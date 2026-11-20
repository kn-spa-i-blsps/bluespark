"""
wait_for.py — launch action that waits for a ROS message before starting more actions.

Lets a launch file start nodes in order based on real readiness instead of fixed
delays:

    WaitForMessage('/mavros/state', State, lambda m: m.connected,
                   timeout=30.0,
                   then=[rc_override, vehicle_manager])

How it works:
  - It subscribes through the launch's own internal ROS node (launch_ros keeps one
    per launch; get_ros_node(context) returns it, spun in a background thread).
  - The subscription callback runs in that ROS thread. On the first message for
    which `predicate(msg)` is True it hands control back to the launch's asyncio
    loop (call_soon_threadsafe), which emits a private event. A one-shot event
    handler then runs the `then` actions. Later messages are ignored.
  - On timeout the action fails: launch logs the reason, shuts everything down and
    exits with return code 1. run_governer.sh treats rc != 0 as a crash (disarm +
    restart). A plain Shutdown() would exit with rc 0 = "mission complete".
  - On launch shutdown (e.g. Ctrl+C) while still waiting, the wait just ends
    quietly, so it does not turn a normal Ctrl+C into rc 1.

Readiness convention for our own nodes:
  A node that others depend on publishes its name (std_msgs/String) on
  READY_TOPIC with READY_QOS once it is ready (e.g. after creating its services).
  READY_QOS is transient_local, so a launch that subscribes later still gets
  every publisher's last "ready" message. Use ready_all({...}) as the predicate.
"""

import asyncio
from typing import Callable, Iterable, List, Optional

import launch
from launch import LaunchContext, LaunchDescriptionEntity, SomeSubstitutionsType
from launch.action import Action
from launch.event import Event
from launch.event_handler import EventHandler
from launch.event_handlers import OnShutdown
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.ros_adapters import get_ros_node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# Must match the topic/QoS used by the nodes that announce readiness
# (bluespark_control: rc_override_node, vehicle_manager_node).
READY_TOPIC = '/bluespark/ready'
READY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ready_all:  # noqa: N801 - used like a function: ready_all({...})
    """Predicate for READY_TOPIC: True once every name in `names` has been seen."""

    def __init__(self, names: Iterable[str]) -> None:
        self.required = set(names)
        self.seen = set()
        # Picked up by WaitForMessage for its log / timeout messages.
        self.description = f'ready: {", ".join(sorted(self.required))}'

    def __call__(self, msg) -> bool:
        if msg.data in self.required:
            self.seen.add(msg.data)
        return self.required <= self.seen


class _MessageMatched(Event):
    """Private event: a WaitForMessage got its message."""

    name = 'bluespark.MessageMatched'

    def __init__(self, action: 'WaitForMessage') -> None:
        super().__init__()
        self.action = action


class WaitForMessage(Action):
    """Run `then` once a message on `topic` satisfies `predicate`."""

    def __init__(
        self,
        topic: SomeSubstitutionsType,
        msg_type,
        predicate: Callable,
        then: List[LaunchDescriptionEntity],
        *,
        timeout: Optional[float] = None,
        qos: Optional[QoSProfile] = None,
        description: Optional[str] = None,
        **kwargs,
    ) -> None:
        super().__init__(**kwargs)
        # May contain substitutions (e.g. ['/', LaunchConfiguration('namespace'), '/x']);
        # resolved to a plain string in execute().
        self.__topic_subs = normalize_to_list_of_substitutions(topic)
        self.__topic = topic if isinstance(topic, str) else '<unresolved>'
        self.__msg_type = msg_type
        self.__predicate = predicate
        self.__then = list(then)
        self.__timeout = timeout
        self.__qos = qos if qos is not None else QoSProfile(depth=10)
        self.__description = (
            description or getattr(predicate, 'description', None) or 'condition')
        self.__logger = launch.logging.get_logger(__name__)

        self.__future: Optional[asyncio.Future] = None
        self.__timer: Optional[asyncio.TimerHandle] = None
        self.__node = None
        self.__subscription = None
        self.__triggered = False  # touched only in the ROS thread

    @property
    def what(self) -> str:
        return f'{self.__topic} ({self.__description})'

    def execute(self, context: LaunchContext):
        loop = context.asyncio_loop
        if loop is None:  # only None before the launch service starts running
            raise RuntimeError('WaitForMessage executed outside a running launch')
        self.__topic = perform_substitutions(context, self.__topic_subs)
        self.__future = loop.create_future()

        # Runs the `then` actions once our private event arrives.
        context.register_event_handler(EventHandler(
            matcher=lambda event: isinstance(event, _MessageMatched) and event.action is self,
            entities=self.__then,
            handle_once=True,
        ))
        # Stop waiting quietly if the launch shuts down first.
        context.register_event_handler(OnShutdown(
            on_shutdown=lambda event, ctx: self.__finish()))

        if self.__timeout is not None:
            self.__timer = loop.call_later(self.__timeout, self.__on_timeout)

        def on_message(msg) -> None:  # ROS thread
            if self.__triggered:
                return
            try:
                matched = self.__predicate(msg)
            except Exception as exc:
                self.__logger.error(f'Predicate for {self.what} raised: {exc}')
                return
            if matched:
                self.__triggered = True
                loop.call_soon_threadsafe(self.__on_matched, context)

        self.__node = get_ros_node(context)
        self.__subscription = self.__node.create_subscription(
            self.__msg_type, self.__topic, on_message, self.__qos)

        self.__logger.info(f'Waiting for {self.what}...')
        return None

    def get_asyncio_future(self) -> Optional[asyncio.Future]:
        # Keeps launch aware of the pending wait; an exception set here (timeout)
        # makes launch shut down with return code 1.
        return self.__future

    # --- everything below runs in the launch's asyncio thread ---

    def __on_matched(self, context: LaunchContext) -> None:
        if self.__future is None or self.__future.done():
            return  # already timed out or shutting down
        self.__logger.info(f'Got {self.what}.')
        context.emit_event_sync(_MessageMatched(self))
        self.__finish()

    def __on_timeout(self) -> None:
        if self.__future is None or self.__future.done():
            return
        reason = f'Timed out after {self.__timeout:g} s waiting for {self.what}'
        self.__logger.error(reason)
        self.__cleanup()
        self.__future.set_exception(TimeoutError(reason))

    def __finish(self) -> None:
        """End the wait successfully (matched or launch shutting down)."""
        self.__cleanup()
        if self.__future is not None and not self.__future.done():
            self.__future.set_result(None)

    def __cleanup(self) -> None:
        if self.__timer is not None:
            self.__timer.cancel()
            self.__timer = None
        if self.__node is not None and self.__subscription is not None:
            try:
                self.__node.destroy_subscription(self.__subscription)
            except Exception:
                pass  # node may already be shutting down
            self.__subscription = None
