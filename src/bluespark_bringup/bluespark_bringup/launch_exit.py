"""
launch_exit.py — end the whole launch when one key process exits, with ITS result.

    RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=end_launch_on_exit))

  * process exits 0      -> Shutdown(): launch ends with return code 0
  * process exits != 0   -> launch logs "Caught exception in launch: <name> exited
                            with code N", shuts everything down, return code 1
  * launch already shutting down (Ctrl+C) -> nothing, keeps launch's own result

Why not just on_exit=Shutdown(): that always gives return code 0, so a
supervisor (run_governer.sh, docker restart) could not tell a crash from a
clean finish. An exception raised while launch handles an event is its
documented way to fail with return code 1.
"""

from launch.actions import Shutdown


def end_launch_on_exit(event, context):
    if context.is_shutdown:
        return None
    if event.returncode == 0:
        return [Shutdown(reason=f"{event.process_name} finished")]
    raise RuntimeError(f"{event.process_name} exited with code {event.returncode}")
