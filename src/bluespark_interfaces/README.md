# bluespark_interfaces

Custom ROS 2 message and service types shared by the BlueSpark packages. No code,
only definitions; colcon generates the Python/C++ classes from them.

---

## Messages

| Type                  | Purpose                                                                                                                                                                                 | Published by                   | Used by                                |
| :-------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :----------------------------- | :------------------------------------- |
| `DetectedObject`      | One YOLO detection (see fields below)                                                                                                                                                   | - (element of the array)       | -                                      |
| `DetectedObjectArray` | All detections from one frame: `header` (stamp, camera `frame_id`) + `DetectedObject[] objects` (list of `DetectedObject`). Topics: `/front/detected_objects`, `/side/detected_objects` | `bluespark_vision/vision_node` | `bluespark_autonomy/BlackboardManager` |
| `VisionTarget`        | Common format for the down-camera detectors (pipeline / path marker / bin): offset from image center, angle, size, symbol                                                               | nobody yet                     | nobody yet                             |

`VisionTarget` is kept as the planned interface for future down-camera detectors. It replaced the
older per-detector messages (`PipelineData`, `PathMarkerData`, `BinData`), which were removed. The
`source` field says which detector sent it (`"pipeline"`, `"path_marker"`, `"bin"`).

The detectors that published it were removed after RoboSub (commit `7a859a7`). Their code is a good
starting point and is still in git history:

```bash
git show 7a859a7^:src/bluespark_vision/bluespark_vision/pipeline_node.py
git ls-tree -r --name-only 7a859a7^ -- src/bluespark_vision   # list all files from that version
```

### `DetectedObject` fields

`DetectedObject` has no header of its own: the timestamp and camera come from the
`DetectedObjectArray` it is sent in.

| Field                                | Unit    | Meaning                                              |
| :----------------------------------- | :------ | :--------------------------------------------------- |
| `label`, `confidence`                | -       | class name and score from YOLO                       |
| `pos_x`, `pos_y`, `pos_z`            | meters  | position relative to the camera                      |
| `cam_h_angle_deg`, `cam_v_angle_deg` | degrees | horizontal / vertical angle from the image center    |
| `obj_rotation_deg`                   | degrees | estimated rotation of the object around its own axis |
| `x_center`, `y_center`               | pixels  | center of the bounding box                           |

---

## Services

| Type            | Request            | Response       | Used by                                                          |
| :-------------- | :----------------- | :------------- | :--------------------------------------------------------------- |
| `SetRCOverride` | `uint16 pwm_value` | `bool success` | `control/set_*` services of `bluespark_control/rc_override_node` |

---

## Adding a new type

1. Add the `.msg` / `.srv` file to `msg/` or `srv/`.
2. Add it to `rosidl_generate_interfaces(...)` in `CMakeLists.txt`.
3. Rebuild (`colcon build`) and re-source `install/setup.bash`.

---

## TODO

- `DetectedObject` has no unique id, and `BlackboardManager` stores objects in a dict
  keyed by `label`, so two objects of the same class in one frame overwrite each other.
- Possible new types from planned work: heartbeat / control command topic (dead man
  switch in `bluespark_control`), a payload service that returns success
  (`bluespark_peripherals`).
- `package.xml`: add a description, maintainer name is `root`.
