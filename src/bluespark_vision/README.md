# bluespark_vision

Python package for real-time object detection, 3D pose estimation, and distance calculation using YOLO ml models. This package provides vision capabilities for the Bluespark robot, supporting multiple camera types (USB, Raspberry Pi CSI, and TCP).

## Overview

The `bluespark_vision` package provides real-time computer vision processing for object detection and 3D object localization. It includes:

- **Object Detection**: Using YOLO models for real-time inference
- **3D Pose Estimation**: Calculates 3D position and orientation of detected objects relative to the camera
- **Distance Calculation**: Computes distance to objects using camera calibration and known object dimensions
- **Classic task detectors**: HSV/contour-based detectors for the pipeline, the path marker, and the bin, publishing a shared `VisionTarget` message
- **Multi-Camera Support**: Supports USB cameras, Raspberry Pi CSI cameras, and TCP camera streams
- **Publishing**: Publishes detected objects with their 3D poses at configurable rates

### Architecture at a glance

The camera is owned by a single node and shared by every detector. Detectors do
**not** open the camera themselves — they subscribe to the frames it publishes:

```
                             ┌─▶ vision_node        ─▶ /detected_objects   (YOLO, 3D)
camera_node ─▶ /camera/image_raw ─┼─▶ pipeline_node      ─▶ /pipeline/data      (VisionTarget)
                             ├─▶ path_marker_node   ─▶ /path_marker/data   (VisionTarget)
                             └─▶ bin_detector_node  ─▶ /bin/data           (VisionTarget)
```

This keeps a single camera as one shared resource: several detectors can run at
once (e.g. following the pipeline on the bottom camera while YOLO watches for a
gate), which would be impossible if each node opened its own camera.

## Dependencies

### ROS 2 Dependencies

- `rclpy`
- `bluespark_interfaces`: Custom ROS 2 message definitions

### Python Dependencies

The following Python packages are required:

- `ultralytics>=8.0.0`
- `opencv-python>=4.5.0`
- `numpy>=1.19.0`
- `picamera2`
- `torch` *(installed with ultralytics)*

Install Python dependencies using:

```bash
pip install ultralytics opencv-python numpy
pip install picamera2  # For Raspberry Pi camera support
```

Note: the classic detectors and the frame conversion helper (`image_utils`) rely
only on `numpy` + `opencv-python`, so they do **not** require `cv_bridge`.


## Nodes

### camera_node

Owns the camera hardware and publishes frames for every other vision node to
consume. **Every detector depends on this node** — if it is not running and
publishing, the detectors receive no frames and produce no detections.

#### Node Name
`camera_node`

#### Executable
```bash
ros2 run bluespark_vision camera_node
```

#### Published Topics

| Topic Name | Message Type | Description |
|-----------|--------------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw BGR frames. The `frame_id` in the header identifies which physical camera the frame came from (relevant once multiple cameras are used). |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_mode` | string | `"auto"` | Camera selection mode: `"auto"` (auto-detect, rpi>tcp>usb), `"usb"` (USB camera), `"rpi"` (Raspberry Pi CSI), or `"tcp"` (TCP stream at 127.0.0.1:5000). |
| `camera_width` | int | 640 | Camera frame width in pixels. |
| `camera_height` | int | 480 | Camera frame height in pixels. |

> Note: camera selection and resolution belong to `camera_node`. The detector
> nodes never touch camera hardware; they only read `/camera/image_raw`.

---

### vision_node

The main node that runs the YOLO detection and 3D pose estimation pipeline.

#### Node Name
`vision_node_publisher`

#### Executable
```bash
ros2 run bluespark_vision vision_node
```

#### Published Topics

| Topic Name | Message Type | Description |
|-----------|--------------|-------------|
| `/detected_objects` | `bluespark_interfaces/DetectedObjectArray` | Array of detected objects with 3D poses and distances. |

**Message Structure** (`DetectedObjectArray`):
- `header` (std_msgs/Header): Timestamp and frame ID
- `objects` (DetectedObject[]): Array of detected objects

**Message Structure** (`DetectedObject`):
- `label` (string): Class name
- `confidence` (float32): Detection confidence score (0.0 to 1.0)
- `pos_x`, `pos_y`, `pos_z` (float32): 3D position in meters relative to camera frame
- `cam_h_angle_deg`, `cam_v_angle_deg` (float32): Horizontal and vertical angles from camera center in degrees
- `obj_rotation_deg` (float32): Estimated rotation angle of object in degrees
- `x_center`, `y_center` (int16): Center of bounding box in image pixels

#### Subscribed Topics

| Topic Name | Message Type | Description |
|-----------|--------------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw frames from `camera_node`. |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `detection_threshold` | float | 0.5 | Objects detected with confidence below this threshold are filtered out. |
| `inference_size` | int | 224 | Input image size for YOLO inference. |

#### Processing Pipeline

1. **Frame intake**: Receives a frame from `/camera/image_raw`
2. **Object Detection**: Runs YOLO inference to detect objects in the frame
3. **Pose Estimation**: For each detected object:
   - Calculates 3D position relative to camera using pinhole camera model
   - Computes horizontal and vertical angles from camera center
   - Estimates object rotation based on aspect ratio
4. **Publishing**: Publishes detected objects as a `DetectedObjectArray` message

---

### Classic detectors: pipeline_node / path_marker_node / bin_detector_node

Three lightweight HSV/contour detectors, one per competition task. They share the
same shape: subscribe to `/camera/image_raw`, run their (untouched) detector
class, smooth the result, and publish a **shared** `VisionTarget` message. Each
one only **reports** what it sees — none of them commands motion.

#### Executables
```bash
ros2 run bluespark_vision pipeline_node
ros2 run bluespark_vision path_marker_node
ros2 run bluespark_vision bin_detector_node
```

#### The shared VisionTarget message

All three detectors publish `bluespark_interfaces/VisionTarget`. Because it is
shared across three different tasks, **each detector fills only a subset of the
fields, and leaves the rest at defaults (0 / empty / false).** A consumer must
know which fields are meaningful for the `source` it is reading.

| Field | Type | Meaning |
|-------|------|---------|
| `header` | std_msgs/Header | `frame_id` is copied from the incoming frame → identifies the source camera. |
| `source` | string | Which detector produced this: `"pipeline"`, `"path_marker"`, or `"bin"`. Read this to know how to interpret the rest. |
| `detected` | bool | Whether the primary target was found this frame. **Always check first.** |
| `offset_x` | float32 | Horizontal offset of the target from image center, `[-1, 1]`. Negative = left, positive = right. |
| `offset_y` | float32 | Vertical offset, `[-1, 1]`. (Pipeline leaves this at 0.) |
| `angle_deg` | float32 | Orientation of the target/line, `[-90, 90]`. (Bin leaves this at 0.) |
| `area_fraction` | float32 | Fraction of the frame the target occupies (proximity). (Pipeline leaves this at 0.) |
| `symbol` | string | Bin only: `"flame"` / `"drop"` / `""`. Empty for the others. |
| `confidence` | float32 | **Meaning depends on source — see per-detector notes below.** |
| `tip_direction` | float32 | Path marker only: arrow pointing angle `[0, 360]`. |
| `tip_direction_valid` | bool | Path marker only: whether `tip_direction` is meaningful. |
| `bbox_x/y/w/h` | int32 | Pixel bounding box. **Meaning depends on source — see notes.** |

##### Three non-obvious things every consumer must know

Because the message is shared, three fields are overloaded in ways that are not
visible from the field name alone:

1. **`pipeline` reuses `bbox_w` for line width.** `VisionTarget` has no dedicated
   "line width" field, so for `source == "pipeline"` the pixel width of the
   detected line is carried in `bbox_w`. The other bbox fields are 0. Do **not**
   read it as a bounding-box width for the pipeline.

2. **`path_marker` has two independent validity flags.** `detected` means "I see
   the marker"; `tip_direction_valid` means "I also know which way it points".
   These can differ: the marker can be reliably detected (`detected == true`)
   while its direction is undetermined (`tip_direction_valid == false`, e.g. the
   arrow is too symmetric or too small). If you need the pointing direction,
   check `tip_direction_valid`, **not** `detected`.

3. **`bin` overloads `confidence` to mean *symbol* confidence, and `symbol` can be
   empty even when detected.** For the bin, `confidence` is how sure we are about
   the **symbol**, not about seeing the container. And `detected == true` with
   `symbol == ""` is a valid state: "I see the bin but haven't determined the
   symbol yet". So "I see the bin" (`detected`) and "I know the symbol"
   (`symbol != ""`) are two separate conditions.

##### Shared smoothing behavior

All three apply an exponential moving average (EMA): `smoothed = alpha * new +
(1 - alpha) * previous` (default `alpha = 0.7`). When a frame has **no**
detection, they do **not** snap offsets back to zero — instead `confidence`
**decays** (`confidence *= (1 - alpha)`) while the last geometry persists. A brief
loss of the target therefore fades confidence rather than throwing the control
sideways; a consumer should gate on a confidence threshold.

---

#### pipeline_node

Detects the pipeline (a bright line on the pool floor) so the robot can follow it.

- **Publishes**: `/pipeline/data`, `source = "pipeline"`
- **Fills**: `detected`, `offset_x` (sway correction), `angle_deg` (yaw correction), `confidence` (line length ÷ frame diagonal), `bbox_w` (= line width in px, see note 1)
- **Algorithm**: HSV threshold → morphological cleanup → Canny + probabilistic Hough → pick the **longest** line segment → measure its center offset, tilt, and length.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hsv_h_min`/`hsv_h_max` | 0 / 180 | Hue range of the line. |
| `hsv_s_min`/`hsv_s_max` | 0 / 35 | Saturation range (low = washed-out / white). |
| `hsv_v_min`/`hsv_v_max` | 200 / 255 | Value range (high = bright). |
| `hough_threshold` | 50 | Min votes for a Hough line. |
| `hough_min_line_length` | 80 | Shortest accepted segment (px). |
| `hough_max_line_gap` | 30 | Max gap bridged within one line (px). |
| `ema_alpha` | 0.7 | Smoothing factor. |

#### path_marker_node

Detects the orange arrow-shaped path marker and the direction it points.

- **Publishes**: `/path_marker/data`, `source = "path_marker"`
- **Fills**: `detected`, `offset_x`, `angle_deg`, `confidence`, `area_fraction`, `tip_direction`, `tip_direction_valid` (see note 2)
- **Algorithm**: HSV threshold → morphology → largest contour → reject if too small (`min_area_fraction`) or not elongated enough (`min_aspect_ratio`) → `minAreaRect` gives the marker angle → **tip direction** is derived from contour moments (the arrow's mass is asymmetric, so the vector from the geometric center to the center of mass points toward the tip).

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hsv_h_min`/`hsv_h_max` | 10 / 25 | Hue range (orange). |
| `hsv_s_min`/`hsv_s_max` | 100 / 255 | Saturation range. |
| `hsv_v_min`/`hsv_v_max` | 100 / 255 | Value range. |
| `min_area_fraction` | 0.005 | Reject contours smaller than this fraction of the frame. |
| `min_aspect_ratio` | 2.0 | Reject contours that are not elongated (filters non-arrow blobs). |
| `ema_alpha` | 0.7 | Smoothing factor. |

#### bin_detector_node

Detects the bin (a colored container) and identifies the symbol inside it.

- **Publishes**: `/bin/data`, `source = "bin"`
- **Fills**: `detected`, `offset_x`, `offset_y`, `area_fraction`, `symbol` (`"flame"`/`"drop"`/`""`), `confidence` (= symbol confidence, see note 3), full `bbox_x/y/w/h` (real container box)
- **Algorithm** (two stages):
  1. **Find the container** by HSV → morphology → largest contour → bounding box → offsets + area.
  2. **Identify the symbol** inside that box, using one of two methods (`symbol_method`): **HSV** (compare how many pixels match the flame color vs the drop color) or **template matching** (match provided flame/drop template images).
- **Symbol stabilization**: a single frame can misread the symbol, so the node keeps a **majority vote over a sliding window** (`vote_window` frames) and publishes the winner. The geometry (`offset_x/y`, `area_fraction`, symbol confidence) is EMA-smoothed as usual. When the bin leaves the frame, the vote history is cleared and the symbol resets to empty.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `symbol_method` | `"hsv"` | `"hsv"` or `"template"`. |
| `bin_hsv_h/s/v_min/max` | 100–130 / 80–255 / 30–150 | HSV range of the container. |
| `flame_hsv_*` | 0–25 / 100–255 / 150–255 | HSV range of the flame symbol. |
| `drop_hsv_*` | 140–170 / 80–255 / 150–255 | HSV range of the drop symbol. |
| `flame_template_path` / `drop_template_path` | `""` | Template images (only for `template` method). |
| `template_threshold` | 0.7 | Min match score for template method. |
| `min_bin_area_fraction` | 0.01 | Reject containers smaller than this fraction of the frame. |
| `ema_alpha` | 0.7 | Smoothing factor. |
| `vote_window` | 10 | Frames in the symbol majority-vote window. |

#### Inspecting any detector

```bash
ros2 topic echo /pipeline/data
ros2 topic echo /path_marker/data
ros2 topic echo /bin/data
ros2 interface show bluespark_interfaces/msg/VisionTarget
```

## Calibration

### Camera Calibration

To obtain accurate 3D position estimates, the camera must be calibrated. Camera calibration can be performed using designed calibration module from [bluespark vision repository r&d](https://github.com/MaciejBorowiecki/BlueSpark-autonomy.git):

1. Use the `BlueSpark-autonomy/bluespark_distance_estiation/calibration/calibrate_charuco.py` *requires charuco calibration board, which can be created from the same directory with `generate_charuco` files.*
2. Export the calibration to JSON format (camera matrix and distortion coefficients)
3. Place the JSON file in `calibration_files/camera_calibration.json`

Example calibration JSON structure:
```json
{
  "camera_matrix": [
    [521.5, 0.0, 320.0],
    [0.0, 520.0, 240.0],
    [0.0, 0.0, 1.0]
  ],
  "dist_coeffs": [0.1, -0.2, 0.0, 0.0, 0.0]
}
```

### Object Dimensions Calibration

Measure the real-world width and height of objects you want to detect and add them to `calibration_files/object_config.json`. This is critical for accurate distance estimation.

Measure objects in their typical orientation relative to the camera and record both width and height in meters.

## File Overviews

### Structure

```
bluespark_vision/
├── bluespark_vision/
│   ├── camera_node.py                      # Owns the camera, publishes /camera/image_raw
│   ├── vision_node.py                      # YOLO detection + 3D pose node
│   ├── pipeline_node.py                    # Pipeline follower detector node
│   ├── path_marker_node.py                 # Path marker detector node
│   ├── bin_detector_node.py                # Bin + symbol detector node
│   ├── image_utils.py                      # ROS Image <-> numpy conversion (no cv_bridge)
│   ├── detector.py                         # YOLO object detector wrapper
│   ├── simple_distance_calculator.py       # 3D pose and distance calculator
│   ├── camera.py                           # Universal camera interface
│   ├── exceptions.py                       # Custom camera exception classes
│   └── detectors/                          # Classic detector logic (task-specific)
│       ├── pipeline.py                     # HSV + Hough line detector
│       ├── path_marker.py                  # HSV + contour arrow detector
│       └── bin_detector.py                 # HSV/template bin + symbol detector
├── calibration_files/
│   ├── camera_calibration.json             # Camera parameters
│   └── object_config.json                  # Object dimensions configuration
├── ml_models/
│   └── model.pt                            # Yolo type model
├── test/                                   # Unit and integration tests
├── package.xml
├── setup.py
└── README.md
```

### Core Modules

**camera_node.py**: Owns the camera hardware and publishes frames on `/camera/image_raw`. The single source of frames for every detector.

**vision_node.py**: ROS 2 node implementing the YOLO detection + pose estimation pipeline. Subscribes to `/camera/image_raw`, publishes `DetectedObjectArray`.

**pipeline_node.py / path_marker_node.py / bin_detector_node.py**: The three classic-detector nodes. Each subscribes to `/camera/image_raw`, runs its detector from `detectors/`, smooths the result, and publishes a `VisionTarget`.

**image_utils.py**: Lightweight conversion between `sensor_msgs/Image` and numpy arrays, using only numpy (a `cv_bridge` replacement).

**detector.py**: Wrapper around YOLO for object detection. Handles model loading, inference, and bounding box extraction.

**simple_distance_calculator.py**: Implements 3D pose estimation using pinhole camera model. Calculates position, angles, and object rotation from bounding boxes and camera calibration.

**camera.py**: Universal camera interface supporting multiple camera types (USB, Raspberry Pi CSI, TCP). Abstracts camera hardware differences behind a single API.

**exceptions.py**: Custom exception classes for camera initialization errors.

**detectors/**: The task-specific detection logic (pure OpenCV/numpy, no ROS). Each detector takes a frame and returns a small dataclass; the corresponding node wraps it in a `VisionTarget`.

## Related Packages

- **bluespark_interfaces**: Defines custom ROS 2 message types used by this package
- **bluespark_main**: Main package orchestrating robot behavior