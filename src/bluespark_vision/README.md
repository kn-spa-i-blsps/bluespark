# bluespark_vision

Python package for real-time object detection, 3D pose estimation, and distance calculation using YOLO ml models. This package provides vision capabilities for the Bluespark robot, supporting multiple camera types (USB, Raspberry Pi CSI, and TCP).

## Overview

The `bluespark_vision` package provides real-time computer vision processing for object detection and 3D object localization. It includes:

- **Object Detection**: Using YOLO models for real-time inference
- **3D Pose Estimation**: Calculates 3D position and orientation of detected objects relative to the camera
- **Distance Calculation**: Computes distance to objects using camera calibration and known object dimensions
- **Multi-Camera Support**: Supports USB cameras, Raspberry Pi CSI cameras, and TCP camera streams
- **Publishing**: Publishes detected objects with their 3D poses at configurable rates

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


## Nodes

### vision_node

The main node that runs the vision processing pipeline.

#### Node Name
`vision_node_publisher`

#### Executable
```bash
ros2 run bluespark_vision vision_node
```

#### Published Topics

| Topic Name | Message Type | Description |
|-----------|--------------|-------------|
| `/detected_objects` | `bluespark_interfaces/DetectedObjectArray` | Array of detected objects with 3D poses and distances. Published at ~5 Hz (timer period: 0.2s). |

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

None. The node operates independently, reading from camera hardware.

#### Parameters

The following parameters can be set in the ROS 2 parameter server or passed as arguments when launching the node:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `detection_threshold` | float | 0.5 | Objects detected with confidence below this threshold are filtered out. |
| `inference_size` | int | 224 | Input image size for YOLO inference | 
| `camera_mode` | string | `"auto"` | Camera selection mode: `"auto"` (auto-detect, rpi>tcp>usb), `"usb"` (USB camera), `"rpi"` (Raspberry Pi CSI), or `"tcp"` (TCP stream at 127.0.0.1:5000). |
| `camera_width` | int | 640 | Camera frame width in pixels. |
| `camera_height` | int | 480 | Camera frame height in pixels. |
| `timer_period` | float | 0.2 | Time period between consecutive frames in seconds (5 Hz publishing rate). |

#### Processing Pipeline

1. **Camera Capture**: Reads a frame from the configured camera source
2. **Object Detection**: Runs YOLO inference to detect objects in the frame
3. **Pose Estimation**: For each detected object:
   - Calculates 3D position relative to camera using pinhole camera model
   - Computes horizontal and vertical angles from camera center
   - Estimates object rotation based on aspect ratio
4. **Publishing**: Publishes detected objects as a `DetectedObjectArray` message

## Configuration Files

### Camera Calibration (`calibration_files/camera_calibration.json`)

Contains camera intrinsic parameters and distortion coefficients in JSON format:

```json
{
  "camera_matrix": [
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]
  ],
  "dist_coeffs": [k1, k2, p1, p2, k3]
}
```

Where:
- `fx`, `fy`: Focal length in pixels
- `cx`, `cy`: camera center point in pixels
- `k1`, `k2`, `k3`: Radial distortion coefficients
- `p1`, `p2`: Tangential distortion coefficients

### Object Configuration (`calibration_files/object_config.json`)

Defines real-world dimensions of objects for distance calculation:

```json
{
  "objects": {
    "object_name": {
      "real_width": <width_in_meters>,
      "real_height": <height_in_meters>
    }
  }
}
```

Example:
```json
{
  "objects": {
    "bottle": {
      "real_width": 0.08,
      "real_height": 0.30
    },
    "person": {
      "real_width": 0.45,
      "real_height": 1.70
    }
  }
}
```

## Usage Examples

### Basic Launch

```bash
ros2 run bluespark_vision vision_node
```

### Launch with USB Camera

```bash
ros2 run bluespark_vision vision_node --ros-args -p camera_mode:=usb -p camera_width:=640 -p camera_height:=480
```

### Launch with Raspberry Pi Camera

```bash
ros2 run bluespark_vision vision_node --ros-args -p camera_mode:=rpi
```

### Launch with TCP Camera Stream

```bash
ros2 run bluespark_vision vision_node --ros-args -p camera_mode:=tcp
```

### Launch with Custom Detection Threshold

```bash
ros2 run bluespark_vision vision_node --ros-args -p detection_threshold:=0.7 -p inference_size:=320
```

### Subscribe to Detections in Another Node

```python
import rclpy
from rclpy.node import Node
from bluespark_interfaces.msg import DetectedObjectArray

class DetectionListener(Node):
    def __init__(self):
        super().__init__('detection_listener')
        self.subscription = self.create_subscription(
            DetectedObjectArray,
            'detected_objects',
            self.detections_callback,
            10
        )
    
    def detections_callback(self, msg):
        self.get_logger().info(f'Detected {len(msg.objects)} objects')
        for obj in msg.objects:
            self.get_logger().info(
                f'  - {obj.label}: confidence={obj.confidence:.2f}, '
                f'distance={obj.pos_z:.2f}m, '
                f'h_angle={obj.cam_h_angle_deg:.1f}°, '
                f'v_angle={obj.cam_v_angle_deg:.1f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    listener = DetectionListener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Monitor Detections with ROS 2 CLI

```bash
# View all published messages
ros2 topic echo /detected_objects

# Show topic information
ros2 topic info /detected_objects

# Monitor publishing rate
ros2 topic hz /detected_objects

# Display message type details
ros2 interface show bluespark_interfaces/msg/DetectedObjectArray
ros2 interface show bluespark_interfaces/msg/DetectedObject
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
│   ├── vision_node.py                      # Main ROS 2 node
│   ├── detector.py                         # YOLO object detector wrapper
│   ├── simple_distance_calculator.py       # 3D pose and distance calculator
│   ├── camera.py                           # Universal camera interface
│   └── exceptions.py                       # Custom camera exception classes
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

**vision_node.py**: Main ROS 2 node implementing the vision processing pipeline. Manages node lifecycle, timers, publishers, and orchestrates the detection and pose estimation workflow.

**detector.py**: Wrapper around YOLO for object detection. Handles model loading, inference, and bounding box extraction.

**simple_distance_calculator.py**: Implements 3D pose estimation using pinhole camera model. Calculates position, angles, and object rotation from bounding boxes and camera calibration.

**camera.py**: Universal camera interface supporting multiple camera types (USB, Raspberry Pi CSI, TCP). Abstracts camera hardware differences behind a single API.

**exceptions.py**: Custom exception classes for camera initialization errors.

## Related Packages

- **bluespark_interfaces**: Defines custom ROS 2 message types used by this package
- **bluespark_main**: Main package orchestrating robot behavior
