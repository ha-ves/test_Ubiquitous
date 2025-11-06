# YOLOv8 Detection ROS1 Integration

This package provides ROS1 integration for YOLOv8 object detection using ONNX Runtime.

## Overview

The YOLOv8_Detection component has been migrated to ROS1, allowing it to be used as a standard ROS node. The node subscribes to camera images and publishes detected object information.

## Features

- Real-time object detection using YOLOv8 ONNX model
- GPU acceleration support via OpenCL (if available)
- Configurable confidence and IoU thresholds
- Multiple output formats:
  - Object count (std_msgs/UInt16)
  - Visualization markers (visualization_msgs/MarkerArray)
  - Debug images with bounding boxes (sensor_msgs/Image)

## Installation

### Prerequisites

- ROS Melodic or Noetic
- OpenCV 4.x
- ONNX Runtime
- Camera driver (e.g., usb_cam, realsense2_camera)

### Building

```bash
# Navigate to your catkin workspace
cd ~/catkin_ws/src

# Clone or copy this package
# cd ~/catkin_ws/src
# ln -s /path/to/YOLOv8_Detection yolov8_detection

# Build
cd ~/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash
```

## Usage

### Basic Usage

```bash
# Launch the YOLOv8 detection node
roslaunch yolov8_detection yolov8_detection.launch
```

### Custom Parameters

You can customize the node behavior by modifying the launch file or passing parameters:

```bash
roslaunch yolov8_detection yolov8_detection.launch confidence_threshold:=0.8 iou_threshold:=0.5
```

### Parameters

- `onnx_file` (string, default: "best_full.onnx"): Path to the ONNX model file
- `confidence_threshold` (float, default: 0.75): Minimum confidence threshold for detections (0.01-1.0)
- `iou_threshold` (float, default: 0.75): IoU threshold for non-maximum suppression (0.01-1.0)

### Topics

#### Subscribed Topics

- `image_raw` (sensor_msgs/Image): Input camera images (BGR8 encoding)

#### Published Topics

- `detected_objects` (std_msgs/UInt16): Count of detected objects (specifically pins)
- `detection_markers` (visualization_msgs/MarkerArray): Visualization markers for detected objects
- `debug_image` (sensor_msgs/Image): Debug image with bounding boxes and labels (published only if subscribed)

### Detected Classes

The default model detects three classes:
1. **ball** - Bowling ball (red markers)
2. **pin** - Bowling pin (green markers)
3. **board** - Bowling lane board (blue markers)

## Example

### With USB Camera

```bash
# Terminal 1: Start camera
rosrun usb_cam usb_cam_node

# Terminal 2: Start detection
roslaunch yolov8_detection yolov8_detection.launch

# Terminal 3: View debug image
rosrun image_view image_view image:=/yolov8/debug_image

# Terminal 4: Monitor detections
rostopic echo /yolov8/detected_objects
```

### With RealSense Camera

```bash
# Terminal 1: Start RealSense
roslaunch realsense2_camera rs_camera.launch

# Terminal 2: Start detection (remap to RealSense color topic)
roslaunch yolov8_detection yolov8_detection.launch

# Note: Modify the launch file to remap:
# <remap from="image_raw" to="/camera/color/image_raw" />
```

## Performance

The node provides timing information for each processing stage:
- Preprocessing time
- Inference time
- Postprocessing time
- Total processing time

These are logged at 1 Hz to avoid cluttering the console.

## Comparison with OpenRTM Component

The ROS1 node maintains the same core functionality as the original OpenRTM component:

| Feature | OpenRTM | ROS1 |
|---------|---------|------|
| Input | RTC::CameraImage | sensor_msgs/Image |
| Output | RTC::TimedUShort | std_msgs/UInt16 |
| Visualization | - | MarkerArray + debug image |
| Configuration | Runtime parameters | ROS parameters |
| Middleware | OpenRTM-aist | ROS1 |

## Troubleshooting

### ONNX Runtime not found

Make sure ONNX Runtime is installed and the `FindOnnxRuntime.cmake` file correctly points to the installation location.

### Model file not found

Ensure the ONNX model file (`best_full.onnx`) is in the correct location or update the `onnx_file` parameter to point to the correct path.

### No detections

- Check that the camera is publishing images: `rostopic hz /camera/image_raw`
- Verify the confidence threshold is not too high
- Ensure the model file is compatible with your ONNX Runtime version

### Performance issues

- Enable GPU acceleration if available (OpenCL support)
- Reduce image resolution from the camera
- Adjust the confidence threshold to filter more detections

## License

LGPL (same as the original component)

## References

- Original OpenRTM component: YOLOv8_Detection
- YOLOv8: https://github.com/ultralytics/ultralytics
- ONNX Runtime: https://onnxruntime.ai/
