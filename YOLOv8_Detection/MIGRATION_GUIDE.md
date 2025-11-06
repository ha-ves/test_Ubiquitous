# Migration Guide: YOLOv8_Detection from OpenRTM to ROS1

This document describes the migration of the YOLOv8_Detection component from OpenRTM-aist to ROS1.

## What Changed

### Architecture

- **Before**: OpenRTM-aist DataFlowComponent
- **After**: Standard ROS1 node using `roscpp`

### Communication

| Aspect | OpenRTM | ROS1 |
|--------|---------|------|
| Middleware | OpenRTM-aist / CORBA | ROS1 / XMLRPC |
| Input Port | `RTC::InPort<RTC::CameraImage>` | `image_transport::Subscriber` |
| Output Port | `RTC::OutPort<RTC::TimedUShort>` | `ros::Publisher` |
| Data Type (Input) | `RTC::CameraImage` | `sensor_msgs::Image` |
| Data Type (Output) | `RTC::TimedUShort` | `std_msgs::UInt16` |

### Configuration

- **OpenRTM**: Runtime configuration via `rtc.conf` and configuration parameters
- **ROS1**: Launch-time configuration via ROS parameters in launch files

### Build System

- **OpenRTM**: CMake with OpenRTM-specific modules
- **ROS1**: Catkin (CMake + ROS-specific conventions)

## What Stayed the Same

1. **Core Algorithm**: The YOLOv8 detection algorithm using ONNX Runtime remains unchanged
2. **Dependencies**: 
   - OpenCV 4.x (with OpenCL support)
   - ONNX Runtime
3. **Model**: Same ONNX model file (`best_full.onnx`)
4. **Detection Classes**: ball, pin, board
5. **Thresholds**: Confidence and IoU thresholds with same defaults (0.75)

## File Structure

```
YOLOv8_Detection/
├── CMakeLists.txt               # Dual-mode CMake (ROS1/OpenRTM detection)
├── CMakeLists_openrtm.txt       # Original OpenRTM CMakeLists (backup)
├── package.xml                  # ROS1 package manifest
├── README.md                    # Updated main documentation
├── ROS1_README.md              # ROS1-specific documentation
├── launch/
│   └── yolov8_detection.launch # ROS1 launch file
├── ros1/
│   └── yolov8_detection_node.cpp # ROS1 node implementation
├── src/                         # Original OpenRTM source files
│   ├── YOLOv8_Detection.cpp
│   └── YOLOv8_DetectionComp.cpp
└── include/                     # Original OpenRTM headers
    └── YOLOv8_Detection/
        └── YOLOv8_Detection.h
```

## Key Implementation Changes

### 1. Initialization

**OpenRTM**:
```cpp
RTC::ReturnCode_t YOLOv8_Detection::onActivated(RTC::UniqueId ec_id) {
    // Initialize ONNX session
    infWrap = Ort::Session(ortEnvWrap, ...);
    return RTC::RTC_OK;
}
```

**ROS1**:
```cpp
YOLOv8DetectionNode() {
    // Initialize in constructor
    if (!initializeONNX()) {
        ros::shutdown();
    }
}
```

### 2. Image Processing

**OpenRTM**:
```cpp
RTC::ReturnCode_t YOLOv8_Detection::onExecute(RTC::UniqueId ec_id) {
    if (m_camImageIn.isNew()) {
        m_camImageIn.read();
        img_src = cv::Mat(m_camImage.height, m_camImage.width, 
                         CV_8UC3, m_camImage.pixels.get_buffer());
        // Process...
    }
    return RTC::RTC_OK;
}
```

**ROS1**:
```cpp
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, 
        sensor_msgs::image_encodings::BGR8);
    img_src_ = cv_ptr->image;
    // Process...
}
```

### 3. Publishing Results

**OpenRTM**:
```cpp
m_outObjects.data = object_count;
m_outObjectsOut.write(m_outObjects);
```

**ROS1**:
```cpp
std_msgs::UInt16 count_msg;
count_msg.data = object_count;
object_count_pub_.publish(count_msg);
```

## Additional ROS1 Features

The ROS1 implementation includes additional features not present in the OpenRTM version:

1. **Visualization Markers**: Publishes `visualization_msgs::MarkerArray` for RViz visualization
2. **Debug Images**: Publishes annotated images with bounding boxes and labels
3. **Performance Logging**: Detailed timing information using `ROS_INFO_THROTTLE`
4. **Better Error Handling**: ROS-style error reporting with `ROS_ERROR` and `ROS_WARN`

## Building

### OpenRTM Build

```bash
cd YOLOv8_Detection
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

### ROS1 Build

```bash
# In catkin workspace
cd ~/catkin_ws/src
ln -s /path/to/YOLOv8_Detection yolov8_detection
cd ~/catkin_ws
catkin_make
```

## Running

### OpenRTM

```bash
# Start name server
rtm-naming

# Run component
./YOLOv8_DetectionComp
```

### ROS1

```bash
# Launch node
roslaunch yolov8_detection yolov8_detection.launch

# Or run directly
rosrun yolov8_detection yolov8_detection_node
```

## Migration Notes

1. **No Breaking Changes to OpenRTM Version**: The original OpenRTM component remains fully functional and can be built using `CMakeLists_openrtm.txt`

2. **Shared Dependencies**: Both versions use the same ONNX model and have similar OpenCV/ONNX Runtime requirements

3. **Different Execution Models**: 
   - OpenRTM uses a state machine (inactive → active → executing)
   - ROS1 uses a callback-driven model

4. **Configuration Flexibility**: ROS1 version supports runtime parameter updates via `rosparam` and `dynamic_reconfigure` (if extended)

## Testing

To test the ROS1 migration:

1. **With Bag Files**:
   ```bash
   # Play a rosbag with camera images
   rosbag play camera_data.bag
   roslaunch yolov8_detection yolov8_detection.launch
   ```

2. **With Live Camera**:
   ```bash
   # Terminal 1: Camera driver
   roslaunch usb_cam usb_cam-test.launch
   
   # Terminal 2: Detection
   roslaunch yolov8_detection yolov8_detection.launch
   
   # Terminal 3: Visualization
   rosrun image_view image_view image:=/yolov8/debug_image
   ```

3. **Monitor Performance**:
   ```bash
   rostopic hz /yolov8/detected_objects
   rostopic echo /yolov8/detected_objects
   ```

## Future Enhancements

Potential improvements for the ROS1 implementation:

1. **Dynamic Reconfigure**: Add runtime parameter adjustment
2. **Action Server**: For on-demand detection requests
3. **TF Integration**: Publish detection positions in 3D space
4. **Custom Message Types**: Create dedicated detection message type
5. **Service Interface**: Add service for single-frame detection
6. **Multi-threading**: Optimize performance with threaded spinning

## Conclusion

The migration to ROS1 maintains all core functionality while providing better integration with the ROS ecosystem. Both versions can coexist, allowing users to choose based on their middleware requirements.
