// ROS1 node for YOLOv8 Detection using ONNX Runtime
// Migrated from OpenRTM component

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <onnxruntime_cxx_api.h>

#include <numeric>
#include <functional>
#include <chrono>

struct Detection {
    cv::Rect box;
    float score;
    int class_id;
};

class YOLOv8DetectionNode {
private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher object_count_pub_;
    ros::Publisher marker_pub_;
    image_transport::Publisher debug_image_pub_;

    // ONNX Runtime
    Ort::Env ort_env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_;
    
    std::vector<int64_t> input_dims_;
    std::vector<int64_t> output_dims_;
    int64_t input_size_;
    int64_t output_size_;
    
    cv::Size input_size_cv_;
    std::vector<std::string> labels_;
    int64_t channels_;
    
    std::vector<float> output_tensor_;
    
    // OpenCV buffers
    cv::Rect pad_roi_rect_;
    cv::Mat img_src_;
    cv::UMat img_src_n_, img_in_, blob_;
    
    // Parameters
    std::string onnx_file_;
    float confidence_threshold_;
    float iou_threshold_;
    
    // Image dimensions
    int in_w_, in_h_;
    float scale_;
    
    bool initialized_;

public:
    YOLOv8DetectionNode() 
        : nh_()
        , pnh_("~")
        , it_(nh_)
        , ort_env_(ORT_LOGGING_LEVEL_WARNING, "YOLOv8_Detection_ROS")
        , memory_info_(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault))
        , in_w_(0)
        , in_h_(0)
        , scale_(1.0)
        , initialized_(false)
    {
        // Load parameters
        pnh_.param<std::string>("onnx_file", onnx_file_, "best_full.onnx");
        pnh_.param<float>("confidence_threshold", confidence_threshold_, 0.75);
        pnh_.param<float>("iou_threshold", iou_threshold_, 0.75);
        
        ROS_INFO("YOLOv8 Detection Node Starting...");
        ROS_INFO("ONNX file: %s", onnx_file_.c_str());
        ROS_INFO("Confidence threshold: %.2f", confidence_threshold_);
        ROS_INFO("IoU threshold: %.2f", iou_threshold_);
        
        // Initialize ONNX Runtime
        if (!initializeONNX()) {
            ROS_ERROR("Failed to initialize ONNX Runtime");
            ros::shutdown();
            return;
        }
        
        // Check OpenCL support
        if (cv::ocl::haveOpenCL()) {
            ROS_INFO("Using OpenCV %s (OpenCL-capable)", cv::getVersionString().c_str());
            cv::ocl::useOpenCL();
            cv::ocl::setUseOpenCL(true);
            
            cv::ocl::Context context;
            if (context.create(cv::ocl::Device::TYPE_ALL)) {
                std::vector<cv::ocl::PlatformInfo> platforms;
                cv::ocl::getPlatfomsInfo(platforms);
                
                for (const auto& platform : platforms) {
                    ROS_INFO("Platform Name: %s", platform.name().c_str());
                    
                    for (int i = 0; i < platform.deviceNumber(); i++) {
                        cv::ocl::Device device;
                        platform.getDevice(device, i);
                        
                        ROS_INFO("Device Name: %s", device.name().c_str());
                        ROS_INFO("Vendor: %s", device.vendorName().c_str());
                    }
                }
            }
        } else {
            ROS_INFO("Using OpenCV %s", cv::getVersionString().c_str());
        }
        
        // Setup ROS communication
        image_sub_ = it_.subscribe("image_raw", 1, &YOLOv8DetectionNode::imageCallback, this);
        object_count_pub_ = nh_.advertise<std_msgs::UInt16>("detected_objects", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detection_markers", 10);
        debug_image_pub_ = it_.advertise("debug_image", 1);
        
        initialized_ = true;
        ROS_INFO("YOLOv8 Detection Node initialized successfully");
    }
    
    ~YOLOv8DetectionNode() {
    }

private:
    bool initializeONNX() {
        try {
            ROS_INFO("Using ONNX Runtime %s", Ort::GetVersionString());
            
            ROS_INFO("Available providers:");
            for (auto& prov : Ort::GetAvailableProviders()) {
                ROS_INFO("  [%s]", prov.c_str());
            }
            
            // Create session options
            Ort::SessionOptions session_options;
            session_options.SetLogSeverityLevel(ORT_LOGGING_LEVEL_WARNING);
            session_options.SetIntraOpNumThreads(1);
            
            // Load model
            std::wstring onnx_file_w(onnx_file_.begin(), onnx_file_.end());
            session_ = std::make_unique<Ort::Session>(ort_env_, onnx_file_w.c_str(), session_options);
            
            // Get input/output info
            input_dims_ = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            input_size_ = std::accumulate(input_dims_.begin(), input_dims_.end(), 1, std::multiplies<int64_t>());
            
            output_dims_ = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            output_size_ = std::accumulate(output_dims_.begin(), output_dims_.end(), 1, std::multiplies<int64_t>());
            
            input_size_cv_ = cv::Size(input_dims_[3], input_dims_[2]);
            channels_ = output_dims_[2];
            labels_ = {"ball", "pin", "board"};
            
            output_tensor_.resize(output_size_);
            
            img_in_ = cv::UMat(input_size_cv_, CV_8UC3, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
            img_in_.setTo(cv::Scalar(0, 0, 0));
            
            ROS_INFO("Model loaded successfully");
            ROS_INFO("Input shape: [%ld, %ld, %ld, %ld]", input_dims_[0], input_dims_[1], input_dims_[2], input_dims_[3]);
            ROS_INFO("Output shape: [%ld, %ld, %ld]", output_dims_[0], output_dims_[1], output_dims_[2]);
            
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize ONNX: %s", e.what());
            return false;
        }
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!initialized_) {
            return;
        }
        
        try {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            
            img_src_ = cv_ptr->image;
            
            // Check if image dimensions changed
            if (in_w_ != img_src_.cols || in_h_ != img_src_.rows) {
                in_w_ = img_src_.cols;
                in_h_ = img_src_.rows;
                
                // Calculate padding (maintain aspect ratio)
                if ((float)img_src_.rows / img_src_.cols < 1.0f) {
                    // Height padding
                    scale_ = (float)input_size_cv_.width / img_src_.cols;
                    int scaled_height = scale_ * img_src_.rows;
                    pad_roi_rect_ = cv::Rect(
                        cv::Point(0, (input_size_cv_.width - scaled_height) / 2),
                        cv::Size(input_size_cv_.width, scaled_height)
                    );
                } else {
                    // Width padding
                    scale_ = (float)input_size_cv_.height / img_src_.rows;
                    int scaled_width = scale_ * img_src_.cols;
                    pad_roi_rect_ = cv::Rect(
                        cv::Point((input_size_cv_.height - scaled_width) / 2, 0),
                        cv::Size(scaled_width, input_size_cv_.height)
                    );
                }
            }
            
            // Preprocessing
            img_src_n_ = img_src_.getUMat(cv::ACCESS_FAST, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
            cv::resize(img_src_n_, img_in_(pad_roi_rect_), 
                      cv::Size(pad_roi_rect_.width, pad_roi_rect_.height), 
                      0, 0, cv::INTER_AREA);
            
            // Create blob
            cv::dnn::blobFromImage(img_in_, blob_, 1.0 / 255.0, input_size_cv_, 
                                  cv::Scalar(), true, false, CV_32F);
            
            auto preprocess_time = std::chrono::high_resolution_clock::now();
            auto preprocess_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                preprocess_time - start_time).count();
            
            // Run inference
            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                memory_info_, 
                (float*)blob_.getMat(cv::ACCESS_FAST).data, 
                input_size_, 
                input_dims_.data(), 
                input_dims_.size()
            );
            
            Ort::Value output_tensor = Ort::Value::CreateTensor<float>(
                memory_info_, 
                output_tensor_.data(), 
                output_size_, 
                output_dims_.data(), 
                output_dims_.size()
            );
            
            const char* input_names[] = {"images"};
            const char* output_names[] = {"output0"};
            
            session_->Run(
                Ort::RunOptions{nullptr}, 
                input_names, &input_tensor, 1,
                output_names, &output_tensor, 1
            );
            
            auto inference_time = std::chrono::high_resolution_clock::now();
            auto inference_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                inference_time - preprocess_time).count();
            
            // Post-processing
            std::vector<float> scores;
            std::vector<int> types;
            std::vector<cv::Rect> boxes;
            std::vector<int> indices;
            
            for (int i = 0; i < channels_; i++) {
                int label_offset = i + channels_ * 4;
                
                for (int l = 0; l < labels_.size(); l++, label_offset += channels_) {
                    float confidence = output_tensor_[label_offset];
                    
                    if (confidence < confidence_threshold_) {
                        continue;
                    }
                    
                    scores.push_back(confidence);
                    types.push_back(l);
                    
                    float x_center = output_tensor_[i];
                    float y_center = output_tensor_[i + channels_];
                    float width = output_tensor_[i + channels_ * 2];
                    float height = output_tensor_[i + channels_ * 3];
                    
                    boxes.push_back(cv::Rect(
                        x_center - width / 2,
                        y_center - height / 2,
                        width,
                        height
                    ));
                }
            }
            
            // Apply NMS
            std::vector<float> nms_scores;
            cv::dnn::softNMSBoxes(boxes, scores, nms_scores, 
                                 confidence_threshold_, iou_threshold_, indices);
            
            // Count objects (specifically pins, class_id == 1)
            uint16_t object_count = 0;
            std::vector<Detection> detections;
            
            for (auto& id : indices) {
                if (types[id] == 1) {  // Pin class
                    object_count++;
                }
                detections.push_back({boxes[id], scores[id], types[id]});
            }
            
            auto postprocess_time = std::chrono::high_resolution_clock::now();
            auto postprocess_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                postprocess_time - inference_time).count();
            
            // Publish results
            std_msgs::UInt16 count_msg;
            count_msg.data = object_count;
            object_count_pub_.publish(count_msg);
            
            // Publish markers for visualization
            publishMarkers(detections, msg->header);
            
            // Publish debug image if there are subscribers
            if (debug_image_pub_.getNumSubscribers() > 0) {
                publishDebugImage(detections, msg->header);
            }
            
            auto total_time = std::chrono::high_resolution_clock::now();
            auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                total_time - start_time).count();
            
            ROS_INFO_THROTTLE(1.0, "Detection times - Preprocess: %ld ms, Inference: %ld ms, Postprocess: %ld ms, Total: %ld ms, Objects: %d",
                             preprocess_duration, inference_duration, postprocess_duration, 
                             total_duration, object_count);
                             
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in image callback: %s", e.what());
        }
    }
    
    void publishMarkers(const std::vector<Detection>& detections, const std_msgs::Header& header) {
        visualization_msgs::MarkerArray marker_array;
        
        int marker_id = 0;
        for (const auto& det : detections) {
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.header.frame_id = "camera_link";  // Adjust as needed
            marker.ns = "yolov8_detections";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // Position (simplified - would need camera calibration for proper 3D position)
            marker.pose.position.x = det.box.x / scale_;
            marker.pose.position.y = det.box.y / scale_;
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1.0;
            
            // Scale
            marker.scale.x = det.box.width / scale_;
            marker.scale.y = det.box.height / scale_;
            marker.scale.z = 0.1;
            
            // Color based on class
            switch (det.class_id) {
                case 0:  // ball
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    break;
                case 1:  // pin
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    break;
                case 2:  // board
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    break;
                default:
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
            }
            marker.color.a = 0.5;
            
            marker.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(marker);
        }
        
        marker_pub_.publish(marker_array);
    }
    
    void publishDebugImage(const std::vector<Detection>& detections, const std_msgs::Header& header) {
        cv::Mat debug_img = img_src_.clone();
        
        for (const auto& det : detections) {
            // Scale box back to original image coordinates
            cv::Rect scaled_box(
                (det.box.x - pad_roi_rect_.x) / scale_,
                (det.box.y - pad_roi_rect_.y) / scale_,
                det.box.width / scale_,
                det.box.height / scale_
            );
            
            // Choose color based on class
            cv::Scalar color;
            switch (det.class_id) {
                case 0: color = cv::Scalar(255, 0, 0); break;    // ball - blue
                case 1: color = cv::Scalar(0, 255, 0); break;    // pin - green
                case 2: color = cv::Scalar(0, 0, 255); break;    // board - red
                default: color = cv::Scalar(255, 255, 255);
            }
            
            cv::rectangle(debug_img, scaled_box, color, 2);
            
            // Add label
            std::string label = labels_[det.class_id] + ": " + 
                               std::to_string(static_cast<int>(det.score * 100)) + "%";
            cv::putText(debug_img, label, 
                       cv::Point(scaled_box.x, scaled_box.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
        
        // Publish debug image
        sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(header, "bgr8", debug_img).toImageMsg();
        debug_image_pub_.publish(debug_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolov8_detection_node");
    
    try {
        YOLOv8DetectionNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}
