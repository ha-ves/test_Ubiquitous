// ROS1 node for YOLOv8 Detection using ONNX Runtime
// Migrated from OpenRTM component
// YOLOv8物体検出用ROS1ノード（ONNX Runtimeを使用）
// OpenRTMコンポーネントから移植

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

// 検出結果を格納する構造体
// Structure to store detection results
struct Detection {
    cv::Rect box;       // バウンディングボックス / Bounding box
    float score;        // 信頼度スコア / Confidence score
    int class_id;       // クラスID / Class ID
};

// YOLOv8物体検出ノードクラス
// YOLOv8 Detection Node Class
class YOLOv8DetectionNode {
private:
    // ROSハンドル / ROS handles
    ros::NodeHandle nh_;        // ノードハンドル / Node handle
    ros::NodeHandle pnh_;       // プライベートノードハンドル / Private node handle
    image_transport::ImageTransport it_;  // 画像転送 / Image transport
    image_transport::Subscriber image_sub_;  // 画像サブスクライバ / Image subscriber
    ros::Publisher object_count_pub_;  // 物体数パブリッシャ / Object count publisher
    ros::Publisher marker_pub_;  // マーカーパブリッシャ / Marker publisher
    image_transport::Publisher debug_image_pub_;  // デバッグ画像パブリッシャ / Debug image publisher

    // ONNX Runtime関連 / ONNX Runtime related
    Ort::Env ort_env_;  // ONNX環境 / ONNX environment
    std::unique_ptr<Ort::Session> session_;  // 推論セッション / Inference session
    Ort::MemoryInfo memory_info_;  // メモリ情報 / Memory info
    
    std::vector<int64_t> input_dims_;   // 入力次元 / Input dimensions
    std::vector<int64_t> output_dims_;  // 出力次元 / Output dimensions
    int64_t input_size_;   // 入力サイズ / Input size
    int64_t output_size_;  // 出力サイズ / Output size
    
    cv::Size input_size_cv_;  // OpenCV用入力サイズ / OpenCV input size
    std::vector<std::string> labels_;  // クラスラベル / Class labels
    int64_t channels_;  // チャンネル数 / Number of channels
    
    std::vector<float> output_tensor_;  // 出力テンソル / Output tensor
    
    // OpenCVバッファ / OpenCV buffers
    cv::Rect pad_roi_rect_;  // パディング領域 / Padding ROI
    cv::Mat img_src_;        // 入力画像 / Source image
    cv::UMat img_src_n_, img_in_, blob_;  // GPU用画像バッファ / GPU image buffers
    
    // パラメータ / Parameters
    std::string onnx_file_;        // ONNXモデルファイルパス / ONNX model file path
    float confidence_threshold_;   // 信頼度閾値 / Confidence threshold
    float iou_threshold_;          // IoU閾値 / IoU threshold
    
    // 画像寸法 / Image dimensions
    int in_w_, in_h_;  // 入力画像の幅と高さ / Input image width and height
    float scale_;      // スケール係数 / Scale factor
    
    bool initialized_;  // 初期化フラグ / Initialization flag

public:
    // コンストラクタ / Constructor
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
        // パラメータ読み込み / Load parameters
        pnh_.param<std::string>("onnx_file", onnx_file_, "best_full.onnx");
        pnh_.param<float>("confidence_threshold", confidence_threshold_, 0.75);
        pnh_.param<float>("iou_threshold", iou_threshold_, 0.75);
        
        ROS_INFO("YOLOv8 Detection Node Starting...");
        ROS_INFO("ONNX file: %s", onnx_file_.c_str());
        ROS_INFO("Confidence threshold: %.2f", confidence_threshold_);
        ROS_INFO("IoU threshold: %.2f", iou_threshold_);
        
        // ONNX Runtimeの初期化 / Initialize ONNX Runtime
        if (!initializeONNX()) {
            ROS_ERROR("Failed to initialize ONNX Runtime");
            ros::shutdown();
            return;
        }
        
        // OpenCLサポートの確認 / Check OpenCL support
        if (cv::ocl::haveOpenCL()) {
            ROS_INFO("Using OpenCV %s (OpenCL-capable)", cv::getVersionString().c_str());
            cv::ocl::useOpenCL();
            cv::ocl::setUseOpenCL(true);
            
            // OpenCLデバイス情報の取得 / Get OpenCL device information
            cv::ocl::Context context;
            if (context.create(cv::ocl::Device::TYPE_ALL)) {
                std::vector<cv::ocl::PlatformInfo> platforms;
                cv::ocl::getPlatformsInfo(platforms);
                
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
        
        // ROS通信のセットアップ / Setup ROS communication
        image_sub_ = it_.subscribe("image_raw", 1, &YOLOv8DetectionNode::imageCallback, this);
        object_count_pub_ = nh_.advertise<std_msgs::UInt16>("detected_objects", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detection_markers", 10);
        debug_image_pub_ = it_.advertise("debug_image", 1);
        
        initialized_ = true;
        ROS_INFO("YOLOv8 Detection Node initialized successfully");
    }
    
    // デストラクタ / Destructor
    ~YOLOv8DetectionNode() {
    }

private:
    // ONNX Runtimeの初期化 / Initialize ONNX Runtime
    bool initializeONNX() {
        try {
            ROS_INFO("Using ONNX Runtime %s", Ort::GetVersionString());
            
            // 利用可能なプロバイダーの表示 / Display available providers
            ROS_INFO("Available providers:");
            for (auto& prov : Ort::GetAvailableProviders()) {
                ROS_INFO("  [%s]", prov.c_str());
            }
            
            // セッションオプションの作成 / Create session options
            Ort::SessionOptions session_options;
            session_options.SetLogSeverityLevel(ORT_LOGGING_LEVEL_WARNING);
            session_options.SetIntraOpNumThreads(1);
            
            // モデルの読み込み / Load model
            std::wstring onnx_file_w(onnx_file_.begin(), onnx_file_.end());
            session_ = std::make_unique<Ort::Session>(ort_env_, onnx_file_w.c_str(), session_options);
            
            // 入出力情報の取得 / Get input/output info
            input_dims_ = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            input_size_ = std::accumulate(input_dims_.begin(), input_dims_.end(), 1, std::multiplies<int64_t>());
            
            output_dims_ = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            output_size_ = std::accumulate(output_dims_.begin(), output_dims_.end(), 1, std::multiplies<int64_t>());
            
            input_size_cv_ = cv::Size(input_dims_[3], input_dims_[2]);
            channels_ = output_dims_[2];
            labels_ = {"ball", "pin", "board"};  // クラスラベル：ボール、ピン、ボード
            
            output_tensor_.resize(output_size_);
            
            // GPU用画像バッファの初期化 / Initialize GPU image buffer
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
    
    // 画像コールバック関数 / Image callback function
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!initialized_) {
            return;
        }
        
        try {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            // ROS画像をOpenCVに変換 / Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            
            img_src_ = cv_ptr->image;
            
            // 画像サイズの変更を確認 / Check if image dimensions changed
            if (in_w_ != img_src_.cols || in_h_ != img_src_.rows) {
                in_w_ = img_src_.cols;
                in_h_ = img_src_.rows;
                
                // パディングの計算（アスペクト比を維持） / Calculate padding (maintain aspect ratio)
                if ((float)img_src_.rows / img_src_.cols < 1.0f) {
                    // 高さ方向のパディング / Height padding
                    scale_ = (float)input_size_cv_.width / img_src_.cols;
                    int scaled_height = scale_ * img_src_.rows;
                    pad_roi_rect_ = cv::Rect(
                        cv::Point(0, (input_size_cv_.width - scaled_height) / 2),
                        cv::Size(input_size_cv_.width, scaled_height)
                    );
                } else {
                    // 幅方向のパディング / Width padding
                    scale_ = (float)input_size_cv_.height / img_src_.rows;
                    int scaled_width = scale_ * img_src_.cols;
                    pad_roi_rect_ = cv::Rect(
                        cv::Point((input_size_cv_.height - scaled_width) / 2, 0),
                        cv::Size(scaled_width, input_size_cv_.height)
                    );
                }
            }
            
            // 前処理 / Preprocessing
            img_src_n_ = img_src_.getUMat(cv::ACCESS_FAST, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
            cv::resize(img_src_n_, img_in_(pad_roi_rect_), 
                      cv::Size(pad_roi_rect_.width, pad_roi_rect_.height), 
                      0, 0, cv::INTER_AREA);
            
            // Blobの作成（正規化：0-255 → 0-1） / Create blob (normalize: 0-255 → 0-1)
            cv::dnn::blobFromImage(img_in_, blob_, 1.0 / 255.0, input_size_cv_, 
                                  cv::Scalar(), true, false, CV_32F);
            
            auto preprocess_time = std::chrono::high_resolution_clock::now();
            auto preprocess_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                preprocess_time - start_time).count();
            
            // 推論実行 / Run inference
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
            
            // ONNX推論の実行 / Execute ONNX inference
            session_->Run(
                Ort::RunOptions{nullptr}, 
                input_names, &input_tensor, 1,
                output_names, &output_tensor, 1
            );
            
            auto inference_time = std::chrono::high_resolution_clock::now();
            auto inference_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                inference_time - preprocess_time).count();
            
            // 後処理 / Post-processing
            std::vector<float> scores;      // 信頼度スコア / Confidence scores
            std::vector<int> types;         // クラスタイプ / Class types
            std::vector<cv::Rect> boxes;    // バウンディングボックス / Bounding boxes
            std::vector<int> indices;       // NMS後のインデックス / Indices after NMS
            
            // 検出結果の抽出 / Extract detection results
            for (int i = 0; i < channels_; i++) {
                int label_offset = i + channels_ * 4;
                
                for (int l = 0; l < labels_.size(); l++, label_offset += channels_) {
                    float confidence = output_tensor_[label_offset];
                    
                    // 信頼度閾値によるフィルタリング / Filter by confidence threshold
                    if (confidence < confidence_threshold_) {
                        continue;
                    }
                    
                    scores.push_back(confidence);
                    types.push_back(l);
                    
                    // バウンディングボックスの計算 / Calculate bounding box
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
            
            // NMS（非最大値抑制）の適用 / Apply NMS (Non-Maximum Suppression)
            std::vector<float> nms_scores;
            cv::dnn::softNMSBoxes(boxes, scores, nms_scores, 
                                 confidence_threshold_, iou_threshold_, indices);
            
            // 物体のカウント（特にピン、class_id == 1） / Count objects (specifically pins, class_id == 1)
            uint16_t object_count = 0;
            std::vector<Detection> detections;
            
            for (auto& id : indices) {
                if (types[id] == 1) {  // ピンクラス / Pin class
                    object_count++;
                }
                detections.push_back({boxes[id], scores[id], types[id]});
            }
            
            auto postprocess_time = std::chrono::high_resolution_clock::now();
            auto postprocess_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                postprocess_time - inference_time).count();
            
            // 結果の公開 / Publish results
            std_msgs::UInt16 count_msg;
            count_msg.data = object_count;
            object_count_pub_.publish(count_msg);
            
            // 可視化用マーカーの公開 / Publish markers for visualization
            publishMarkers(detections, msg->header);
            
            // サブスクライバがいる場合はデバッグ画像を公開 / Publish debug image if there are subscribers
            if (debug_image_pub_.getNumSubscribers() > 0) {
                publishDebugImage(detections, msg->header);
            }
            
            auto total_time = std::chrono::high_resolution_clock::now();
            auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                total_time - start_time).count();
            
            // 処理時間の表示（1秒ごと） / Display processing times (every 1 second)
            ROS_INFO_THROTTLE(1.0, "Detection times - Preprocess: %ld ms, Inference: %ld ms, Postprocess: %ld ms, Total: %ld ms, Objects: %d",
                             preprocess_duration, inference_duration, postprocess_duration, 
                             total_duration, object_count);
                             
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in image callback: %s", e.what());
        }
    }
    
    // RViz用マーカーの公開 / Publish markers for RViz
    void publishMarkers(const std::vector<Detection>& detections, const std_msgs::Header& header) {
        visualization_msgs::MarkerArray marker_array;
        
        int marker_id = 0;
        for (const auto& det : detections) {
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.header.frame_id = "camera_link";  // 必要に応じて調整 / Adjust as needed
            marker.ns = "yolov8_detections";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // 位置（簡易版 - 正確な3D位置にはカメラキャリブレーションが必要）
            // Position (simplified - would need camera calibration for proper 3D position)
            marker.pose.position.x = det.box.x / scale_;
            marker.pose.position.y = det.box.y / scale_;
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1.0;
            
            // スケール / Scale
            marker.scale.x = det.box.width / scale_;
            marker.scale.y = det.box.height / scale_;
            marker.scale.z = 0.1;
            
            // クラスに応じた色 / Color based on class
            switch (det.class_id) {
                case 0:  // ボール / ball
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    break;
                case 1:  // ピン / pin
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    break;
                case 2:  // ボード / board
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    break;
                default:
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
            }
            marker.color.a = 0.5;  // 透明度 / Transparency
            
            marker.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(marker);
        }
        
        marker_pub_.publish(marker_array);
    }
    
    // デバッグ画像の公開（バウンディングボックス付き） / Publish debug image (with bounding boxes)
    void publishDebugImage(const std::vector<Detection>& detections, const std_msgs::Header& header) {
        cv::Mat debug_img = img_src_.clone();
        
        for (const auto& det : detections) {
            // ボックスを元の画像座標にスケール変換 / Scale box back to original image coordinates
            cv::Rect scaled_box(
                (det.box.x - pad_roi_rect_.x) / scale_,
                (det.box.y - pad_roi_rect_.y) / scale_,
                det.box.width / scale_,
                det.box.height / scale_
            );
            
            // クラスに応じた色を選択 / Choose color based on class
            cv::Scalar color;
            switch (det.class_id) {
                case 0: color = cv::Scalar(255, 0, 0); break;    // ボール - 青 / ball - blue
                case 1: color = cv::Scalar(0, 255, 0); break;    // ピン - 緑 / pin - green
                case 2: color = cv::Scalar(0, 0, 255); break;    // ボード - 赤 / board - red
                default: color = cv::Scalar(255, 255, 255);
            }
            
            // バウンディングボックスの描画 / Draw bounding box
            cv::rectangle(debug_img, scaled_box, color, 2);
            
            // ラベルの追加 / Add label
            std::string label = labels_[det.class_id] + ": " + 
                               std::to_string(static_cast<int>(det.score * 100)) + "%";
            cv::putText(debug_img, label, 
                       cv::Point(scaled_box.x, scaled_box.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
        
        // デバッグ画像の公開 / Publish debug image
        sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(header, "bgr8", debug_img).toImageMsg();
        debug_image_pub_.publish(debug_msg);
    }
};

// メイン関数 / Main function
int main(int argc, char** argv) {
    // ROSノードの初期化 / Initialize ROS node
    ros::init(argc, argv, "yolov8_detection_node");
    
    try {
        YOLOv8DetectionNode node;
        ros::spin();  // ノードの実行 / Run node
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}
