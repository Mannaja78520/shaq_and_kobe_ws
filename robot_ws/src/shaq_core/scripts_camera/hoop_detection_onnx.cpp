#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

#include <chrono>
#include <vector>
#include <algorithm>
#include <memory>
#include <string>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <mutex>


using namespace std::chrono_literals;

class HoopDetectionNode : public rclcpp::Node {
public:
  HoopDetectionNode()
  : Node("hoop_detection"),
    env_(ORT_LOGGING_LEVEL_WARNING, "hoop_detection"),
    session_(nullptr),
    x_(0.0),
    y_(0.0),
    center_x_(0.0),
    center_y_(0.0),
    led_state_(false)
  {
    // Initialize ONNX Runtime session
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // Get model path from package share directory
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("shaq_core");
    std::string model_path = package_share_dir + "/models/trainvschair.onnx";
    
    RCLCPP_INFO(this->get_logger(), "Loading model from: %s", model_path.c_str());
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options);

    Ort::AllocatorWithDefaultOptions allocator;
    
    // --- FIX/DEBUGGING START ---
    // Get input names
    size_t num_inputs = session_->GetInputCount();
    RCLCPP_INFO(this->get_logger(), "Number of model inputs: %zu", num_inputs);
    if (num_inputs == 0) {
        RCLCPP_ERROR(this->get_logger(), "Model has no inputs!");
        throw std::runtime_error("ONNX model has no inputs.");
    }

    // Retrieve input name and store it in a std::string to ensure validity
    input_name_str_ = session_->GetInputNameAllocated(0, allocator).get();
    input_names_c_str_ = {input_name_str_.c_str()}; // Store the C-style string for session_->Run
    RCLCPP_INFO(this->get_logger(), "Retrieved input name: '%s'", input_name_str_.c_str());

    // Get input shape
    auto input_type_info = session_->GetInputTypeInfo(0);
    auto tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    input_shape_ = tensor_info.GetShape();
    
    if (input_shape_.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "Expected input shape of 4 dimensions (N,C,H,W), but got %zu", input_shape_.size());
        throw std::runtime_error("Unexpected input shape dimension count.");
    }

    input_height_ = static_cast<int>(input_shape_[2]);
    input_width_ = static_cast<int>(input_shape_[3]);
    RCLCPP_INFO(this->get_logger(), "Model input dimensions: Height=%d, Width=%d", input_height_, input_width_);
    // --- FIX/DEBUGGING END ---


    // Get output names
    size_t num_outputs = session_->GetOutputCount();
    RCLCPP_INFO(this->get_logger(), "Number of model outputs: %zu", num_outputs);
    for (size_t i = 0; i < num_outputs; i++) {
      output_names_str_.push_back(session_->GetOutputNameAllocated(i, allocator).get());
      output_names_.push_back(output_names_str_.back().c_str());
      RCLCPP_INFO(this->get_logger(), "Retrieved output name %zu: '%s'", i, output_names_str_.back().c_str());
    }

    // Publishers
    pub_hoop_pos_ = this->create_publisher<geometry_msgs::msg::Twist>("/shaq/send_where_hoop", rclcpp::SensorDataQoS());
    pub_annotated_image_ = this->create_publisher<sensor_msgs::msg::Image>("/shaq/image/annotated_image", rclcpp::SensorDataQoS());
    pub_led_ = this->create_publisher<geometry_msgs::msg::Twist>("/shaq/led", rclcpp::QoS(10));

    // Subscriber
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/shaq/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&HoopDetectionNode::imageCallback, this, std::placeholders::_1));

    // Timer to publish data periodically (every 50ms)
    timer_ = this->create_wall_timer(50ms, std::bind(&HoopDetectionNode::sendData, this));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Lock guard to protect access to processing flag
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (processing) {
            return;  // skip if busy
        }
        processing = true;
    }

    auto start_time = std::chrono::steady_clock::now();

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;

    // Resize to model input size
    cv::Mat frame_resized;
    cv::resize(frame, frame_resized, cv::Size(input_width_, input_height_));

    // Convert BGR to RGB and normalize
    std::vector<float> input_tensor_values(3 * input_height_ * input_width_);
    for (int c = 0; c < 3; ++c) {
      for (int h = 0; h < input_height_; ++h) {
        for (int w = 0; w < input_width_; ++w) {
          input_tensor_values[c * input_height_ * input_width_ + h * input_width_ + w] =
            frame_resized.at<cv::Vec3b>(h, w)[2 - c] / 255.0f;
        }
      }
    }

    // Use the actual input_shape_ obtained from the model
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info,
      input_tensor_values.data(),
      input_tensor_values.size(),
      input_shape_.data(), // Use input_shape_
      input_shape_.size());

    // Run inference
    auto outputs = session_->Run(
      Ort::RunOptions{nullptr},
      input_names_c_str_.data(), // Use input_names_c_str_
      &input_tensor,
      1,
      output_names_.data(),
      output_names_.size());

    float* raw_output = outputs[0].GetTensorMutableData<float>();
    auto output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();

    // Output shape expected [features, num_predictions] or [batch, features, num_predictions]
    // Assuming output_shape[0] is batch_size (which is 1) and output_shape[1] is features, output_shape[2] is num_predictions.
    // If your model output is structured differently (e.g., [1, num_predictions, features]), you'll need to adjust indexing.
    // For YOLOv8, it's typically [1, features, num_predictions] where features includes bbox (4) + confidence (1) + class_scores (num_classes).
    // Given your loop: float x = raw_output[0 * num_predictions + i]; etc., it assumes [features, num_predictions] where features are rows.
    // Let's stick with your current indexing if it works with your model, but be aware of this common variation.
    
    // Original indexing:
    // int features_per_detection = static_cast<int>(output_shape[1]); // e.g. 5
    // int num_predictions = static_cast<int>(output_shape[2]);       // e.g. 1344

    // More robust indexing for YOLOv8 type output (batch, features, num_predictions)
    int num_predictions_from_output = static_cast<int>(output_shape[2]); // Assuming output_shape is [1, Features, N]
    int features_per_detection_from_output = static_cast<int>(output_shape[1]); // Features count (e.g., 5 for x,y,w,h,conf or 5+classes)

    std::vector<std::vector<float>> valid_detections;
    // Iterate through predictions (columns)
    for (int i = 0; i < num_predictions_from_output; ++i) {
      // Accessing elements as if features are rows and predictions are columns.
      // This is a common representation from some YOLOv8 ONNX exports where the output is (Batch, Features, NumDetections).
      // So, raw_output[feature_index * num_predictions_from_output + current_prediction_index]
      float x = raw_output[0 * num_predictions_from_output + i];
      float y = raw_output[1 * num_predictions_from_output + i];
      float w = raw_output[2 * num_predictions_from_output + i];
      float h = raw_output[3 * num_predictions_from_output + i];
      float conf = raw_output[4 * num_predictions_from_output + i]; // Assuming the 5th feature is confidence

      if (conf > 0.35f) {
        valid_detections.push_back({x, y, w, h, conf});
      }
    }

    if (!valid_detections.empty()) {
      auto best = *std::max_element(valid_detections.begin(), valid_detections.end(),
                                    [](const auto& a, const auto& b) { return a[4] < b[4]; });

      int x1 = static_cast<int>((best[0] - best[2] / 2.0f) * input_width_);
      int y1 = static_cast<int>((best[1] - best[3] / 2.0f) * input_height_);
      int x2 = static_cast<int>((best[0] + best[2] / 2.0f) * input_width_);
      int y2 = static_cast<int>((best[1] + best[3] / 2.0f) * input_height_);

      // Update detected position in pixels
      x_ = best[0] * input_width_;
      y_ = best[1] * input_height_;

      // Draw bounding box and label
      cv::rectangle(frame_resized, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
      char label[50];
      snprintf(label, sizeof(label), "hoop: %.2f", best[4]);
      cv::putText(frame_resized, label, cv::Point(x1, std::max(y1 - 10, 0)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
    } else {
      x_ = 0.0;
      y_ = 0.0;
    }

    center_x_ = input_width_ / 2.0;
    center_y_ = input_height_ / 2.0;

    // Publish annotated image
    try {
      auto annotated_msg = cv_bridge::CvImage(msg->header, "bgr8", frame_resized).toImageMsg();
      pub_annotated_image_->publish(*annotated_msg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to publish annotated image: %s", e.what());
    }

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(this->get_logger(), "Process time: %ld ms", elapsed_ms);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        processing = false;
    }
  }

  void sendData() {
    auto msg = geometry_msgs::msg::Twist();
    auto led_msg = geometry_msgs::msg::Twist();

    msg.linear.x = x_;
    msg.linear.y = y_;
    msg.angular.x = center_x_;
    msg.angular.y = center_y_;

    if (x_ > 265 && x_ < 275) {
      led_state_ = true;
    } else {
      led_state_ = false;
    }

    led_msg.linear.x = led_state_ ? 1.0 : 0.0;

    pub_hoop_pos_->publish(msg);
    pub_led_->publish(led_msg);
  }

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_hoop_pos_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_image_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_led_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ONNX Runtime
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  
  // Storing input name as std::string for lifetime management
  std::string input_name_str_; 
  std::vector<const char*> input_names_c_str_; // C-style string for Ort::Session::Run
  std::vector<int64_t> input_shape_; // Store the full input shape

  std::vector<const char*> output_names_;
  std::vector<std::string> output_names_str_; // To manage string lifetime

  int input_height_;
  int input_width_;

  // Detection data
  float x_;
  float y_;
  float center_x_;
  float center_y_;
  bool led_state_;

  // thread data
  bool processing = false;
  std::mutex mutex_;   
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HoopDetectionNode>();

  // This parameter declaration is actually not needed if you are getting the package share directory directly
  // via ament_index_cpp. It's more for parameters you would set from the launch file.
  // node->declare_parameter<std::string>("package_share_dir", ""); 

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}