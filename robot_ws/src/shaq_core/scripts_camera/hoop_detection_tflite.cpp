#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class HoopDetectionNode : public rclcpp::Node {
public:
  HoopDetectionNode()
  : Node("hoop_detection_tflite"), x_(0.0), y_(0.0), center_x_(0.0), center_y_(0.0), led_state_(false), processing_(false)
  {
    // Load TFLite model
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("shaq_core");
    std::string model_path = package_share_dir + "/models/trainvschair_float16.tflite";

    RCLCPP_INFO(this->get_logger(), "Loading TFLite model from: %s", model_path.c_str());

    model_ = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
    if (!model_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load TFLite model");
      throw std::runtime_error("Failed to load TFLite model");
    }

    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model_, resolver);
    builder(&interpreter_);
    if (!interpreter_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct TFLite interpreter");
      throw std::runtime_error("Failed to construct TFLite interpreter");
    }

    if (interpreter_->AllocateTensors() != kTfLiteOk) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate tensors");
      throw std::runtime_error("Failed to allocate tensors");
    }

    input_tensor_ = interpreter_->input_tensor(0);
    if (input_tensor_->dims->size != 4) {
      RCLCPP_ERROR(this->get_logger(), "Expected 4D input tensor, got %dD", input_tensor_->dims->size);
      throw std::runtime_error("Unexpected input tensor dimension");
    }

    input_height_ = input_tensor_->dims->data[1];
    input_width_ = input_tensor_->dims->data[2];
    input_channels_ = input_tensor_->dims->data[3];

    RCLCPP_INFO(this->get_logger(), "Model input size: %d x %d x %d", input_width_, input_height_, input_channels_);

    // Publishers
    pub_hoop_pos_ = this->create_publisher<geometry_msgs::msg::Twist>("/shaq/send_where_hoop", rclcpp::SensorDataQoS());
    pub_annotated_image_ = this->create_publisher<sensor_msgs::msg::Image>("/shaq/image/annotated_image", rclcpp::SensorDataQoS());
    pub_led_ = this->create_publisher<geometry_msgs::msg::Twist>("/shaq/led", rclcpp::QoS(10));

    // Subscriber to camera image
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/shaq/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&HoopDetectionNode::imageCallback, this, std::placeholders::_1));

    // Timer for publishing data periodically
    timer_ = this->create_wall_timer(50ms, std::bind(&HoopDetectionNode::sendData, this));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Prevent overlapping processing
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (processing_) return;
      processing_ = true;
    }

    auto start = std::chrono::steady_clock::now();

    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      processing_ = false;
      return;
    }
    cv::Mat frame = cv_ptr->image;

    // Resize to model input
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(input_width_, input_height_));

    // Convert BGR to RGB and normalize [0,1]
    std::vector<float> input_data(input_height_ * input_width_ * input_channels_);
    for (int h = 0; h < input_height_; ++h) {
      for (int w = 0; w < input_width_; ++w) {
        cv::Vec3b pixel = resized.at<cv::Vec3b>(h, w);
        input_data[(h * input_width_ + w) * 3 + 0] = pixel[2] / 255.0f; // R
        input_data[(h * input_width_ + w) * 3 + 1] = pixel[1] / 255.0f; // G
        input_data[(h * input_width_ + w) * 3 + 2] = pixel[0] / 255.0f; // B
      }
    }

    // Copy input data to tensor
    float* input_tensor_ptr = interpreter_->typed_input_tensor<float>(0);
    std::copy(input_data.begin(), input_data.end(), input_tensor_ptr);

    // Run inference
    if (interpreter_->Invoke() != kTfLiteOk) {
      RCLCPP_ERROR(this->get_logger(), "Failed to invoke TFLite interpreter");
      processing_ = false;
      return;
    }

    // Get output tensor - assuming single output
    TfLiteTensor* output_tensor = interpreter_->output_tensor(0);
    const float* output_data = output_tensor->data.f;

    // Output shape handling (depends on your model, adjust accordingly)
    int output_dims = output_tensor->dims->size; // usually 3: [1, features, num_predictions]
    int features = output_tensor->dims->data[1];
    int num_predictions = output_tensor->dims->data[2];

    // Parse detections (assuming format: [x, y, w, h, conf, ...])
    struct Detection { float x, y, w, h, conf; };
    std::vector<Detection> detections;

    for (int i = 0; i < num_predictions; ++i) {
      float x = output_data[0 * num_predictions + i];
      float y = output_data[1 * num_predictions + i];
      float w = output_data[2 * num_predictions + i];
      float h = output_data[3 * num_predictions + i];
      float conf = output_data[4 * num_predictions + i];

      if (conf > 0.35f) {
        detections.push_back({x, y, w, h, conf});
      }
    }

    if (!detections.empty()) {
      // Pick best detection by confidence
      auto best = *std::max_element(detections.begin(), detections.end(),
                                    [](const Detection& a, const Detection& b) { return a.conf < b.conf; });

      int x1 = static_cast<int>((best.x - best.w / 2.0f) * input_width_);
      int y1 = static_cast<int>((best.y - best.h / 2.0f) * input_height_);
      int x2 = static_cast<int>((best.x + best.w / 2.0f) * input_width_);
      int y2 = static_cast<int>((best.y + best.h / 2.0f) * input_height_);

      x_ = best.x * input_width_;
      y_ = best.y * input_height_;

      // Draw box and label
      cv::rectangle(resized, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
      std::string label = "hoop: " + std::to_string(best.conf);
      cv::putText(resized, label, cv::Point(x1, std::max(y1 - 10, 0)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
    } else {
      x_ = 0.0;
      y_ = 0.0;
    }

    center_x_ = input_width_ / 2.0;
    center_y_ = input_height_ / 2.0;

    // Publish annotated image
    try {
      auto annotated_msg = cv_bridge::CvImage(msg->header, "bgr8", resized).toImageMsg();
      pub_annotated_image_->publish(*annotated_msg);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception on annotated image: %s", e.what());
    }

    // Measure inference time
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

    RCLCPP_INFO(this->get_logger(), "Process Time: %ld ms", duration_ms);

    // Release processing lock
    std::lock_guard<std::mutex> lock(mutex_);
    processing_ = false;
  }

  void sendData() {
    geometry_msgs::msg::Twist pos_msg;
    pos_msg.linear.x = x_;
    pos_msg.linear.y = y_;
    pos_msg.linear.z = 0.0;

    pos_msg.angular.x = center_x_;
    pos_msg.angular.y = center_y_;
    pos_msg.angular.z = 0.0;

    pub_hoop_pos_->publish(pos_msg);

    geometry_msgs::msg::Twist led_msg;
    led_msg.linear.x = led_state_ ? 1.0 : 0.0;
    pub_led_->publish(led_msg);
  }

  // Members
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_hoop_pos_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_image_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_led_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
  TfLiteTensor* input_tensor_;

  int input_width_, input_height_, input_channels_;

  std::mutex mutex_;
  bool processing_;
  bool led_state_;

  float x_, y_;
  float center_x_, center_y_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HoopDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
