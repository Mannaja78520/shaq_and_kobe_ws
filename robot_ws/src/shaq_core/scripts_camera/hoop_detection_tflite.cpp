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
#include <thread>
#include <atomic>
#include <condition_variable>

using namespace std::chrono_literals;

class HoopDetectionNode : public rclcpp::Node {
public:
  HoopDetectionNode()
  : Node("hoop_detection_tflite"), x_(0.0), y_(0.0), center_x_(0.0), center_y_(0.0), led_state_(false), stop_processing_(false)
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
    interpreter_->SetNumThreads(2);

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

    // Start processing thread
    processing_thread_ = std::thread(&HoopDetectionNode::processingLoop, this);
  }

  ~HoopDetectionNode() {
    stop_processing_ = true;
    cv_image_available_cv_.notify_one();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    try {
      // Convert ROS image to OpenCV and store the latest image
      latest_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image.clone();
      latest_header_ = msg->header;
      cv_image_available_cv_.notify_one();
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void processingLoop() {
    while (!stop_processing_) {
      cv::Mat image_to_process;
      std_msgs::msg::Header header_to_use;

      // Wait until new image available or stop requested
      {
        std::unique_lock<std::mutex> lock(image_mutex_);
        cv_image_available_cv_.wait(lock, [this]() { return !latest_image_.empty() || stop_processing_; });

        if (stop_processing_) break;

        // Grab the latest image and clear stored image to indicate processing started
        image_to_process = latest_image_.clone();
        header_to_use = latest_header_;
        latest_image_ = cv::Mat();
      }

      auto start = std::chrono::steady_clock::now();

      // Preprocessing: blur, resize
      cv::GaussianBlur(image_to_process, image_to_process, cv::Size(3,3), 0);
      cv::Mat resized;
      cv::resize(image_to_process, resized, cv::Size(input_width_, input_height_));

      // Convert BGR to RGB and normalize to [0,1]
      std::vector<float> input_data(input_height_ * input_width_ * input_channels_);
      for (int h = 0; h < input_height_; ++h) {
        for (int w = 0; w < input_width_; ++w) {
          cv::Vec3b pixel = resized.at<cv::Vec3b>(h, w);
          input_data[(h * input_width_ + w) * 3 + 0] = pixel[2] / 255.0f; // R
          input_data[(h * input_width_ + w) * 3 + 1] = pixel[1] / 255.0f; // G
          input_data[(h * input_width_ + w) * 3 + 2] = pixel[0] / 255.0f; // B
        }
      }

      // Copy input to tensor
      float* input_tensor_ptr = interpreter_->typed_input_tensor<float>(0);
      std::copy(input_data.begin(), input_data.end(), input_tensor_ptr);

      // Run inference
      if (interpreter_->Invoke() != kTfLiteOk) {
        RCLCPP_ERROR(this->get_logger(), "Failed to invoke TFLite interpreter");
        continue;
      }

      // Parse output
      TfLiteTensor* output_tensor = interpreter_->output_tensor(0);
      const float* output_data = output_tensor->data.f;

      int features = output_tensor->dims->data[1];
      int num_predictions = output_tensor->dims->data[2];

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

      cv::Mat annotated = resized.clone();

      if (!detections.empty()) {
        auto best = *std::max_element(detections.begin(), detections.end(),
                                      [](const Detection& a, const Detection& b) { return a.conf < b.conf; });

        int x1 = static_cast<int>((best.x - best.w / 2.0f) * input_width_);
        int y1 = static_cast<int>((best.y - best.h / 2.0f) * input_height_);
        int x2 = static_cast<int>((best.x + best.w / 2.0f) * input_width_);
        int y2 = static_cast<int>((best.y + best.h / 2.0f) * input_height_);

        // Update shared variables for position
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          x_ = best.x * input_width_;
          y_ = best.y * input_height_;
          led_state_ = true;
        }

        cv::rectangle(annotated, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
        std::string label = "hoop: " + std::to_string(best.conf);
        cv::putText(annotated, label, cv::Point(x1, std::max(y1 - 10, 0)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
      } else {
        std::lock_guard<std::mutex> lock(data_mutex_);
        x_ = 0.0;
        y_ = 0.0;
        led_state_ = false;
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        center_x_ = input_width_ / 2.0;
        center_y_ = input_height_ / 2.0;
      }

      // Publish annotated image
      try {
        auto annotated_msg = cv_bridge::CvImage(header_to_use, "bgr8", annotated).toImageMsg();
        pub_annotated_image_->publish(*annotated_msg);
      } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception on annotated image: %s", e.what());
      }

      // Log processing time
      auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
      RCLCPP_INFO(this->get_logger(), "Inference Process Time: %ld ms", duration_ms);
    }
  }

  void sendData() {
    geometry_msgs::msg::Twist pos_msg;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      pos_msg.linear.x = x_;
      pos_msg.linear.y = y_;
      pos_msg.linear.z = 0.0;

      pos_msg.angular.x = center_x_;
      pos_msg.angular.y = center_y_;
      pos_msg.angular.z = 0.0;
    }
    pub_hoop_pos_->publish(pos_msg);

    geometry_msgs::msg::Twist led_msg;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      led_msg.linear.x = led_state_ ? 1.0 : 0.0;
    }
    pub_led_->publish(led_msg);
  }

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_hoop_pos_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_image_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_led_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TFLite
  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
  TfLiteTensor* input_tensor_;
  int input_width_;
  int input_height_;
  int input_channels_;

  // Image processing thread
  std::thread processing_thread_;
  std::atomic<bool> stop_processing_;

  // Latest image & header, protected by mutex and condition variable
  cv::Mat latest_image_;
  std_msgs::msg::Header latest_header_;
  std::mutex image_mutex_;
  std::condition_variable cv_image_available_cv_;

  // Shared data protected by mutex
  std::mutex data_mutex_;
  double x_, y_, center_x_, center_y_;
  bool led_state_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HoopDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
