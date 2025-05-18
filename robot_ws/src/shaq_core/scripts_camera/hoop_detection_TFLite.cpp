#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class HoopDetectionNode : public rclcpp::Node
{
public:
    HoopDetectionNode()
    : Node("Hoop_Detection_TFLite"), x_(0.0), y_(0.0), led_state_(false)
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("shaq_core");
        std::string model_path = package_share_directory + "/models/trainvschair_int8.tflite";

        model_ = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
        if (!model_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load TFLite model");
            rclcpp::shutdown();
            return;
        }

        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
        if (!interpreter_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to construct TFLite interpreter");
            rclcpp::shutdown();
            return;
        }

        if (interpreter_->AllocateTensors() != kTfLiteOk) {
            RCLCPP_FATAL(this->get_logger(), "Failed to allocate TFLite tensors");
            rclcpp::shutdown();
            return;
        }

        input_height_ = interpreter_->tensor(interpreter_->inputs()[0])->dims->data[1];
        input_width_ = interpreter_->tensor(interpreter_->inputs()[0])->dims->data[2];
        input_channels_ = interpreter_->tensor(interpreter_->inputs()[0])->dims->data[3];

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/shaq/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&HoopDetectionNode::imageCallback, this, _1)
        );

        data_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/shaq/send_where_hoop", rclcpp::SensorDataQoS());
        led_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/shaq/led", 10);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/shaq/image/hoop_annotated", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&HoopDetectionNode::publishData, this));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            cv::Mat resized;
            cv::resize(frame, resized, cv::Size(input_width_, input_height_));
            resized.convertTo(resized, CV_32FC3, 1.0 / 255.0);
            cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

            float* input = interpreter_->typed_input_tensor<float>(0);
            std::memcpy(input, resized.data, sizeof(float) * input_height_ * input_width_ * input_channels_);

            if (interpreter_->Invoke() != kTfLiteOk) {
                RCLCPP_ERROR(this->get_logger(), "TFLite inference failed");
                return;
            }

            float* output = interpreter_->typed_output_tensor<float>(0);
            float conf = output[4];

            if (conf > 0.2) {
                x_ = output[0];
                y_ = output[1];
                float w = output[2];
                float h = output[3];

                // Convert normalized (x, y, w, h) to pixel bounding box
                int x_pixel = static_cast<int>(x_);
                int y_pixel = static_cast<int>(y_);
                int width = static_cast<int>(w);
                int height = static_cast<int>(h);

                int x1 = x_pixel - width / 2;
                int y1 = y_pixel - height / 2;
                int x2 = x_pixel + width / 2;
                int y2 = y_pixel + height / 2;

                // Draw bounding box on the original image
                cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, "Hoop", cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 0, 0}, 1);
            } else {
                x_ = y_ = 0.0;
            }

            center_x_ = frame.cols / 2.0;
            center_y_ = frame.rows / 2.0;

            // Publish annotated image
            auto image_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
            image_pub_->publish(*image_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void publishData()
    {
        geometry_msgs::msg::Twist hoop_msg;
        geometry_msgs::msg::Twist led_msg;

        hoop_msg.linear.x = x_;
        hoop_msg.linear.y = y_;
        hoop_msg.angular.x = 270.0;
        hoop_msg.angular.y = center_y_;

        led_state_ = (x_ >= 265.0 && x_ <= 275.0);
        led_msg.linear.x = static_cast<double>(led_state_);

        data_pub_->publish(hoop_msg);
        led_pub_->publish(led_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr data_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr led_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, center_x_, center_y_;
    bool led_state_;

    std::unique_ptr<tflite::FlatBufferModel> model_;
    std::unique_ptr<tflite::Interpreter> interpreter_;

    int input_height_, input_width_, input_channels_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoopDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
