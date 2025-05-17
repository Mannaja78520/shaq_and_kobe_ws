#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class HoopDetectionNode : public rclcpp::Node
{
public:
    HoopDetectionNode()
    : Node("Hoop_Detection_onnx"), x_(0.0), y_(0.0), led_state_(false)
    {
        // Load the ONNX model
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("shaq_core");
        std::string model_path = package_share_directory + "/models/trainvschair.onnx";

        cv::dnn::Net net_ = cv::dnn::readNetFromONNX(model_path);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::cuda::getCudaEnabledDeviceCount() > 0 ? cv::dnn::DNN_TARGET_CUDA : cv::dnn::DNN_TARGET_CPU);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/shaq/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&HoopDetectionNode::imageCallback, this, _1)
        );

        data_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/shaq/send_where_hoop", rclcpp::SensorDataQoS());
        led_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/shaq/led", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&HoopDetectionNode::publishData, this));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(640, 480), cv::Scalar(), true, false);

            net_.setInput(blob);
            cv::Mat outputs = net_.forward();  // Assumes single output

            // Parse the outputs based on your model structure
            if (!outputs.empty() && outputs.rows > 0) {
                // Assuming YOLOv8 ONNX returns [x_center, y_center, w, h, conf, class_id...]
                const float* data = (float*)outputs.data;
                float conf = data[4];
                if (conf > 0.4) {
                    x_ = data[0];
                    y_ = data[1];
                } else {
                    x_ = y_ = 0.0;
                }
            }

            center_x_ = frame.cols / 2.0;
            center_y_ = frame.rows / 2.0;

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void publishData()
    {
        geometry_msgs::msg::Twist hoop_msg;
        geometry_msgs::msg::Twist led_msg;

        hoop_msg.linear.x = x_;  // Assuming cvx = x_
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
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, center_x_, center_y_;
    bool led_state_;
    cv::dnn::Net net_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoopDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
