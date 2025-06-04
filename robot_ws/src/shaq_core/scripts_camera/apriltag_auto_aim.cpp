#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <map>

class AprilTagDetector : public rclcpp::Node
{
public:
    AprilTagDetector()
    : Node("apriltag_detector")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/shaq/distance/kobe", 10);
        
        rclcpp::QoS best_effort_qos(rclcpp::KeepLast(10));
        best_effort_qos.best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/shaq/image_raw", best_effort_qos,
            std::bind(&AprilTagDetector::image_callback, this, std::placeholders::_1));

        tag_size_ = 0.1;       // meters
        focal_length_ = 653.0; // pixels (adjust after calibration)

        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);
    }

    ~AprilTagDetector()
    {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

private:

    const double wanted_distance = 3.0;    // meters
    const double max_offset = 1.0;      // deadband tolerance in meters


    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        image_u8_t im = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td_, &im);

        int frame_w = gray.cols;
        int frame_h = gray.rows;

        std::map<int, std::pair<int, int>> screen_centers = {
            {0, {320, 0}},
            {1, {148, 0}},
            {2, {340, 0}},
            {3, {350, 0}}
        };


        std::pair<int, int> default_center = {frame_w / 2, frame_h / 2};

        if (zarray_size(detections) == 0) {
            auto out = geometry_msgs::msg::Twist();
            out.linear.x = 0.0;
            out.linear.y = 0.0;
            out.linear.z = 0.0;
            out.angular.y = 123123.0; //Dont want to be 0 Cuz 0 For Led error = 0m
            publisher_->publish(out);
        } else {
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                int tag_id = det->id;
                int center_x = static_cast<int>(det->c[0]);
                int center_y = static_cast<int>(det->c[1]);

                std::pair<int, int> screen_center = default_center;
                if (screen_centers.count(tag_id)) {
                    screen_center = screen_centers[tag_id];
                }

                double dx = det->p[0][0] - det->p[1][0];
                double dy = det->p[0][1] - det->p[1][1];
                double perceived_width = std::hypot(dx, dy);

                dx = det->p[1][0] - det->p[2][0];
                dy = det->p[1][1] - det->p[2][1];
                double perceived_height = std::hypot(dx, dy);

                double perceived_size = (perceived_width + perceived_height) / 2.0;
                double distance = (tag_size_ * focal_length_) / perceived_size;
                double error = distance - wanted_distance;          // positive = too far, negative = too close
                double normalized_error = std::clamp(error / max_offset, -1.0, 1.0);

                auto out = geometry_msgs::msg::Twist();
                out.linear.x = center_x;
                out.linear.y = center_y;
                out.linear.z = distance;

                out.angular.x = screen_center.first;
                out.angular.y = normalized_error;
                out.angular.z = tag_id;

                publisher_->publish(out);
            }
        }

        apriltag_detections_destroy(detections);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    apriltag_family_t *tf_;
    apriltag_detector_t *td_;

    double tag_size_;
    double focal_length_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagDetector>());
    rclcpp::shutdown();
    return 0;
}
