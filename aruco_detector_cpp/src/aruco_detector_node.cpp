#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.hpp>

#include "aruco_processor.hpp"  // Incluye la lógica externa

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector() : Node("aruco_detector")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));

        image_pub_ = image_transport::create_publisher(this, "/aruco/image_marked");

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        RCLCPP_INFO(this->get_logger(), "Nodo ArUco iniciado.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    image_transport::Publisher image_pub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Usa función del archivo separado
        cv::Mat frame_marked = detectar_y_dibujar_aruco(frame, dictionary_);
        

        sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
            msg->header, "bgr8", frame_marked).toImageMsg();
        image_pub_.publish(output_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}
