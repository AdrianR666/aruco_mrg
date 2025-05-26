#ifndef ARUCO_PROCESSOR_HPP
#define ARUCO_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>

struct MarkerPose {
    int id;
    geometry_msgs::msg::TransformStamped transform;
};

// Detecta y dibuja, además devuelve las poses estimadas
cv::Mat detectar_dibujar_y_estimar_aruco(
    const cv::Mat& frame,
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    std::vector<MarkerPose>& marker_poses
);

#endif // ARUCO_PROCESSOR_HPP
