#ifndef ARUCO_PROCESSOR_HPP
#define ARUCO_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

cv::Mat detectar_y_dibujar_aruco(const cv::Mat& frame, const cv::Ptr<cv::aruco::Dictionary>& dictionary);

#endif // ARUCO_PROCESSOR_HPP
