#include "aruco_processor.hpp"

cv::Mat detectar_y_dibujar_aruco(const cv::Mat& frame, const cv::Ptr<cv::aruco::Dictionary>& dictionary)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat frame_marked = frame.clone();

    cv::aruco::detectMarkers(frame_marked, dictionary, corners, ids);

    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(frame_marked, corners, ids);
    }

    //Debug opcional:
    cv::imshow("Aruco Debug", frame);
    cv::waitKey(1);

    return frame_marked;
}
