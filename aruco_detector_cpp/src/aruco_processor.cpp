#include "aruco_processor.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

cv::Mat detectar_dibujar_y_estimar_aruco(
    const cv::Mat& frame,
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    std::vector<MarkerPose>& marker_poses
) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Mat frame_marked = frame.clone();

    cv::aruco::detectMarkers(frame_marked, dictionary, corners, ids);

    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(frame_marked, corners, ids);
        cv::aruco::estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); ++i) {
            cv::aruco::drawAxis(frame_marked, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03);

            tf2::Quaternion q;
            cv::Mat rotMat;
            cv::Rodrigues(rvecs[i], rotMat);
            tf2::Matrix3x3(tf2::Matrix3x3(
                rotMat.at<double>(0,0), rotMat.at<double>(0,1), rotMat.at<double>(0,2),
                rotMat.at<double>(1,0), rotMat.at<double>(1,1), rotMat.at<double>(1,2),
                rotMat.at<double>(2,0), rotMat.at<double>(2,1), rotMat.at<double>(2,2)
            )).getRotation(q);

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = rclcpp::Clock().now();
            tf.header.frame_id = "camera_frame";
            tf.child_frame_id = "aruco_" + std::to_string(ids[i]);

            tf.transform.translation.x = tvecs[i][0];
            tf.transform.translation.y = tvecs[i][1];
            tf.transform.translation.z = tvecs[i][2];

            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();

            marker_poses.push_back({ ids[i], tf });
        }
    }

    return frame_marked;
}
