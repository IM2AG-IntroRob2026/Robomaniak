#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// DockDetectorNode
//
// S'abonne à /camera/image_raw, cherche un marqueur ArUco d'ID donné,
// estime sa pose 3D via solvePnP (cv::aruco::estimatePoseSingleMarkers),
// et publie :
//   - /dock/detected    (std_msgs/Bool)             -> flag de présence à chaque frame
//   - /dock/pose        (geometry_msgs/PoseStamped) -> pose du marqueur dans le repère caméra (OpenCV : +X droite, +Y bas, +Z avant)
//   - /dock/debug_image (option)                    -> image annotée pour vérifier la détection
//
// Les intrinsèques sont chargés depuis un fichier YAML compatible ROS
// (format produit par `ros2 run camera_calibration cameracalibrator`).
// ─────────────────────────────────────────────────────────────────────────────

class DockDetectorNode : public rclcpp::Node
{
private:
    int    marker_id_{0};
    double marker_size_m_{0.08};
    bool   publish_debug_{false};

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               detected_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           debug_pub_;

public:
    DockDetectorNode();

private:
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    void loadCalibration(const std::filesystem::path& path);

    [[nodiscard]] static int dictionaryFromString(const std::string& name);

    [[nodiscard]] static geometry_msgs::msg::PoseStamped makePose(
        const cv::Vec3d& rvec,
        const cv::Vec3d& tvec,
        const std_msgs::msg::Header& header);
};