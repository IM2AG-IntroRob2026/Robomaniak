#include "robot_vision/dock_detector_node.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

DockDetectorNode::DockDetectorNode() : Node("dock_detector_node")
{
    declare_parameter<std::string>("camera_info_path", "");
    declare_parameter<int>        ("marker_id",        0);
    declare_parameter<double>     ("marker_size_m",    0.08);
    declare_parameter<std::string>("dictionary",       "DICT_4X4_50");
    declare_parameter<bool>       ("publish_debug",    false);

    const auto info_path = get_parameter("camera_info_path").as_string();
    marker_id_           = get_parameter("marker_id").as_int();
    marker_size_m_       = get_parameter("marker_size_m").as_double();
    const auto dict_name = get_parameter("dictionary").as_string();
    publish_debug_       = get_parameter("publish_debug").as_bool();

    if (info_path.empty()) {
        throw std::runtime_error("Parameter 'camera_info_path' is required.");
    }
    loadCalibration(info_path);

    dictionary_      = cv::aruco::getPredefinedDictionary(dictionaryFromString(dict_name));
    detector_params_ = cv::aruco::DetectorParameters::create();

    detector_params_->adaptiveThreshWinSizeMin  = 3;
    detector_params_->adaptiveThreshWinSizeMax  = 23;
    detector_params_->adaptiveThreshWinSizeStep = 10;
    detector_params_->cornerRefinementMethod    = cv::aruco::CORNER_REFINE_SUBPIX;
    detector_params_->minMarkerPerimeterRate    = 0.02;

    image_sub_    = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&DockDetectorNode::onImage, this, std::placeholders::_1));

    pose_pub_     = create_publisher<geometry_msgs::msg::PoseStamped>("/dock/pose", 10);
    detected_pub_ = create_publisher<std_msgs::msg::Bool>            ("/dock/detected", 10);

    if (publish_debug_) {
        debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/dock/debug_image", 10);
    }

    RCLCPP_INFO(get_logger(),
        "DockDetectorNode ready. marker_id=%d size=%.3fm dict=%s debug=%s",
        marker_id_, marker_size_m_, dict_name.c_str(), publish_debug_ ? "y" : "n");
}

int DockDetectorNode::dictionaryFromString(const std::string& name)
{
    if (name == "DICT_4X4_50")    { return cv::aruco::DICT_4X4_50; }
    if (name == "DICT_4X4_100")   { return cv::aruco::DICT_4X4_100; }
    if (name == "DICT_4X4_250")   { return cv::aruco::DICT_4X4_250; }
    if (name == "DICT_5X5_50")    { return cv::aruco::DICT_5X5_50; }
    if (name == "DICT_5X5_100")   { return cv::aruco::DICT_5X5_100; }
    if (name == "DICT_5X5_250")   { return cv::aruco::DICT_5X5_250; }
    if (name == "DICT_6X6_50")    { return cv::aruco::DICT_6X6_50; }
    if (name == "DICT_6X6_100")   { return cv::aruco::DICT_6X6_100; }
    if (name == "DICT_6X6_250")   { return cv::aruco::DICT_6X6_250; }
    if (name == "DICT_7X7_50")    { return cv::aruco::DICT_7X7_50; }
    if (name == "DICT_APRILTAG_36h11") { return cv::aruco::DICT_APRILTAG_36h11; }
    throw std::invalid_argument("Unknown ArUco dictionary: " + name);
}

void DockDetectorNode::loadCalibration(const std::filesystem::path& path)
{
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("Calibration file not found: " + path.string());
    }

    const YAML::Node root = YAML::LoadFile(path.string());

    std::vector<double> k_data;
    if (const auto& cm = root["camera_matrix"]; cm) {
        if (cm.IsMap() && cm["data"]) {
            k_data = cm["data"].as<std::vector<double>>();
        } else if (cm.IsSequence()) {
            k_data = cm.as<std::vector<double>>();
        }
    }
    if (k_data.size() != 9) {
        throw std::runtime_error(
            "Invalid 'camera_matrix' in " + path.string() +
            " (expected 9 values, got " + std::to_string(k_data.size()) + ").");
    }
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        camera_matrix_.at<double>(i / 3, i % 3) = k_data[static_cast<std::size_t>(i)];
    }

    std::vector<double> d_data;
    if (const auto& dc = root["distortion_coefficients"]; dc) {
        if (dc.IsMap() && dc["data"]) {
            d_data = dc["data"].as<std::vector<double>>();
        } else if (dc.IsSequence()) {
            d_data = dc.as<std::vector<double>>();
        }
    }
    if (d_data.empty()) {
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        RCLCPP_WARN(get_logger(), "No 'distortion_coefficients' found, using zeros.");
    } else {
        dist_coeffs_ = cv::Mat(1, static_cast<int>(d_data.size()), CV_64F);
        for (std::size_t i = 0; i < d_data.size(); ++i) {
            dist_coeffs_.at<double>(0, static_cast<int>(i)) = d_data[i];
        }
    }

    RCLCPP_INFO(get_logger(),
        "Calibration loaded: fx=%.1f fy=%.1f cx=%.1f cy=%.1f (D has %d coeffs)",
        camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
        camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2),
        dist_coeffs_.cols);
}

void DockDetectorNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    cv::Mat frame;
    try {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
        return;
    }
    if (frame.empty()) { return; }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_);

    int target_idx = -1;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == marker_id_) {
            target_idx = static_cast<int>(i);
            break;
        }
    }

    std_msgs::msg::Bool detected_msg;
    detected_msg.data = (target_idx >= 0);
    detected_pub_->publish(detected_msg);

    if (target_idx >= 0) {
        std::vector<std::vector<cv::Point2f>> target_corners{corners[static_cast<std::size_t>(target_idx)]};
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
            target_corners, static_cast<float>(marker_size_m_),
            camera_matrix_, dist_coeffs_, rvecs, tvecs);

        pose_pub_->publish(makePose(rvecs[0], tvecs[0], msg->header));
    }

    if (publish_debug_ && !ids.empty()) {
        cv::Mat debug_frame = frame.clone();
        cv::aruco::drawDetectedMarkers(debug_frame, corners, ids);

        if (target_idx >= 0) {
            std::vector<std::vector<cv::Point2f>> target_corners{corners[static_cast<std::size_t>(target_idx)]};
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                target_corners, static_cast<float>(marker_size_m_),
                camera_matrix_, dist_coeffs_, rvecs, tvecs);
            cv::drawFrameAxes(debug_frame, camera_matrix_, dist_coeffs_,
                rvecs[0], tvecs[0], static_cast<float>(marker_size_m_ * 0.5));
        }

        auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_frame).toImageMsg();
        debug_pub_->publish(*debug_msg);
    }
}

geometry_msgs::msg::PoseStamped DockDetectorNode::makePose(
    const cv::Vec3d& rvec, const cv::Vec3d& tvec,
    const std_msgs::msg::Header& header)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    tf2::Matrix3x3 rot(
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    tf2::Quaternion q;
    rot.getRotation(q);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = tvec[0];
    pose.pose.position.y = tvec[1];
    pose.pose.position.z = tvec[2];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<DockDetectorNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("dock_detector_node"), "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}