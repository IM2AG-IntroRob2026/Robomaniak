#include "robot_vision/dock_calibration_node.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>

DockCalibrationNode::DockCalibrationNode() : Node("dock_calibration_node")
{
    declare_parameter<std::string>("mode",             "raw");
    declare_parameter<int>        ("samples_target",   30);
    declare_parameter<double>     ("known_distance_m", 1.0);
    declare_parameter<double>     ("known_height_m",   0.0);
    declare_parameter<std::string>("log_path",         "/tmp/dock_pose_log.csv");

    mode_             = get_parameter("mode").as_string();
    samples_target_   = get_parameter("samples_target").as_int();
    known_distance_m_ = get_parameter("known_distance_m").as_double();
    known_height_m_   = get_parameter("known_height_m").as_double();
    log_path_         = get_parameter("log_path").as_string();

    if (mode_ != "raw" && mode_ != "pitch" && mode_ != "log") {
        throw std::invalid_argument("Unknown mode '" + mode_ + "'. Use 'raw', 'pitch' or 'log'.");
    }

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/dock/pose", 10,
        std::bind(&DockCalibrationNode::onPose, this, std::placeholders::_1));

    start_time_ = this->now();
}

void DockCalibrationNode::onPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    Sample s{
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    };

    if (mode_ == "raw")        { handleRaw(s); }
    else if (mode_ == "pitch") { handlePitch(s); }
    else                       { handleLog(s); }
}

void DockCalibrationNode::handleRaw(const Sample& s) const
{
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 500,
        "marker pose [cam OpenCV] : x=%+.3fm  y=%+.3fm  z=%+.3fm "
        "(distance euclidienne = %.3fm)",
        s.x, s.y, s.z, std::sqrt(s.x*s.x + s.y*s.y + s.z*s.z));
}

void DockCalibrationNode::handlePitch(const Sample& s)
{
    samples_.push_back(s);
    if (static_cast<int>(samples_.size()) < samples_target_) {
        if (samples_.size() % 5 == 0) {
            RCLCPP_INFO(get_logger(), "Collected %zu / %d samples...",
                samples_.size(), samples_target_);
        }
        return;
    }

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto& sample : samples_) {
        sum_x += sample.x;
        sum_y += sample.y;
        sum_z += sample.z;
    }
    const double n = static_cast<double>(samples_.size());
    const double avg_x = sum_x / n;
    const double avg_y = sum_y / n;
    const double avg_z = sum_z / n;

    double var_x = 0.0, var_y = 0.0, var_z = 0.0;
    for (const auto& sample : samples_) {
        var_x += (sample.x - avg_x) * (sample.x - avg_x);
        var_y += (sample.y - avg_y) * (sample.y - avg_y);
        var_z += (sample.z - avg_z) * (sample.z - avg_z);
    }
    const double std_x = std::sqrt(var_x / n);
    const double std_y = std::sqrt(var_y / n);
    const double std_z = std::sqrt(var_z / n);

    const double pitch_rad = estimatePitchRad(avg_y, avg_z, known_distance_m_, known_height_m_);
    const double pitch_deg = pitch_rad * 180.0 / M_PI;

    std::ostringstream oss;
    oss << "\n"
        << "═════════════════════════════════════════════════════════════════\n"
        << "             RÉSULTAT DE CALIBRATION DU PITCH\n"
        << "═════════════════════════════════════════════════════════════════\n"
        << " Échantillons utilisés     : " << samples_.size() << "\n"
        << " Distance réelle annoncée  : " << std::fixed << std::setprecision(3)
                                           << known_distance_m_ << " m\n"
        << " Hauteur signée annoncée   : " << known_height_m_   << " m\n"
        << "\n"
        << " Pose moyenne (cam OpenCV) :\n"
        << "   x_cv = " << std::setw(7) << std::showpos << avg_x
                        << " m   (σ = " << std::noshowpos << std_x << ")\n"
        << "   y_cv = " << std::setw(7) << std::showpos << avg_y
                        << " m   (σ = " << std::noshowpos << std_y << ")\n"
        << "   z_cv = " << std::setw(7) << std::showpos << avg_z
                        << " m   (σ = " << std::noshowpos << std_z << ")\n"
        << "\n"
        << " >>> camera_pitch_deg estimé = "
                << std::showpos << std::setprecision(2) << pitch_deg << "°\n"
        << "═════════════════════════════════════════════════════════════════";

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

    samples_.clear();
}

void DockCalibrationNode::handleLog(const Sample& s)
{
    samples_.push_back(s);
    if (static_cast<int>(samples_.size()) < samples_target_) { return; }

    std::ofstream out(log_path_);
    if (!out.is_open()) {
        RCLCPP_ERROR(get_logger(), "Cannot open log file '%s'", log_path_.c_str());
        return;
    }
    out << "x_cv,y_cv,z_cv\n";
    for (const auto& sample : samples_) {
        out << sample.x << ',' << sample.y << ',' << sample.z << '\n';
    }
    out.close();

    RCLCPP_INFO(get_logger(), "Wrote %zu samples to %s", samples_.size(), log_path_.c_str());
    samples_.clear();
}

double DockCalibrationNode::estimatePitchRad(double avg_y_cv, double avg_z_cv, double known_dist_m, double known_height_m)
{
    const double alpha_apparent = std::atan2(-avg_y_cv, avg_z_cv);
    const double alpha_vrai     = std::atan2(known_height_m, known_dist_m);
    const double pitch_physique = alpha_apparent - alpha_vrai;
    return -pitch_physique;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<DockCalibrationNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("dock_calibration_node"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}