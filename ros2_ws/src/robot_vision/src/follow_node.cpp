#include <chrono>
#include <format>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "robot_vision/yolo_detector.hpp"


class FollowNode : public rclcpp::Node
{
public:
    FollowNode() : Node("follow_node")
    {
        
        this->declare_parameter<std::string>("strategy", "default");
        this->declare_parameter<double>("follow_speed", 0.5);
        this->declare_parameter<double>("follow_distance", 1.0); // % de la taille de la boxe dans l'image, (l'image est toujours centrée)

        const auto  strategy          = this->get_parameter("strategy").as_string();
        const double follow_speed     = this->get_parameter("follow_speed").as_double();
        const double follow_distance  = this->get_parameter("follow_distance").as_double();

        RCLCPP_INFO(this->get_logger(), "FollowNode initialized with strategy '%s', follow_speed=%.2f, follow_distance=%.2f",
            strategy.c_str(), follow_speed, follow_distance);

        human_pub_ = this->create_subscription<std_msgs::msg::Bool>("/detection/human_present", 10);
        coordinate_pub_ = this->create_subscription<robot_vision::Detection>("/detection/coordinates", 10);
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_pub_;
    rclcpp::Subscription<robot_vision::Detection>::SharedPtr coordinate_pub_; // A voir si Dectection est passable
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;

    void PersonCenter(const robot_vision::Detection& detection)
    {
        // Mettre le centre de la bbox et la mettre au centre
        if (detection.cx < 1920/2) {
            // Personne à gauche, tourner à gauche
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0; // Pas de déplacement linéaire
            cmd.angular.z = 0.5; // Tourner à gauche
            RCLCPP_INFO(this->get_logger(), "Person detected on the left, turning left.");
            velocity_pub_->publish(cmd);
        } 
        else if (detection.cx > 1920/2) {
            // Personne à droite, tourner à droite
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0; // Pas de déplacement linéaire
            cmd.angular.z = -0.5; // Tourner à droite
            RCLCPP_INFO(this->get_logger(), "Person detected on the right, turning right.");
            velocity_pub_->publish(cmd);
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<FollowNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("follow_node"), "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

