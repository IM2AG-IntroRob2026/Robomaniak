#include "robot_square_server.h"

RobotSquareServer::RobotSquareServer(const rclcpp::NodeOptions& options)
    : Node("robot_square_server", options)
{
    // Initialize parameters
    this->declare_parameter<double>("default_side_length", 2.0);
    this->declare_parameter<double>("default_speed", 1.0);
    this->get_parameter("default_side_length", default_side_length_);
    this->get_parameter("default_speed", default_speed_);

    // Create publisher and subscriber
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&RobotSquareServer::odom_callback, this, std::placeholders::_1)
);
    // Create action server
    action_server_ = rclcpp_action::create_server<DrawSquare>(
        this,
        "draw_square",
        std::bind(&RobotSquareServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&RobotSquareServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&RobotSquareServer::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse RobotSquareServer::handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DrawSquare::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with side_length: %.2f and speed: %.2f", goal->side_length, goal->speed);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotSquareServer::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotSquareServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread{std::bind(&RobotSquareServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void RobotSquareServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting square execution");

    // Récupération des paramètres
    auto goal = goal_handle->get_goal();
    const double side_length = (goal->side_length > 0.0f) ?
                               goal->side_length : default_side_length_;
    const double speed = (goal->speed > 0.0f) ?
                         goal->speed : default_speed_;

    // Paramètres de rotation
    constexpr double TURN_ANGLE = M_PI / 2.0;  // 90°
    constexpr double ANGULAR_SPEED = 1.0;      // rad/s

    // Calcul distance totale du carré
    const double total_distance = 4.0 * side_length;

    // Dessiner les 4 côtés
    for (int i = 0; i < 4; ++i)
    {
        // Calcul distance restante (pour feedback)
        const double remaining = total_distance - (i * side_length);

        // Avancer d'un côté
        if (!move_forward(side_length, speed, goal_handle, remaining))
        {
            // Annulation détectée
            return;
        }

        // Tourner de 90° (sauf après le dernier côté)
        if (i < 3)
        {
            if (!turn(TURN_ANGLE, ANGULAR_SPEED, goal_handle))
            {
                // Annulation détectée
                return;
            }
        }
    }

    // Succès : retour au point de départ
    auto result = std::make_shared<DrawSquare::Result>();
    result->success = true;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Square completed successfully");
}

bool RobotSquareServer::move_forward(
    double distance,
    double speed,
    const std::shared_ptr<GoalHandle>& goal_handle,
    double distance_offset)
{
    // Sauvegarde position de départ
    const double start_x = current_pose_.x;
    const double start_y = current_pose_.y;

    double distance_traveled = 0.0;
    rclcpp::Rate rate(10);  // 10 Hz

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

    while (distance_traveled < distance)
    {
        // Vérification annulation
        if (goal_handle->is_canceling())
        {
            // Arrêt d'urgence
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);

            auto result = std::make_shared<DrawSquare::Result>();
            result->success = false;
            goal_handle->canceled(result);

            RCLCPP_WARN(this->get_logger(), "Goal canceled during move_forward");
            return false;
        }

        // Calcul distance parcourue (euclidienne)
        const double dx = current_pose_.x - start_x;
        const double dy = current_pose_.y - start_y;
        distance_traveled = std::sqrt(dx * dx + dy * dy);

        // Publication commande de vitesse
        cmd.header.stamp = this->now();
        cmd.twist.linear.x = speed;
        cmd.twist.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);

        // Publication feedback
        auto feedback = std::make_shared<DrawSquare::Feedback>();
        feedback->remaining_distance = static_cast<float>(
            distance_offset - distance_traveled);
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }

    // Arrêt propre
    cmd.header.stamp = this->now();
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);

    // Petit délai pour stabilisation
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    return true;
}

bool RobotSquareServer::turn(
    double angle,
    double angular_speed,
    const std::shared_ptr<GoalHandle>& goal_handle)
{
    // Calcul angle cible
    const double start_theta = current_pose_.theta;
    const double target_theta = normalize_angle(start_theta + angle);

    // Tolérance (en radians) : ~0.57°
    constexpr double ANGLE_TOLERANCE = 0.04;

    rclcpp::Rate rate(10);  // 10 Hz
    geometry_msgs::msg::TwistStamped cmd;

    while (true)
    {
        // Vérification annulation
        if (goal_handle->is_canceling())
        {
            // Arrêt d'urgence
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);

            auto result = std::make_shared<DrawSquare::Result>();
            result->success = false;
            goal_handle->canceled(result);

            RCLCPP_WARN(this->get_logger(), "Goal canceled during turn");
            return false;
        }

        // Calcul différence angulaire
        const double angle_diff = normalize_angle(target_theta - current_pose_.theta);

        // Vérification atteinte de la cible
        if (std::abs(angle_diff) < ANGLE_TOLERANCE)
        {
            break;
        }

        // Publication commande de rotation
        // Sens de rotation selon signe de angle_diff
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = (angle_diff > 0.0) ? angular_speed : -angular_speed;
        cmd_vel_pub_->publish(cmd);

        rate.sleep();
    }

    // Arrêt propre
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);

    // Délai stabilisation
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    return true;
}

double RobotSquareServer::normalize_angle(double angle)
{
    // Méthode optimisée avec fmod
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0)
        angle += 2.0 * M_PI;
    return angle - M_PI;
}

void RobotSquareServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 1. Position classique (X, Y)
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;

    // 2. Conversion du Quaternion (3D) vers Theta (Angle 2D)
    double q_x = msg->pose.pose.orientation.x;
    double q_y = msg->pose.pose.orientation.y;
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;

    // Formule pour extraire le Yaw (rotation autour de Z)
    double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
    current_pose_.theta = std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSquareServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}