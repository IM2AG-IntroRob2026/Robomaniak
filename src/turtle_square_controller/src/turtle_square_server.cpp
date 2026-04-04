#include "turtle_square_server.h"

TurtleSquareServer::TurtleSquareServer(const rclcpp::NodeOptions& options)
    : Node("turtle_square_server", options)
{
    // Initialize parameters
    this->declare_parameter<double>("default_side_length", 2.0);
    this->declare_parameter<double>("default_speed", 1.0);
    this->get_parameter("default_side_length", default_side_length_);
    this->get_parameter("default_speed", default_speed_);

    // Create publisher and subscriber
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&TurtleSquareServer::pose_callback, this, std::placeholders::_1));

    // Create action server
    action_server_ = rclcpp_action::create_server<DrawSquare>(
        this,
        "draw_square",
        std::bind(&TurtleSquareServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TurtleSquareServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&TurtleSquareServer::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse TurtleSquareServer::handle_goal(const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const DrawSquare::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with side_length: %.2f and speed: %.2f", goal->side_length, goal->speed);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurtleSquareServer::handle_cancel(const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtleSquareServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread{std::bind(&TurtleSquareServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void TurtleSquareServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
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

void TurtleSquareServer::pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
    current_pose_ = *msg;
}

bool TurtleSquareServer::move_forward(
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

    geometry_msgs::msg::Twist cmd;

    while (distance_traveled < distance)
    {
        // Vérification annulation
        if (goal_handle->is_canceling())
        {
            // Arrêt d'urgence
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
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
        cmd.linear.x = speed;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);

        // Publication feedback
        auto feedback = std::make_shared<DrawSquare::Feedback>();
        feedback->remaining_distance = static_cast<float>(
            distance_offset - distance_traveled);
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }

    // Arrêt propre
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);

    // Petit délai pour stabilisation
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    return true;
}

bool TurtleSquareServer::turn(
    double angle,
    double angular_speed,
    const std::shared_ptr<GoalHandle>& goal_handle)
{
    // Calcul angle cible
    const double start_theta = current_pose_.theta;
    const double target_theta = normalize_angle(start_theta + angle);

    // Tolérance (en radians) : ~0.57°
    constexpr double ANGLE_TOLERANCE = 0.01;

    rclcpp::Rate rate(10);  // 10 Hz
    geometry_msgs::msg::Twist cmd;

    while (true)
    {
        // Vérification annulation
        if (goal_handle->is_canceling())
        {
            // Arrêt d'urgence
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
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
        cmd.linear.x = 0.0;
        cmd.angular.z = (angle_diff > 0.0) ? angular_speed : -angular_speed;
        cmd_vel_pub_->publish(cmd);

        rate.sleep();
    }

    // Arrêt propre
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);

    // Délai stabilisation
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    return true;
}

double TurtleSquareServer::normalize_angle(double angle)
{
    // Méthode optimisée avec fmod
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0)
        angle += 2.0 * M_PI;
    return angle - M_PI;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSquareServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}