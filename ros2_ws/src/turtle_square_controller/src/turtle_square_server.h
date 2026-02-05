#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtle_square_interfaces/action/draw_square.hpp>


class TurtleSquareServer : public rclcpp::Node
{
public:
    using DrawSquare = turtle_square_interfaces::action::DrawSquare;
    using GoalHandle = rclcpp_action::ServerGoalHandle<DrawSquare>;

    explicit TurtleSquareServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Action server
    rclcpp_action::Server<DrawSquare>::SharedPtr action_server_;

    // Publisher/Subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

    // État
    turtlesim::msg::Pose current_pose_;

    // Parameters
    double default_side_length_;
    double default_speed_;

    // Callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DrawSquare::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);

    /**
     * @brief Déplace la tortue en ligne droite
     * @param distance Distance à parcourir (mètres)
     * @param speed Vitesse linéaire (m/s)
     * @param goal_handle Handle pour feedback et annulation
     * @param distance_offset Distance déjà parcourue (pour feedback)
     * @return true si succès, false si annulé
     */
    bool move_forward(
        double distance,
        double speed,
        const std::shared_ptr<GoalHandle>& goal_handle,
        double distance_offset = 0.0);

    /**
     * @brief Fait tourner la tortue d'un angle donné
     * @param angle Angle de rotation en radians (positif = sens antihoraire)
     * @param angular_speed Vitesse angulaire (rad/s)
     * @param goal_handle Handle pour annulation
     * @return true si succès, false si annulé
     */
    bool turn(
        double angle,
        double angular_speed,
        const std::shared_ptr<GoalHandle>& goal_handle);

    /**
     * @brief Normalise un angle dans l'intervalle [-π, π]
     * @param angle Angle en radians
     * @return Angle normalisé
     */
    static double normalize_angle(double angle);
};