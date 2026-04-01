#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <turtle_boundaries_interfaces/action/draw_boundaries.hpp>
#include <turtlesim/msg/color.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/set_pen.hpp>

/**
 * @brief Serveur d'action ROS 2 pour tracer le contour d'un mur/objet.
 *
 * Strategie implementee:
 * 1) Deplacement autonome avec stylo OFF jusqu'a rencontrer un "mur".
 *    Un mur est detecte soit par capteur couleur (rouge), soit par proximite d'un bord domaine.
 * 2) Au premier contact mur: stylo ON.
 * 3) Suivi du contour en continu (avance quand le mur est detecte, sinon rotation de recherche).
 * 4) Arret quand la tortue revient dans la zone de depart du trace,
 *    ce qui signifie qu'elle a fait une boucle complete autour de la piece/objet.
 *
 * Contrat action DrawBoundaries:
 * - Goal:
 *   - margin: marge de securite utilisee pour la detection de bord domaine
 *   - speed: vitesse lineaire de suivi (>0, sinon vitesse par defaut)
 *   - clockwise: sens prefere de rotation pendant la recherche du mur
 * - Result:
 *   - success: true si le tour est boucle
 *   - perimeter: distance effectivement tracee
 *   - message: raison de succes/echec
 * - Feedback:
 *   - remaining_distance: inconnue dans cette strategie (valeur -1)
 *   - side_index: 0 = phase recherche mur, 1 = phase suivi contour
 */
class TurtleBoundaries : public rclcpp::Node
{
public:
  using DrawBoundaries = turtle_boundaries_interfaces::action::DrawBoundaries;
  using GoalHandle = rclcpp_action::ServerGoalHandle<DrawBoundaries>;

  explicit TurtleBoundaries(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Infrastructure ROS
  rclcpp_action::Server<DrawBoundaries>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<turtlesim::msg::Color>::SharedPtr color_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pause_toggle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;

  // Etat capteurs
  turtlesim::msg::Pose current_pose_{};
  turtlesim::msg::Color current_color_{};
  bool pose_received_{false};
  bool color_received_{false};

  // Parametres
  double default_margin_{0.5};
  double default_speed_{1.2};
  double search_speed_{0.8};
  double follow_turn_speed_{1.2};
  double domain_min_{0.5};
  double domain_max_{10.5};
  double loop_closure_radius_{0.35};
  double min_loop_distance_{4.0};
  double search_timeout_s_{20.0};
  double trace_timeout_s_{120.0};
  double lost_wall_grace_s_{0.45};
  double manual_cmd_timeout_s_{0.25};
  double resume_reacquire_timeout_s_{8.0};
  double resume_reacquire_radius_{0.22};

  int red_min_{180};
  int red_max_green_{100};
  int red_max_blue_{100};
  int trace_max_rgb_{60};

  mutable std::mutex manual_control_mutex_;
  bool manual_pause_active_{false};
  geometry_msgs::msg::Twist manual_cmd_{};
  rclcpp::Time last_manual_cmd_time_{0, 0, RCL_ROS_TIME};

  // Action callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DrawBoundaries::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // Capteurs
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);
  void color_callback(const turtlesim::msg::Color::SharedPtr msg);
  void pause_toggle_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void manual_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  bool is_manual_pause_active() const;
  geometry_msgs::msg::Twist get_manual_cmd(double * age_s = nullptr) const;

  // Aides pilotage
  bool set_pen(bool off, uint8_t width = 2, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0);
  bool is_red_wall_detected() const;
  bool is_own_trace_detected() const;
  bool is_near_domain_boundary(double margin) const;
  bool is_wall_detected(double margin) const;
  static double distance_between(const turtlesim::msg::Pose & a, const turtlesim::msg::Pose & b);
  static double normalize_angle(double angle);
  double boundary_tangent_heading(bool clockwise) const;
  void safe_publish_feedback(const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<DrawBoundaries::Feedback> & feedback);
  void safe_succeed(const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<DrawBoundaries::Result> & result);
  void safe_abort(const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<DrawBoundaries::Result> & result);
  void safe_cancel(const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<DrawBoundaries::Result> & result);
  void stop_turtle();
};
