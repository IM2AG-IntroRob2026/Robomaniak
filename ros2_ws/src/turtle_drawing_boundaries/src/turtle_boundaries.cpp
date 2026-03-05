#include "turtle_boundaries.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

TurtleBoundaries::TurtleBoundaries(const rclcpp::NodeOptions & options)
: Node("turtle_boundaries", options)
{
  this->declare_parameter<double>("default_margin", 0.5);
  this->declare_parameter<double>("default_speed", 1.2);
  this->declare_parameter<double>("search_speed", 0.8);
  this->declare_parameter<double>("follow_turn_speed", 1.2);
  this->declare_parameter<double>("domain_min", 0.0);
  this->declare_parameter<double>("domain_max", 11.0);
  this->declare_parameter<double>("loop_closure_radius", 0.45);
  this->declare_parameter<double>("min_loop_distance", 4.0);
  this->declare_parameter<double>("search_timeout_s", 20.0);
  this->declare_parameter<double>("trace_timeout_s", 240.0);

  this->declare_parameter<int>("red_min", 180);
  this->declare_parameter<int>("red_max_green", 100);
  this->declare_parameter<int>("red_max_blue", 100);
  this->declare_parameter<int>("trace_max_rgb", 60);

  this->get_parameter("default_margin", default_margin_);
  this->get_parameter("default_speed", default_speed_);
  this->get_parameter("search_speed", search_speed_);
  this->get_parameter("follow_turn_speed", follow_turn_speed_);
  this->get_parameter("domain_min", domain_min_);
  this->get_parameter("domain_max", domain_max_);
  this->get_parameter("loop_closure_radius", loop_closure_radius_);
  this->get_parameter("min_loop_distance", min_loop_distance_);
  this->get_parameter("search_timeout_s", search_timeout_s_);
  this->get_parameter("trace_timeout_s", trace_timeout_s_);

  this->get_parameter("red_min", red_min_);
  this->get_parameter("red_max_green", red_max_green_);
  this->get_parameter("red_max_blue", red_max_blue_);
  this->get_parameter("trace_max_rgb", trace_max_rgb_);

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

  pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
    "/turtle1/pose", 10,
    std::bind(&TurtleBoundaries::pose_callback, this, std::placeholders::_1));

  color_sub_ = this->create_subscription<turtlesim::msg::Color>(
    "/turtle1/color_sensor", 10,
    std::bind(&TurtleBoundaries::color_callback, this, std::placeholders::_1));

  set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

  action_server_ = rclcpp_action::create_server<DrawBoundaries>(
    this,
    "draw_boundaries",
    std::bind(&TurtleBoundaries::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TurtleBoundaries::handle_cancel, this, std::placeholders::_1),
    std::bind(&TurtleBoundaries::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse TurtleBoundaries::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const DrawBoundaries::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(
    this->get_logger(),
    "Goal recu: margin=%.2f speed=%.2f clockwise=%s",
    goal->margin, goal->speed, goal->clockwise ? "true" : "false");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurtleBoundaries::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Demande d'annulation recue");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtleBoundaries::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&TurtleBoundaries::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void TurtleBoundaries::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  try {
  auto result = std::make_shared<DrawBoundaries::Result>();

  // Attend pose + color sensor avant de piloter.
  for (int retry = 0; retry < 30 && (!pose_received_ || !color_received_) && rclcpp::ok(); ++retry) {
    rclcpp::sleep_for(100ms);
  }

  if (!pose_received_ || !color_received_) {
    result->success = false;
    result->perimeter = 0.0F;
    result->message = "Capteurs indisponibles (pose/couleur)";
    safe_abort(goal_handle, result);
    return;
  }

  const auto goal = goal_handle->get_goal();
  const double follow_speed = goal->speed > 0.0 ? goal->speed : default_speed_;
  const double effective_margin = goal->margin > 0.0 ? goal->margin : default_margin_;
  const double search_linear_speed = std::max(0.2, std::min(search_speed_, follow_speed));
  const double contact_margin = std::clamp(effective_margin * 0.10, 0.02, 0.08);
  const double follow_margin = std::clamp(effective_margin * 0.12, 0.03, 0.10);

  // Phase 1: on se deplace sans dessiner jusqu'a detecter un mur.
  if (!set_pen(true)) {
    result->success = false;
    result->perimeter = 0.0F;
    result->message = "Service /turtle1/set_pen indisponible";
    safe_abort(goal_handle, result);
    return;
  }

  const rclcpp::Time search_start = this->now();
  rclcpp::Rate search_rate(20);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stop_turtle();
      auto cancel_result = std::make_shared<DrawBoundaries::Result>();
      cancel_result->success = false;
      cancel_result->perimeter = 0.0F;
      cancel_result->message = "Action annulee pendant la recherche du mur";
      safe_cancel(goal_handle, cancel_result);
      return;
    }

    // Contact "mur" strict: evite d'activer le stylo trop loin du bord.
    if (is_wall_detected(contact_margin)) {
      break;
    }

    geometry_msgs::msg::Twist cmd;

    // Si proche du bord pendant la recherche, tourne pour rester dans la zone.
    if (is_near_domain_boundary(0.03)) {
      cmd.linear.x = 0.0;
      cmd.angular.z = goal->clockwise ? -follow_turn_speed_ : follow_turn_speed_;
    } else {
      cmd.linear.x = search_linear_speed;
      cmd.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(cmd);

    auto feedback = std::make_shared<DrawBoundaries::Feedback>();
    feedback->remaining_distance = -1.0F;
    feedback->side_index = 0U;
    safe_publish_feedback(goal_handle, feedback);

    if ((this->now() - search_start).seconds() > search_timeout_s_) {
      stop_turtle();
      result->success = false;
      result->perimeter = 0.0F;
      result->message = "Mur non detecte dans le delai";
      safe_abort(goal_handle, result);
      return;
    }

    search_rate.sleep();
  }

  stop_turtle();

  // Contact mur atteint: on commence le trace.
  if (!set_pen(false)) {
    result->success = false;
    result->perimeter = 0.0F;
    result->message = "Impossible d'activer le stylo";
    safe_abort(goal_handle, result);
    return;
  }

  const turtlesim::msg::Pose trace_start_pose = current_pose_;
  turtlesim::msg::Pose previous_pose = current_pose_;
  double traced_distance = 0.0;
  bool has_left_start_zone = false;

  const rclcpp::Time trace_start = this->now();
  rclcpp::Rate trace_rate(30);

  // Phase 2: suivi du contour jusqu'a fermeture de boucle.
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stop_turtle();
      auto cancel_result = std::make_shared<DrawBoundaries::Result>();
      cancel_result->success = false;
      cancel_result->perimeter = static_cast<float>(traced_distance);
      cancel_result->message = "Action annulee pendant le suivi du contour";
      safe_cancel(goal_handle, cancel_result);
      return;
    }

    traced_distance += distance_between(previous_pose, current_pose_);
    previous_pose = current_pose_;

    const double dist_to_start = distance_between(current_pose_, trace_start_pose);

    // On force un vrai depart avant d'autoriser la fermeture de boucle.
    if (dist_to_start > loop_closure_radius_) {
      has_left_start_zone = true;
    }

    const bool loop_closed =
      has_left_start_zone &&
      (dist_to_start <= loop_closure_radius_) &&
      (traced_distance >= min_loop_distance_) &&
      (is_own_trace_detected() || (dist_to_start <= (loop_closure_radius_ * 0.6)));

    if (loop_closed) {
      stop_turtle();
      result->success = true;
      result->perimeter = static_cast<float>(traced_distance);
      result->message = "Boucle fermee: contour complet trace";
      safe_succeed(goal_handle, result);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    const bool near_boundary = is_near_domain_boundary(follow_margin);
    const bool red_wall = is_red_wall_detected();

    if (near_boundary) {
      // Si on est au bord du domaine, on suit la tangente du bord
      // au lieu de pousser vers le mur.
      const double target_heading = boundary_tangent_heading(goal->clockwise);
      const double heading_error = normalize_angle(
        target_heading - static_cast<double>(current_pose_.theta));
      const double turn_cmd = std::clamp(
        3.5 * heading_error,
        -follow_turn_speed_,
        follow_turn_speed_);

      if (std::abs(heading_error) > 0.25) {
        cmd.linear.x = 0.0;
      } else {
        cmd.linear.x = std::min(
          std::min(0.55, follow_speed),
          follow_speed * (1.0 - std::min(std::abs(heading_error) / 1.2, 0.6)));
      }
      cmd.angular.z = turn_cmd;
    } else if (red_wall) {
      // Mur objet (rouge): biais angulaire leger pour longer le contour.
      cmd.linear.x = follow_speed * 0.8;
      cmd.angular.z = goal->clockwise ? -0.25 : 0.25;
    } else {
      // Mur perdu: petite avance + rotation dans le sens choisi pour reacquerir.
      cmd.linear.x = std::min(0.25, follow_speed * 0.25);
      cmd.angular.z = goal->clockwise ? -follow_turn_speed_ : follow_turn_speed_;
    }

    cmd_vel_pub_->publish(cmd);

    auto feedback = std::make_shared<DrawBoundaries::Feedback>();
    feedback->remaining_distance = -1.0F;
    feedback->side_index = 1U;
    safe_publish_feedback(goal_handle, feedback);

    if ((this->now() - trace_start).seconds() > trace_timeout_s_) {
      stop_turtle();
      result->success = false;
      result->perimeter = static_cast<float>(traced_distance);
      result->message = "Timeout pendant le suivi du contour";
      safe_abort(goal_handle, result);
      return;
    }

    trace_rate.sleep();
  }

  stop_turtle();
  if (!rclcpp::ok()) {
    return;
  }
  result->success = false;
  result->perimeter = static_cast<float>(traced_distance);
  result->message = "Execution interrompue";
  safe_abort(goal_handle, result);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "execute interrompu: %s", e.what());
    stop_turtle();
  }
}

void TurtleBoundaries::pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
  current_pose_ = *msg;
  pose_received_ = true;
}

void TurtleBoundaries::color_callback(const turtlesim::msg::Color::SharedPtr msg)
{
  current_color_ = *msg;
  color_received_ = true;
}

bool TurtleBoundaries::set_pen(bool off, uint8_t width, uint8_t r, uint8_t g, uint8_t b)
{
  if (!set_pen_client_->wait_for_service(2s)) {
    return false;
  }

  auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
  request->off = off ? 1U : 0U;
  request->width = width;
  request->r = r;
  request->g = g;
  request->b = b;

  auto future = set_pen_client_->async_send_request(request);
  return future.wait_for(2s) == std::future_status::ready;
}

bool TurtleBoundaries::is_red_wall_detected() const
{
  if (!color_received_) {
    return false;
  }

  return
    static_cast<int>(current_color_.r) >= red_min_ &&
    static_cast<int>(current_color_.g) <= red_max_green_ &&
    static_cast<int>(current_color_.b) <= red_max_blue_;
}

bool TurtleBoundaries::is_own_trace_detected() const
{
  if (!color_received_) {
    return false;
  }

  return
    static_cast<int>(current_color_.r) <= trace_max_rgb_ &&
    static_cast<int>(current_color_.g) <= trace_max_rgb_ &&
    static_cast<int>(current_color_.b) <= trace_max_rgb_;
}

bool TurtleBoundaries::is_near_domain_boundary(double margin) const
{
  if (!pose_received_) {
    return false;
  }

  const double min_domain = std::min(domain_min_, domain_max_);
  const double max_domain = std::max(domain_min_, domain_max_);
  const double max_margin = std::max(0.02, ((max_domain - min_domain) / 2.0) - 0.05);
  const double m = std::clamp(margin, 0.02, max_margin);

  return
    current_pose_.x <= (min_domain + m) ||
    current_pose_.x >= (max_domain - m) ||
    current_pose_.y <= (min_domain + m) ||
    current_pose_.y >= (max_domain - m);
}

bool TurtleBoundaries::is_wall_detected(double margin) const
{
  return is_red_wall_detected() || is_near_domain_boundary(margin);
}

double TurtleBoundaries::distance_between(const turtlesim::msg::Pose & a, const turtlesim::msg::Pose & b)
{
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  return std::sqrt((dx * dx) + (dy * dy));
}

double TurtleBoundaries::normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double TurtleBoundaries::boundary_tangent_heading(bool clockwise) const
{
  const double min_domain = std::min(domain_min_, domain_max_);
  const double max_domain = std::max(domain_min_, domain_max_);
  constexpr double edge_eps = 0.08;

  const double x = static_cast<double>(current_pose_.x);
  const double y = static_cast<double>(current_pose_.y);
  const bool near_left = x <= (min_domain + edge_eps);
  const bool near_right = x >= (max_domain - edge_eps);
  const bool near_bottom = y <= (min_domain + edge_eps);
  const bool near_top = y >= (max_domain - edge_eps);

  // Coins: cap explicite pour sortir proprement du coin sans s'enfoncer.
  if (near_top && near_right) {
    return clockwise ? (-kPi / 2.0) : kPi;
  }
  if (near_bottom && near_right) {
    return clockwise ? kPi : (kPi / 2.0);
  }
  if (near_bottom && near_left) {
    return clockwise ? (kPi / 2.0) : 0.0;
  }
  if (near_top && near_left) {
    return clockwise ? 0.0 : (-kPi / 2.0);
  }

  // Bords simples.
  if (near_right) {
    return clockwise ? (-kPi / 2.0) : (kPi / 2.0);
  }
  if (near_left) {
    return clockwise ? (kPi / 2.0) : (-kPi / 2.0);
  }
  if (near_top) {
    return clockwise ? 0.0 : kPi;
  }
  if (near_bottom) {
    return clockwise ? kPi : 0.0;
  }

  // Fallback si aucune paroi n'est strictement proche.
  const double d_left = std::abs(x - min_domain);
  const double d_right = std::abs(max_domain - x);
  const double d_bottom = std::abs(y - min_domain);
  const double d_top = std::abs(max_domain - y);

  if (d_right <= d_left && d_right <= d_bottom && d_right <= d_top) {
    return clockwise ? (-kPi / 2.0) : (kPi / 2.0);
  }
  if (d_left <= d_right && d_left <= d_bottom && d_left <= d_top) {
    return clockwise ? (kPi / 2.0) : (-kPi / 2.0);
  }
  if (d_top <= d_left && d_top <= d_right && d_top <= d_bottom) {
    return clockwise ? 0.0 : kPi;
  }
  return clockwise ? kPi : 0.0;
}

void TurtleBoundaries::safe_publish_feedback(
  const std::shared_ptr<GoalHandle> & goal_handle,
  const std::shared_ptr<DrawBoundaries::Feedback> & feedback)
{
  if (!goal_handle || !rclcpp::ok()) {
    return;
  }
  if (!goal_handle->is_active()) {
    return;
  }
  try {
    goal_handle->publish_feedback(feedback);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "feedback ignore (shutdown): %s", e.what());
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "feedback ignore (unknown error)");
  }
}

void TurtleBoundaries::safe_succeed(
  const std::shared_ptr<GoalHandle> & goal_handle,
  const std::shared_ptr<DrawBoundaries::Result> & result)
{
  if (!goal_handle || !rclcpp::ok()) {
    return;
  }
  if (!goal_handle->is_active()) {
    return;
  }
  try {
    goal_handle->succeed(result);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "succeed ignore (shutdown): %s", e.what());
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "succeed ignore (unknown error)");
  }
}

void TurtleBoundaries::safe_abort(
  const std::shared_ptr<GoalHandle> & goal_handle,
  const std::shared_ptr<DrawBoundaries::Result> & result)
{
  if (!goal_handle || !rclcpp::ok()) {
    return;
  }
  if (!goal_handle->is_active()) {
    return;
  }
  try {
    goal_handle->abort(result);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "abort ignore (shutdown): %s", e.what());
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "abort ignore (unknown error)");
  }
}

void TurtleBoundaries::safe_cancel(
  const std::shared_ptr<GoalHandle> & goal_handle,
  const std::shared_ptr<DrawBoundaries::Result> & result)
{
  if (!goal_handle || !rclcpp::ok()) {
    return;
  }
  if (!goal_handle->is_active()) {
    return;
  }
  try {
    goal_handle->canceled(result);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "cancel ignore (shutdown): %s", e.what());
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "cancel ignore (unknown error)");
  }
}

void TurtleBoundaries::stop_turtle()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleBoundaries>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
