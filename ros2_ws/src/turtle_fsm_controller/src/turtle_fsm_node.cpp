#include "turtle_fsm_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

TurtleFsmNode::TurtleFsmNode(): Node("turtle_fsm_node")
{
  // -----------------------------
  // 1) Declaration des parametres
  // -----------------------------
  declare_parameter<double>("forward_speed", 1.2);
  declare_parameter<double>("turn_speed", 1.6);
  declare_parameter<double>("reverse_speed", 1.0);
  declare_parameter<double>("boundary_margin", 0.6);
  declare_parameter<double>("boundary_turn_duration", 1.2);
  declare_parameter<double>("boundary_escape_duration", 0.6);
  declare_parameter<double>("boundary_heading_tolerance", 0.35);
  declare_parameter<double>("red_reverse_duration", 1.0);
  declare_parameter<double>("red_turn_duration", 1.2);
  declare_parameter<double>("red_boost_duration", 0.8);
  declare_parameter<double>("red_event_cooldown", 0.8);
  declare_parameter<int>("red_threshold", 200);
  declare_parameter<int>("green_max_for_red", 100);
  declare_parameter<int>("blue_max_for_red", 100);

  // ------------------------
  // 2) Lecture des parametres
  // ------------------------
  // Les paramètres sont lus une fois au démarrage et stockés dans des variables membres.
  // Paramètres de comment doit réagir la tortue dans son environnement (vitesse, durées, seuils de couleur).
  // Sinon on prend les valeurs par défaut déjà initialisées dans la classe.
  forward_speed_ = get_parameter("forward_speed").as_double();
  turn_speed_ = get_parameter("turn_speed").as_double();
  reverse_speed_ = get_parameter("reverse_speed").as_double();
  boundary_margin_ = get_parameter("boundary_margin").as_double();
  boundary_turn_duration_ = get_parameter("boundary_turn_duration").as_double();
  boundary_escape_duration_ = get_parameter("boundary_escape_duration").as_double();
  boundary_heading_tolerance_ = get_parameter("boundary_heading_tolerance").as_double();
  red_reverse_duration_ = get_parameter("red_reverse_duration").as_double();
  red_turn_duration_ = get_parameter("red_turn_duration").as_double();
  red_boost_duration_ = get_parameter("red_boost_duration").as_double();
  red_event_cooldown_ = get_parameter("red_event_cooldown").as_double();

  // Rouge que l'on veut détecter
  red_threshold_ = get_parameter("red_threshold").as_int();
  green_max_for_red_ = get_parameter("green_max_for_red").as_int();
  blue_max_for_red_ = get_parameter("blue_max_for_red").as_int();

  // ------------------------
  // 3) Interfaces ROS2
  // ------------------------
  // Publisher de commande vitesse vers turtle1.
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

  // Subscriber de pose: permet de connaitre la position/heading courants.
  pose_sub_ = create_subscription<turtlesim::msg::Pose>(
    "/turtle1/pose", 
    10,
    std::bind(&TurtleFsmNode::on_pose, this, std::placeholders::_1)
  );

  // Subscriber du capteur de couleur sous la tortue.
  color_sub_ = create_subscription<turtlesim::msg::Color>(
    "/turtle1/color_sensor", 
    10,
    std::bind(&TurtleFsmNode::on_color, this, std::placeholders::_1)
  );

  // Timer périodique: cadence de décision de la FSM.
  // Permet d'exécuter control_loop() à une fréquence régulière :
  // - crée un minuteur timer_
  // - qui appelle control_loop() toutes les 50ms
  // - sur this (le noeud courant)
  timer_ = create_wall_timer(50ms, std::bind(&TurtleFsmNode::control_loop, this));

  // Etat initial explicite, pour clarifier les logs dès le démarrage.
  state_ = State::WAITING_DATA;
  state_start_time_ = now();
  last_red_event_time_ = now();
  // On écrit un message dans les logs de ROS2!
  RCLCPP_INFO(get_logger(), "FSM init: state=%s", state_to_cstr(state_));
}

void TurtleFsmNode::on_pose(const turtlesim::msg::Pose::SharedPtr msg)
{
  // Ce callback ne prend pas de décision, il met juste à jour la mesure.
  // La logique de décision reste centralisée dans control_loop().
  pose_ = *msg;
  has_pose_ = true;
}

void TurtleFsmNode::on_color(const turtlesim::msg::Color::SharedPtr msg)
{
  // Ici, on met à jour la dernière couleur mesurée.
  // Cette donnée sera utilisée au prochain tick de control_loop().
  color_ = *msg;
  has_color_ = true;

  // Mise à jour du flag : j'ai vu du rouge ou non !
  // - R doit être assez grand
  // - G et B doivent rester faibles
  const bool is_red =
    (msg->r >= red_threshold_) &&
    (msg->g <= green_max_for_red_) &&
    (msg->b <= blue_max_for_red_);

  // - Si on était NON rouge et qu'on devient rouge, alors on déclenche l'événement.
  // - Sinon (on était déjà rouge), on ne redéclenche pas.
  // Si cet événement arrive, ça veut dire: "on vient d'entrer sur une zone rouge".
  const double since_last_red_event = (now() - last_red_event_time_).seconds();
  if (is_red && !on_red_now_ && since_last_red_event >= red_event_cooldown_) {
    red_event_pending_ = true;
    last_red_event_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "Evenement couleur: rouge detecte (r=%u g=%u b=%u)",
      msg->r, msg->g, msg->b);
  }

  // On mémorise l'état actuel pour pouvoir détecter le prochain front montant.
  on_red_now_ = is_red;
}

void TurtleFsmNode::control_loop()
{
  // 1) Préconditions.
  // Là, on attend que la pose ET la couleur aient été reçues au moins une fois.
  // Si ce n'est pas encore le cas: on arrête la tortue et on ne fait rien d'autre.
  // Sinon: on continue la logique FSM.
  if (!has_pose_ || !has_color_) {
    // Si on est là, ça veut dire "données incomplètes", donc sécurité maximale.
    publish_stop();
    return;
  }

  // 2) Sortie de WAITING_DATA.
  // Si les données sont prêtes, on démarre la navigation.
  if (state_ == State::WAITING_DATA) {
    set_state(State::FORWARD, "premières données reçues");
  }

  // 3) Priorité à l'événement rouge.
  // Si un événement rouge est en attente, on interrompt le comportement courant
  // (FORWARD/BOUNDARY_TURN) pour exécuter la séquence spéciale rouge.
  // Sinon, on reste dans la logique normale de l'état courant.
  if (red_event_pending_ &&
      (state_ == State::FORWARD ||
       state_ == State::BOUNDARY_TURN ||
       state_ == State::BOUNDARY_ESCAPE))
  {
    red_event_pending_ = false;
    set_state(State::RED_REVERSE, "rouge détecté -> recul");
  }

  // Sécurité "mur rouge":
  // si on est déjà sur rouge en navigation normale, on déclenche aussi RED_REVERSE.
  // Cela évite de traverser la bande rouge lorsque le front montant a été raté.
  const double since_last_red_event = (now() - last_red_event_time_).seconds();
  if (on_red_now_ &&
      since_last_red_event >= red_event_cooldown_ &&
      (state_ == State::FORWARD || state_ == State::BOUNDARY_ESCAPE))
  {
    last_red_event_time_ = now();
    set_state(State::RED_REVERSE, "rouge sous la tortue -> recul");
  }

  // 4) Action associée à l'état courant + transitions temporisées.
  switch (state_) {
    case State::WAITING_DATA:
      // Cas défensif: en théorie on en sort juste avant, mais on garde ce stop.
      publish_stop();
      break;

    case State::FORWARD:
      // Là, on avance.
      // Si on approche d'un bord -> on tourne pour éviter la sortie.
      // Sinon -> on continue à avancer.
      if (is_near_boundary()) {
        set_state(State::BOUNDARY_TURN, "proche bord -> rotation");
        publish_stop();
        break;
      }
      publish_twist(forward_speed_, 0.0);
      break;

    case State::BOUNDARY_TURN:
      // Là, on est en évitement de bord.
      // Si la durée de rotation est atteinte -> on passe en BOUNDARY_ESCAPE.
      // Cela évite la boucle "FORWARD -> BOUNDARY_TURN" immédiate.
      // Sinon -> on continue à tourner.
      if (elapsed_in_state() >= boundary_turn_duration_) {
        set_state(State::BOUNDARY_ESCAPE, "rotation bord terminee -> degagement");
        publish_stop();
        break;
      }
      publish_twist(0.0, turn_speed_);
      break;

    case State::BOUNDARY_ESCAPE:
      // Avance forcée pendant un court instant pour vraiment quitter la zone de bord.
      // Si la durée de dégagement est terminée -> retour en FORWARD.
      // Sinon -> on continue d'avancer.
      if (!is_near_boundary() || elapsed_in_state() >= boundary_escape_duration_) {
        set_state(State::FORWARD, "degagement bord termine");
        publish_stop();
        break;
      }
      {
        // Pendant le dégagement, on corrige aussi l'orientation vers le centre.
        // Cela évite de glisser le long du mur en formant des mini-triangles.
        const double error = heading_error_to_center();
        const double angular_cmd = std::clamp(2.0 * error, -turn_speed_, turn_speed_);
        // Si l'orientation est mauvaise, on tourne d'abord sur place.
        // Sinon, on avance en gardant une correction angulaire.
        if (std::abs(error) > boundary_heading_tolerance_) {
          publish_twist(0.0, angular_cmd);
        } else {
          publish_twist(0.8 * forward_speed_, angular_cmd);
        }
      }
      break;

    case State::RED_REVERSE:
      // Si rouge détecté, première réaction: reculer.
      // Si la durée de recul est atteinte -> on passe à RED_TURN.
      // Sinon -> on continue à reculer.
      if (elapsed_in_state() >= red_reverse_duration_) {
        set_state(State::RED_TURN, "recul terminé -> rotation");
        publish_stop();
        break;
      }
      publish_twist(-reverse_speed_, 0.0);
      break;

    case State::RED_TURN:
      // Deuxième réaction: tourner pour changer rapidement de direction.
      // Si la durée est terminée -> on passe au boost.
      // Sinon -> on continue à tourner.
      if (elapsed_in_state() >= red_turn_duration_) {
        set_state(State::RED_BOOST, "rotation rouge terminée -> boost");
        publish_stop();
        break;
      }
      publish_twist(0.0, turn_speed_);
      break;

    case State::RED_BOOST:
      // Troisième réaction: boost temporaire (vitesse x2).
      // Si le boost est fini -> retour au mode nominal FORWARD.
      // Sinon -> on garde l'accélération.
      // Sécurité: si on est proche d'un bord, on coupe le boost et on repasse
      // en évitement de bord pour ne pas taper la paroi.
      if (is_near_boundary()) {
        set_state(State::BOUNDARY_TURN, "boost interrompu: proche bord");
        publish_stop();
        break;
      }
      // Si on est encore sur rouge après la séquence, on ne booste pas,
      // on repart en recul pour vraiment sortir de la zone rouge.
      if (on_red_now_) {
        set_state(State::RED_REVERSE, "boost interrompu: toujours sur rouge");
        publish_stop();
        break;
      }
      if (elapsed_in_state() >= red_boost_duration_) {
        set_state(State::FORWARD, "boost terminé");
        publish_stop();
        break;
      }
      publish_twist(forward_speed_ * 2.0, 0.0);
      break;
  }
}

bool TurtleFsmNode::is_near_boundary() const
{
  // Dans turtlesim, la zone utile est environ [0, 11] sur x/y.
  // La marge boundary_margin_ crée une "zone d'alerte" proche des bords.
  return pose_.x < boundary_margin_ || pose_.x > (11.0 - boundary_margin_) ||
         pose_.y < boundary_margin_ || pose_.y > (11.0 - boundary_margin_);
}

double TurtleFsmNode::heading_error_to_center() const
{
  // Centre de la zone turtlesim.
  constexpr double kCenter = 5.5;
  const double desired_theta = std::atan2(kCenter - pose_.y, kCenter - pose_.x);
  return normalize_angle(desired_theta - pose_.theta);
}

void TurtleFsmNode::set_state(State new_state, const std::string & reason)
{
  // Aucun effet si on redemande le même état.
  if (state_ == new_state) {
    return;
  }

  // Log de transition pour debug/trace TP.
  RCLCPP_INFO(
    get_logger(), "Transition FSM: %s -> %s (%s)",
    state_to_cstr(state_), state_to_cstr(new_state), reason.c_str());

  // Mémorise le nouvel état + l'instant d'entrée.
  // Cet horodatage sert aux transitions temporisées.
  state_ = new_state;
  state_start_time_ = now();
}

double TurtleFsmNode::elapsed_in_state() const
{
  // Renvoie le temps ecoule depuis state_start_time_.
  // Utilise pour les etats bases sur une duree.
  return (now() - state_start_time_).seconds();
}

void TurtleFsmNode::publish_stop()
{
  // Facteur commun: évite de dupliquer les affectations Twist à zéro.
  publish_twist(0.0, 0.0);
}

void TurtleFsmNode::publish_twist(double linear_x, double angular_z)
{
  // Une commande Twist contient plusieurs composantes.
  // Ici on n'utilise que linear.x (avance/recul) et angular.z (rotation plane).
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  cmd_vel_pub_->publish(msg);
}

const char * TurtleFsmNode::state_to_cstr(State s)
{
  // Conversion pour avoir des logs lisibles.
  switch (s) {
    case State::WAITING_DATA:
      return "WAITING_DATA";
    case State::FORWARD:
      return "FORWARD";
    case State::BOUNDARY_TURN:
      return "BOUNDARY_TURN";
    case State::BOUNDARY_ESCAPE:
      return "BOUNDARY_ESCAPE";
    case State::RED_REVERSE:
      return "RED_REVERSE";
    case State::RED_TURN:
      return "RED_TURN";
    case State::RED_BOOST:
      return "RED_BOOST";
    default:
      return "UNKNOWN";
  }
}

double TurtleFsmNode::normalize_angle(double angle)
{
  // Variante classique: replie l'angle dans [-pi, pi].
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

int main(int argc, char ** argv)
{
  // Point d'entrée standard ROS2.
  // 1) init middleware
  // 2) création du noeud
  // 3) spin (boucle callbacks)
  // 4) shutdown propre
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleFsmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown(); // Lance la boucle d'exécution ROS2, qui gère les callbacks (pose, couleur, timer).
  return 0;
}
