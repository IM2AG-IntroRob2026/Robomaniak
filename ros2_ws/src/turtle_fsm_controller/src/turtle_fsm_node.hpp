#pragma once

#include <cmath>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/color.hpp>
#include <turtlesim/msg/pose.hpp>

/**
 * @brief Noeud ROS2 qui pilote turtle1 avec une machine a etats finis (FSM).
 *
 * Ce noeud combine :
 * - un comportement de navigation de base (avancer + eviter les bords),
 * - des reactions evenementielles basées sur la couleur rouge detectee au sol.
 *
 * La FSM publie des commandes de vitesse sur /turtle1/cmd_vel
 * et lit la pose et la couleur depuis turtlesim.
 */
class TurtleFsmNode : public rclcpp::Node
{
public:
  /**
   * @brief Construit le noeud, declare/charge les parametres et initialise ROS2.
   *
   * Initialisations effectées :
   * - déclaration des paramètres de vitesse, durée et seuils couleur,
   * - création des publisher/subscribers,
   * - création du timer de contrôle périodique,
   * - placement de la FSM dans l'état WAITING_DATA.
   */
  TurtleFsmNode();

private:
  /**
   * @brief Etats de la machine a etats.
   */
  enum class State
  {
    WAITING_DATA,   // Attente des premiers messages pose + couleur.
    FORWARD,        // Avance en ligne droite.
    BOUNDARY_TURN,  // Rotation pour éviter une sortie de zone.
    BOUNDARY_ESCAPE, // Avance forcée pour sortir de la marge de bord.
    RED_REVERSE,    // Recul suite à la détection de rouge.
    RED_TURN,       // Rotation juste après le recul.
    RED_BOOST       // Avance temporaire à vitesse double.
  };

  /**
   * @brief Callback de pose.
   * @param msg Dernière pose de turtle1.
   *
   * Met à jour l'état interne de localisation (x, y, theta),
   * et active le flag indiquant que la pose est disponible.
   */
  void on_pose(const turtlesim::msg::Pose::SharedPtr msg);

  /**
   * @brief Callback du capteur de couleur.
   * @param msg Couleur detectee sous turtle1.
   *
   * Détecte un événement "rouge" selon des seuils paramétrés
   * et ne déclenche l'événement que si le rouge est assez grand et que le vert/bleu sont assez faibles.
   */
  void on_color(const turtlesim::msg::Color::SharedPtr msg);

  /**
   * @brief Boucle de contrôle périodique de la FSM.
   *
   * Cette méthode:
   * - vérifie les préconditions (pose/couleur reçues),
   * - traite les événements prioritaires (rouge),
   * - applique les transitions d'état,
   * - publie la commande de vitesse associée à l'état courant.
   */
  void control_loop();

  /**
   * @brief Vérifie si la tortue est proche d'un bord de la fenêtre turtlesim.
   * @return true si la tortue est dans la marge de sécurité, false sinon.
   */
  bool is_near_boundary() const;

  /**
   * @brief Calcule l'angle entre l'orientation actuelle et la direction du centre.
   * @return Erreur angulaire normalisée dans [-pi, pi].
   */
  double heading_error_to_center() const;

  /**
   * @brief Change l'état courant et mémorise l'horodatage d'entrée.
   * @param new_state Etat cible.
   * @param reason Texte explicatif pour les logs.
   */
  void set_state(State new_state, const std::string & reason);

  /**
   * @brief Temps écoulé dans l'état courant.
   * @return Durée en secondes depuis l'entrée dans l'état.
   */
  double elapsed_in_state() const;

  /**
   * @brief Publie une commande d'arrêt.
   *
   * Equivalent à publier un Twist nul (vitesse linéaire et angulaire à zéro).
   */
  void publish_stop();

  /**
   * @brief Publie une commande de vitesse.
   * @param linear_x Vitesse linéaire suivant x (m/s).
   * @param angular_z Vitesse angulaire autour de z (rad/s).
   */
  void publish_twist(double linear_x, double angular_z);

  /**
   * @brief Conversion d'un état en chaîne lisible pour les logs.
   * @param s État à convertir.
   * @return Nom de l'état sous forme C string.
   */
  static const char * state_to_cstr(State s);

  /**
   * @brief Normalise un angle dans l'intervalle [-pi, pi].
   * @param angle Angle en radians.
   * @return Angle normalisé.
   */
  static double normalize_angle(double angle);

  // Communication ROS2.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<turtlesim::msg::Color>::SharedPtr color_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Dernières mesures reçues.
  turtlesim::msg::Pose pose_{};
  turtlesim::msg::Color color_{};

  // Flags de disponibilité des données et détection d'événement couleur.
  bool has_pose_ = false;    // Indique si une mesure de pose a été reçue au moins une fois.
  bool has_color_ = false;   // Indique si une mesure de couleur a été reçue au moins une fois.
  bool on_red_now_ = false;  // Indique si la couleur actuelle est considérée comme "rouge" selon les seuils.
  bool red_event_pending_ = false; // Indique qu'un événement "rouge" a été détecté et doit être traité dans la boucle de contrôle.

  // Etat de la FSM.
  State state_ = State::WAITING_DATA;  // Etat courant de la machine à états finis.
  rclcpp::Time state_start_time_{0, 0, RCL_ROS_TIME};

  // Paramètres de mouvement et de logique FSM.
  double forward_speed_ = 1.2;          // vitesse d'avance normale
  double turn_speed_ = 1.6;             // vitesse de rotation
  double reverse_speed_ = 1.0;          // vitesse de recul
  double boundary_margin_ = 0.6;        // marge de sécurité pour éviter le bord
  double boundary_turn_duration_ = 1.2; // durée de rotation en cas de proximité du bord
  double boundary_escape_duration_ = 0.6; // durée d'avance forcée après rotation de bord
  double boundary_heading_tolerance_ = 0.35; // tolérance d'orientation vers le centre (rad)
  double red_reverse_duration_ = 1.0;   // durée de recul en cas de détection de rouge
  double red_turn_duration_ = 1.2;      // durée de rotation en cas de détection de rouge
  double red_boost_duration_ = 0.8;     // durée de boost en cas de détection de rouge
  double red_event_cooldown_ = 0.8;     // délai minimal entre deux événements rouge

  // Seuils de detection "rouge".
  int red_threshold_ = 200;          // Seuil de détection du rouge
  int green_max_for_red_ = 100;      // Valeur maximale de vert pour considérer comme rouge
  int blue_max_for_red_ = 100;       // Valeur maximale de bleu pour considérer comme rouge

  // Horodatage du dernier événement rouge effectivement déclenché.
  rclcpp::Time last_red_event_time_{0, 0, RCL_ROS_TIME};
};
