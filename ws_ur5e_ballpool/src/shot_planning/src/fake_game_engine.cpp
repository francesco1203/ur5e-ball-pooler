// ============================================================
//  fake_game_engine.cpp
//  Nodo ROS2 che calcola e pubblica i parametri di tiro leggendo le TF.
// ============================================================

#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "shared_headers_pkg/ros2_architecture.hpp"     // Header per topic name
#include "interfaces_pkg/msg/shot_params.hpp"           // Custom message

using namespace std::chrono_literals;

class FakeGameEngine : public rclcpp::Node
{
    public:
        /* Alias */
        using ShotParamsMsg = interfaces_pkg::msg::ShotParams;   
        using ShotParamsPublisher = rclcpp::Publisher<ShotParamsMsg>::SharedPtr;

        /* Costruttore */
        FakeGameEngine() : Node("fake_game_engine")
        {
            // 1. Parametri fisici ed ambientali
            this->declare_parameter<double>("cloth_sliding_friction", 0.2); // u_s
            this->declare_parameter<double>("gravity", 9.81);               // g (m/s^2)
            this->declare_parameter<double>("ball_radius", 0.015);          // Raggio palla (m)
            this->declare_parameter<double>("cue_ball_mass", 0.10);         // m1 (kg) - Bianca
            this->declare_parameter<double>("target_ball_mass", 0.15);      // m2 (kg) - Rossa

            // 2. Inizializzazione Listener TF2
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // 3. Publisher del messaggio
            publisher_ = this->create_publisher<ShotParamsMsg>(SHOT_PARAMS_TOPIC, 10);

            // 4. Timer per il calcolo e la pubblicazione ogni 2 secondi
            timer_ = this->create_wall_timer(
                2000ms, std::bind(&FakeGameEngine::publish_params, this));

            RCLCPP_INFO(this->get_logger(), "Fake Engine avviato. In attesa delle TF per il calcolo...");
        }

    private:
        /* ATTRIBUTI */
        ShotParamsPublisher publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        // Callback di pubblicazione
        void publish_params()
        {
            geometry_msgs::msg::TransformStamped tf_white, tf_red, tf_pocket;

            // --- 1. LETTURA POSE TRAMITE TF2 ---
            try {
                tf_white = tf_buffer_->lookupTransform("billiard_center_field", "white_solid_ball", tf2::TimePointZero);
                tf_red = tf_buffer_->lookupTransform("billiard_center_field", "red_solid_ball", tf2::TimePointZero);
                tf_pocket = tf_buffer_->lookupTransform("billiard_center_field", "hole_top_left", tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "In attesa che le TF siano disponibili: %s", ex.what());
                return;
            }

            double w_x = tf_white.transform.translation.x;
            double w_y = tf_white.transform.translation.y;

            double t_x = tf_red.transform.translation.x;
            double t_y = tf_red.transform.translation.y;

            double pocket_x = tf_pocket.transform.translation.x;
            double pocket_y = tf_pocket.transform.translation.y;

            // Lettura parametri fisici
            double u_s = this->get_parameter("cloth_sliding_friction").as_double();
            double g = this->get_parameter("gravity").as_double();
            double b_rad = this->get_parameter("ball_radius").as_double();
            double m1 = this->get_parameter("cue_ball_mass").as_double();
            double m2 = this->get_parameter("target_ball_mass").as_double();

            // --- 2. GEOMETRIA: Vettori e Angoli ---
            double dx_pocket = pocket_x - t_x;
            double dy_pocket = pocket_y - t_y;
            double pocket_distance = std::hypot(dx_pocket, dy_pocket);
            double pocket_angle_rad = std::atan2(dy_pocket, dx_pocket);

            // Punto di contatto "Ghost Ball"
            double ball_diameter = b_rad * 2.0;
            double contact_x = t_x - ball_diameter * std::cos(pocket_angle_rad);
            double contact_y = t_y - ball_diameter * std::sin(pocket_angle_rad);

            // Vettore Bianca -> Punto di contatto
            double dx_cue = contact_x - w_x;
            double dy_cue = contact_y - w_y;
            double cue_distance = std::hypot(dx_cue, dy_cue);
            double cue_angle_rad = std::atan2(dy_cue, dx_cue);

            // Angolo di taglio
            double cut_angle_rad = cue_angle_rad - pocket_angle_rad;

            // --- 3. FISICA: Calcolo Velocità e Direzione ---
            double v2f = std::sqrt(2.0 * u_s * g * pocket_distance);
            double cos_alpha = std::cos(cut_angle_rad);

            // Gestione del limite matematico se il coseno tende a zero
            if (std::abs(cos_alpha) < 0.001) {
                return;
            }

            double mass_factor = (m1 + m2) / (2.0 * m1);
            double v1i_impact = mass_factor * (v2f / cos_alpha);

            double v_white_start = std::sqrt(std::pow(v1i_impact, 2) + 2.0 * u_s * g * cue_distance);
            double shot_velocity = 1.5 * v_white_start;
            double direction_deg = cue_angle_rad * (180.0 / M_PI);

            // --- 4. POPOLAZIONE E PUBBLICAZIONE MESSAGGIO ---
            auto msg = ShotParamsMsg();
            msg.direction_angle_deg = direction_deg;
            msg.impact_shot_velocity = shot_velocity;
            
            publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), 
                "Tiro Pubblicato! Vel: %.3f m/s, Dir: %.2f deg", shot_velocity, direction_deg);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeGameEngine>());
    rclcpp::shutdown();
    return 0;
}