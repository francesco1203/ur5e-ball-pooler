// ============================================================
//  game_engine.cpp
//  Nodo ROS2 che seleziona la buca ottimale e calcola i parametri di tiro.
// ============================================================

#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "interfaces_pkg/msg/shot_params.hpp"

using namespace std::chrono_literals;

class GameEngine : public rclcpp::Node
{
    public:
        /* Alias */
        using ShotParamsMsg = interfaces_pkg::msg::ShotParams;   
        using ShotParamsPublisher = rclcpp::Publisher<ShotParamsMsg>::SharedPtr;

        /* Costruttore */
        GameEngine() : Node("game_engine")
        {

            //iperaparametri
            this->declare_parameter<double>("velocity_factor", 1.2);
            velocity_factor_ = this->get_parameter("velocity_factor").as_double();

            // Inizializzazione Listener TF2
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Publisher del messaggio
            publisher_ = this->create_publisher<ShotParamsMsg>(SHOT_PARAMS_TOPIC, 10);

            // Lista delle 6 buche sul tavolo
            pocket_frames_ = {
                HOLE_TOP_RIGHT_FRAME,
                HOLE_TOP_LEFT_FRAME,
                HOLE_MID_RIGHT_FRAME,
                HOLE_MID_LEFT_FRAME,
                HOLE_BOTTOM_RIGHT_FRAME,
                HOLE_BOTTOM_LEFT_FRAME
            };

            // Timer per il calcolo e la pubblicazione ogni 2 secondi
            timer_ = this->create_wall_timer(
                2000ms, std::bind(&GameEngine::publish_params, this));

            RCLCPP_INFO(this->get_logger(), "Game Engine avviato. Valutazione buche e sponde attiva.");
        }

    private:
        /* ATTRIBUTI */
        ShotParamsPublisher publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::vector<std::string> pocket_frames_;


        double velocity_factor_;


        // Utility per normalizzare l'angolo tra -PI e +PI
        double normalize_angle(double angle)
        {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

        // Callback di pubblicazione
        void publish_params()
        {
            geometry_msgs::msg::TransformStamped tf_white, tf_red;

            // --- 1. LETTURA POSE PALLINE TRAMITE TF2 ---
            try {
                tf_white = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, WHITE_SOLID_BALL_FRAME, tf2::TimePointZero);
                tf_red   = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, RED_SOLID_BALL_FRAME,   tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "In attesa delle TF per le palline: %s", ex.what());
                return;
            }

            double w_x = tf_white.transform.translation.x;
            double w_y = tf_white.transform.translation.y;

            double t_x = tf_red.transform.translation.x;
            double t_y = tf_red.transform.translation.y;

            // Limiti geometrici del campo da biliardo (sponde)
            double half_field_length = POOL_TABLE_FIELD_LENGTH / 2.0;
            double half_field_width  = POOL_TABLE_FIELD_WIDTH / 2.0;


            // Variabili per tracciare il tiro migliore
            std::string best_pocket = "";
            double best_cost = std::numeric_limits<double>::max();
            double best_shot_velocity = 0.0;
            double best_direction_deg = 0.0;
            bool valid_shot_found = false;

            // --- 2. VALUTAZIONE DI OGNI BUCA ---
            for (const auto& pocket_frame : pocket_frames_)
            {
                geometry_msgs::msg::TransformStamped tf_pocket;
                try {
                    tf_pocket = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, pocket_frame, tf2::TimePointZero);
                } catch (const tf2::TransformException & ex) {
                    continue; // Salta la buca se la TF non è disponibile
                }

                double pocket_x = tf_pocket.transform.translation.x;
                double pocket_y = tf_pocket.transform.translation.y;

                // Geometria Rossa -> Buca
                double dx_pocket = pocket_x - t_x;
                double dy_pocket = pocket_y - t_y;

                double pocket_distance = std::hypot(dx_pocket, dy_pocket);      //distanza rossa buca

                double pocket_angle_rad = std::atan2(dy_pocket, dx_pocket);

                // Calcolo posizione della "Ghost Ball" (Punto di contatto sulla Rossa)
                double ball_diameter = BALL_RADIUS * 2.0;
                double contact_x = t_x - ball_diameter * std::cos(pocket_angle_rad);
                double contact_y = t_y - ball_diameter * std::sin(pocket_angle_rad);


                // --- CONTROLLO SPONDE (Cushions) ---
                // Se il punto di contatto esce dal campo o si incastra nella sponda, il tiro è impossibile
                double margin = BALL_RADIUS; // Margine di sicurezza dalle sponde
                if (std::abs(contact_x) >= (half_field_length - margin) || 
                    std::abs(contact_y) >= (half_field_width - margin)) 
                {
                    continue; // Tiro scartato: la Ghost Ball interseca la sponda
                }

                // Vettore Bianca -> Ghost Ball
                double dx_cue = contact_x - w_x;
                double dy_cue = contact_y - w_y;
                double cue_distance = std::hypot(dx_cue, dy_cue);
                double cue_angle_rad = std::atan2(dy_cue, dx_cue);

                // Angolo di taglio (Cut Angle)
                double cut_angle_rad = normalize_angle(cue_angle_rad - pocket_angle_rad);
                double abs_cut_angle = std::abs(cut_angle_rad);

                // --- FILTRO ANGOLO DI TAGLIO ---
                // Se l'angolo di taglio è >= 85 gradi (1.48 rad), il tiro è fisicamente impossibile
                constexpr double MAX_CUT_ANGLE_RAD = 85.0 * (M_PI / 180.0);
                if (abs_cut_angle >= MAX_CUT_ANGLE_RAD) {
                    continue; 
                }

                // --- FUNZIONE DI COSTO ---
                // Pesi per la valutazione della difficoltà
                constexpr double WEIGHT_CUT_ANGLE = 3.5; // Alta penalità agli angoli stretti
                constexpr double WEIGHT_POCKET_DIST = 1.0; // Penalità alla distanza Rossa-Buca
                constexpr double WEIGHT_CUE_DIST = 0.5;   // Penalità alla distanza Bianca-Rossa

                // Calcolo della penalità per prossimità alle sponde (se la pallina rossa è quasi attaccata alla sponda)
                double dist_to_rail_x = half_field_length - std::abs(t_x);
                double dist_to_rail_y = half_field_width - std::abs(t_y);
                double min_rail_dist = std::min(dist_to_rail_x, dist_to_rail_y);
                double rail_penalty = (min_rail_dist < 2.0 * BALL_RADIUS) ? 2.0 : 0.0;

                // Costo totale
                double total_cost = (WEIGHT_CUT_ANGLE * abs_cut_angle) + 
                                   (WEIGHT_POCKET_DIST * pocket_distance) + 
                                   (WEIGHT_CUE_DIST * cue_distance) + 
                                   rail_penalty;

                // --- 3. FISICA E CALCOLO VELOCITÀ PER QUESTA BUCA ---
                double cos_alpha = std::cos(cut_angle_rad);
                if (std::abs(cos_alpha) < 0.001) continue;


                double v2f = std::sqrt(2.0 * CLOTH_SLIDING_FRICTION * GRAVITY * pocket_distance);
                double v1i_impact = (v2f / cos_alpha);
                double v_white_start = std::sqrt(std::pow(v1i_impact, 2) + 2.0 * CLOTH_SLIDING_FRICTION * GRAVITY * cue_distance);

                // Applicazione dello scaling di sicurezza per MoveIt
                double shot_velocity = velocity_factor_ * v_white_start;            //aumentato di un coefficiente velocity_factor
                double direction_deg = normalize_angle(cue_angle_rad + M_PI) * (180.0 / M_PI);

                // Aggiornamento del tiro ottimale se questo ha il costo minore
                if (total_cost < best_cost) {
                    best_cost = total_cost;
                    best_pocket = pocket_frame;
                    best_shot_velocity = shot_velocity;
                    best_direction_deg = direction_deg;
                    valid_shot_found = true;
                }
            }

            // --- 4. PUBBLICAZIONE DEL TIRO VINCENTE ---
            if (valid_shot_found) {
                auto msg = ShotParamsMsg();
                msg.direction_angle_deg = best_direction_deg;
                msg.impact_shot_velocity = best_shot_velocity;
                
                publisher_->publish(msg);

                RCLCPP_INFO(this->get_logger(), 
                    "Buca scelta: [%s] (Costo: %.2f) | Vel: %.3f m/s, Dir: %.2f deg", 
                    best_pocket.c_str(), best_cost, best_shot_velocity, best_direction_deg);
            } else {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Nessuna buca raggiungibile! Tutti i tiri violano i limiti di angolo o sponda.");
            }
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameEngine>());
    rclcpp::shutdown();
    return 0;
}
