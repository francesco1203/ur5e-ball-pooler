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
#include <tf2/LinearMath/Vector3.h>

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "interfaces_pkg/msg/shot_params.hpp"

using namespace std::chrono_literals;

class GameEngine : public rclcpp::Node
{
    public:
        using ShotParamsMsg = interfaces_pkg::msg::ShotParams;   
        using ShotParamsPublisher = rclcpp::Publisher<ShotParamsMsg>::SharedPtr;

        GameEngine() : Node("game_engine")
        {
            this->declare_parameter<double>("velocity_factor", 1.2);
            
            // Offset di yaw del tip. 
            // Se asse blu (Z) del tip va verso destra e rosso (X) verso il basso, 
            // questo parametro compensa l'orientamento per MoveIt.
            this->declare_parameter<double>("tip_yaw_offset_deg", 180.0); 

            velocity_factor_ = this->get_parameter("velocity_factor").as_double();
            tip_yaw_offset_deg_ = this->get_parameter("tip_yaw_offset_deg").as_double();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            publisher_ = this->create_publisher<ShotParamsMsg>(SHOT_PARAMS_TOPIC, 10);

            pocket_frames_ = {
                HOLE_TOP_RIGHT_FRAME,
                HOLE_TOP_LEFT_FRAME,
                HOLE_MID_RIGHT_FRAME,
                HOLE_MID_LEFT_FRAME,
                HOLE_BOTTOM_RIGHT_FRAME,
                HOLE_BOTTOM_LEFT_FRAME
            };

            timer_ = this->create_wall_timer(
                2000ms, std::bind(&GameEngine::publish_params, this));

            RCLCPP_INFO(this->get_logger(), "Game Engine avviato. Valutazione basata su Vettori.");
        }

    private:
        ShotParamsPublisher publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::vector<std::string> pocket_frames_;

        double velocity_factor_;
        double tip_yaw_offset_deg_;

        double normalize_angle(double angle)
        {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

        void publish_params()
        {
            geometry_msgs::msg::TransformStamped tf_white, tf_red;

            try {
                tf_white = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, WHITE_SOLID_BALL_FRAME, tf2::TimePointZero);
                tf_red   = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, RED_SOLID_BALL_FRAME,   tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                return;
            }

            // Trasformazione in Vettori (Elimina errori umani su coordinate X, Y separate)
            tf2::Vector3 pos_white(tf_white.transform.translation.x, tf_white.transform.translation.y, 0.0);
            tf2::Vector3 pos_red(tf_red.transform.translation.x, tf_red.transform.translation.y, 0.0);

            double half_field_length = POOL_TABLE_FIELD_LENGTH / 2.0;
            double half_field_width  = POOL_TABLE_FIELD_WIDTH / 2.0;
            double ball_diameter = BALL_RADIUS * 2.0;

            std::string best_pocket = "";
            double best_cost = std::numeric_limits<double>::max();
            double best_shot_velocity = 0.0;
            double best_direction_deg = 0.0;
            bool valid_shot_found = false;

            for (const auto& pocket_frame : pocket_frames_)
            {
                geometry_msgs::msg::TransformStamped tf_pocket;
                try {
                    tf_pocket = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, pocket_frame, tf2::TimePointZero);
                } catch (const tf2::TransformException & ex) {
                    continue; 
                }

                tf2::Vector3 pos_pocket(tf_pocket.transform.translation.x, tf_pocket.transform.translation.y, 0.0);

                // --- 1. VETTORE DIREZIONE ROSSA -> BUCA ---
                tf2::Vector3 vec_red_to_pocket = pos_pocket - pos_red;
                double pocket_distance = vec_red_to_pocket.length();
                if (pocket_distance < 0.001) continue;

                // Vettore normalizzato (Lunghezza 1) che punta verso la buca
                tf2::Vector3 dir_pocket = vec_red_to_pocket.normalized();

                // --- 2. POSIZIONE INFALLIBILE DELLA GHOST BALL ---
                // Si calcola arrettrando dalla rossa lungo la linea *opposta* alla buca.
                // Usando i vettori, questa operazione funziona perfettamente in qualsiasi quadrante.
                tf2::Vector3 pos_ghost = pos_red - (dir_pocket * ball_diameter);

                // Controllo sponde per la Ghost Ball
                double margin = BALL_RADIUS; 
                if (std::abs(pos_ghost.x()) >= (half_field_length - margin) || 
                    std::abs(pos_ghost.y()) >= (half_field_width - margin)) 
                {
                    continue; 
                }

                // --- 3. VETTORE BIANCA -> GHOST BALL (Traiettoria della stecca) ---
                tf2::Vector3 vec_white_to_ghost = pos_ghost - pos_white;
                double cue_distance = vec_white_to_ghost.length();
                if (cue_distance < 0.001) continue;

                tf2::Vector3 dir_shot = vec_white_to_ghost.normalized();

                // --- 4. ANGOLO DI TAGLIO TRAMITE PRODOTTO SCALARE ---
                // Il prodotto scalare trova il coseno dell'angolo tra la traiettoria della bianca 
                // e la traiettoria desiderata della rossa. Evita tutti i bug di atan2!
                double cos_cut_angle = dir_shot.dot(dir_pocket);

                // Se cos_cut_angle <= 0, la bianca spingerebbe la rossa "al contrario" (tiro da dietro)
                // cos(85°) ≈ 0.087. Filtriamo i tiri più larghi di 85 gradi.
                if (cos_cut_angle <= 0.087) {
                    continue; 
                }

                double cut_angle_rad = std::acos(cos_cut_angle);

                // --- 5. FUNZIONE DI COSTO ---
                constexpr double WEIGHT_CUT_ANGLE = 3.5; 
                constexpr double WEIGHT_POCKET_DIST = 1.0; 
                constexpr double WEIGHT_CUE_DIST = 0.5;   

                double dist_to_rail_x = half_field_length - std::abs(pos_red.x());
                double dist_to_rail_y = half_field_width - std::abs(pos_red.y());
                double rail_penalty = (std::min(dist_to_rail_x, dist_to_rail_y) < ball_diameter) ? 2.0 : 0.0;

                double total_cost = (WEIGHT_CUT_ANGLE * cut_angle_rad) + 
                                   (WEIGHT_POCKET_DIST * pocket_distance) + 
                                   (WEIGHT_CUE_DIST * cue_distance) + 
                                   rail_penalty;

                // --- 6. FISICA E VELOCITÀ ---
                double v2f = std::sqrt(2.0 * CLOTH_SLIDING_FRICTION * GRAVITY * pocket_distance);
                double v1i_impact = (v2f / cos_cut_angle);
                double v_white_start = std::sqrt(std::pow(v1i_impact, 2) + 2.0 * CLOTH_SLIDING_FRICTION * GRAVITY * cue_distance);
                double shot_velocity = velocity_factor_ * v_white_start;            

                // --- 7. YAW PER IL ROBOT ---
                // L'angolo reale della bianca nel piano del tavolo
                double cue_angle_rad = std::atan2(dir_shot.y(), dir_shot.x());
                
                // Aggiungiamo l'offset richiesto dal setup cinematico del tuo end-effector
                double tip_offset_rad = tip_yaw_offset_deg_ * (M_PI / 180.0);
                double final_yaw_rad = normalize_angle(cue_angle_rad + tip_offset_rad);
                double direction_deg = final_yaw_rad * (180.0 / M_PI);

                if (total_cost < best_cost) {
                    best_cost = total_cost;
                    best_pocket = pocket_frame;
                    best_shot_velocity = shot_velocity;
                    best_direction_deg = direction_deg;
                    valid_shot_found = true;
                }
            }

            if (valid_shot_found) {
                auto msg = ShotParamsMsg();
                msg.direction_angle_deg = best_direction_deg;
                msg.impact_shot_velocity = best_shot_velocity;
                
                publisher_->publish(msg);

                RCLCPP_INFO(this->get_logger(), 
                    "Buca: [%s] | Vel: %.3f m/s | Yaw Robot: %.2f deg", 
                    best_pocket.c_str(), best_shot_velocity, best_direction_deg);
            } else {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Nessuna buca raggiungibile fisicamente.");
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