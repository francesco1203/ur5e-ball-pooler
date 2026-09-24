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

#include <std_srvs/srv/set_bool.hpp>

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "interfaces_pkg/msg/shot_params.hpp"

using namespace std::chrono_literals;

class GameEngine : public rclcpp::Node
{
    public:
        using ShotParamsMsg = interfaces_pkg::msg::ShotParams;   
        using ShotParamsPublisher = rclcpp::Publisher<ShotParamsMsg>::SharedPtr;
        
        // Alias per il servizio
        using SetBoolSrv = std_srvs::srv::SetBool;

        GameEngine() : Node("game_engine")
        {
            /*IPER PARAMETRI*/

            // Per scalare velocità stecca
            this->declare_parameter<double>("velocity_factor", 1.2);
         
            // Offset di yaw del tip. 
            this->declare_parameter<double>("tip_yaw_offset_deg", 180.0); 
            tip_yaw_offset_deg_ = this->get_parameter("tip_yaw_offset_deg").as_double();

            // Parametri per la gestione dinamica dell'inclinazione dell'asta (pitch)
            this->declare_parameter<double>("rail_proximity_threshold", 0.04);
            this->declare_parameter<double>("normal_impact_angle_deg", 10.0);
            this->declare_parameter<double>("steep_impact_angle_deg", 15.0);   
            this->declare_parameter<double>("cloth_sliding_friction", 0.02);   
            
            // Parametro per decidere se il motore parte già attivo o disattivato di default
            this->declare_parameter<bool>("start_active", false);

            velocity_factor_ = this->get_parameter("velocity_factor").as_double();
            tip_yaw_offset_deg_ = this->get_parameter("tip_yaw_offset_deg").as_double();
            rail_proximity_threshold_ = this->get_parameter("rail_proximity_threshold").as_double();
            normal_impact_angle_deg_ = this->get_parameter("normal_impact_angle_deg").as_double();
            steep_impact_angle_deg_ = this->get_parameter("steep_impact_angle_deg").as_double();
            cloth_sliding_friction_ = this->get_parameter("cloth_sliding_friction").as_double();
            
            is_active_ = this->get_parameter("start_active").as_bool();

            /*tf*/
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            /*publisher verso task node*/
            publisher_ = this->create_publisher<ShotParamsMsg>(SHOT_PARAMS_TOPIC, 10);

            /* Servizio per attivare/disattivare il calcolo */
            toggle_service_ = this->create_service<SetBoolSrv>(
                TOGGLE_GAME_ENGINE_SERVICE, 
                std::bind(&GameEngine::handle_activation, this, std::placeholders::_1, std::placeholders::_2)
            );
            
            //altro
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

            RCLCPP_INFO(this->get_logger(), "Game Engine avviato (Stato iniziale: %s).", is_active_ ? "ATTIVO" : "INATTIVO");
        }

    private:
        ShotParamsPublisher publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::vector<std::string> pocket_frames_;
        
        rclcpp::Service<SetBoolSrv>::SharedPtr toggle_service_;

        double velocity_factor_;
        double tip_yaw_offset_deg_;
        
        // Variabili per l'inclinazione dinamica
        double rail_proximity_threshold_;
        double steep_impact_angle_deg_;
        double normal_impact_angle_deg_;
        double cloth_sliding_friction_;
        
        bool is_active_; // Flag di stato

        double normalize_angle(double angle)
        {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

        // Callback del servizio SetBool
        void handle_activation(const std::shared_ptr<SetBoolSrv::Request> request,
                               std::shared_ptr<SetBoolSrv::Response> response)
        {
            is_active_ = request->data;
            response->success = true;
            
            if (is_active_) {
                response->message = "Game Engine ATTIVATO.";
                RCLCPP_INFO(this->get_logger(), "Servizio chiamato: Game Engine ATTIVATO.");
            } else {
                response->message = "Game Engine DISATTIVATO.";
                RCLCPP_INFO(this->get_logger(), "Servizio chiamato: Game Engine DISATTIVATO.");
            }
        }

        void publish_params()
        {
            // Se inattivo, usciamo subito dal timer senza consumare CPU né inviare messaggi
            if (!is_active_) {
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "--- Inizio ciclo calcolo tiro ---");

            /* TROVIAMO LE POSIZIONI DELLE PALLINE BIANCA E ROSSA*/
            geometry_msgs::msg::TransformStamped tf_white, tf_red;

            try {
                tf_white = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, WHITE_SOLID_BALL_FRAME, tf2::TimePointZero);
                tf_red   = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, RED_SOLID_BALL_FRAME,   tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                // Questo è il motivo più comune di blocco: le TF non sono pubblicate o i nomi dei frame non coincidono
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Errore TF palline: impossibile trovare le trasformate. Motivo: %s", ex.what());
                return;
            }

            tf2::Vector3 pos_white(tf_white.transform.translation.x, tf_white.transform.translation.y, 0.0);
            tf2::Vector3 pos_red(tf_red.transform.translation.x, tf_red.transform.translation.y, 0.0);

            RCLCPP_INFO(this->get_logger(), "TF Palline trovate. Bianca (%.2f, %.2f) - Rossa (%.2f, %.2f)", 
                        pos_white.x(), pos_white.y(), pos_red.x(), pos_red.y());

            /* CALCOLO INCLINAZIONE ASTA (PITCH) */
            double half_field_length = POOL_TABLE_FIELD_LENGTH / 2.0;
            double half_field_width  = POOL_TABLE_FIELD_WIDTH / 2.0;
            double ball_diameter = BALL_RADIUS * 2.0;
           
            double dist_white_to_rail_x = half_field_length - std::abs(pos_white.x());
            double dist_white_to_rail_y = half_field_width - std::abs(pos_white.y());
            double min_dist_white_to_rail = std::min(dist_white_to_rail_x, dist_white_to_rail_y);

            double chosen_impact_angle = normal_impact_angle_deg_;
            if (min_dist_white_to_rail < rail_proximity_threshold_) {
                chosen_impact_angle = steep_impact_angle_deg_;
            }

            /* SCELTA AUTOMATICA DELLA BUCA MIGLIORE E CALCOLO DELLA VELCOCITÀ PER IL TIRO*/
            std::string best_pocket = "";
            double best_cost = std::numeric_limits<double>::max();
            double best_shot_velocity_planar = 0.0;
            double best_shot_velocity = 0.0;
            double best_direction_deg = 0.0;
            bool valid_shot_found = false;

            for (const auto& pocket_frame : pocket_frames_)
            {
                geometry_msgs::msg::TransformStamped tf_pocket;
                try {
                    tf_pocket = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, pocket_frame, tf2::TimePointZero);
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "Errore TF buca [%s]: %s", pocket_frame.c_str(), ex.what());
                    continue; 
                }

                tf2::Vector3 pos_pocket(tf_pocket.transform.translation.x, tf_pocket.transform.translation.y, 0.0);
                tf2::Vector3 vec_red_to_pocket = pos_pocket - pos_red;
                double pocket_distance = vec_red_to_pocket.length();
                
                if (pocket_distance < 0.001) continue;

                tf2::Vector3 dir_pocket = vec_red_to_pocket.normalized();
                tf2::Vector3 pos_ghost = pos_red - (dir_pocket * ball_diameter);
                double margin = BALL_RADIUS; 
                
                if (std::abs(pos_ghost.x()) >= (half_field_length - margin) || 
                    std::abs(pos_ghost.y()) >= (half_field_width - margin)) 
                {
                    RCLCPP_INFO(this->get_logger(), "Buca [%s] scartata: Ghost ball fuori o troppo vicina al bordo.", pocket_frame.c_str());
                    continue; 
                }

                tf2::Vector3 vec_white_to_ghost = pos_ghost - pos_white;
                double cue_distance = vec_white_to_ghost.length();
                if (cue_distance < 0.001) continue;

                tf2::Vector3 dir_shot = vec_white_to_ghost.normalized();
                double cos_cut_angle = dir_shot.dot(dir_pocket);
                
                if (cos_cut_angle <= 0.087) {
                    RCLCPP_INFO(this->get_logger(), "Buca [%s] scartata: Angolo di taglio non realistico (cos <= 0.087).", pocket_frame.c_str());
                    continue; 
                }

                double cut_angle_rad = std::acos(cos_cut_angle);

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

                double v2f = std::sqrt(2.0 * cloth_sliding_friction_ * GRAVITY * pocket_distance);
                double v1i_impact = (v2f / cos_cut_angle);
                double v_white_start = std::sqrt(std::pow(v1i_impact, 2) + 2.0 * cloth_sliding_friction_ * GRAVITY * cue_distance);
                double shot_velocity_planar = velocity_factor_ * v_white_start ;   
                double shot_velocity = shot_velocity_planar / cos(chosen_impact_angle * (M_PI / 180.0)); 

                double cue_angle_rad = std::atan2(dir_shot.y(), dir_shot.x());
                double tip_offset_rad = tip_yaw_offset_deg_ * (M_PI / 180.0);
                double final_yaw_rad = normalize_angle(cue_angle_rad + tip_offset_rad);
                double direction_deg = final_yaw_rad * (180.0 / M_PI);

                RCLCPP_INFO(this->get_logger(), "Buca [%s] valida | Costo calcolato: %.3f", pocket_frame.c_str(), total_cost);

                if (total_cost < best_cost) {
                    best_cost = total_cost;
                    best_pocket = pocket_frame;
                    best_shot_velocity_planar = shot_velocity_planar;
                    best_shot_velocity = shot_velocity;
                    best_direction_deg = direction_deg;
                    valid_shot_found = true;
                }
            }

            if (valid_shot_found) {
                auto msg = ShotParamsMsg();
                msg.direction_angle_deg = best_direction_deg;
                msg.impact_shot_velocity = best_shot_velocity;
                msg.impact_angle_deg = chosen_impact_angle; 
                
                publisher_->publish(msg);

                RCLCPP_INFO(this->get_logger(), 
                    "+++ SCELTA FINALE: Buca [%s] | Vel: %.3f m/s (planar %.3f m/s) | Yaw: %.2f deg | Pitch: %.2f deg | Dist. sponda: %.3f m +++", 
                    best_pocket.c_str(), best_shot_velocity, best_shot_velocity_planar, best_direction_deg, chosen_impact_angle, min_dist_white_to_rail);
            } else {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Nessuna buca raggiungibile fisicamente (tutte scartate).");
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