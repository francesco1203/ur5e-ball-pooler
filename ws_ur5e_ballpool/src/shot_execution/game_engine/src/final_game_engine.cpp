// ============================================================
//  game_engine.cpp
//  Nodo ROS2 che seleziona la combinazione Pallina-Buca ottimale.
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
        
        using SetBoolSrv = std_srvs::srv::SetBool;

        GameEngine() : Node("game_engine")
        {
            /* IPER PARAMETRI */
            this->declare_parameter<double>("velocity_factor", 1.2);
            this->declare_parameter<double>("tip_yaw_offset_deg", 180.0); 
            this->declare_parameter<double>("rail_proximity_threshold", 0.04);
            this->declare_parameter<double>("normal_impact_angle_deg", 10.0);
            this->declare_parameter<double>("steep_impact_angle_deg", 15.0);   
            this->declare_parameter<bool>("start_active", false);

            velocity_factor_ = this->get_parameter("velocity_factor").as_double();
            tip_yaw_offset_deg_ = this->get_parameter("tip_yaw_offset_deg").as_double();
            rail_proximity_threshold_ = this->get_parameter("rail_proximity_threshold").as_double();
            normal_impact_angle_deg_ = this->get_parameter("normal_impact_angle_deg").as_double();
            steep_impact_angle_deg_ = this->get_parameter("steep_impact_angle_deg").as_double();
            is_active_ = this->get_parameter("start_active").as_bool();

            /* tf */
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            /* publisher e servizi */
            publisher_ = this->create_publisher<ShotParamsMsg>(SHOT_PARAMS_TOPIC, 10);
            toggle_service_ = this->create_service<SetBoolSrv>(
                TOGGLE_GAME_ENGINE_SERVICE, 
                std::bind(&GameEngine::handle_activation, this, std::placeholders::_1, std::placeholders::_2)
            );
            
            // Frame delle buche
            pocket_frames_ = {
                HOLE_TOP_RIGHT_FRAME, HOLE_TOP_LEFT_FRAME,
                HOLE_MID_RIGHT_FRAME, HOLE_MID_LEFT_FRAME,
                HOLE_BOTTOM_RIGHT_FRAME, HOLE_BOTTOM_LEFT_FRAME
            };

            // Frame delle palline bersaglio (da valutare)
            target_balls_frames_ = {
                RED_SOLID_BALL_FRAME, 
                BLUE_SOLID_BALL_FRAME, 
                YELLOW_SOLID_BALL_FRAME
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
        std::vector<std::string> target_balls_frames_;
        rclcpp::Service<SetBoolSrv>::SharedPtr toggle_service_;

        double velocity_factor_;
        double tip_yaw_offset_deg_;
        double rail_proximity_threshold_;
        double steep_impact_angle_deg_;
        double normal_impact_angle_deg_;
        bool is_active_; 

        double normalize_angle(double angle)
        {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

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
            if (!is_active_) return;

            /* CERCHIAMO LA PALLINA BIANCA (Indispensabile) */
            geometry_msgs::msg::TransformStamped tf_white;
            try {
                tf_white = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, WHITE_SOLID_BALL_FRAME, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                return; // Se non vedo la bianca, non posso tirare
            }
            tf2::Vector3 pos_white(tf_white.transform.translation.x, tf_white.transform.translation.y, 0.0);

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

            /* VARIABILI PER SALVARE IL TIRO MIGLIORE IN ASSOLUTO */
            std::string best_pocket = "";
            std::string best_ball = "";
            double best_cost = std::numeric_limits<double>::max();
            double best_shot_velocity_planar = 0.0;
            double best_shot_velocity = 0.0;
            double best_direction_deg = 0.0;
            bool valid_shot_found = false;

            // CICLO ESTERNO: Analizziamo una ad una le palline bersaglio disponibili
            for (const auto& target_frame : target_balls_frames_)
            {
                geometry_msgs::msg::TransformStamped tf_target;
                try {
                    // Provo a leggere la TF della pallina. Se non c'è, passo alla prossima.
                    tf_target = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, target_frame, tf2::TimePointZero);
                } catch (const tf2::TransformException & ex) {
                    continue; 
                }

                tf2::Vector3 pos_target(tf_target.transform.translation.x, tf_target.transform.translation.y, 0.0);

                // CICLO INTERNO: Valutiamo tutte le buche per la pallina target corrente
                for (const auto& pocket_frame : pocket_frames_)
                {
                    geometry_msgs::msg::TransformStamped tf_pocket;
                    try {
                        tf_pocket = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, pocket_frame, tf2::TimePointZero);
                    } catch (const tf2::TransformException & ex) {
                        continue; 
                    }

                    tf2::Vector3 pos_pocket(tf_pocket.transform.translation.x, tf_pocket.transform.translation.y, 0.0);

                    tf2::Vector3 vec_target_to_pocket = pos_pocket - pos_target;
                    double pocket_distance = vec_target_to_pocket.length();
                    if (pocket_distance < 0.001) continue;

                    tf2::Vector3 dir_pocket = vec_target_to_pocket.normalized();

                    // Calcolo della Ghost Ball
                    tf2::Vector3 pos_ghost = pos_target - (dir_pocket * ball_diameter);

                    // Scarta il tiro se la ghost ball finisce fuori o contro le sponde
                    double margin = BALL_RADIUS; 
                    if (std::abs(pos_ghost.x()) >= (half_field_length - margin) || 
                        std::abs(pos_ghost.y()) >= (half_field_width - margin)) 
                    {
                        continue; 
                    }

                    tf2::Vector3 vec_white_to_ghost = pos_ghost - pos_white;
                    double cue_distance = vec_white_to_ghost.length();
                    if (cue_distance < 0.001) continue;

                    tf2::Vector3 dir_shot = vec_white_to_ghost.normalized();

                    double cos_cut_angle = dir_shot.dot(dir_pocket);
                    if (cos_cut_angle <= 0.087) { // Evita angoli di taglio impossibili (vicini a 90 gradi)
                        continue; 
                    }

                    double cut_angle_rad = std::acos(cos_cut_angle);

                    // Pesi dei Costi
                    constexpr double WEIGHT_CUT_ANGLE = 3.5; 
                    constexpr double WEIGHT_POCKET_DIST = 1.0; 
                    constexpr double WEIGHT_CUE_DIST = 0.5;   

                    double dist_to_rail_x = half_field_length - std::abs(pos_target.x());
                    double dist_to_rail_y = half_field_width - std::abs(pos_target.y());
                    double rail_penalty = (std::min(dist_to_rail_x, dist_to_rail_y) < ball_diameter) ? 2.0 : 0.0;

                    double total_cost = (WEIGHT_CUT_ANGLE * cut_angle_rad) + 
                                       (WEIGHT_POCKET_DIST * pocket_distance) + 
                                       (WEIGHT_CUE_DIST * cue_distance) + 
                                       rail_penalty;

                    // Se questo tiro è migliore di tutti i precedenti, salvalo!
                    if (total_cost < best_cost) {
                        
                        double v2f = std::sqrt(2.0 * CLOTH_SLIDING_FRICTION * GRAVITY * pocket_distance);
                        double v1i_impact = (v2f / cos_cut_angle);
                        double v_white_start = std::sqrt(std::pow(v1i_impact, 2) + 2.0 * CLOTH_SLIDING_FRICTION * GRAVITY * cue_distance);
                        double shot_velocity_planar = velocity_factor_ * v_white_start ;   
                        double shot_velocity = shot_velocity_planar / cos(chosen_impact_angle * (M_PI / 180.0)); 

                        double cue_angle_rad = std::atan2(dir_shot.y(), dir_shot.x());
                        double tip_offset_rad = tip_yaw_offset_deg_ * (M_PI / 180.0);
                        double final_yaw_rad = normalize_angle(cue_angle_rad + tip_offset_rad);
                        double direction_deg = final_yaw_rad * (180.0 / M_PI);

                        best_cost = total_cost;
                        best_pocket = pocket_frame;
                        best_ball = target_frame; // Salviamo quale pallina abbiamo scelto
                        best_shot_velocity_planar = shot_velocity_planar;
                        best_shot_velocity = shot_velocity;
                        best_direction_deg = direction_deg;
                        valid_shot_found = true;
                    }
                }
            }

            if (valid_shot_found) {
                auto msg = ShotParamsMsg();
                msg.direction_angle_deg = best_direction_deg;
                msg.impact_shot_velocity = best_shot_velocity;
                msg.impact_angle_deg = chosen_impact_angle; 
                
                publisher_->publish(msg);

                // Mostra un bel log riassuntivo che include la pallina bersaglio
                RCLCPP_INFO(this->get_logger(), 
                    "BERSAGLIO: [%s] -> BUCA: [%s] | Costo Ottimale: %.2f", 
                    best_ball.c_str(), best_pocket.c_str(), best_cost);
                RCLCPP_INFO(this->get_logger(),
                    "PARAMETRI: Vel: %.3f m/s | Yaw: %.2f deg | Pitch: %.2f deg",
                    best_shot_velocity, best_direction_deg, chosen_impact_angle);
            } else {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Nessuna pallina ha un tiro fisicamente raggiungibile verso le buche.");
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