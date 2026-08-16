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
#include <tf2_ros/transform_broadcaster.h> 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>     

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
            this->declare_parameter<double>("tip_yaw_offset_deg", 180.0); 

            velocity_factor_ = this->get_parameter("velocity_factor").as_double();
            tip_yaw_offset_deg_ = this->get_parameter("tip_yaw_offset_deg").as_double();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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

            tf2::Vector3 pos_white(tf_white.transform.translation.x, tf_white.transform.translation.y, 0.0);
            tf2::Vector3 pos_red(tf_red.transform.translation.x, tf_red.transform.translation.y, 0.0);

            double half_field_length = POOL_TABLE_FIELD_LENGTH / 2.0;
            double half_field_width  = POOL_TABLE_FIELD_WIDTH / 2.0;
            double ball_diameter = BALL_RADIUS * 2.0;

            std::string best_pocket = "";
            double best_cost = std::numeric_limits<double>::max();
            double best_shot_velocity = 0.0;
            double best_direction_deg = 0.0;
            
            tf2::Vector3 best_pos_ghost; 
            tf2::Vector3 best_pos_ghost_impact; // NUOVO: Variabile per salvare il punto di impatto

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

                tf2::Vector3 vec_red_to_pocket = pos_pocket - pos_red;
                double pocket_distance = vec_red_to_pocket.length();
                if (pocket_distance < 0.001) continue;

                tf2::Vector3 dir_pocket = vec_red_to_pocket.normalized();

                // Centro della Ghost Ball (a un diametro di distanza)
                tf2::Vector3 pos_ghost = pos_red - (dir_pocket * ball_diameter);
                
                // NUOVO: Punto di contatto fisico sulla superficie della pallina rossa (a un raggio di distanza)
                tf2::Vector3 pos_ghost_impact = pos_red - (dir_pocket * BALL_RADIUS);

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

                if (cos_cut_angle <= 0.087) {
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

                double v2f = std::sqrt(2.0 * CLOTH_SLIDING_FRICTION * GRAVITY * pocket_distance);
                double v1i_impact = (v2f / cos_cut_angle);
                double v_white_start = std::sqrt(std::pow(v1i_impact, 2) + 2.0 * CLOTH_SLIDING_FRICTION * GRAVITY * cue_distance);
                double shot_velocity = velocity_factor_ * v_white_start;            

                double cue_angle_rad = std::atan2(dir_shot.y(), dir_shot.x());
                
                double tip_offset_rad = tip_yaw_offset_deg_ * (M_PI / 180.0);
                double final_yaw_rad = normalize_angle(cue_angle_rad + tip_offset_rad);
                double direction_deg = final_yaw_rad * (180.0 / M_PI);

                if (total_cost < best_cost) {
                    best_cost = total_cost;
                    best_pocket = pocket_frame;
                    best_shot_velocity = shot_velocity;
                    best_direction_deg = direction_deg;
                    
                    best_pos_ghost = pos_ghost; 
                    best_pos_ghost_impact = pos_ghost_impact; // NUOVO: Salvataggio della coordinata di impatto migliore
                    
                    valid_shot_found = true;
                }
            }

            if (valid_shot_found) {
                auto msg = ShotParamsMsg();
                msg.direction_angle_deg = best_direction_deg;
                msg.impact_shot_velocity = best_shot_velocity;
                
                publisher_->publish(msg);

                // =========================================================
                // Pubblicazione dei TF: Ghost Ball & Punto di Impatto
                // =========================================================
                rclcpp::Time now = this->get_clock()->now();
                tf2::Quaternion q;
                q.setRPY(0, 0, 0); // Nessuna rotazione aggiuntiva per i marker visivi

                // 1. TF Ghost Ball (Centro della pallina fantasma)
                geometry_msgs::msg::TransformStamped t_ghost;
                t_ghost.header.stamp = now;
                t_ghost.header.frame_id = BILLIARD_TABLE_FRAME; 
                t_ghost.child_frame_id = "ghost_ball_frame";
                t_ghost.transform.translation.x = best_pos_ghost.x();
                t_ghost.transform.translation.y = best_pos_ghost.y();
                t_ghost.transform.translation.z = 0.0; 
                t_ghost.transform.rotation.x = q.x();
                t_ghost.transform.rotation.y = q.y();
                t_ghost.transform.rotation.z = q.z();
                t_ghost.transform.rotation.w = q.w();

                // 2. TF Ghost Impact (Punto esatto di contatto sulla rossa)
                geometry_msgs::msg::TransformStamped t_impact;
                t_impact.header.stamp = now;
                t_impact.header.frame_id = BILLIARD_TABLE_FRAME;
                t_impact.child_frame_id = "ghost_impact_frame";
                t_impact.transform.translation.x = best_pos_ghost_impact.x();
                t_impact.transform.translation.y = best_pos_ghost_impact.y();
                t_impact.transform.translation.z = 0.0;
                t_impact.transform.rotation.x = q.x();
                t_impact.transform.rotation.y = q.y();
                t_impact.transform.rotation.z = q.z();
                t_impact.transform.rotation.w = q.w();

                // Invia entrambe le trasformazioni
                tf_broadcaster_->sendTransform(t_ghost);
                tf_broadcaster_->sendTransform(t_impact);
                // =========================================================

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