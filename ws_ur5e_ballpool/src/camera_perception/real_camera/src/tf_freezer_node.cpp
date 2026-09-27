// ============================================================
//  tf_freezer_node.cpp
//  Congela le TF realtime in TF Statiche (con Graveyard per pulizia)
// ============================================================
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"

using namespace std::chrono_literals;

class TfFreezerNode : public rclcpp::Node
{
public:
    TfFreezerNode() : Node("tf_freezer_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        this->declare_parameter<std::string>("reference_frame", "world");
        reference_frame_ = this->get_parameter("reference_frame").as_string();

        ball_frames_ = {
            WHITE_SOLID_BALL_FRAME,
            RED_SOLID_BALL_FRAME,
            BLUE_SOLID_BALL_FRAME,
            YELLOW_SOLID_BALL_FRAME
        };

        freeze_service_ = this->create_service<std_srvs::srv::Trigger>(
            FREEZE_TF_SERVICE,
            std::bind(&TfFreezerNode::freeze_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "TF Freezer Node (Static) avviato. In attesa del servizio '/freeze_balls_tf'...");
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr freeze_service_;
    
    std::string reference_frame_;
    std::vector<std::string> ball_frames_;

    void freeze_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Ricevuta richiesta di freeze delle palline.");

        std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
        std::string frozen_balls_list = "";
        int success_count = 0;

        for (const auto& ball_name : ball_frames_)
        {
            std::string realtime_frame = REALTIME_PREFIX + ball_name;
            geometry_msgs::msg::TransformStamped transform_stamped;

            try {
                // Tenta di leggere la posizione corrente
                transform_stamped = tf_buffer_->lookupTransform(
                    reference_frame_, 
                    realtime_frame, 
                    rclcpp::Time(0), 
                    rclcpp::Duration::from_seconds(0.5) 
                );

                transform_stamped.header.frame_id = reference_frame_;
                transform_stamped.header.stamp = this->get_clock()->now();
                transform_stamped.child_frame_id = ball_name;

                static_transforms.push_back(transform_stamped);
                frozen_balls_list += ball_name + " ";
                success_count++;
                
                RCLCPP_INFO(this->get_logger(), "Congelata TF: [%s]", ball_name.c_str());

            } catch (const tf2::TransformException & ex) {
                // PALLINA NON TROVATA -> CIMITERO
                RCLCPP_WARN(this->get_logger(), "Pallina %s non trovata. Spostata a Z = -10.0 (Cimitero).", ball_name.c_str());
                
                geometry_msgs::msg::TransformStamped graveyard_tf;
                graveyard_tf.header.frame_id = reference_frame_;
                graveyard_tf.header.stamp = this->get_clock()->now();
                graveyard_tf.child_frame_id = ball_name;
                
                graveyard_tf.transform.translation.x = 0.0;
                graveyard_tf.transform.translation.y = 0.0;
                graveyard_tf.transform.translation.z = -10.0; // Sotto terra
                
                graveyard_tf.transform.rotation.x = 0.0;
                graveyard_tf.transform.rotation.y = 0.0;
                graveyard_tf.transform.rotation.z = 0.0;
                graveyard_tf.transform.rotation.w = 1.0;
                
                static_transforms.push_back(graveyard_tf);
            }
        }

        // Pubblica SEMPRE le 4 palline: quelle attuali al loro posto, quelle assenti sotto terra.
        // Questo forza il TF tree a sovrascrivere i vecchi valori.
        static_broadcaster_->sendTransform(static_transforms);

        if (success_count > 0) {
            response->success = true;
            response->message = "Trovate e congelate " + std::to_string(success_count) + " palline: " + frozen_balls_list;
        } else {
            response->success = false;
            response->message = "Nessuna pallina trovata. Campo ripulito (tutte nel cimitero).";
            RCLCPP_ERROR(this->get_logger(), "Nessuna pallina trovata per il freeze. Campo ripulito.");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfFreezerNode>());
    rclcpp::shutdown();
    return 0;
}