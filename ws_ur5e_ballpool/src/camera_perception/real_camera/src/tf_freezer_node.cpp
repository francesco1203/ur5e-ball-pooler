// ============================================================
//  tf_freezer_node.cpp
//  Congela le TF temporanee (TEMP_*) delle palline in TF Statiche
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

// Assumo che tu abbia questi header dal tuo pacchetto come nel vision_node
#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"

using namespace std::chrono_literals;

class TfFreezerNode : public rclcpp::Node
{
public:
    TfFreezerNode() : Node("tf_freezer_node")
    {
        // Inizializzazione TF Buffer e Listener per LEGGERE le TF dinamiche
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Inizializzazione Broadcaster per SCRIVERE le TF statiche
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Parametro per decidere rispetto a quale frame freezare le palline.
        // E' IMPORTANTISSIMO freezarle rispetto al mondo o al tavolo, NON alla camera.
        // In questo modo, anche se la camera subisce micro-vibrazioni, le palline restano fisse.
        this->declare_parameter<std::string>("reference_frame", "world");
        reference_frame_ = this->get_parameter("reference_frame").as_string();

        // Lista delle palline da cercare (usando le macro del tuo header)
        ball_frames_ = {
            WHITE_SOLID_BALL_FRAME,
            RED_SOLID_BALL_FRAME,
            BLUE_SOLID_BALL_FRAME,
            YELLOW_SOLID_BALL_FRAME
        };

        // Creazione del servizio Trigger
        freeze_service_ = this->create_service<std_srvs::srv::Trigger>(
            "freeze_balls_tf",
            std::bind(&TfFreezerNode::freeze_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "TF Freezer Node avviato. In attesa del servizio '/freeze_balls_tf'...");
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
            std::string temp_frame = "TEMP_" + ball_name;
            geometry_msgs::msg::TransformStamped transform_stamped;

            try {
                // Cerchiamo la trasformazione dalla TEMP_* al reference_frame (es. "world")
                // tf2::TimePointZero prende l'ultima TF disponibile
                // CORRETTO (tutto rclcpp)
                transform_stamped = tf_buffer_->lookupTransform(
                    reference_frame_, 
                    temp_frame, 
                    rclcpp::Time(0), // 0 significa "ultima TF disponibile"
                    rclcpp::Duration::from_seconds(0.5) 
                );

                // Modifichiamo il frame_id e child_frame_id per la pubblicazione statica
                // Il frame padre diventa il reference_frame (es. world)
                transform_stamped.header.frame_id = reference_frame_;
                transform_stamped.header.stamp = this->get_clock()->now();
                
                // Il frame figlio diventa il nome pulito della pallina (rimuoviamo "TEMP_")
                transform_stamped.child_frame_id = ball_name;

                static_transforms.push_back(transform_stamped);
                frozen_balls_list += ball_name + " ";
                success_count++;
                
                RCLCPP_INFO(this->get_logger(), "Congelata TF: [%s] rispetto a [%s]", ball_name.c_str(), reference_frame_.c_str());

            } catch (const tf2::TransformException & ex) {
                // E' normale che alcune palline non ci siano (magari sono state imbucate)
                RCLCPP_WARN(this->get_logger(), "Impossibile congelare %s: %s", temp_frame.c_str(), ex.what());
            }
        }

        if (success_count > 0) {
            // Pubblichiamo tutte le TF congelate in un colpo solo
            static_broadcaster_->sendTransform(static_transforms);
            
            response->success = true;
            response->message = "Trovate e congelate " + std::to_string(success_count) + " palline: " + frozen_balls_list;
        } else {
            response->success = false;
            response->message = "Nessuna pallina 'TEMP_*' trovata nell'albero TF.";
            RCLCPP_ERROR(this->get_logger(), "Nessuna pallina trovata per il freeze.");
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