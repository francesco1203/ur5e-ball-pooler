// // ============================================================
// //  fake_game_engine.cpp
// //  Nodo ROS2 che simula il comportamento di un motore di gioco per la pianificazione dei colpi.
// //
// //  Eseguire con: ros2 run shot_planning fake_game_engine --ros-args --params-file $(ros2 pkg prefix shot_planning)/share/shot_planning/config/fake_game_engine_params.yaml
// //
// // ============================================================


// #include <chrono>
// #include <memory>
// #include "rclcpp/rclcpp.hpp"

// #include "shared_headers_pkg/ros2_architecture.hpp"     // header per topic name
// #include "interfaces_pkg/msg/shot_params.hpp"                // custom message


// using namespace std::chrono_literals;


// class FakeGameEngine : public rclcpp::Node
// {
//     public:
//         /*Alias*/
//         using ShotParamsMsg = interfaces_pkg::msg::ShotParams;   
//         using ShotParamsPublisher = rclcpp::Publisher<ShotParamsMsg>::SharedPtr;

//         /* Builder */
//         FakeGameEngine() : Node("fake_game_engine")
//         {
//             // Dichiara e leggi i parametri (verranno presi dal file YAML)
//             this->declare_parameter<double>("direction_angle_deg", 35.0);
//             this->declare_parameter<double>("impact_shot_velocity", 0.10);

//             direction_ = this->get_parameter("direction_angle_deg").as_double();
//             velocity_ = this->get_parameter("impact_shot_velocity").as_double();

//             RCLCPP_INFO(this->get_logger(), "Fake Engine avviato. Direction: %.2f, Velocity: %.2f", direction_, velocity_);

//             // Crea il publisher
//             publisher_ = this->create_publisher<ShotParamsMsg>(SHOT_PARAMS_TOPIC , 10);

//             // Timer per pubblicare il tiro a ogni 2s
//             timer_ = this->create_wall_timer(
//                 2000ms, std::bind(&FakeGameEngine::publish_params, this));
//         }

//     private:
//         /*ATTRIBUTI*/
//         ShotParamsPublisher publisher_;
//         rclcpp::TimerBase::SharedPtr timer_;
//         double direction_;
//         double velocity_;
        
//         //callback di pubblication
//         void publish_params()
//         {
//             auto msg = ShotParamsMsg();
//             msg.direction_angle_deg = direction_;
//             msg.impact_shot_velocity = velocity_;
            
//             publisher_->publish(msg);
//             RCLCPP_DEBUG(this->get_logger(), "Parametri di tiro pubblicati.");
//         }

// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FakeGameEngine>());
//     rclcpp::shutdown();
//     return 0;
// }



// ============================================================
//  fake_game_engine.cpp
//  Nodo ROS2 che simula il comportamento di un motore di gioco puntando 
//  direttamente la pallina bianca verso una buca selezionata tramite TF2.
//
//  Eseguire con: ros2 run shot_planning fake_game_engine --ros-args --params-file $(ros2 pkg prefix shot_planning)/share/shot_planning/config/fake_game_engine_params.yaml
// ============================================================

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "shared_headers_pkg/ros2_architecture.hpp"     // header per topic name e frame names
#include "shared_headers_pkg/scene_description.hpp"
#include "interfaces_pkg/msg/shot_params.hpp"                // custom message

using namespace std::chrono_literals;

class FakeGameEngine : public rclcpp::Node
{
    public:
        using ShotParamsMsg = interfaces_pkg::msg::ShotParams;   
        using ShotParamsPublisher = rclcpp::Publisher<ShotParamsMsg>::SharedPtr;

        FakeGameEngine() : Node("fake_game_engine")
        {
            // Dichiara e legge i parametri
            this->declare_parameter<std::string>("target_pocket_frame", HOLE_BOTTOM_LEFT_FRAME);
            this->declare_parameter<double>("impact_shot_velocity", 0.5);
            this->declare_parameter<double>("tip_yaw_offset_deg", 180.0);

            target_pocket_frame_ = this->get_parameter("target_pocket_frame").as_string();
            velocity_ = this->get_parameter("impact_shot_velocity").as_double();
            tip_yaw_offset_deg_ = this->get_parameter("tip_yaw_offset_deg").as_double();

            RCLCPP_INFO(this->get_logger(), 
                "Fake Game Engine avviato. Target Pocket: [%s], Velocity: %.2f", 
                target_pocket_frame_.c_str(), velocity_);

            // Inizializzazione Listener TF2
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Crea il publisher
            publisher_ = this->create_publisher<ShotParamsMsg>(SHOT_PARAMS_TOPIC, 10);

            // Timer per leggere le TF e pubblicare il tiro ogni 2 secondi
            timer_ = this->create_wall_timer(
                2000ms, std::bind(&FakeGameEngine::publish_params, this));
        }

    private:
        ShotParamsPublisher publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string target_pocket_frame_;
        double velocity_;
        double tip_yaw_offset_deg_;

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
            geometry_msgs::msg::TransformStamped tf_white, tf_pocket;

            // Lettura delle TF della pallina bianca e della buca selezionata rispetto al tavolo
            try {
                tf_white = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, WHITE_SOLID_BALL_FRAME, tf2::TimePointZero);
                tf_pocket = tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, target_pocket_frame_, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "In attesa delle TF (Bianca o Buca): %s", ex.what());
                return;
            }

            double w_x = tf_white.transform.translation.x;
            double w_y = tf_white.transform.translation.y;

            double p_x = tf_pocket.transform.translation.x;
            double p_y = tf_pocket.transform.translation.y;

            // Calcolo del vettore dalla pallina bianca alla buca
            double dx = p_x - w_x;
            double dy = p_y - w_y;
            double distance = std::hypot(dx, dy);

            if (distance < 0.001) {
                RCLCPP_WARN(this->get_logger(), "La pallina bianca è troppo vicina alla buca target!");
                return;
            }

            // Calcolo dell'angolo di direzione della traiettoria
            double angle_rad = std::atan2(dy, dx);
            
            // Applicazione dell'offset meccanico del tip del robot
            double tip_offset_rad = tip_yaw_offset_deg_ * (M_PI / 180.0);
            double final_yaw_rad = normalize_angle(angle_rad + tip_offset_rad);
            double direction_deg = final_yaw_rad * (180.0 / M_PI);

            // Creazione e pubblicazione del messaggio
            auto msg = ShotParamsMsg();
            msg.direction_angle_deg = direction_deg;
            msg.impact_shot_velocity = velocity_;
            
            publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), 
                "Fake Shot -> Buca: [%s] | Distanza: %.3f m | Yaw Robot: %.2f deg | Vel: %.3f m/s", 
                target_pocket_frame_.c_str(), distance, direction_deg, velocity_);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeGameEngine>());
    rclcpp::shutdown();
    return 0;
}