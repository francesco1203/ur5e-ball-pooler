// ============================================================
//  fake_game_engine.cpp
//  Nodo ROS2 che simula il comportamento di un motore di gioco per la pianificazione dei colpi.
//
//  Eseguire con: ros2 run shot_planning fake_game_engine --ros-args --params-file $(ros2 pkg prefix shot_planning)/share/shot_planning/config/fake_game_engine_params.yaml
//
// ============================================================


#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "shared_headers_pkg/ros2_architecture.hpp"     // header per topic name
#include "interfaces_pkg/msg/shot_params.hpp"                // custom message


using namespace std::chrono_literals;


class FakeGameEngine : public rclcpp::Node
{
    public:
        /*Alias*/
        using ShotParamsMsg = interfaces_pkg::msg::ShotParams;   
        using ShotParamsPublisher = rclcpp::Publisher<ShotParamsMsg>::SharedPtr;

        /* Builder */
        FakeGameEngine() : Node("fake_game_engine")
        {
            // Dichiara e leggi i parametri (verranno presi dal file YAML)
            this->declare_parameter<double>("direction_angle_deg", 35.0);
            this->declare_parameter<double>("impact_shot_velocity", 0.10);

            direction_ = this->get_parameter("direction_angle_deg").as_double();
            velocity_ = this->get_parameter("impact_shot_velocity").as_double();

            RCLCPP_INFO(this->get_logger(), "Fake Engine avviato. Direction: %.2f, Velocity: %.2f", direction_, velocity_);

            // Crea il publisher
            publisher_ = this->create_publisher<ShotParamsMsg>(SHOT_PARAMS_TOPIC , 10);

            // Timer per pubblicare il tiro a ogni 2s
            timer_ = this->create_wall_timer(
                2000ms, std::bind(&FakeGameEngine::publish_params, this));
        }

    private:
        /*ATTRIBUTI*/
        ShotParamsPublisher publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        double direction_;
        double velocity_;
        
        //callback di pubblication
        void publish_params()
        {
            auto msg = ShotParamsMsg();
            msg.direction_angle_deg = direction_;
            msg.impact_shot_velocity = velocity_;
            
            publisher_->publish(msg);
            RCLCPP_DEBUG(this->get_logger(), "Parametri di tiro pubblicati.");
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeGameEngine>());
    rclcpp::shutdown();
    return 0;
}