// ============================================================
//  cartesian_velocity_publisher.cpp
//  Nodo ROS2 che logga ESCLUSIVAMENTE la Velocità del TCP (Flangia/Tool0)
// ============================================================

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// Librerie Eigen per calcoli matriciali
#include <Eigen/Core>
#include <Eigen/Dense>

// Librerie MoveIt per calcolo del Jacobiano
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"

using namespace std::chrono_literals;

class CartesianVelocityPublisher : public rclcpp::Node
{
    public:
        using JointStateMsg = sensor_msgs::msg::JointState;
        using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
        
        CartesianVelocityPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
            : Node("cartesian_velocity_publisher", options), 
              is_logging_(true),
              target_link_("tool0") 
        {
            /* Sottoscrizioni e Publishers */
            joint_sub_ = this->create_subscription<JointStateMsg>(
                JOINT_STATES_TOPIC, rclcpp::QoS(10),
                std::bind(&CartesianVelocityPublisher::joint_state_callback, this, std::placeholders::_1));

            twist_pub_  = this->create_publisher<TwistStampedMsg>(CARTESIAN_TWIST_TOPIC, 10);
        }

        void init()
        {
            RCLCPP_INFO(this->get_logger(), "Caricamento modello robot per calcolo Jacobiano su tool0...");
            
            robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(), "robot_description");
            robot_model_ = robot_model_loader_->getModel();
            
            if (!robot_model_) {
                RCLCPP_ERROR(this->get_logger(), "Impossibile caricare il robot_model!");
            } else {
                robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
                robot_state_->setToDefaultValues();
                
                // Gruppo cinematico standard UR
                joint_model_group_ = robot_model_->getJointModelGroup("manipulator");
                
                if(!joint_model_group_) {
                    RCLCPP_ERROR(this->get_logger(), "JointModelGroup 'manipulator' non trovato!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Cartesian Velocity Publisher pronto all'uso.");
                }
            }
        }

    private:
        std::atomic<bool> is_logging_; 
        std::string target_link_;
        
        rclcpp::Subscription<JointStateMsg>::SharedPtr joint_sub_; 
        rclcpp::Publisher<TwistStampedMsg>::SharedPtr twist_pub_;

        std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
        moveit::core::RobotModelPtr robot_model_;
        moveit::core::RobotStatePtr robot_state_;
        const moveit::core::JointModelGroup* joint_model_group_;

        void joint_state_callback(const JointStateMsg::SharedPtr msg)
        {
            if (!is_logging_ || !robot_state_ || !joint_model_group_) return;

            // Verifichiamo se il messaggio contiene le velocità
            bool has_vel = (msg->velocity.size() == msg->position.size());
            if (!has_vel) return; // Se mancano le velocità di giunto, usciamo

            // 1. Inseriamo le POSIZIONI per calcolare il Jacobiano corretto per la configurazione attuale
            robot_state_->setVariablePositions(msg->name, msg->position);
            // 2. Inseriamo le VELOCITÀ
            robot_state_->setVariableVelocities(msg->name, msg->velocity);
            
            // Ricalcoliamo lo stato del robot
            robot_state_->update();

            // ==========================================
            // CALCOLO VELOCITÀ CARTESIANA (Jacobiano)
            // ==========================================
            
            // Estraiamo il vettore q_dot (velocità giunti) e lo ordiniamo secondo il gruppo MoveIt
            std::vector<double> q_dot_vec;
            robot_state_->copyJointGroupVelocities(joint_model_group_, q_dot_vec);
            Eigen::Map<Eigen::VectorXd> q_dot(q_dot_vec.data(), 6); // Robot a 6 DOF

            // Calcoliamo la Matrice Jacobiana su tool0
            Eigen::MatrixXd jacobian;
            robot_state_->getJacobian(joint_model_group_, 
                                      robot_state_->getLinkModel(target_link_), 
                                      Eigen::Vector3d::Zero(), // Nessun offset
                                      jacobian);

            // x_dot = J * q_dot
            Eigen::VectorXd x_dot = jacobian * q_dot;

            // Creazione e pubblicazione del messaggio
            TwistStampedMsg twist_msg;
            twist_msg.header.stamp = msg->header.stamp; 
            twist_msg.header.frame_id = robot_model_->getModelFrame(); 
            
            // x_dot contiene: [v_x, v_y, v_z, omega_x, omega_y, omega_z]
            twist_msg.twist.linear.x = x_dot(0);
            twist_msg.twist.linear.y = x_dot(1);
            twist_msg.twist.linear.z = x_dot(2);
            twist_msg.twist.angular.x = x_dot(3);
            twist_msg.twist.angular.y = x_dot(4);
            twist_msg.twist.angular.z = x_dot(5);
            
            twist_pub_->publish(twist_msg);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<CartesianVelocityPublisher>(node_options);
    
    node->init();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}