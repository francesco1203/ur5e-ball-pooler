#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Librerie MoveIt per calcolo della Cinematica Diretta (FK)
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"

using namespace std::chrono_literals;

class CartesianLogger : public rclcpp::Node
{
    public:
        /*ALIAS*/
        using JointStateMsg = sensor_msgs::msg::JointState;
        using JointStateSub = rclcpp::Subscription<JointStateMsg>::SharedPtr;

        using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
        using PoseStampedPub = rclcpp::Publisher<PoseStampedMsg>::SharedPtr;
        
        /* COSTRUTTORE */
        CartesianLogger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
            : Node("cartesian_pose_logger", options), 
              is_logging_(true),
              target_frame_(EE_LINK)
        {
            /* Sottoscrizione allo stato dei giunti */
            joint_sub_ = this->create_subscription<JointStateMsg>(
                JOINT_STATES_TOPIC, rclcpp::QoS(10),
                std::bind(&CartesianLogger::joint_state_callback, this, std::placeholders::_1));

            /* Publisher per la posa cartesiana */
            pose_pub_ = this->create_publisher<PoseStampedMsg>(
                CARTESIAN_POSE_TOPIC, rclcpp::QoS(10));

            //RCLCPP_INFO(this->get_logger(), "Cartesian Publisher istanziato. In attesa di abilitazione per pubblicare su topic.");
        }

        // METODO INIT: da chiamare nel main (iniziaizzazione in differita di moveit, necessità architetturale)
        void init()
        {
            RCLCPP_INFO(this->get_logger(), "Caricamento modello robot per calcolo FK...");
            
            robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(), "robot_description");
            robot_model_ = robot_model_loader_->getModel();
            
            if (!robot_model_) {
                RCLCPP_ERROR(this->get_logger(), "Impossibile caricare il robot_model! Impossibile calcolare la FK.");
            } else {
                robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
                robot_state_->setToDefaultValues();
                RCLCPP_INFO(this->get_logger(), "Cartesian Publisher pronto.");
            }
        }


    private:
        std::atomic<bool> is_logging_; 
        std::string target_frame_;
        
        JointStateSub joint_sub_; 
        PoseStampedPub pose_pub_;

        // Oggetti MoveIt
        std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
        moveit::core::RobotModelPtr robot_model_;
        moveit::core::RobotStatePtr robot_state_;


        /* CALLBACK SUI JOINT STATES: Calcola FK ed estrae la posa reale */
        void joint_state_callback(const JointStateMsg::SharedPtr msg)
        {
            // Esce se non stiamo loggando o se il modello non è stato caricato
            if (!is_logging_ || !robot_state_) return;

            // 1. Aggiorna la configurazione interna del robot con i veri angoli correnti letti dai motori
            robot_state_->setVariablePositions(msg->name, msg->position);
            robot_state_->update(); // Ricalcola l'albero cinematico interno (Forward Kinematics)

            // 2. Ottieni la posizione cartesiana globale dell'End-Effector reale
            const Eigen::Isometry3d& ee_transform = robot_state_->getGlobalLinkTransform(target_frame_);
            
            // 3. Crea e popola il messaggio PoseStamped
            PoseStampedMsg pose_msg;
            pose_msg.header.stamp = msg->header.stamp; // Sincronizzato con il timestamp dei joint
            pose_msg.header.frame_id = robot_model_->getModelFrame(); // Frame di base (es. "world" o "base_link")
            
            // Traduzione (x, y, z)
            pose_msg.pose.position.x = ee_transform.translation().x();
            pose_msg.pose.position.y = ee_transform.translation().y();
            pose_msg.pose.position.z = ee_transform.translation().z();

            // Rotazione (quaternione)
            Eigen::Quaterniond q(ee_transform.rotation());
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            // 4. Pubblica il messaggio
            pose_pub_->publish(pose_msg);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<CartesianLogger>(node_options);
    
    node->init();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}