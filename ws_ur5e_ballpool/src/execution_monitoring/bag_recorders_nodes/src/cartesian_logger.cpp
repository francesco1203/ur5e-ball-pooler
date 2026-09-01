#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <sstream>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Librerie MoveIt per calcolo della Cinematica Diretta (FK)
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "interfaces_pkg/srv/log_on_file.hpp"

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"

using namespace std::chrono_literals;

class CartesianLogger : public rclcpp::Node
{
    public:
        using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;
        using LogOnFileServiceServer = rclcpp::Service<LogOnFileSrv>::SharedPtr;
        

        CartesianLogger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
            : Node("cartesian_logger", options), 
              is_logging_(false),
              target_frame_(EE_LINK),
              first_point_logged_(false)
        {
            /* Setup Servizio richiesta di monitoring */
            log_service_ = this->create_service<LogOnFileSrv>(
                LOG_CARTESIAN_ON_OFF_SERVICE, 
                std::bind(&CartesianLogger::handle_logging_request, this, std::placeholders::_1, std::placeholders::_2));

            /* Sottoscrizione al VERO stato dei giunti (Sostituisce /tf) */
            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                JOINT_STATES_TOPIC, rclcpp::QoS(10),
                std::bind(&CartesianLogger::joint_state_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Cartesian Logger istanziato.");
        }

        // NUOVO METODO INIT: da chiamare nel main
        void init()
        {
            RCLCPP_INFO(this->get_logger(), "Caricamento modello robot per calcolo FK...");
            
            // Qui ora shared_from_this() è sicuro da usare!
            robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(), "robot_description");
            robot_model_ = robot_model_loader_->getModel();
            
            if (!robot_model_) {
                RCLCPP_ERROR(this->get_logger(), "Impossibile caricare il robot_model! Impossibile calcolare la FK.");
            } else {
                robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
                robot_state_->setToDefaultValues();
                RCLCPP_INFO(this->get_logger(), "Cartesian Logger FK Event-Driven avviato e pronto.");
            }
        }

        ~CartesianLogger()
        {
            if (csv_file_.is_open()) csv_file_.close();
        }

    private:
        std::atomic<bool> is_logging_; 
        std::ofstream csv_file_;
        
        std::string target_frame_;
        rclcpp::Time start_time_;
        bool first_point_logged_;    
        
        LogOnFileServiceServer log_service_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_; 

        // Oggetti MoveIt
        std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
        moveit::core::RobotModelPtr robot_model_;
        moveit::core::RobotStatePtr robot_state_;

        /* CALLBACK DEL SERVIZIO*/
        void handle_logging_request(const std::shared_ptr<LogOnFileSrv::Request> request,
                                    std::shared_ptr<LogOnFileSrv::Response> response)
        {
            if (request->enable) {
                if (is_logging_) {
                    response->logging_state_on = true;
                    return;
                }

                csv_file_.open(request->filename);
                if (!csv_file_.is_open()) {
                    response->logging_state_on = false;
                    RCLCPP_ERROR(this->get_logger(), "Impossibile aprire %s", request->filename.c_str());
                    return;
                }

                csv_file_ << "time_sec,x,y,z\n";
                first_point_logged_ = false;

                is_logging_ = true;
                response->logging_state_on = true;
                RCLCPP_INFO(this->get_logger(), "Logging cartesiano (via FK) avviato su %s", request->filename.c_str());
            } 
            else 
            {
                if (!is_logging_) {
                    response->logging_state_on = false;
                    return;
                }

                is_logging_ = false;
                csv_file_.close();
                response->logging_state_on = false;
                RCLCPP_INFO(this->get_logger(), "Logging cartesiano interrotto. File salvato.");
            }
        }

        /* CALLBACK SUI JOINT STATES: Calcola FK ed estrae la posa reale */
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            // Esce se non stiamo loggando o se il modello non è stato caricato
            if (!is_logging_ || !robot_state_) return;

            // 1. Aggiorna la configurazione interna del robot con i veri angoli correnti letti dai motori
            robot_state_->setVariablePositions(msg->name, msg->position);
            robot_state_->update(); // Ricalcola l'albero cinematico interno (Forward Kinematics)

            // 2. Ottieni la posizione cartesiana globale dell'End-Effector reale
            const Eigen::Isometry3d& ee_transform = robot_state_->getGlobalLinkTransform(target_frame_);
            
            rclcpp::Time current_stamp = msg->header.stamp;

            if (!first_point_logged_) {
                start_time_ = current_stamp;
                first_point_logged_ = true;
            }

            // 3. Calcola il tempo trascorso rispetto al primo punto
            double elapsed_time = (current_stamp - start_time_).seconds();

            // 4. Salva le coordinate effettivamente percorse su file
            csv_file_ << elapsed_time << "," 
                      << ee_transform.translation().x() << "," 
                      << ee_transform.translation().y() << "," 
                      << ee_transform.translation().z() << "\n";
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    // 1. Creiamo il nodo (che genera lo shared_ptr in modo sicuro)
    auto node = std::make_shared<CartesianLogger>(node_options);
    
    // 2. Inizializziamo MoveIt ORA, dopo che lo shared_ptr esiste
    node->init();
    
    // 3. Facciamo spin del nodo
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}