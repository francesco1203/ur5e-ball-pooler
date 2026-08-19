#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <vector>
#include <atomic>
#include <sstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "interfaces_pkg/srv/log_on_file.hpp"

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"

using namespace std::chrono_literals;

class JointLogger : public rclcpp::Node
{
    public:
        using JointStateMsg = sensor_msgs::msg::JointState;
        using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;

        JointLogger() : Node("joint_monitor"), 
            is_logging_(false),
            ur5e_joint_names_(UR5e_JOINT_NAMES),
            first_point_logged_(false)
        {
            /* Setup Subscriber Giunti Event-Driven (Trigger) */
            joint_sub_ = this->create_subscription<JointStateMsg>(
                JOINT_STATES_TOPIC, rclcpp::SensorDataQoS(), 
                std::bind(&JointLogger::joint_callback, this, std::placeholders::_1));
        
            /* Setup Servizio richiesta di monitoring */
            log_service_ = this->create_service<LogOnFileSrv>(
                LOG_JOINT_ON_OFF_SERVICE, 
                std::bind(&JointLogger::handle_logging_request, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "Joint Logger Event-Driven avviato. In attesa...");
        }

        ~JointLogger()
        {
            if (csv_file_.is_open()) csv_file_.close();
        }

    private:
        std::atomic<bool> is_logging_; 
        std::ofstream csv_file_;
        std::vector<std::string> ur5e_joint_names_;
        
        rclcpp::Time start_time_;
        bool first_point_logged_;
        
        rclcpp::Subscription<JointStateMsg>::SharedPtr joint_sub_;
        rclcpp::Service<LogOnFileSrv>::SharedPtr log_service_;

        /* CALLBACK DEL SERVIZIO */
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

                // Header aggiornato per includere posizione (q) e sforzo (tau)
                csv_file_ << "time_sec,q0,q1,q2,q3,q4,q5,tau0,tau1,tau2,tau3,tau4,tau5\n";

                first_point_logged_ = false;
                is_logging_ = true;
                response->logging_state_on = true;
                RCLCPP_INFO(this->get_logger(), "Logging giunti avviato su %s", request->filename.c_str());
            } 
            else {                      
                if (!is_logging_) {     
                    response->logging_state_on = false;
                    return; 
                }

                is_logging_ = false;    
                csv_file_.close();
                response->logging_state_on = false;
                RCLCPP_INFO(this->get_logger(), "Logging giunti interrotto. File salvato.");
            }
        }

        /* CALLBACK EVENT-DRIVEN SUI GIUNTI */
        void joint_callback(const JointStateMsg::SharedPtr msg)
        {
            if (!is_logging_) return;

            rclcpp::Time current_stamp = msg->header.stamp;

            // Sincronizzazione iniziale
            if (!first_point_logged_) {
                start_time_ = current_stamp;
                first_point_logged_ = true;
            }

            double elapsed_time = (current_stamp - start_time_).seconds();
            std::stringstream ss;
            ss << elapsed_time;

            // Vettori di buffer per mantenere l'ordine corretto
            std::vector<double> positions(6, 0.0);
            std::vector<double> efforts(6, 0.0);

            // Mappatura sicura: il broadcaster potrebbe inviare i giunti in ordine sparso
            for (size_t i = 0; i < ur5e_joint_names_.size(); ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), ur5e_joint_names_[i]);
                if (it != msg->name.end()) {
                    size_t index = static_cast<size_t>(std::distance(msg->name.begin(), it));
                    
                    if (index < msg->position.size()) positions[i] = msg->position[index];
                    
                    // Fallback a 0.0 se l'effort non è pubblicato (es. con MockHardware)
                    if (index < msg->effort.size()) efforts[i] = msg->effort[index];
                }
            }

            // Scrittura efficiente concatenata
            for (double p : positions) ss << "," << p;
            for (double tau : efforts) ss << "," << tau;

            ss << "\n";
            csv_file_ << ss.str();
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointLogger>());
    rclcpp::shutdown();
    return 0;
}