#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <sstream>

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
        /* Alias */
        using JointStateMsg = sensor_msgs::msg::JointState;
        using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;

        using JointStateSub = rclcpp::Subscription<JointStateMsg>::SharedPtr;
        using LogOnFileServiceServer = rclcpp::Service<LogOnFileSrv>::SharedPtr;


        /* Builder */
        JointLogger() : Node("joint_monitor"), 
            is_logging_(false),
            ur5e_joint_names_(UR5e_JOINT_NAMES)
        {
            /* Parametri di base */
            this->declare_parameter<double>("logging_period", 0.01); // T = 0.01 s, f = 100 Hz
            double logging_period = this->get_parameter("logging_period").as_double();  

            /* Setup Subscriber Giunti */
            joint_sub_ = this->create_subscription<JointStateMsg>(
                JOINT_STATES_TOPIC, 10, std::bind(&JointLogger::joint_callback, this, std::placeholders::_1));
        
            /* Setup Servizio richiesta di monitoring */
            log_service_ = this->create_service<LogOnFileSrv>(
                LOG_JOINT_ON_OFF_SERVICE, std::bind(&JointLogger::handle_logging_request, this, std::placeholders::_1, std::placeholders::_2));

            /* Avvio del Timer di Logging a frequenza fissa */
            auto period = std::chrono::duration<double>(logging_period);
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(period),
                std::bind(&JointLogger::log_data, this));

            RCLCPP_INFO(this->get_logger(), "Joint Logger Node avviato. In attesa sul servizio 'log_joint_move'...");
        }

        ~JointLogger()
        {
            if (csv_file_.is_open()) csv_file_.close();
        }

    private:
        /* attributi */
        bool is_logging_;
        std::ofstream csv_file_;
        std::vector<std::string> ur5e_joint_names_;
        rclcpp::Time start_time_;
        
        JointStateSub joint_sub_;
        LogOnFileServiceServer log_service_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::mutex joint_mutex_;        //*
        std::map<std::string, double> current_joint_positions_;
        
        //*NOTA DI CODICE: La mappa current_joint_positions_ memorizza le posizioni correnti dei giunti.
        //                 È acceduta da due callback diverse che leggono e scrivono in maniera asimmetrica.
        //                 Per questo è protetta da un mutex per garantire l'accesso thread-safe.



        /* CALLBACKS */

        //callback di subscription dei giunti, aggiorna la mappa dei giunti correnti
        void joint_callback(const JointStateMsg::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            for (size_t i = 0; i < msg->name.size(); i++) {
                current_joint_positions_[msg->name[i]] = msg->position[i];
            }
        }

        //callback del servizio per avviare/interrompere il logging
        void handle_logging_request(const std::shared_ptr<LogOnFileSrv::Request> request,
                                    std::shared_ptr<LogOnFileSrv::Response> response)
        {
            if (request->enable) {      //voglio attivarlo
                if (is_logging_) {      //è già attivo, non faccio nulla
                    response->logging_state_on = true;
                    return; // Già in esecuzione
                }

                csv_file_.open(request->filename);  //lo attivo
                if (!csv_file_.is_open()) {         
                    response->logging_state_on = false; //se non riesce ad aprire il file, rimane disattivato
                    RCLCPP_ERROR(this->get_logger(), "Impossibile aprire %s", request->filename.c_str());
                    return;
                }

                // Scrive l'header specifico
                csv_file_ << "time_sec,q0,q1,q2,q3,q4,q5\n";

                start_time_ = this->get_clock()->now();
                is_logging_ = true;
                response->logging_state_on = true;
                RCLCPP_INFO(this->get_logger(), "Logging giunti avviato su %s", request->filename.c_str());
            } 
            else {                      // lo voglio disattivare
                if (!is_logging_) {     // era già disattivato
                    response->logging_state_on = false;
                    return; // Era già fermo
                }

                is_logging_ = false;    //disattivo
                csv_file_.close();
                response->logging_state_on = false;
                RCLCPP_INFO(this->get_logger(), "Logging giunti interrotto. File salvato.");
            }
        }

        //callback del timer per il logging dei dati dei giunti
        void log_data()
        {
            if (!is_logging_) return;

            double elapsed_time = (this->get_clock()->now() - start_time_).seconds();
            std::stringstream ss;   //concateno i valori in una riga per scriverli tutti insieme nel file, evitando di fare più scritture separate
            ss << elapsed_time;

            {
                std::lock_guard<std::mutex> lock(joint_mutex_);
                for (const auto& j_name : ur5e_joint_names_) {
                    // Se manca un giunto per qualche ragione, evitiamo di scrivere dati corrotti
                    if (current_joint_positions_.find(j_name) == current_joint_positions_.end()) return;
                    
                    ss << "," << current_joint_positions_[j_name];
                }
            }

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