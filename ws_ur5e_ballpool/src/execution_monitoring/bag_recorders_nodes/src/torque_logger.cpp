#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <sstream>
#include <atomic> // Per thread-safety sul flag, come in CartesianLogger

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "interfaces_pkg/srv/log_on_file.hpp"

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"


using namespace std::chrono_literals;


class TorqueLogger : public rclcpp::Node
{
    public:
        // Alias
        using JointStateMsg = sensor_msgs::msg::JointState;
        using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;
        using LogOnFileServiceServer = rclcpp::Service<LogOnFileSrv>::SharedPtr;

        TorqueLogger() : Node("torque_logger"),
            is_logging_(false),
            header_written_(false),
            first_point_logged_(false)
        {
            /* Setup Servizio richiesta di monitoring */
            log_service_ = this->create_service<LogOnFileSrv>(
                LOG_TORQUE_ON_OFF_SERVICE,
                std::bind(&TorqueLogger::handle_logging_request, this, std::placeholders::_1, std::placeholders::_2));

            /* Setup Sottoscrizione Event-Driven a /mujoco_actuators_states */
            joint_state_sub_ = this->create_subscription<JointStateMsg>(
                ACTUATORS_STATES_MUJOCO_TOPIC, rclcpp::QoS(100),
                std::bind(&TorqueLogger::joint_state_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Torque Logger Event-Driven avviato. In attesa sul servizio...");
        }

        ~TorqueLogger()
        {
            if (csv_file_.is_open()) csv_file_.close();
        }

    private:
        std::atomic<bool> is_logging_; // Atomico per evitare collisioni tra callback
        std::ofstream csv_file_;

        rclcpp::Time start_time_;
        bool header_written_;       // Serve per scrivere l'header CSV una sola volta, al primo messaggio utile
        bool first_point_logged_;   // Serve per agganciare start_time_ al primo header.stamp ricevuto, non al clock del nodo

        LogOnFileServiceServer log_service_;
        rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;

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

                header_written_ = false;
                start_time_ = this->get_clock()->now();
                is_logging_ = true;
                response->logging_state_on = true;
                RCLCPP_INFO(this->get_logger(), "Logging torque avviato su %s", request->filename.c_str());
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
                RCLCPP_INFO(this->get_logger(), "Logging torque interrotto. File salvato.");
            }
        }

        /* CALLBACK EVENT-DRIVEN DI /mujoco_actuators_states */
        void joint_state_callback(const JointStateMsg::SharedPtr msg)
        {
            // Se non stiamo loggando, usciamo subito senza consumare risorse
            if (!is_logging_) return;

            // Alcuni publisher pubblicano JointState senza il campo effort: scartiamo il campione se manca o non combacia in lunghezza con i nomi.
            if (msg->effort.empty() || msg->effort.size() != msg->name.size()) return;

            // Scriviamo l'header CSV al primo messaggio utile, usando i nomi dei giunti presenti nel messaggio (stesso ordine dell'array effort).
            if (!header_written_) {
                csv_file_ << "time_sec";
                for (const auto & joint_name : msg->name) {
                    csv_file_ << "," << joint_name;
                }
                csv_file_ << "\n";
                header_written_ = true;
            }

            rclcpp::Time current_stamp = msg->header.stamp;

            // Il primo messaggio utile fissa l'origine dei tempi: cosi' elapsed_time parte da 0
            // e resta coerente con il clock della sorgente del messaggio (es. tempo di simulazione),
            // senza mescolarlo con il clock di sistema del nodo.
            if (!first_point_logged_) {
                start_time_ = current_stamp;
                first_point_logged_ = true;
            }
 
            double elapsed_time = (current_stamp - start_time_).seconds();
 
            csv_file_ << elapsed_time;
            for (const auto & effort_value : msg->effort) {
                csv_file_ << "," << effort_value;
            }
            csv_file_ << "\n";

        }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueLogger>());
    rclcpp::shutdown();
    return 0;
}