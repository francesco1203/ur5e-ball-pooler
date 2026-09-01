#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

#include "interfaces_pkg/srv/log_on_file.hpp"

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"

using namespace std::chrono_literals;

// ── ControllerStateLogger ─────────────────────────────────────────
// Logga il topic .../controller_state del JointTrajectoryController.

//   reference.position / reference.velocity → cosa il controller STA
//                                              COMANDANDO in quell'istante
//                                              (spline interna costruita a
//                                              partire dai punti Ruckig)
//   feedback.position  / feedback.velocity  → stato reale letto dagli
//                                              state_interfaces (posizione/
//                                              velocità effettiva, prima
//                                              ancora della TF)
//   error.position     / error.velocity     → differenza feedback-reference,
//                                              calcolata internamente dal JTC
//   output                                   → valore scritto sulle
//                                              command_interfaces (qui solo
//                                              position, come da tuo YAML)
//
// Confrontando "desired" con la curva ideale di Ruckig isoli se il JTC
// sta seguendo fedelmente il piano; confrontando "actual" con "desired"
// isoli se il problema è nell'esecuzione/interpolazione del controller.
// ────────────────────────────────────────────────────────────────
class ControllerStateLogger : public rclcpp::Node
{
    public:
        using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
        using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;
        using LogOnFileServiceServer = rclcpp::Service<LogOnFileSrv>::SharedPtr;

        ControllerStateLogger() : Node("controller_state_logger"),
            is_logging_(false),
            first_point_logged_(false)
        {
         
            /* Setup Servizio richiesta di monitoring (stesso pattern del CartesianLogger) */
            log_service_ = this->create_service<LogOnFileSrv>(
                LOG_CONTROLLER_STATE_ON_OFF_SERVICE,
                std::bind(&ControllerStateLogger::handle_logging_request, this, std::placeholders::_1, std::placeholders::_2));

            /* Sottoscrizione event-driven a controller_state */
            state_sub_ = this->create_subscription<ControllerStateMsg>(
                CONTROLLER_STATE_TOPIC, rclcpp::QoS(100),
                std::bind(&ControllerStateLogger::state_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Controller State Logger avviato su topic '%s'. In attesa sul servizio...",
                        CONTROLLER_STATE_TOPIC.c_str());
        }

        ~ControllerStateLogger()
        {
            if (csv_file_.is_open()) csv_file_.close();
        }

    private:
        std::atomic<bool> is_logging_;
        std::ofstream csv_file_;

        rclcpp::Time start_time_;
        bool first_point_logged_;

        LogOnFileServiceServer log_service_;
        rclcpp::Subscription<ControllerStateMsg>::SharedPtr state_sub_;

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

                // Header dinamico: joint_name_desired_pos, joint_name_actual_pos, ... per ogni giunto
                csv_file_ << "time_sec";
                header_written_ = false; // scriveremo le colonne per-giunto al primo messaggio, quando conosciamo joint_names

                first_point_logged_ = false;
                is_logging_ = true;
                response->logging_state_on = true;
                RCLCPP_INFO(this->get_logger(), "Logging controller_state avviato su %s", request->filename.c_str());
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
                RCLCPP_INFO(this->get_logger(), "Logging controller_state interrotto. File salvato.");
            }
        }

        /* CALLBACK EVENT-DRIVEN DEL CONTROLLER STATE */
        void state_callback(const ControllerStateMsg::SharedPtr msg)
        {
            if (!is_logging_) return;

            const rclcpp::Time current_stamp(msg->header.stamp);

            if (!first_point_logged_) {
                start_time_ = current_stamp;
                first_point_logged_ = true;
            }

            // Scriviamo l'header solo al primo messaggio, quando conosciamo i nomi dei giunti
            if (!header_written_) {
                for (const auto & name : msg->joint_names) {
                    csv_file_ << "," << name << "_desired_pos"
                              << "," << name << "_desired_vel"
                              << "," << name << "_actual_pos"
                              << "," << name << "_actual_vel"
                              << "," << name << "_error_pos"
                              << "," << name << "_error_vel";
                }
                csv_file_ << "\n";
                header_written_ = true;
            }
            
            double elapsed_time = (current_stamp - start_time_).seconds();
            csv_file_ << elapsed_time;
 
            const size_t n = msg->joint_names.size();
            for (size_t j = 0; j < n; ++j) {
                double reference_pos = (j < msg->reference.positions.size())  ? msg->reference.positions[j]  : 0.0;
                double reference_vel = (j < msg->reference.velocities.size()) ? msg->reference.velocities[j] : 0.0;
                double feedback_pos  = (j < msg->feedback.positions.size())   ? msg->feedback.positions[j]   : 0.0;
                double feedback_vel  = (j < msg->feedback.velocities.size())  ? msg->feedback.velocities[j]  : 0.0;
                double error_pos     = (j < msg->error.positions.size())      ? msg->error.positions[j]      : (feedback_pos - reference_pos);
                double error_vel     = (j < msg->error.velocities.size())     ? msg->error.velocities[j]     : (feedback_vel - reference_vel);
 
                csv_file_ << "," << reference_pos << "," << reference_vel
                          << "," << feedback_pos  << "," << feedback_vel
                          << "," << error_pos     << "," << error_vel;
            }
            csv_file_ << "\n";

        }

        bool header_written_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerStateLogger>());
    rclcpp::shutdown();
    return 0;
}