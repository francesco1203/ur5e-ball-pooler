#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "interfaces_pkg/srv/log_on_file.hpp"

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"

using namespace std::chrono_literals;

class CartesianLogger : public rclcpp::Node
{
    public:
        /* Alias */

        //messaggi
        using TransformStampedMsg = geometry_msgs::msg::TransformStamped;

        //servizio
        using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;
        using LogOnFileServiceServer = rclcpp::Service<LogOnFileSrv>::SharedPtr;

        /* Builder */
        CartesianLogger() : Node("cartesian_logger"), 
            is_logging_(false),
            target_frame_(EE_LINK),
            reference_frame_(WORLD_FRAME),
            first_point_logged_(false)
        {
            /* Parametri di base */
            this->declare_parameter<double>("logging_period", 0.01); // T = 0.01 s, f = 100 Hz
            double logging_period = this->get_parameter("logging_period").as_double();  

            /* Setup TF */
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            /* Setup Servizio richiesta di monitoring */
            log_service_ = this->create_service<LogOnFileSrv>(
                LOG_CARTESIAN_ON_OFF_SERVICE, std::bind(&CartesianLogger::handle_logging_request, this, std::placeholders::_1, std::placeholders::_2));

            /* Avvio del Timer di Logging a frequenza fissa */
            auto period = std::chrono::duration<double>(logging_period);
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(period),
                std::bind(&CartesianLogger::log_data, this));

            RCLCPP_INFO(this->get_logger(), "Cartesian Logger Node avviato. In attesa sul servizio 'log_cartesian_move'...");
        }

        ~CartesianLogger()
        {
            if (csv_file_.is_open()) csv_file_.close();
        }

    private:
        /* attributi */
        bool is_logging_;
        std::ofstream csv_file_;
        
        std::string target_frame_;
        std::string reference_frame_;
        rclcpp::Time start_time_;
        rclcpp::Time last_tf_stamp_; 
        bool first_point_logged_;    
        
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        LogOnFileServiceServer log_service_;
        rclcpp::TimerBase::SharedPtr timer_;


        /* CALLBACKS */

        //callback del servizio per avviare/interrompere il logging
        void handle_logging_request(const std::shared_ptr<LogOnFileSrv::Request> request,
                                    std::shared_ptr<LogOnFileSrv::Response> response)
        {
            if (request->enable) {      //se voglio attivare il logging
                if (is_logging_) {      //già attivo, non faccio nulla
                    response->logging_state_on = true;
                    return; // Già in esecuzione
                }

                // lo devo attivare, apro il file e scrivo l'header
                csv_file_.open(request->filename);
                if (!csv_file_.is_open()) {
                    response->logging_state_on = false;
                    RCLCPP_ERROR(this->get_logger(), "Impossibile aprire %s", request->filename.c_str());
                    return;
                }

                // Scrive l'header specifico
                csv_file_ << "time_sec,x,y,z\n";

                first_point_logged_ = false; // RESET FLAG per il primo punto

                start_time_ = this->get_clock()->now();
                is_logging_ = true;
                response->logging_state_on = true;
                RCLCPP_INFO(this->get_logger(), "Logging cartesiano avviato su %s", request->filename.c_str());
            } 
            else            //se voglio disattivare il logging
            {
                if (!is_logging_) {     //era già disattivato
                    response->logging_state_on = false;
                    return; // Era già fermo
                }

                is_logging_ = false;
                csv_file_.close();
                response->logging_state_on = false;
                RCLCPP_INFO(this->get_logger(), "Logging cartesiano interrotto. File salvato.");
            }
        }

        // Callback del timer per il logging dei dati cartesiani
        void log_data()
        {
            if (!is_logging_) return;

            try {
                TransformStampedMsg t = tf_buffer_->lookupTransform(
                    reference_frame_, target_frame_, tf2::TimePointZero);
                
                // Estraiamo il timestamp esatto in cui questa TF è stata generata dal controller
                rclcpp::Time current_tf_stamp = t.header.stamp;


                // SCARTA I DUPLICATI: se il tempo della TF è uguale a quello di prima, significa
                // che il controller non ha ancora pubblicato una nuova posa, ma ne sto leggendo una che è già stata loggata.
                // In questo caso, non salvo nulla e ritorno.
                //
                if (first_point_logged_ && current_tf_stamp == last_tf_stamp_) {
                    return; 
                }

                // SINCRONIZZAZIONE INIZIALE: se è il primo punto, allineiamo lo start_time_
                if (!first_point_logged_) {
                    start_time_ = current_tf_stamp;
                    first_point_logged_ = true;
                }

                last_tf_stamp_ = current_tf_stamp;

                //calcolo del tempo trascorso dall'inizio del logging
                double elapsed_time = (current_tf_stamp - start_time_).seconds();

                // Scrive i dati nel file CSV
                csv_file_ << elapsed_time << "," 
                          << t.transform.translation.x << "," 
                          << t.transform.translation.y << "," 
                          << t.transform.translation.z << "\n";
            } catch (const tf2::TransformException & ex) {
                // Se TF fallisce un ciclo, saltiamo
                return; 
            }
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianLogger>());
    rclcpp::shutdown();
    return 0;
}