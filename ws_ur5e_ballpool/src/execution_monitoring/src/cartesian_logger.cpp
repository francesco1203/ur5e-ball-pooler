#include <chrono>
#include <memory>
#include <fstream>
#include <string>
#include <sstream>
#include <atomic> // AGGIUNTO per thread-safety sul flag

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp" // AGGIUNTO per ascoltare /tf

#include "interfaces_pkg/srv/log_on_file.hpp"

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"

using namespace std::chrono_literals;

class CartesianLogger : public rclcpp::Node
{
    public:
        // Alias
        using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
        using TFMessage = tf2_msgs::msg::TFMessage; // Alias per il messaggio TF
        using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;
        using LogOnFileServiceServer = rclcpp::Service<LogOnFileSrv>::SharedPtr;

        CartesianLogger() : Node("cartesian_logger"), 
            is_logging_(false),
            target_frame_(EE_LINK),
            reference_frame_(WORLD_FRAME),
            first_point_logged_(false)
        {
            /* Setup TF */
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            /* Setup Servizio richiesta di monitoring */
            log_service_ = this->create_service<LogOnFileSrv>(
                LOG_CARTESIAN_ON_OFF_SERVICE, 
                std::bind(&CartesianLogger::handle_logging_request, this, std::placeholders::_1, std::placeholders::_2));

            /* Setup Sottoscrizione Event-Driven a /tf (Sostituisce il Timer) */
            tf_sub_ = this->create_subscription<TFMessage>(
                "/tf", rclcpp::QoS(100),
                std::bind(&CartesianLogger::tf_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Cartesian Logger Event-Driven avviato. In attesa sul servizio...");
        }

        ~CartesianLogger()
        {
            if (csv_file_.is_open()) csv_file_.close();
        }

    private:
        std::atomic<bool> is_logging_; // Atomico per evitare collisioni tra callback
        std::ofstream csv_file_;
        
        std::string target_frame_;
        std::string reference_frame_;
        rclcpp::Time start_time_;
        rclcpp::Time last_tf_stamp_; 
        bool first_point_logged_;    
        
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        LogOnFileServiceServer log_service_;
        rclcpp::Subscription<TFMessage>::SharedPtr tf_sub_; // Il nostro trigger

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

                start_time_ = this->get_clock()->now();
                is_logging_ = true;
                response->logging_state_on = true;
                RCLCPP_INFO(this->get_logger(), "Logging cartesiano avviato su %s", request->filename.c_str());
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

        /* CALLBACK EVENT-DRIVEN DELLE TF (Sostituisce log_data) */
        void tf_callback(const TFMessage::SharedPtr /*msg*/)
        {
            // Se non stiamo loggando, usciamo subito senza consumare risorse
            if (!is_logging_) return;

            try {
                // Interroghiamo il buffer per ottenere l'ultima TF risolta
                TransformStampedMsg t = tf_buffer_->lookupTransform(
                    reference_frame_, target_frame_, tf2::TimePointZero);
                
                rclcpp::Time current_tf_stamp = t.header.stamp;

                // Il filtro duplicati è fondamentale qui: /tf riceve aggiornamenti per
                // TUTTI i frame del robot. Scartiamo i calcoli se la posa finale è identica.
                if (first_point_logged_ && current_tf_stamp == last_tf_stamp_) {
                    return; 
                }

                if (!first_point_logged_) {
                    start_time_ = current_tf_stamp;
                    first_point_logged_ = true;
                }

                last_tf_stamp_ = current_tf_stamp;
                double elapsed_time = (current_tf_stamp - start_time_).seconds();

                csv_file_ << elapsed_time << "," 
                          << t.transform.translation.x << "," 
                          << t.transform.translation.y << "," 
                          << t.transform.translation.z << "\n";
                          
            } catch (const tf2::TransformException & ex) {
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