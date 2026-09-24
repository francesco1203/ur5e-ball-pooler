#include <rclcpp/rclcpp.hpp> 
#include <rosbag2_cpp/writer.hpp> 
#include <rosbag2_storage/storage_options.hpp> 
#include <chrono> 
#include <ctime> 
#include <sstream> 
#include <iomanip> 

// servizio custom 
#include "interfaces_pkg/srv/log_on_file.hpp" 

// libreria 
#include "shared_headers_pkg/ros2_architecture.hpp" 

// tipi di messaggio 
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <geometry_msgs/msg/twist_stamped.hpp> // <-- AGGIUNTO
#include <sensor_msgs/msg/joint_state.hpp> 
#include "control_msgs/msg/joint_trajectory_controller_state.hpp" 

#include <filesystem> 
#include <mutex> 
#include <memory> 

// NOTA: Se non hai già definito questa macro nel tuo ros2_architecture.hpp, 
// decommenta la riga seguente o aggiungila al tuo file di header.
// #define CARTESIAN_TWIST_TOPIC "/tcp_velocity"

class BagWriterNode : public rclcpp::Node { 
    public: 
        /*ALIAS*/ 
        using PoseStampedMsg = geometry_msgs::msg::PoseStamped; 
        using TwistStampedMsg = geometry_msgs::msg::TwistStamped; // <-- AGGIUNTO
        using JointStateMsg = sensor_msgs::msg::JointState; 
        using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState; 

        using LogOnFileSrv = interfaces_pkg::srv::LogOnFile; 
        using LogOnFileServiceServer = rclcpp::Service<LogOnFileSrv>::SharedPtr; 

        /* COSTRUTTORE */ 
        BagWriterNode() : Node("bag_writer_node"), is_recording_(false) { 

            // Dichiarazione dei parametri
            this->declare_parameter<std::string>("bag_base_path", "/home/francesco/log/ros2_bagdata_recovery");       
            this->declare_parameter<std::string>("bag_format", "mcap"); 
            this->declare_parameter<std::string>("test_title", "titolo_default"); 

            // 2. Inizializzazione Sottoscrizioni 
            cartesian_pose_sub_ = this->create_subscription<PoseStampedMsg>( 
                CARTESIAN_POSE_TOPIC, 10, std::bind(&BagWriterNode::cartesian_pose_cb, this, std::placeholders::_1)); // Rinominata callback
             
            // <-- AGGIUNTO: Subscriber del Twist
            cartesian_twist_sub_ = this->create_subscription<TwistStampedMsg>( 
                CARTESIAN_TWIST_TOPIC, 10, std::bind(&BagWriterNode::cartesian_twist_cb, this, std::placeholders::_1)); 

            joint_sub_ = this->create_subscription<JointStateMsg>( 
                JOINT_STATES_TOPIC, 10, std::bind(&BagWriterNode::joint_cb, this, std::placeholders::_1)); 
                 
            mujoco_sub_ = this->create_subscription<JointStateMsg>( 
                ACTUATORS_STATES_MUJOCO_TOPIC, 10, std::bind(&BagWriterNode::mujoco_cb, this, std::placeholders::_1)); 
                 
            controller_sub_ = this->create_subscription<ControllerStateMsg>( 
                CONTROLLER_STATE_TOPIC, 10, std::bind(&BagWriterNode::controller_cb, this, std::placeholders::_1)); 

            // 3. Inizializzazione Servizio 
            log_service_ = this->create_service<LogOnFileSrv>( 
                LOG_ON_OFF_SERVICE, 
                std::bind(&BagWriterNode::handle_logging_request, this, std::placeholders::_1, std::placeholders::_2)); 

            // 4. Inizializzazione Writer 
            writer_ = std::make_unique<rosbag2_cpp::Writer>(); 
            RCLCPP_INFO(this->get_logger(), "Bag Writer Node avviato e in attesa di richieste."); 
        } 

    private: 
         
        /*Variabili private*/ 
        std::unique_ptr<rosbag2_cpp::Writer> writer_; 
        std::mutex writer_mutex_; 
        bool is_recording_; 

        bool log_cartesian_ = false; 
        bool log_joints_ = false; 
        bool log_torque_ = false; 
        bool log_controller_ = false; 

        rclcpp::Service<LogOnFileSrv>::SharedPtr log_service_; 
        rclcpp::Subscription<PoseStampedMsg>::SharedPtr cartesian_pose_sub_; 
        rclcpp::Subscription<TwistStampedMsg>::SharedPtr cartesian_twist_sub_; // <-- AGGIUNTO
        rclcpp::Subscription<JointStateMsg>::SharedPtr joint_sub_; 
        rclcpp::Subscription<JointStateMsg>::SharedPtr mujoco_sub_; 
        rclcpp::Subscription<ControllerStateMsg>::SharedPtr controller_sub_; 

        /** CALLBACK DEL SERVIZIO --- */ 
        void handle_logging_request( 
            const std::shared_ptr<LogOnFileSrv::Request> req, 
            std::shared_ptr<LogOnFileSrv::Response> res)  
        { 
            std::lock_guard<std::mutex> lock(writer_mutex_); 

            bool is_enabling = req->enable_cartesian_logging || 
                               req->enable_joint_logging || 
                               req->enable_torque_logging || 
                               req->enable_controller_logging; 

            if (!is_enabling) {
                if (is_recording_) { 
                    writer_->close(); 
                    is_recording_ = false; 
                    RCLCPP_INFO(this->get_logger(), "Registrazione fermata e bag salvata."); 
                } 
                res->logging_state_on = false; 
            }  
            else
            { 
                if(req->filename.empty()) { 
                    RCLCPP_ERROR(this->get_logger(), "Nome del file di log non valido. La registrazione non può essere avviata."); 
                    res->logging_state_on = false; 
                    return; 
                } 

                if (is_recording_) { 
                    RCLCPP_WARN(this->get_logger(), "Attenzione: stavi già registrando un altro file. Il file precedente verrà chiuso."); 
                    writer_->close(); 
                } 

                std::string bag_format = this->get_parameter("bag_format").as_string(); 
                std::string base_path_str = this->get_parameter("bag_base_path").as_string(); 
                std::string test_title = this->get_parameter("test_title").as_string(); 

                auto now = std::chrono::system_clock::now(); 
                std::time_t now_time = std::chrono::system_clock::to_time_t(now); 
                std::tm tm_buf; 
                localtime_r(&now_time, &tm_buf);

                std::ostringstream timestamp_ss; 
                timestamp_ss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S"); 

                std::string filename_with_timestamp = req->filename + "_" + timestamp_ss.str(); 
                std::filesystem::path bag_path = std::filesystem::path(base_path_str) / test_title / filename_with_timestamp; 

                rosbag2_storage::StorageOptions storage_options; 
                storage_options.uri = bag_path.string(); 
                storage_options.storage_id = bag_format;            

                rosbag2_cpp::ConverterOptions converter_options; 
                converter_options.input_serialization_format = "cdr"; 
                converter_options.output_serialization_format = "cdr"; 

                try { 
                    writer_->open(storage_options, converter_options); 
                } catch (const std::exception& e) { 
                    RCLCPP_ERROR(this->get_logger(), "Impossibile aprire la bag: %s", e.what()); 
                    res->logging_state_on = false; 
                    return; 
                } 
                RCLCPP_INFO(this->get_logger(), "Bag aperta con successo. Avviata registrazione su: %s", bag_path.c_str()); 

                // Registrazione dinamica dei topic selezionati 
                if (req->enable_cartesian_logging) { 
                    register_topic(CARTESIAN_POSE_TOPIC, "geometry_msgs/msg/PoseStamped"); 
                    register_topic(CARTESIAN_TWIST_TOPIC, "geometry_msgs/msg/TwistStamped"); // <-- AGGIUNTO
                } 
                if (req->enable_joint_logging) { 
                    register_topic(JOINT_STATES_TOPIC, "sensor_msgs/msg/JointState"); 
                } 
                if (req->enable_torque_logging) { 
                    register_topic(ACTUATORS_STATES_MUJOCO_TOPIC, "sensor_msgs/msg/JointState");  
                } 
                if (req->enable_controller_logging) { 
                    register_topic(CONTROLLER_STATE_TOPIC, "control_msgs/msg/JointTrajectoryControllerState"); 
                } 

                // Salvo i flag in locale 
                log_cartesian_ = req->enable_cartesian_logging; 
                log_joints_ = req->enable_joint_logging; 
                log_torque_ = req->enable_torque_logging; 
                log_controller_ = req->enable_controller_logging; 
                 
                is_recording_ = true; 
                res->logging_state_on = true; 
            } 
        } 

        // Helper per registrare un topic nel database 
        void register_topic(const std::string& name, const std::string& type) { 
            rosbag2_storage::TopicMetadata tm; 
            tm.name = name; 
            tm.type = type; 
            tm.serialization_format = "cdr"; 
            writer_->create_topic(tm); 
        } 

        /*CALLBACK DEI TOPIC */ 
         
        void cartesian_pose_cb(const PoseStampedMsg::SharedPtr msg) { // <-- Rinominata leggermente per chiarezza
            std::lock_guard<std::mutex> lock(writer_mutex_); 
            if (is_recording_ && log_cartesian_) { 
                writer_->write(*msg, CARTESIAN_POSE_TOPIC, this->now()); 
            } 
        } 

        // <-- AGGIUNTA: Callback Twist
        void cartesian_twist_cb(const TwistStampedMsg::SharedPtr msg) { 
            std::lock_guard<std::mutex> lock(writer_mutex_); 
            if (is_recording_ && log_cartesian_) { 
                writer_->write(*msg, CARTESIAN_TWIST_TOPIC, this->now()); 
            } 
        } 

        void joint_cb(const JointStateMsg::SharedPtr msg) { 
            std::lock_guard<std::mutex> lock(writer_mutex_); 
            if (is_recording_ && log_joints_) { 
                writer_->write(*msg, JOINT_STATES_TOPIC, this->now()); 
            } 
        } 

        void mujoco_cb(const JointStateMsg::SharedPtr msg) { 
            std::lock_guard<std::mutex> lock(writer_mutex_); 
            if (is_recording_ && log_torque_) { 
                writer_->write(*msg, ACTUATORS_STATES_MUJOCO_TOPIC, this->now()); 
            } 
        } 

        void controller_cb(const ControllerStateMsg::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(writer_mutex_); 
            if (is_recording_ && log_controller_) { 
                writer_->write(*msg, CONTROLLER_STATE_TOPIC, this->now()); 
            } 
        } 
}; 

int main(int argc, char **argv) { 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<BagWriterNode>(); 

    rclcpp::executors::MultiThreadedExecutor executor; 
    executor.add_node(node); 
    executor.spin(); 
    rclcpp::shutdown(); 
    return 0; 
}