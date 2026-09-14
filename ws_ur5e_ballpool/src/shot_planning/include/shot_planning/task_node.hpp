// ============================================================
//  task_node.hpp 
//  Nodo ROS2 che pianifica e ESEGUE movimenti del braccio usando MoveIt! MoveGroupInterface.
//
//  Struttura e metodi più importanti:
//    - TaskNode (classe nodo ROS2)
//        ├── moveToJointConfig    → pianifica ed esegue verso una configurazione di giunti specifica
//        ├── moveToNamedTarget()  → va a una posizione predefinita (es. "home")
//        ├── moveCartesianPath()  → va a una posa cartesiana in linea retta (cartesian path) con parametrizzazione temporale ignota
//        ├── moveCartesianPathAsymmTriangle → va a una posa cartesiana in linea retta (cartesian path) con profilo di velocità triangolare smussato (ad S) asimmetrico (Ruckig)
// ============================================================



#ifndef TASK_NODE_HPP
#define TASK_NODE_HPP

#include <memory>
#include <vector>
#include <fstream>
#include <future>

#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>

// Ruckig
#include <ruckig/ruckig.hpp>

// Servizi e Messaggi
#include <std_srvs/srv/trigger.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "geometry_msgs/msg/pose.hpp" 
#include "sensor_msgs/msg/joint_state.hpp"

// TF2
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Header custom (assicurati che i path siano corretti)
#include "shared_headers_pkg/eigen_utilities.hpp"
#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"
#include "interfaces_pkg/msg/shot_params.hpp" 
#include "interfaces_pkg/srv/log_on_file.hpp"

using joint_config = std::vector<double>; 

class TaskNode : public rclcpp::Node
{
public:
    /* Alias pubblci utili anche per l'esterno se necessario */
    using MoveGroupInterface    = moveit::planning_interface::MoveGroupInterface;
    using MoveGroupInterfacePtr = std::unique_ptr<MoveGroupInterface>;
    using Plan                  = MoveGroupInterface::Plan;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PoseMsg = geometry_msgs::msg::Pose;
    using JointStateMsg  = sensor_msgs::msg::JointState;   
    using RobotTrajectoryMsg = moveit_msgs::msg::RobotTrajectory;
    using ShotParamsMsg = interfaces_pkg::msg::ShotParams;
    using ShotParamsSubscription = rclcpp::Subscription<ShotParamsMsg>::SharedPtr;
    using TriggerSrv = std_srvs::srv::Trigger;
    using TriggerClient = rclcpp::Client<TriggerSrv>::SharedPtr;
    using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;
    using LogOnFileClient = rclcpp::Client<LogOnFileSrv>::SharedPtr;
    using TimerPtr              = rclcpp::TimerBase::SharedPtr;

    /* Costruttore */
    explicit TaskNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions());

    /* Metodi Pubblici */
    void start();
    void waitInit();
    void waitForParams();

    bool moveToJointConfig(const joint_config& joint_values, double planning_time = -1.0);
    bool moveToNamedTarget(const std::string& target_name, double planning_time = -1.0);
    
    double moveCartesianPath(const Vector3d& posizione, const Quaternion& orientamento,
                             const std::string& frame_id = WORLD_FRAME,
                             double success_execute_threshold = 0.00);
                             
    bool moveCartesianPathAsymmTriangle(const Vector3d& posizione, const Quaternion& orientamento,
                                        const std::string& frame_id = WORLD_FRAME,
                                        double vel_max = -1.0, double acceleration = -1.0, double deceleration = -1.0);

    bool ExecuteShot(const Vector3d& posizione_arresto, const Quaternion& orientamento,
                     const std::string& frame_id = WORLD_FRAME,
                     double vel_impact = -1.0, double distance_acceleration = -1.0, double distance_deceleration = -1.0);

    bool build_scene();
    bool disable_white_ball_collision();
    
    bool startLogging(const std::string& filename, bool joint_logging_enabled, bool cartesian_logging_enabled, bool torque_logging_enabled, bool controller_logging_enabled);
    bool stopLogging();

    double getDirectionAngle() const;
    double getImpactShotVelocity() const;
    double getImpactAngle() const;
    
    double getEEFDistance();
    Vector3d getEEFRelativePosition();
    
    void printEEFDebugInfo();
    void print_and_wait(const std::string & message);

private:
    /* Metodi Privati */
    void paramsCallback(const ShotParamsMsg::SharedPtr msg);
    bool send_logging_request(const std::string& filename, bool joint_logging_enabled, bool cartesian_logging_enabled, bool torque_logging_enabled, bool controller_logging_enabled);
    bool send_trigger_request(const TriggerClient& client, const std::string& service_name);

    /* Variabili Privati (Copia qui tutte le tue variabili private) */
    MoveGroupInterfacePtr move_group_; 
    TimerPtr start_timer_;             
    std::promise<void> init_done_;     
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_cli_;
    ShotParamsSubscription param_sub_;
    TriggerClient build_scene_client_;
    TriggerClient remove_white_ball_client_;
    LogOnFileClient log_client_;
    rclcpp::CallbackGroup::SharedPtr logging_cb_group_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parametri...
    double max_velocity_acceleration_scaling_factor_;   
    double goal_joint_tolerance_;                       
    double goal_position_tolerance_;                    
    double goal_orientation_tolerance_;                 
    double def_joint_planning_time_;                    
    std::string joint_planning_algorithm_;              
    double resolution_step_;                             
    double resolution_step_Ruckig_;                      
    double success_threshold_Ruckig_;                    
    double Ruckig_dt_;                                   
    double max_jerk_;                                    
    bool log_ruckig_trajectory_;                         
    std::string csv_ruckig_trajectory_path_;             
    std::promise<void> params_promise_;
    bool params_received_ = false;
    double direction_angle_deg_;
    double impact_shot_velocity_;
    double impact_angle_deg_;
    double vel_factor_for_jerk_compensation_;
    double accel_decel_factor_for_jerk_compensation_;
    char c_in; 
};

#endif // TASK_NODE_HPP