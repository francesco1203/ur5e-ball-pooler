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
#include <limits>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <iomanip>
#include <sstream>

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
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

// Ruckig
#include <ruckig/ruckig.hpp>

// Servizi e Messaggi standard
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "geometry_msgs/msg/pose.hpp" 
#include "sensor_msgs/msg/joint_state.hpp"

// TF2
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Header custom 
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
    using SetBoolSrv = std_srvs::srv::SetBool;
    using SetBoolClient = rclcpp::Client<SetBoolSrv>::SharedPtr;
    using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;
    using LogOnFileClient = rclcpp::Client<LogOnFileSrv>::SharedPtr;
    using TimerPtr              = rclcpp::TimerBase::SharedPtr;

    /* Costruttore */
    explicit TaskNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions());

    /* Metodi Pubblici */
    void start();
    void waitInit();
    void waitForBilliardIdentification(const std::string& reference_frame = WORLD_FRAME);
    void waitForGameEngineParams();

    bool using_sim_time() const;
    
    bool moveToJointConfig(const joint_config& joint_values, double planning_time = -1.0);
    bool moveToNamedTarget(const std::string& target_name, double planning_time = -1.0);

   double moveCartesianPath(const Vector3d& posizione, 
                            const Quaternion& orientamento,
                            const std::string& frame_id = WORLD_FRAME,
                            double success_execute_threshold = 0.00
                           );        
                 
    bool moveCartesianPathAsymmTriangle(const Vector3d& posizione, const Quaternion& orientamento,
                                        const std::string& frame_id = WORLD_FRAME,
                                        double vel_max = -1.0, 
                                        double acceleration = -1.0, 
                                        double deceleration = -1.0
                                      );


    double findOptimalPitchAngle(double min_pitch_deg, 
                                 double max_pitch_deg, 
                                 double step_deg, 
                                 double robustness_delta_deg,
                                 double direction_angle_deg,
                                 double dist_backshot,
                                 double dist_arresto,
                                 double offset_z);
                                 
    void printShotParams(double vel_impact = -1.0, 
                         double distance_acceleration = -1.0, 
                         double distance_deceleration = -1.0
                        );
    
    bool ExecuteShot(const Vector3d& posizione_arresto, const Quaternion& orientamento,
                     const std::string& frame_id = WORLD_FRAME,
                     double vel_impact = -1.0,
                     double distance_acceleration = -1.0,
                     double distance_deceleration = -1.0
                    );


    void printGameMoveParams(double  direction_angle_deg_ = -1.0, 
                             double planar_impact_shot_velocity_ = -1.0, 
                             double  impact_angle_deg_ = -1.0,
                             const std::string& target_ball_color_ = "none"
                            );


    bool freeze_balls();
    bool checkRealtimeSceneIdentification(const std::string& reference_frame = WORLD_FRAME);
    

    bool build_scene();
    bool disable_white_ball_collision();
    bool enable_white_ball_collision();

    bool startLogging(const std::string& filename, bool joint_logging_enabled, bool cartesian_logging_enabled, bool controller_logging_enabled, bool wrench_logging_enabled);
    bool stopLogging();


    bool start_game_engine();
    bool stop_game_engine();
    
    double getDirectionAngle() const;
    double getPlanarImpactShotVelocity() const;
    std::string getTargetBallColor() const;
    
    double getEEFDistance();
    Vector3d getEEFRelativePosition();
    void printEEFDebugInfo();


    char print_and_wait(const std::string & message);

private:
    /* Metodi Privati */
    double planCartesianPathFromAtoB(const Vector3d& pos_A, 
                                     const Vector3d& pos_B,
                                     const Quaternion& orientamento,
                                     const std::string& frame_id = WORLD_FRAME
                                    );

    void paramsCallback(const ShotParamsMsg::SharedPtr msg);
    bool send_logging_request(const std::string& filename, bool joint_logging_enabled, bool cartesian_logging_enabled, bool controller_logging_enabled, bool wrench_logging_enabled);
    bool send_trigger_request(const TriggerClient& client, const std::string& service_name);
    bool set_game_engine_state(bool state);


    /* Variabili Privati */
    MoveGroupInterfacePtr move_group_; 
    moveit::planning_interface::MoveGroupInterface::Plan computed_plan_;
    TimerPtr start_timer_;             
    std::promise<void> init_done_;     
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_cli_;
    ShotParamsSubscription param_sub_;
    TriggerClient build_scene_client_;
    TriggerClient remove_white_ball_client_;
    TriggerClient add_white_ball_client_;
    TriggerClient freeze_balls_client_;
    LogOnFileClient log_client_;
    SetBoolClient toggle_game_engine_client_;
    rclcpp::CallbackGroup::SharedPtr logging_cb_group_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parametri...
    double max_velocity_scaling_factor_; 
    double max_acceleration_scaling_factor_;  
    double goal_joint_tolerance_;                       
    double goal_position_tolerance_;                    
    double goal_orientation_tolerance_;                 
    double def_joint_planning_time_;                    
    std::string joint_planning_algorithm_;  
    double resolution_step_;                             
    double resolution_step_Ruckig_;                      
    double success_threshold_Ruckig_;                    
    double Ruckig_dt_;        

    bool cartesian_limits_enabled_;
    double cartesian_limits_scaling_factor_;
    double max_cartesian_velocity_;
    double max_cartesian_acceleration_;
    double max_cartesian_deceleration_;
    double max_cartesian_jerk_;
                                                             
    bool log_ruckig_trajectory_;                         
    std::string csv_ruckig_trajectory_folder_path_;  

    std::mutex params_mutex_;
    std::condition_variable params_cv_;
    bool params_received_ = false;
    double direction_angle_deg_;
    double planar_impact_shot_velocity_;  //velocità che la pallina deve assumere
    double impact_shot_velocity_;         //velocità che la stecca deve avere al momento dell'impatto (calcolata in base alla geometria del tiro)
    double impact_angle_deg_;
    std::string target_ball_color_;
    
    char c_in; 
};

#endif // TASK_NODE_HPP