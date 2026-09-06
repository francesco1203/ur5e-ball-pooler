/*NOMI PER I TOPIC, AZIONI, SERVIZI ecc.. della rete ROS2*/

#pragma once
#include <string>


/*TOPIC*/

   //created
   const std::string SHOT_PARAMS_TOPIC = "/shot_params";                            // topic per i parametri di tiro (direction_angle_deg, impact_shot_velocity)

   const std::string CARTESIAN_POSE_TOPIC = "/logging/cartesian_pose";              // topic per logging della posa cartesiana del robot (geometry_msgs/PoseStamped)

   //from robot simulators
   const std::string JOINT_STATES_TOPIC = "/joint_states";                              // topic per lo stato dei giunti del robot (sensor_msgs/JointState)
   const std::string ACTUATORS_STATES_MUJOCO_TOPIC = "/mujoco_actuators_states";        // topic per lo stato degli attuatori in MuJoCo
   const std::string CONTROLLER_STATE_TOPIC = "/left_arm_controller/controller_state";  // topic per lo stato del controller

/* SERVIZI */
   const std::string BUILD_SCENE_SERVICE = "/build_scene";  // servizio per costruire la scena (biliardo + palline)
   const std::string REMOVE_WHITE_BALL_SERVICE = "/remove_white_ball";  // servizio per rimuovere la pallina bianca

   const std::string LOG_ON_OFF_SERVICE = "/log_on_off";  // servizio per abilitare/disabilitare il logging su file (cartesian, joint, torque, controller)

