/*NOMI PER I TOPIC, AZIONI, SERVIZI ecc.. della rete ROS2*/

#pragma once
#include <string>


/*TOPIC*/

   //created
   const std::string SHOT_PARAMS_TOPIC = "/shot_params";  // topic per i parametri di tiro (direction_angle_deg, impact_shot_velocity) --- IGNORE ---

   //from robot simulators
   const std::string JOINT_STATES_TOPIC = "/joint_states";                              // topic per lo stato dei giunti del robot (sensor_msgs/JointState)
   const std::string ACTUATORS_STATES_MUJOCO_TOPIC = "/mujoco_actuators_states";        // topic per lo stato degli attuatori in MuJoCo
   const std::string CONTROLLER_STATE_TOPIC = "/left_arm_controller/controller_state";  // topic per lo stato del controller

/* SERVIZI */
   const std::string LOG_CARTESIAN_ON_OFF_SERVICE = "/log_cartesian_move";  // servizio per salvare i dati di movimento
   const std::string LOG_JOINT_ON_OFF_SERVICE = "/log_joint_move";          // servizio per salvare i dati di movimento
   const std::string LOG_TORQUE_ON_OFF_SERVICE = "/log_torque_on_off";
   const std::string LOG_CONTROLLER_STATE_ON_OFF_SERVICE = "/log_controller_state_on_off";  // servizio per salvare i dati di movimento
