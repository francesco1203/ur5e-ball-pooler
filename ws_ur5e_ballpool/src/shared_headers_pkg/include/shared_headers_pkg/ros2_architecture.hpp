/*NOMI PER I TOPIC, AZIONI, SERVIZI ecc.. della rete ROS2*/

#pragma once
#include <string>


/*TOPIC*/

   //from simulator
    const std::string READING_JOINT_STATES_TOPIC = "/joint_states";
    const std::string PUBLISH_JOINT_COMMAND_TOPIC = "/cmd/joint_position";

    //clik e cartesian control (creati)
    const std::string CARTESIAN_DESIRED_POSE_TOPIC = "/clik/desired_cartesian_pose";
    const std::string CLIK_RESULT = "/clik/result";


/*SERVIZI*/
    const std::string CLIK_SERVICE  = "compute_ik_clik";