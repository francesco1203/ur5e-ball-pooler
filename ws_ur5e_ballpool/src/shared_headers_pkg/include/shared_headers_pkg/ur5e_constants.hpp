#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard

#include <string>
#include <vector>


// Costanti specifiche del robot (UR5e)
constexpr int N_JOINTS              = 6;

const std::vector<std::string> UR5e_JOINT_NAMES = {
    "left_shoulder_pan_joint", 
    "left_shoulder_lift_joint", 
    "left_elbow_joint",
    "left_wrist_1_joint", 
    "left_wrist_2_joint", 
    "left_wrist_3_joint"
};


// limiti fisici cartesiani
constexpr double MAX_TRANS_VEL = 4.0;      // velocity limit for EE translation in m/s
constexpr double MAX_TRANS_ACC = 2.50;     // acceleration limit for EE translation in m/s^2
constexpr double MAX_TRANS_DEC = -2.50;    // deceleration limit for EE translation in m/s^2


//end effector
const std::string EE_LINK = "left_rod_tip_virtual_link";         // link del tip dell'asta, end-effector (definito in URDF)
// const std::string EE_LINK_PHYSICAL = "left_cut_rod_link";        // link intera asta, per collision detection


//moveit setup
const std::string PLANNING_GROUP    = "left_arm";

const std::string HOME_CONFIG                = "all_zero_home";       // configurazione di home (definita in SRDF)
const std::string READY_TO_APPROACH_CONFIG   = "ready_to_approach";   // configurazione pre-approach (definita in SRDF)

