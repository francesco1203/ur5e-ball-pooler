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


const std::string EE_LINK = "left_rod_tip_virtual_link";  // link del tip dell'asta, end-effector (definito in URDF)
//const std::string LAST_LINK         = "left_wrist_3_link";
// const std::string BASE_LINK         = "left_base_link_inertia";        



//moveit setup
const std::string PLANNING_GROUP    = "left_arm";

const std::string HOME_CONFIG       = "all_zero_home";                // configurazione di home (predefinita in SRDF)
const std::string READY_TO_APPROACH_CONFIG   = "ready_to_approach";   // configurazione pre-approach (definita in SRDF)


//per controllo di singolarità
// constexpr double SINGULARITY_THRESHOLD_WARNING = 0.01;  // soglia per considerare una configurazione di warning
// constexpr double SINGULARITY_THRESHOLD_ERROR = 0.001;   // soglia per considerare una configurazione di errore
