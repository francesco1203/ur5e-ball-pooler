#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard

#include <string>
#include <vector>


// Costanti specifiche del robot (UR5e)
constexpr int N_JOINTS              = 6;

const std::vector<std::string> UR5e_JOINT_NAMES = {
    "shoulder_pan_joint", 
    "shoulder_lift_joint", 
    "elbow_joint",
    "wrist_1_joint", 
    "wrist_2_joint", 
    "wrist_3_joint"
};



// limiti cartesiani spostati nel file shot_execution/shot_planning/config/cartesian_robot_limits.yaml
// Questi valori sono definiti in un file YAML separato e caricati dinamicamente.
// siccome non li conosciamo, non ha senso metterli qui come costanti


// limiti fisici cartesiani (trovati in rete)
// constexpr double MAX_TRANS_VEL = 4.0;      // velocity limit for EE translation in m/s
// constexpr double MAX_TRANS_ACC = 2.50;     // acceleration limit for EE translation in m/s^2
// constexpr double MAX_TRANS_DEC = -2.50;    // deceleration limit for EE translation in m/s^2

// limiti fisici cartesiani (l'ha detto Natale E PKKE)
// constexpr double MAX_TRANS_VEL = 5.0;      // velocity limit for EE translation in m/s
// constexpr double MAX_TRANS_ACC = 10.0;     // acceleration limit for EE translation in m/s^2
// constexpr double MAX_TRANS_DEC = -10.0;    // deceleration limit for EE translation in m/s^2


//end effector
const std::string EE_LINK = "rod_tip_virtual_link";         // link del tip dell'asta, end-effector (definito in URDF)
// const std::string EE_LINK_PHYSICAL = "cut_rod_link";      // link intera asta, per collision detection


//moveit setup
const std::string PLANNING_GROUP    = "arm";
const std::string AWAY_FROM_TABLE_CONFIG     = "discover_game_field_to_camera";   // configurazione per scostarsi dal tavolo (definita in SRDF)
const std::string READY_TO_APPROACH_CONFIG   = "ready_to_approach";               // configurazione pre-approach (definita in SRDF)

