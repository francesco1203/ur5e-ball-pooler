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
   const std::string CONTROLLER_STATE_TOPIC = "/arm_controller/controller_state";       // topic per lo stato del controller

   //CAMERA TOPCIS
   //da realsense
   const std::string RGB_IMAGE_TOPIC = "/camera/camera/color/image_raw";                                // topic per l'immagine RGB della camera
   //const std::string DEPTH_IMAGE_TOPIC = "/camera/camera/depth/image_rect_raw";                       // topic per l'immagine di profondità della camera non allineata
   const std::string DEPTH_IMAGE_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw";             // topic per l'immagine di profondità della camera allineata a color
   const std::string CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info";                            // topic per le informazioni della camera (uguale a quelle di aligned)
   
   //per nuvola di punti (dovremmo eliminare)
   // const std::string DEPTH_POINTCLOUD_TOPIC = "/depth_pointcloud";                    // topic per la point cloud di profondità
   // const std::string DEPTH_IMAGE_VISUAL_TOPIC = "/camera/camera/depth/image_visual";  // topic per l'immagine di profondità visualizzata con mappa di colori (
   
/* SERVIZI */
   const std::string BUILD_SCENE_SERVICE = "/build_scene";  // servizio per costruire la scena (biliardo + palline)
   const std::string REMOVE_WHITE_BALL_SERVICE = "/remove_white_ball";  // servizio per rimuovere la pallina bianca

   const std::string LOG_ON_OFF_SERVICE = "/log_on_off";  // servizio per abilitare/disabilitare il logging su file (cartesian, joint, torque, controller)

   const std::string TOGGLE_GAME_ENGINE_SERVICE = "/toggle_game_engine";  // servizio per attivare/disattivare il calcolo dei parametri di tiro (motore di gioco)

