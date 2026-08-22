// ============================================================
//  task_node.cpp
//  Nodo ROS2 che pianifica e ESEGUE movimenti del braccio usando MoveIt! MoveGroupInterface.
//
//  Eseguire con: ros2 run shot_planning task_node --ros-args --params-file $(ros2 pkg prefix shot_planning)/share/shot_planning/config/task_params.yaml
//
//  Struttura e metodi più importanti:
//    - TaskNode (classe nodo ROS2)
//        ├── moveToJointConfig    → pianifica ed esegue verso una configurazione di giunti specifica
//        ├── moveToNamedTarget()  → va a una posizione predefinita (es. "home")
//        ├── moveCartesianPath()  → va a una posa cartesiana in linea retta (cartesian path) con parametrizzazione temporale ignota
//        ├── moveCartesianPathAsymmTriangle → va a una posa cartesiana in linea retta (cartesian path) con profilo di velocità triangolare smussato (ad S) asimmetrico (Ruckig)
// ============================================================


#include <memory>
#include <vector>

#include <fstream>

#include <rclcpp/rclcpp.hpp>
// #include "rclcpp_action/rclcpp_action.hpp"


// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>

// Ruckig per traiettorie asimmetriche
#include <ruckig/ruckig.hpp>


#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "geometry_msgs/msg/pose.hpp" 
#include "sensor_msgs/msg/joint_state.hpp"


//tf
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// my headers
#include "shared_headers_pkg/eigen_utilities.hpp"
#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"

// my interfaces
#include "interfaces_pkg/msg/shot_params.hpp" 
#include "interfaces_pkg/srv/log_on_file.hpp"


using namespace std::chrono_literals;


using joint_config          = std::vector<double>;      //globale


//Path salvataggio file di log
const std::string LOG_CARTESIAN_PATH = "src/execution_monitoring/data/cartesian_logging/";
const std::string LOG_JOINT_PATH = "src/execution_monitoring/data/joint_logging/";
const std::string LOG_TORQUE_PATH = "src/execution_monitoring/data/torque_logging/";
const std::string LOG_CONTROLLER_PATH = "src/execution_monitoring/data/controller_logging/";
const std::string LOG_RUCKIG_PATH = "src/shot_planning/debug/ruckig_logging/";


class TaskNode : public rclcpp::Node
{
    /* alias*/

    //MoveIt
    using MoveGroupInterface    = moveit::planning_interface::MoveGroupInterface;
    using MoveGroupInterfacePtr = std::unique_ptr<MoveGroupInterface>;
    using Plan                  = MoveGroupInterface::Plan;

    //Msg
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PoseMsg = geometry_msgs::msg::Pose;
    using JointStateMsg  = sensor_msgs::msg::JointState;   
    using RobotTrajectoryMsg = moveit_msgs::msg::RobotTrajectory;

    using ShotParamsMsg = interfaces_pkg::msg::ShotParams;                          //Msg definito da me

    //Subscription
    using ShotParamsSubscription = rclcpp::Subscription<ShotParamsMsg>::SharedPtr;

    //Service di logging
    using LogOnFileSrv = interfaces_pkg::srv::LogOnFile;
    using LogOnFileClient = rclcpp::Client<LogOnFileSrv>::SharedPtr;

    //Other
    using TimerPtr              = rclcpp::TimerBase::SharedPtr;


  public:

    /* Builder */
    TaskNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
        : rclcpp::Node("task_node", opt)
    {

        //-----------------------------------------------------------------------
        /*PLANNING PARAMETERS from task_param.yaml*/ 

        //generic
        this->declare_parameter<double>("max_velocity_acceleration_scaling_factor", 0.3);               // default scaling factor
        this->declare_parameter<double>("goal_joint_tolerance", 0.001);                                 // default joint tolerance in radians (1/20 di grado)
        this->declare_parameter<double>("goal_position_tolerance", 0.0005);                              // default position tolerance in meters (0.5 mm)
        this->declare_parameter<double>("goal_orientation_tolerance", 0.01);                            // default orientation tolerance in radians (1/2 di grado)

        //moveToJointConfig e moveToNamedTarget:    
        this->declare_parameter<double>("def_joint_planning_time", 5.0);                                // default planning time in seconds
        this->declare_parameter<std::string>("joint_planning_algorithm", "RRTConnectkConfigDefault");   // def planner                        
        
        //moveCartesianPath:
        this->declare_parameter<double>("resolution_step", 0.01);                               // default resolution step in meters
        
        //moveCartesianPathAsymmTriangle
        this->declare_parameter<double>("resolution_step_Ruckig", 0.005);                        // default resolution step in meters
        this->declare_parameter<double>("success_threshold_Ruckig", 0.95);                       // default success threshold
        this->declare_parameter<double>("Ruckig_dt", 0.01);                                      // default Ruckig working step in seconds
        this->declare_parameter<double>("max_jerk", 10.0);                                       // default max jerk in m/s^3

        //shot method
        this->declare_parameter<double>("vel_factor_for_jerk_compensation", 1.0);                 // default factor to compensate for jerk
        this->declare_parameter<double>("accel_decel_factor_for_jerk_compensation", 1.0);        // default factor to compensate for jerk


        max_velocity_acceleration_scaling_factor_ = this->get_parameter("max_velocity_acceleration_scaling_factor").as_double();
        goal_joint_tolerance_ = this->get_parameter("goal_joint_tolerance").as_double();
        goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
        goal_orientation_tolerance_ = this->get_parameter("goal_orientation_tolerance").as_double();

        def_joint_planning_time_ = this->get_parameter("def_joint_planning_time").as_double();
        joint_planning_algorithm_ = this->get_parameter("joint_planning_algorithm").as_string();
        
        resolution_step_ = this->get_parameter("resolution_step").as_double();
        
        resolution_step_Ruckig_ = this->get_parameter("resolution_step_Ruckig").as_double();
        success_threshold_Ruckig_ = this->get_parameter("success_threshold_Ruckig").as_double();
        Ruckig_dt_ = this->get_parameter("Ruckig_dt").as_double();
        max_jerk_ = this->get_parameter("max_jerk").as_double();

        vel_factor_for_jerk_compensation_ = this->get_parameter("vel_factor_for_jerk_compensation").as_double();
        accel_decel_factor_for_jerk_compensation_ = this->get_parameter("accel_decel_factor_for_jerk_compensation").as_double();
        //-----------------------------------------------------------------------


        //-----------------------------------------------------------------------
        /*LOGGING AND DEBUG PARAMETERS from task_param.yaml*/ 
        this->declare_parameter<bool>("log_ruckig_trajectory", false);
        log_ruckig_trajectory_ = this->get_parameter("log_ruckig_trajectory").as_bool();
        //-----------------------------------------------------------------------
        


        /* Subscriber ai parametri di tiro */
        param_sub_ = this->create_subscription<ShotParamsMsg>(
            SHOT_PARAMS_TOPIC, 10, std::bind(&TaskNode::paramsCallback, this, std::placeholders::_1));


        /* CLIENT PER LOGGING*/
        cartesian_log_client_ = this->create_client<LogOnFileSrv>(LOG_CARTESIAN_ON_OFF_SERVICE);
        joint_log_client_ = this->create_client<LogOnFileSrv>(LOG_JOINT_ON_OFF_SERVICE);
        torque_log_client_ = this->create_client<LogOnFileSrv>(LOG_TORQUE_ON_OFF_SERVICE);
        controller_log_client_ = this->create_client<LogOnFileSrv>(LOG_CONTROLLER_STATE_ON_OFF_SERVICE);


        /*TF*/
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



        // MoveGroupInterface ha bisogno di shared_from_this(), che non è
        // disponibile dentro il costruttore → usiamo un timer one-shot (WORKAROUND INIZIALIZZAZIONE IN DIFFERITA)
        start_timer_ = this->create_wall_timer(
            100ms, std::bind(&TaskNode::start, this));
    }

    /* Metodo di inizializzazione del nodo, eseguito una sola volta dopo la costruzione del nodo.
       Inizializza MoveIt! e tutte le sue sotto-componenti. */
    void start()
    {
        start_timer_->cancel(); // esegui una sola volta
 


        /*MOVEIT setup*/
        move_group_ = std::make_unique<MoveGroupInterface>(this->shared_from_this(), PLANNING_GROUP);

        move_group_->startStateMonitor(5.0);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        move_group_->setStartStateToCurrentState();

        // Setto end effector link (tip asta) per MoveGroupInterface, altrimenti MoveIt! assume il link finale del gruppo cinematico
        move_group_->setEndEffectorLink(EE_LINK);
 

        // Rilassiamo le tolleranze degli algoritmi di pianificazione, per evitare che MoveIt! fallisca la pianificazione per piccole imprecisioni
        move_group_->setGoalJointTolerance(goal_joint_tolerance_);              
        move_group_->setGoalPositionTolerance(goal_position_tolerance_);        
        move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_);  


        // Velocità e accelerazione — scaling rispetto ai limiti massimi definiti in URDF e joint_limits.yaml
        move_group_->setMaxVelocityScalingFactor(max_velocity_acceleration_scaling_factor_);
        move_group_->setMaxAccelerationScalingFactor(max_velocity_acceleration_scaling_factor_);


        //Scelta planner da usare
        //NON LO FACCIAMO QUI, MA NEI METODI APPOSITI
      

        /* CLIENT PLANNING SCENE */
        planning_scene_diff_cli_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");


        RCLCPP_INFO(this->get_logger(), "TaskNode pronto.");
        init_done_.set_value();  // sblocca il main — init completato
    }


    /* METODI DI SINCRONIZZAZIONE DI FLUSSO*/
    //attesa nel main che moveit sia inizializzato
    void waitInit()
    {
        // Blocca il chiamante finché start() non ha completato l'inizializzazione.
        // Garantisce la sincronizzazione tra thread
        init_done_.get_future().wait();
    }
    //NOTA: waitInit() è chiamato dal main dopo la creazione del nodo, prima di qualsiasi movimento
    //      per assicurarsi che start() abbia completato l'inizializzazione di move_group_ 
    //      e altri componenti necessari, senza i quali i metodi di movimento non funzionerebbero correttamente.

     // Attesa nel main che arrivi una mossa valida
    void waitForParams() {
        params_promise_.get_future().wait();
    }


    /* METODI DI PLANNING */

    // ── moveToJointConfig ────────────────────────────────────────────────
    // Muove il robot impostando direttamente una configurazione di giunti
    // desiderata (espressa in radianti).
    //
    // INPUT:  joint_values → vettore di double contenente gli angoli dei giunti desiderati
    //         planning_time → tempo massimo di pianificazione (in secondi). Se <0, usa il valore di default
    // RETURN: true se pianificazione ed esecuzione hanno avuto successo
    bool moveToJointConfig(const joint_config& joint_values, 
                           double planning_time = -1.0)
    {
        //setting moveit
        move_group_->setPlannerId(joint_planning_algorithm_);

        // Se non è stato specificato (valore sentinella -1.0), usa il membro di classe
        double real_planning_time = (planning_time < 0.0) ? def_joint_planning_time_ : planning_time;
        move_group_->setPlanningTime(real_planning_time);

        // Recuperiamo il modello cinematico associato al gruppo per validare l'input
        auto joint_model_group = move_group_->getCurrentState()->getRobotModel()->getJointModelGroup(move_group_->getName());
        

        // Controllo di sicurezza: verifichiamo che il vettore passato abbia il numero corretto di giunti
        if (joint_values.size() != joint_model_group->getActiveJointModels().size()) {
            RCLCPP_ERROR(this->get_logger(), 
                         "Numero di giunti non valido! Forniti: %zu, richiesti dal gruppo '%s': %zu", 
                         joint_values.size(), move_group_->getName().c_str(), joint_model_group->getActiveJointModels().size());
            return false;
        }



        // Log degli angoli passati 
        std::string log_msg = "→ Planning verso configurazione giunti: [";
        for (size_t i = 0; i < joint_values.size(); ++i) {
            log_msg += std::to_string(joint_values[i]) + (i < joint_values.size() - 1 ? ", " : "");
        }
        log_msg += "]";
        RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());


        // Puliamo i target precedenti prima di impostare la nuova configurazione
        //ridondanza per robustezza codice: assicuriamoci che MoveIt! conosca la configurazione attuale del robot prima di pianificare
        move_group_->setStartStateToCurrentState();
        move_group_->clearPoseTargets();
        

        // Impostiamo la configurazione dei giunti target
        // setJointValueTarget(): MoveIt verifica all'istante (leggendo il file joint_limits.yaml)
        // se i valori passati rispettano i limiti di giunto. Se non lo fanno, restituisce false
        if (!move_group_->setJointValueTarget(joint_values)) {
            RCLCPP_WARN(this->get_logger(), "I valori dei giunti passati superano i joint limits impostati!");
            return false;
        }

        // Pianifico ed eseguo se trovo soluzione
        Plan piano;

        // Catturiamo il codice di errore dettagliato
        moveit::core::MoveItErrorCode error_code = move_group_->plan(piano);
 
        bool ok = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

        if (ok) {
            move_group_->execute(piano);
        } else {
            // Stampiamo il codice numerico esatto per capire se è collisione, IK fallita o altro
            RCLCPP_ERROR(this->get_logger(), 
                         "Pianificazione fallita per la configurazione giunti richiesta. Codice errore MoveIt: %d", 
                         error_code.val);
        }
 
        move_group_->clearPoseTargets();

        return ok;
    }

 
    // ── moveToNamedTarget ────────────────────────────────────────────────
    // Muove il robot in una configurazione predefinita per nome,
    //
    // INPUT:  target_name → nome della configurazione di giunto (es. "home")
    //         planning_time → tempo massimo di pianificazione (in secondi). Se <0, usa il valore di default
    // RETURN: true se pianificazione ed esecuzione hanno avuto successo
    bool moveToNamedTarget(const std::string& target_name, 
                           double planning_time = -1.0)
    {
        //setting moveit
        move_group_->setPlannerId(joint_planning_algorithm_);

         // Se non è stato specificato (valore sentinella -1.0), usa il membro di classe
        double real_planning_time = (planning_time < 0.0) ? def_joint_planning_time_ : planning_time;
        move_group_->setPlanningTime(real_planning_time);

        RCLCPP_INFO(this->get_logger(),
                    "→ Named target: '%s'", target_name.c_str());
 
        // Puliamo i target precedenti prima di impostare la nuova configurazione
        //ridondanza per robustezza codice: assicuriamoci che MoveIt! conosca la configurazione attuale del robot prima di pianificare
        move_group_->setStartStateToCurrentState();
        move_group_->clearPoseTargets();


        move_group_->setNamedTarget(target_name);
 
        //pianifico ed eseguo se trovo soluzione
        Plan piano;

         // Catturiamo il codice di errore dettagliato
        moveit::core::MoveItErrorCode error_code = move_group_->plan(piano);
 
        bool ok = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

        if (ok) {
            move_group_->execute(piano);
        } else {
            // Stampiamo il codice numerico esatto per capire se è collisione, IK fallita o altro
            RCLCPP_ERROR(this->get_logger(), 
                         "Pianificazione fallita per il target richiesto. Codice errore MoveIt: %d", 
                         error_code.val);
        }
 
        move_group_->clearPoseTargets();
 
        return ok;
    }


    // ── moveCartesianPath ───────────────────────────────────────────────────────
    // Muove l'end-effector IN LINEA RETTA verso una posa cartesiana specificata tramite:
    // 
    // In pratica: MoveIt fa un'interpolazione in cartesian space in linea retta, di tanti punti
    //             che distano della risoluzione,. Poi calcola la IK internamente per ogni punto 
    //             e pianifica in spazio dei joint ogni volta
    //
    // INPUT:  posizione    → Vector3d {x, y, z} rispetto a frame_id
    //         orientamento → Quaternion che descrive l'orientamento del EEF
    //         frame_id     → terna di riferimento frame esplicito (world di default)
    //         success_execute_threshold → percentuale minima di traiettoria che dev'essere eseguita (se non trova almeno questa percentuale, fallisce)
    // RETURN: percentuale di pianificazione della traiettoria, -1 se fallita interamente
    double moveCartesianPath(const Vector3d& posizione, 
                    const Quaternion& orientamento,
                    const std::string& frame_id =   WORLD_FRAME,
                    double success_execute_threshold = 0.00)        //se non indicato, default 0.00 (esegue sempre quello che trova)
    {

        // 1. Inizializza la posa originale come PoseStamped
        PoseStampedMsg pose_stamped_in;
        pose_stamped_in.header.frame_id = frame_id;
        pose_stamped_in.header.stamp = this->get_clock()->now();

        pose_stamped_in.pose.position.x = posizione.x();
        pose_stamped_in.pose.position.y = posizione.y();
        pose_stamped_in.pose.position.z = posizione.z();

        Quaternion q_norm = orientamento.normalized();
        pose_stamped_in.pose.orientation.w = q_norm.w();
        pose_stamped_in.pose.orientation.x = q_norm.x();
        pose_stamped_in.pose.orientation.y = q_norm.y();
        pose_stamped_in.pose.orientation.z = q_norm.z();

        //estraiamo Pose, che ci serve per costruire il vettore di waypoints per la pianificazione cartesiana
        PoseMsg target_pose = pose_stamped_in.pose; // Posa di default


        // 2. Moveit pianifica rispetto a world, quindi se il frame_id della posa target non è world, dobbiamo trasformare la posa nel frame di planning di MoveIt!
        std::string planning_frame_moveit = move_group_->getPlanningFrame();
        
        if (frame_id != planning_frame_moveit) {
            // RCLCPP_INFO(this->get_logger(), "Trasformazione coordinate: da '%s' a '%s'...", 
            //             frame_id.c_str(), planning_frame_moveit.c_str());
            try {
                PoseStampedMsg pose_stamped_out;
                
                // Esegue la trasformazione (con un timeout di 100ms per aspettare che l'albero tf sia pronto)
                pose_stamped_out = tf_buffer_->transform(
                    pose_stamped_in, 
                    planning_frame_moveit, 
                    tf2::durationFromSec(0.1)
                );
                
                // // -------------------------------------
                // // --- BLOCCO DI LOG ---
                // RCLCPP_INFO(this->get_logger(), 
                //             "Posa trasformata nel frame '%s': pos[%.3f, %.3f, %.3f]",
                //             planning_frame_moveit.c_str(),
                //             pose_stamped_out.pose.position.x,
                //             pose_stamped_out.pose.position.y,
                //             pose_stamped_out.pose.position.z);                         
                // // -------------------------------------

                // Aggiorna la target_pose con i valori trasformati
                target_pose = pose_stamped_out.pose;
                
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "Errore in tf2: impossibile trasformare la posa. %s", ex.what());
                return -1.0; // Interrompe il processo se non riesce a convertire
            }
        }


         // 3. Calcola la traiettoria cartesiana grezza (senza temporizzazione) usando MoveIt!

        move_group_->setStartStateToCurrentState(); 

        std::vector<PoseMsg> waypoints;
        waypoints.push_back(target_pose);

       
        RobotTrajectoryMsg raw_trajectory;
        const double eef_step = resolution_step_;      // 1 cm di risoluzione default

        // Passiamo un oggetto Constraints vuoto per poter accedere al parametro "avoid_collisions"
        moveit_msgs::msg::Constraints empty_constraints;


        double fraction = move_group_->computeCartesianPath(waypoints, 
                                                             eef_step, 
                                                             raw_trajectory,
                                                             empty_constraints,
                                                             !ignore_cartesian_collisions_);
                                                             
        RCLCPP_INFO(this->get_logger(), "Traiettoria cartesiana calcolata al %.2f%%", fraction * 100.0);

        // 4. Esecuzione se il percorso trovato è soddisfacente (superiore alla soglia di successo)
        if (fraction >= success_execute_threshold) 
        {
            // --- GESTIONE DEL TEMPO E DELLA VELOCITÀ ---
            // Convertiamo il messaggio in un oggetto robot_trajectory
            robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
            rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), raw_trajectory);

            // Applichiamo la parametrizzazione del tempo (TimeOptimalTrajectoryGeneration o Ruckig)
            // Scaliamo la velocità e l'accelerazione massima 
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            bool success = totg.computeTimeStamps(rt, 
                                                  max_velocity_acceleration_scaling_factor_, 
                                                  max_velocity_acceleration_scaling_factor_); 

            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Fallita la parametrizzazione temporale della traiettoria!");
                return -1.0;
            }

            // Riconvertiamo nel messaggio da inviare a MoveIt
            Plan piano;
            rt.getRobotTrajectoryMsg(piano.trajectory);
            
            move_group_->execute(piano);
            RCLCPP_INFO(this->get_logger(), "Esecuzione della traiettoria cartesiana completata al %.2f%%.", fraction * 100.0);
            
            return fraction; // Ritorna la percentuale di traiettoria pianificata
        } else {
            RCLCPP_ERROR(this->get_logger(), "Pianificazione cartesiana fallita o interrotta dagli ostacoli/limiti giunti al %.2f%%.", fraction * 100.0);
            return fraction; // Ritorna la percentuale di traiettoria pianificata, anche se non sufficiente
        }
    }



    // ── moveCartesianPathAsymmTriangle ─────────────────
    // Genera la linea retta cartesiana e le applica un profilo triangolare (smussato ad S-curve) usando Ruckig
    // In pratica: MoveIt fa un'interpolazione in cartesian space in linea retta, di tanti punti
    //             che distano della risoluzione. Poi Ruckig assegna ad ogni punto un tempo di arrivo
    //             così da realizzare esattamente il profilo richiesto (velocità massima, accelerazione e decelerazione asimmetriche).
    //             Sotto, è tutto implementato in spazio giunti, quindi MoveIt! calcola la IK internamente per ogni punto
    //
    // INPUT:  posizione    → Vector3d {x, y, z} rispetto a frame_id
    //         orientamento → Quaternion che descrive l'orientamento del EEF
    //         frame_id     → terna di riferimento frame esplicito (world di default)
    // RETURN: true se pianificazione ed esecuzione hanno avuto successo
    // ──────────────────────────────────────────────────────────────────────
    bool moveCartesianPathAsymmTriangle(const Vector3d& posizione, 
                                        const Quaternion& orientamento,
                                        const std::string& frame_id = WORLD_FRAME,
                                        double vel_max = -1.0,
                                        double acceleration = -1.0,
                                        double deceleration = -1.0)
    {
        // 0. controllo preliminare dei parametri
        if (deceleration <= 0.0 || acceleration <= 0.0 || vel_max <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Parametri di velocità/accelerazione/decelerazione non inseriti o non validi (inserire valori POSITIVI!)");
            return false;
        }

        
        // 1. Setup iniziale e trasformazione coordinate
        PoseStampedMsg pose_stamped_in;
        pose_stamped_in.header.frame_id = frame_id;
        pose_stamped_in.header.stamp = this->get_clock()->now();
        pose_stamped_in.pose.position.x = posizione.x();
        pose_stamped_in.pose.position.y = posizione.y();
        pose_stamped_in.pose.position.z = posizione.z();
        Quaternion q_norm = orientamento.normalized();
        pose_stamped_in.pose.orientation.w = q_norm.w();
        pose_stamped_in.pose.orientation.x = q_norm.x();
        pose_stamped_in.pose.orientation.y = q_norm.y();
        pose_stamped_in.pose.orientation.z = q_norm.z();

        PoseMsg target_pose = pose_stamped_in.pose; 

        std::string planning_frame_moveit = move_group_->getPlanningFrame();        //world
        if (frame_id != planning_frame_moveit) {
            try {
                PoseStampedMsg pose_stamped_out = tf_buffer_->transform(
                    pose_stamped_in, planning_frame_moveit, tf2::durationFromSec(0.1));
                target_pose = pose_stamped_out.pose;
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "Errore in tf2: %s", ex.what());
                return false; 
            }
        }


        // 2. Calcola i waypoint geometrici puri con MoveIt (senza tempi, solo percorso)

        // Legge lo stato corrente (reale) da /joint_states e lo imposta come punto di partenza
        move_group_->setStartStateToCurrentState(); 

        std::vector<PoseMsg> waypoints;
        waypoints.push_back(target_pose);
        RobotTrajectoryMsg raw_trajectory;
        const double eef_step = resolution_step_Ruckig_; // (più fitto è meglio per l'interpolazione)
        

        // Passiamo un oggetto Constraints vuoto per poter accedere al parametro "avoid_collisions"
        moveit_msgs::msg::Constraints empty_constraints;


        double fraction = move_group_->computeCartesianPath(waypoints, 
                                                            eef_step, 
                                                            raw_trajectory,
                                                            empty_constraints,
                                                            !ignore_cartesian_collisions_);
                                                            
        RCLCPP_INFO(this->get_logger(), "Traiettoria geometrica calcolata al %.2f%%", fraction * 100.0);

        if (fraction < success_threshold_Ruckig_) {
            RCLCPP_ERROR(this->get_logger(), "Pianificazione cartesiana interrotta. Ostacoli o limiti cinematici.");
            return false;
        }


        // --- 3. PARAMETRIZZAZIONE TEMPORALE ASIMMETRICA CON RUCKIG  ---
        const size_t num_waypoints = raw_trajectory.joint_trajectory.points.size();
        if (num_waypoints < 2) return false;            //se ci sono meno di due punti sulla traiettoria, non ha senso parametrizzare il tempo

        //const double total_distance = (num_waypoints - 1) * eef_step;       //numero di stemp per risoluzione step = distanza totale percorsa lungo la linea retta

        double total_distance = 0.0;
        // Se vuoi usare la distanza euclidea pura dal target (assumendo linea retta perfetta):
        double dx = target_pose.position.x - move_group_->getCurrentPose().pose.position.x;
        double dy = target_pose.position.y - move_group_->getCurrentPose().pose.position.y;
        double dz = target_pose.position.z - move_group_->getCurrentPose().pose.position.z;
        total_distance = std::sqrt(dx*dx + dy*dy + dz*dz);


        //---------------DEBUG RUCKIG--------------------
        //RCLCPP_INFO(this->get_logger(), "Distanza totale lungo la linea retta: %.4f m", total_distance);
        //---------------DEBUG RUCKIG--------------------



        // Impostiamo Ruckig per uno spazio a 1-Dimensione (lungo la linea del tiro)
        // dt a 0.01s (100Hz) è sufficiente per generare una traiettoria fluida
        const double dt = Ruckig_dt_;            //passo di lavoro di Ruckig in secondi (default 0.01s)
        ruckig::Ruckig<1> otg(dt);              //optimal trajectory generator per 1D
        ruckig::InputParameter<1> input;
        ruckig::OutputParameter<1> output;

        // Partiamo da fermi
        input.current_position = {0.0};
        input.current_velocity = {0.0};
        input.current_acceleration = {0.0};

        // Vogliamo arrivare fermi alla fine della linea
        input.target_position = {total_distance};
        input.target_velocity = {0.0};
        input.target_acceleration = {0.0};

        // I LIMITI ASIMMETRICI: La magia per il colpo di biliardo
        input.max_velocity = {vel_max};             // (m/s) Velocità massima raggiunta all'impatto
        input.max_acceleration = {acceleration};    // (m/s^2) Accelerazione di carica: lenta e progressiva
        input.min_acceleration = {-deceleration};   // (m/s^2) Decelerazione: frenata brusca e immediata dopo l'impatto
        input.max_jerk = {max_jerk_};               // (m/s^3) Limite dello strattone

        // Prepariamo il messaggio finale
        moveit_msgs::msg::RobotTrajectory timed_traj;
        timed_traj.joint_trajectory.joint_names = raw_trajectory.joint_trajectory.joint_names;

        //ruckig lavora a stati, iterativamente 
        ruckig::Result result = ruckig::Result::Working;
        double current_time = 0.0;

        //parametri del ciclo di lavoro
        double s = output.new_position[0];          // Posizione corrente lungo la linea (metri)
        double v = output.new_velocity[0];          // Velocità corrente lungo la linea (metri/secondo)
        double a = output.new_acceleration[0];      // Accelerazione corrente lungo la linea (metri/secondo^2)

        //se è richiesto il log di ruckig
        std::ofstream ruckig_log_file;

        if (log_ruckig_trajectory_) {    
            ruckig_log_file.open(LOG_RUCKIG_PATH + "ruckig_trajectory_log.csv");
    
            if (ruckig_log_file.is_open()) {
                // Scriviamo l'intestazione del CSV
                ruckig_log_file << "Time_s,Position_m,Velocity_ms,Acceleration_ms2\n";

                //Scriviamo il primo stato iniziale
                ruckig_log_file << current_time << "," 
                                << s << "," 
                                << v << "," 
                                << a << "\n";
            } else {
                RCLCPP_WARN(this->get_logger(), "Impossibile aprire il file di log per Ruckig");
            }
        }

        // Generiamo i punti temporali passo-passo
        while (result == ruckig::Result::Working) {
            
            result = otg.update(input, output);    //stato ottimale al prossimo dt (attuale nel ciclo)

            s = output.new_position[0];          // Posizione corrente lungo la linea (metri)
            v = output.new_velocity[0];          // Velocità corrente lungo la linea (m/s)
            a = output.new_acceleration[0];      // Accelerazione corrente lungo la linea (m/s^2)

            // SCRIVIAMO I DATI SUL FILE CSV
            if (ruckig_log_file.is_open()) {
                ruckig_log_file << current_time + dt << "," 
                                << s << "," 
                                << v << "," 
                                << a << "\n";
            }

            // Troviamo quali waypoint di MoveIt corrispondono alla posizione "s"
            double exact_idx = (s / total_distance) * (num_waypoints - 1);
            size_t idx_low = std::floor(exact_idx);
            size_t idx_high = std::ceil(exact_idx);

            // Evitiamo overflow per approssimazioni in virgola mobile
            if (idx_low >= num_waypoints) idx_low = num_waypoints - 1;
            if (idx_high >= num_waypoints) idx_high = num_waypoints - 1;

            double t_interp = exact_idx - idx_low; // Valore tra [0, 1] per l'interpolazione

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.time_from_start = rclcpp::Duration::from_seconds(current_time);

            // Mappiamo lo stato cartesiano 1D nei 6 giunti dell'UR5e
            for (size_t j = 0; j < timed_traj.joint_trajectory.joint_names.size(); ++j) {
                double q_low = raw_trajectory.joint_trajectory.points[idx_low].positions[j];
                double q_high = raw_trajectory.joint_trajectory.points[idx_high].positions[j];
                
                // Interpolazione lineare della posizione del giunto
                double q = q_low + t_interp * (q_high - q_low);
                pt.positions.push_back(q);

                // Regola della catena per la velocità del giunto: dq/dt = (dq/ds) * (ds/dt)
                double dq_ds = (idx_high == idx_low) ? 0.0 : (q_high - q_low) / eef_step;
                pt.velocities.push_back(dq_ds * v);
            }

            timed_traj.joint_trajectory.points.push_back(pt);

            // Prepariamo lo step successivo
            output.pass_to_input(input);
            current_time += dt;
        }

        if (ruckig_log_file.is_open()) {
            ruckig_log_file.close();
            RCLCPP_INFO(this->get_logger(), "Profilo ideale di Ruckig salvato");
        }

        // 4. Esecuzione
        Plan piano;
        piano.trajectory = timed_traj;
        
        RCLCPP_INFO(this->get_logger(), "Esecuzione tiro asimmetrico in corso...");
        move_group_->execute(piano);
        return true;
    }



    /* METODI PER IL TIRO */
    bool ExecuteShot(const Vector3d& posizione_arresto,        //fine tiro, dove si ferma
                     const Quaternion& orientamento,
                     const std::string& frame_id = WORLD_FRAME,
                     double vel_impact = -1.0,
                     double distance_acceleration = -1.0,
                     double distance_deceleration = -1.0)
    {
       // 0. controllo preliminare dei parametri
        if (distance_deceleration <= 0.0 || distance_acceleration <= 0.0 || vel_impact <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Parametri di velocità d'impatto e distanza di accelerazione/decelerazione non inseriti o non validi (inserire valori POSITIVI!)");
            return false;
        }


        // 1. calcolo, accelerazione e decelerazione in base alle distanze fornite per il profilo triangolare
        double accel = vel_impact * vel_impact / (2.0 * distance_acceleration); // a = v^2 / (2 * d)
        double decel = vel_impact * vel_impact / (2.0 * distance_deceleration); // a = v^2 / (2 * d)


        // 2. faccio un controllo di fattibilità in base ai limiti fisici del robot
        double vel_limite = MAX_TRANS_VEL * max_velocity_acceleration_scaling_factor_;
        double accel_limite = MAX_TRANS_ACC * max_velocity_acceleration_scaling_factor_;
        double decel_limite = abs(MAX_TRANS_DEC) * max_velocity_acceleration_scaling_factor_;


        RCLCPP_INFO(this->get_logger(), "\n\n--------------------PARAMETRI DEL TIRO--------------------");

        if (vel_impact < vel_limite) {
            RCLCPP_INFO(this->get_logger(), 
                "Velocità di tiro: %.3f m/s (< limite = %.3f m/s con scaling = %.2f usato)", 
                vel_impact, vel_limite, max_velocity_acceleration_scaling_factor_);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Limite di velocità superato: %.3f m/s > limite = %.3f m/s (con scaling = %.2f usato)", 
                vel_impact, vel_limite, max_velocity_acceleration_scaling_factor_);

                return false;
        }

        if (accel < accel_limite) {
            RCLCPP_INFO(this->get_logger(), 
                "Accelerazione di tiro: %.3f m/s^2 < limite = %.3f m/s^2 (con scaling = %.2f usato)", 
                accel, accel_limite, max_velocity_acceleration_scaling_factor_);
        }
        else{
            RCLCPP_ERROR(this->get_logger(), 
                "Limite di accelerazione superato: %.3f m/s^2 > limite = %.3f m/s^2 (con scaling = %.2f usato)", 
                accel, accel_limite, max_velocity_acceleration_scaling_factor_);
            
            return false;
        }

        if (decel < decel_limite) {
            RCLCPP_INFO(this->get_logger(), 
                "Decelerazione di tiro: %.3f m/s^2 < limite = %.3f m/s^2 (con scaling = %.2f usato)", 
                decel, decel_limite, max_velocity_acceleration_scaling_factor_);
        }
        else{
            RCLCPP_ERROR(this->get_logger(), 
                "Limite di decelerazione superato: %.3f m/s^2 > limite = %.3f m/s^2 (con scaling = %.2f usato)", 
                decel, decel_limite, max_velocity_acceleration_scaling_factor_);

            return false;
        }

        RCLCPP_INFO(this->get_logger(), "\n--------------------PARAMETRI DEL TIRO--------------------\n");


        // 3. eseguo il tiro con profilo asimmetrico
        return moveCartesianPathAsymmTriangle(posizione_arresto, orientamento, frame_id, 
                                              vel_impact * vel_factor_for_jerk_compensation_, 
                                              accel * accel_decel_factor_for_jerk_compensation_, 
                                              decel * accel_decel_factor_for_jerk_compensation_); 
                                              
    }


    /* GESTIONE COLLISIONI */
    bool disable_collision() 
    { 
        RCLCPP_INFO(this->get_logger(), "Disattivazione collisioni...");
        ignore_cartesian_collisions_ = true;
        return true;
    }
    
    bool enable_collision() 
    { 
        RCLCPP_INFO(this->get_logger(), "Riattivazione controlli di collisione ambientali...");
        ignore_cartesian_collisions_ = false;
        return true;
    }


    /*SERVIZI DI MONITORING DEI TIRI*/
    bool startCartesianLogging(const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Avvio logging cartesiano...");
        bool cart_ok = send_logging_request(cartesian_log_client_, true, filename);
        
        return cart_ok;
    }

    bool startJointLogging(const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Avvio logging giunti...");
        bool joint_ok = send_logging_request(joint_log_client_, true, filename);
        
        return joint_ok;
    }

    bool stopCartesianLogging()
    {
        RCLCPP_INFO(this->get_logger(), "Arresto logging cartesiano...");
        // Passiamo una stringa vuota per il file, dato che disattivando non serve
        bool cart_ok = send_logging_request(cartesian_log_client_, false, "");
        
        return cart_ok;
    }

    bool stopJointLogging()
    {
        RCLCPP_INFO(this->get_logger(), "Arresto logging giunti...");
        // Passiamo una stringa vuota per il file, dato che disattivando non serve
        bool joint_ok = send_logging_request(joint_log_client_, false, "");
        
        return joint_ok;
    }

    bool startTorqueLogging(const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Avvio logging coppie di giunto...");
        bool torque_ok = send_logging_request(torque_log_client_, true, filename);
        
        return torque_ok;
    }

    bool stopTorqueLogging()
    {
        RCLCPP_INFO(this->get_logger(), "Arresto logging coppie di giunto...");
        bool torque_ok = send_logging_request(torque_log_client_, false, "");
        return torque_ok;
    }

    bool startControllerLogging(const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Avvio logging controller...");
        bool ctrl_ok = send_logging_request(controller_log_client_, true, filename);
        
        return ctrl_ok;
    }

    bool stopControllerLogging()
    {
        RCLCPP_INFO(this->get_logger(), "Arresto logging controller...");
        bool ctrl_ok = send_logging_request(controller_log_client_, false, "");
        return ctrl_ok;
    }

    /*ALTRI METODI DI UTILITIES*/

    /*metodi getter*/
    double getDirectionAngle() const { return direction_angle_deg_; }
    double getImpactShotVelocity() const { return impact_shot_velocity_; }
    double getImpactAngle() const { return impact_angle_deg_; }


    /* PER CALCOLO GEOMETRICO */

    // calcolo della distanza tra il tip dell'asta e il centro della pallina bianca
    double getEEFDistance()
    {
        try {
            // Cerchiamo la trasformazione dal sistema di riferimento della pallina a quello della punta della stecca.
            // In altre parole: "Dove si trova EE_LINK rispetto a WHITE_SOLID_BALL_FRAME?"
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                WHITE_SOLID_BALL_FRAME, // target frame (il centro della pallina)
                EE_LINK,                // source frame (il tip dell'asta)
                tf2::TimePointZero,       // prendi l'ultima trasformazione disponibile
                tf2::durationFromSec(1.0) // timeout
            );

            double x = t.transform.translation.x;
            double y = t.transform.translation.y;
            double z = t.transform.translation.z;
            
            // Calcolo la distanza euclidea (modulo del vettore)
            double distance = std::sqrt(x*x + y*y + z*z);
            return distance;
                
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), 
                "Impossibile leggere la TF tra pallina ed EE_LINK: %s", 
                ex.what());
            return -1.0; // ritorna un valore negativo per indicare errore
        }
    }


    // Calcolo della posizione relativa tra tip dell'asta e pallina bianca (in formato Eigen::Vector3d)
    Vector3d getEEFRelativePosition()
    {
        try {
            // Cerchiamo la trasformazione dal sistema di riferimento della pallina a quello della punta della stecca.
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                WHITE_SOLID_BALL_FRAME, // target frame (il centro della pallina)
                EE_LINK,                // source frame (il tip dell'asta)
                tf2::TimePointZero,       // prendi l'ultima trasformazione disponibile
                tf2::durationFromSec(1.0) // timeout
            );

            double x = t.transform.translation.x;
            double y = t.transform.translation.y;
            double z = t.transform.translation.z;
            
            // Restituisce direttamente un Vector3d di Eigen
            return Vector3d(x, y, z);
                
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), 
                "Impossibile leggere la TF tra pallina ed EE_LINK: %s", 
                ex.what());
            // Ritorna un vettore con valori NaN (Not a Number) per indicare chiaramente l'errore
            return Vector3d(Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
        }
    }


    /* PER DEBUG*/
    // Metodo di utilità per stampare informazioni sulla poszione dell'EE rispetto la pallina
    void printEEFDebugInfo()
    {
        double distance = getEEFDistance();
        Vector3d rel_pos = getEEFRelativePosition();

        // Stampiamo i dati usando il logger se non ci sono errori
        if (distance >= 0.0 && !rel_pos.hasNaN()) {
            RCLCPP_INFO(this->get_logger(), 
                "[DEBUG EEF] Distanza: %.4f m | Posizione relativa -> X: %.4f, Y: %.4f, Z: %.4f", 
                distance, rel_pos.x(), rel_pos.y(), rel_pos.z());
        } else {
            RCLCPP_WARN(this->get_logger(), 
                "[DEBUG EEF] Impossibile recuperare correttamente la distanza o la posizione relativa del tip dell'asta.");
        }
    }


    /* PER CONTROLLO ESECUZIONE*/
    // Metodo di utilità per stampare un messaggio e aspettare l'input da terminale
    void print_and_wait(const std::string & message)
    {
        // Stampa il messaggio sul logger ROS2
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        RCLCPP_INFO(this->get_logger(), "Premi un tasto e INVIO per continuare...");

        std::cin >> c_in;   // blocca finché l'utente non digita qualcosa e preme INVIO
        std::cin.ignore(); // pulisce il '\n' rimasto nel buffer
    }

   

  private:
    MoveGroupInterfacePtr move_group_; // interfaccia MoveIt! per il gruppo "left_arm"
    TimerPtr start_timer_;             // timer one-shot per inizializzazione differita
    std::promise<void> init_done_;     // segnala al main che start() è completato

    // Client per modificare la scena di pianificazione
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_cli_;

    //subscriber a ShotParam (da game engine)
    ShotParamsSubscription param_sub_;

    // loggin service client
    LogOnFileClient cartesian_log_client_;
    LogOnFileClient joint_log_client_;
    LogOnFileClient torque_log_client_;
    LogOnFileClient controller_log_client_;

    // tf2_ros
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    /* PARAMETRI DI PIANIFICAZIONE */
    double max_velocity_acceleration_scaling_factor_;   // fattore di scala per velocità e accelerazione
    double goal_joint_tolerance_;                       // tolleranza di giunto in obiettivi di giunto (radians)
    double goal_position_tolerance_;                    // tolleranza di posizione in obiettivi cartesiani (metri)
    double goal_orientation_tolerance_;                 // tolleranza di orientamento in obiettivi cartesiani (radians)
   
    double def_joint_planning_time_;                    // tempo di pianificazione di default (secondi)
    std::string joint_planning_algorithm_;              // planner per pianificazione in spazio giunti
    
    double resolution_step_;                             // passo di risoluzione per pianificazione cartesian path (metri)
    
    double resolution_step_Ruckig_;                      // passo di risoluzione per pianificazione cartesian path con Ruckig (metri)
    double success_threshold_Ruckig_;                    // soglia di successo per pianificazione cartesian path con Ruckig (0.0 - 1.0)
    double Ruckig_dt_;                                   // passo di lavoro per Ruckig (secondi)
    double max_jerk_;                                    // limite di jerk per pianificazione cartesian path con Ruckig (m/s^3)

    //parametri di loggin e debug
    bool log_ruckig_trajectory_;                         // flag per abilitare/disabilitare il logging della traiettoria generata da Ruckig


    //lettura da subscriber per mossa da motore di gioco
    std::promise<void> params_promise_;
    bool params_received_ = false;
    double direction_angle_deg_;
    double impact_shot_velocity_;
    double impact_angle_deg_;
    double vel_factor_for_jerk_compensation_;
    double accel_decel_factor_for_jerk_compensation_;


    //temporanee
    char c_in; // variabile per input da terminale (usata in print_and_wait)
    bool ignore_cartesian_collisions_ = false; // Flag per abilitare/disabilitare collisioni nel tiro



    /*METODI PRIVATI*/

    /* CALLBACKS */

    // Callback subscriber che riceve i parametri
    void paramsCallback(const ShotParamsMsg::SharedPtr msg) {
        if (!params_received_) {
            direction_angle_deg_ = msg->direction_angle_deg;
            impact_shot_velocity_ = msg->impact_shot_velocity;
            impact_angle_deg_ = msg->impact_angle_deg;
            params_received_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Parametri ricevuti: Angle=%.2f, Velocity=%.2f, Pitch=%.2f", 
                        direction_angle_deg_, impact_shot_velocity_, impact_angle_deg_);
            // Sblocca il main se stavamo aspettando
            params_promise_.set_value();
        }
    }


    /*ALTRO DI UTILITIES*/
    
    // Funzione helper interna per chiamare il servizio
    //
    // input: client → client del servizio a cartesian_logger o joint_logger
    //        enable → true per abilitare il logging, false per disabilitare
    //        filename → nome del file di log (se enable=true)
    // output: true se la richiesta è stata completata con successo, false altrimenti
    bool send_logging_request(LogOnFileClient& client, bool enable, const std::string& filename)
    {
        // Aspettiamo che il servizio sia disponibile (max 1 secondo)
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Servizio di logging non disponibile!");
            return false;
        }

        // Creiamo la richiesta
        auto request = std::make_shared<LogOnFileSrv::Request>();
        request->enable = enable;
        request->filename = filename;

        // Inviamo la richiesta in modo asincrono
        auto future = client->async_send_request(request);

        // Aspettiamo la risposta (sicuro da fare qui perché lo chiamiamo dal main e lo spinner gira in un altro thread)
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto response = future.get();
            if (response->logging_state_on != enable)   //se si trova nello stato diverso da quello richiesto
            {
                RCLCPP_WARN(this->get_logger(), "La richiesta non è andata a buon fine");
                return false;
            }
            return true;
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout in attesa della risposta");
            return false;
        }
    }


};



// ── main ────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    /* inizializzazione */
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<TaskNode>();
 
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });
 


    // Aspetta che start() abbia completato l'inizializzazione di moveit — sincronizzazione
    node->waitInit();
    //adesso sono sicuro che start() ha inizializzato move_group_ e posso chiamare i metodi di movimento



    //------------------------------------------------------
    /* LETTURA MOSSA DI GIOCO*/
    RCLCPP_INFO(node->get_logger(), "In attesa che arrivino i parametri di tiro...");

    // Aspetta che arrivi qualcosa sui topic di parametri di tiro (da motore di gioco)
    node->waitForParams();  

    // Adesso posso usarli
    double direction_angle_deg_ = node->getDirectionAngle(); 
    double impact_shot_velocity_ = node->getImpactShotVelocity();
    double impact_angle_deg_ = node->getImpactAngle();
    //------------------------------------------------------


    //------------------------------------------------------
    /* SHOT PLANNING PARAMETERS */
    node->declare_parameter<double>("approach_distance_from_ball_surface", 0.02);
    node->declare_parameter<double>("shooting_distance_from_ball_surface", 0.05);
    node->declare_parameter<double>("distance_deceleration_phase_fraction_radius", 2.0);
    // node->declare_parameter<double>("impact_angle_deg", 10.0);           //da motore di gioco
    // node->declare_parameter<double>("direction_angle_deg", 0.0);         //da motore di gioco
    // node->declare_parameter<double>("impact_shot_velocity", 0.1);        //da motore di gioco
    node->declare_parameter<double>("offset_correction_center_z", 0.000);
    node->declare_parameter<double>("elevation_escape", 0.05);
    node->declare_parameter<double>("success_threshold_approach", 0.99);
    node->declare_parameter<double>("success_threshold_back", 0.20);


    double approach_distance_from_ball_surface_ = node->get_parameter("approach_distance_from_ball_surface").as_double();
    double shooting_distance_from_ball_surface_ = node->get_parameter("shooting_distance_from_ball_surface").as_double();
    double distance_deceleration_phase_fraction_radius_ = node->get_parameter("distance_deceleration_phase_fraction_radius").as_double();
    // double impact_angle_deg_ = node->get_parameter("impact_angle_deg").as_double();                  //da motore di gioco
    // double direction_angle_deg_ = node->get_parameter("direction_angle_deg").as_double();            //da motore di gioco
    // double impact_shot_velocity_ = node->get_parameter("impact_shot_velocity").as_double();          //da motore di gioco
    double offset_correction_center_z_ = node->get_parameter("offset_correction_center_z").as_double();
    double elevation_escape_ = node->get_parameter("elevation_escape").as_double();
    double success_threshold_approach_ = node->get_parameter("success_threshold_approach").as_double();
    double success_threshold_back_ = node->get_parameter("success_threshold_back").as_double();
    //------------------------------------------------------


    //------------------------------------------------------
    /*CONTROL EXECUTION PARAMETERS*/
    node->declare_parameter<bool>("control_execution_by_user_input", true);
    node->declare_parameter<bool>("using_mujoco_simulation", false);
    node->declare_parameter<int>("mujoco_sync_pause_time_milliseconds", 800);

    bool control_execution_by_user_input_ = node->get_parameter("control_execution_by_user_input").as_bool();
    bool using_mujoco_simulation_ = node->get_parameter("using_mujoco_simulation").as_bool();
    int mujoco_sync_pause_time_milliseconds_ = node->get_parameter("mujoco_sync_pause_time_milliseconds").as_int();
    //------------------------------------------------------


    //------------------------------------------------------
    /*MONITORING PARAMETERS debug + logging on file*/

    node->declare_parameter<bool>("print_EEF_distance_and_position", true);
    bool print_EEF_distance_and_position_ = node->get_parameter("print_EEF_distance_and_position").as_bool();

    node->declare_parameter<bool>("cartesian_logging_enabled_1", false);
    node->declare_parameter<bool>("cartesian_logging_enabled_2", false);
    node->declare_parameter<bool>("cartesian_logging_enabled_3", false);
    node->declare_parameter<bool>("cartesian_logging_enabled_4", false);
    node->declare_parameter<bool>("cartesian_logging_enabled_5", false);
    node->declare_parameter<bool>("joints_logging_enabled_1", false);
    node->declare_parameter<bool>("joints_logging_enabled_2", false);
    node->declare_parameter<bool>("joints_logging_enabled_3", false);
    node->declare_parameter<bool>("joints_logging_enabled_4", false);
    node->declare_parameter<bool>("joints_logging_enabled_5", false);
    node->declare_parameter<bool>("torque_logging_enabled_1", false);
    node->declare_parameter<bool>("torque_logging_enabled_2", false);
    node->declare_parameter<bool>("torque_logging_enabled_3", false);
    node->declare_parameter<bool>("torque_logging_enabled_4", false);
    node->declare_parameter<bool>("torque_logging_enabled_5", false);
    node->declare_parameter<bool>("controller_logging_enabled_1", false);
    node->declare_parameter<bool>("controller_logging_enabled_2", false);
    node->declare_parameter<bool>("controller_logging_enabled_3", false);
    node->declare_parameter<bool>("controller_logging_enabled_4", false);
    node->declare_parameter<bool>("controller_logging_enabled_5", false);

    bool cartesian_logging_enabled_1 = node->get_parameter("cartesian_logging_enabled_1").as_bool();
    bool cartesian_logging_enabled_2 = node->get_parameter("cartesian_logging_enabled_2").as_bool();
    bool cartesian_logging_enabled_3 = node->get_parameter("cartesian_logging_enabled_3").as_bool();
    bool cartesian_logging_enabled_4 = node->get_parameter("cartesian_logging_enabled_4").as_bool();
    bool cartesian_logging_enabled_5 = node->get_parameter("cartesian_logging_enabled_5").as_bool();
    bool joints_logging_enabled_1 = node->get_parameter("joints_logging_enabled_1").as_bool();
    bool joints_logging_enabled_2 = node->get_parameter("joints_logging_enabled_2").as_bool();
    bool joints_logging_enabled_3 = node->get_parameter("joints_logging_enabled_3").as_bool();
    bool joints_logging_enabled_4 = node->get_parameter("joints_logging_enabled_4").as_bool();
    bool joints_logging_enabled_5 = node->get_parameter("joints_logging_enabled_5").as_bool(); 
    bool torque_logging_enabled_1 = node->get_parameter("torque_logging_enabled_1").as_bool() && using_mujoco_simulation_; //solo se sto usando mujoco, altrimenti non funziona
    bool torque_logging_enabled_2 = node->get_parameter("torque_logging_enabled_2").as_bool() && using_mujoco_simulation_;
    bool torque_logging_enabled_3 = node->get_parameter("torque_logging_enabled_3").as_bool() && using_mujoco_simulation_;
    bool torque_logging_enabled_4 = node->get_parameter("torque_logging_enabled_4").as_bool() && using_mujoco_simulation_;
    bool torque_logging_enabled_5 = node->get_parameter("torque_logging_enabled_5").as_bool() && using_mujoco_simulation_;
    bool controller_logging_enabled_1 = node->get_parameter("controller_logging_enabled_1").as_bool();
    bool controller_logging_enabled_2 = node->get_parameter("controller_logging_enabled_2").as_bool();
    bool controller_logging_enabled_3 = node->get_parameter("controller_logging_enabled_3").as_bool();
    bool controller_logging_enabled_4 = node->get_parameter("controller_logging_enabled_4").as_bool();
    bool controller_logging_enabled_5 = node->get_parameter("controller_logging_enabled_5").as_bool();
    //------------------------------------------------------


    //------------------------------------------------------
    /* VARIABILI DELL'ESECUZIONE */
    double perc_success;                // percentuale di successo della pianificazione (0.0 - 1.0)
    bool shot_success;                  // flag per indicare se il tiro è stato eseguito con successo
    //------------------------------------------------------




    //------------------------------------------------------
    /* CALCOLO DELL'ORIENTAMENTO STECCA*/
    // orientamento è costante in molte fasi, dall'approach all'esecuzione tiro.. lo calcolo una sola volta

    // matrice di rotazione di base che allinea z' -> x, y' --> -y, x' --> -z (da posa di pre approach ad approach base)
    Matrix3d R_base;
    R_base <<  0,  0, -1,
               0, -1,  0,
              -1,  0,  0;

    Quaternion Q_base(R_base);  //converto in quaternione


    //direzione d'impatto
    double impact_angle_rad = impact_angle_deg_ * M_PI / 180;           // inclinazione asta -> rotazione attorno asse y (latitudine)
    double direction_angle_rad = direction_angle_deg_ * M_PI / 180;     // direzione asta -> rotazione attorno asse z (longitudine)


    // calcolo il quaternione dell'orientamento stecca
    Quaternion Q_shot = Quaternion(
        RotationAxis(direction_angle_rad, Z_AXIS) *
        RotationAxis(-impact_angle_rad, Y_AXIS)           //- perché per alzarsi dal tavolo, l'asta deve ruotare in senso orario
    ) * Q_base;


    //Risultato: d'ora in avanti Q_shot è l'orientamento per tutte le sequenze di tiro, dall'approach all'esecuzione del tiro stesso
    //------------------------------------------------------


    
    //------------------------------------------------------
    /* SEQUENZA DI TASK */


    // FASE 1 - vado in pre-approach per approcciare la pallina
    {
        if(control_execution_by_user_input_){
            node->print_and_wait("\n\nPosizionamento in 'pre_approach..");
        }
        else{
            RCLCPP_INFO(node->get_logger(), "\n\nPosizionamento in 'pre_approach..");
        }

        //logging
        if(joints_logging_enabled_1) node->startJointLogging(LOG_JOINT_PATH + "joint_log_1_preapproach.csv");
        if(cartesian_logging_enabled_1) node->startCartesianLogging(LOG_CARTESIAN_PATH + "cartesian_log_1_preapproach.csv");
        if(torque_logging_enabled_1) node->startTorqueLogging(LOG_TORQUE_PATH + "torque_log_1_preapproach.csv");
        if(controller_logging_enabled_1) node->startControllerLogging(LOG_CONTROLLER_PATH + "controller_log_1_preapproach.csv");

        node->moveToNamedTarget(READY_TO_APPROACH_CONFIG);

        if(using_mujoco_simulation_){
            //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
            node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

            //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
        }
        
        if(joints_logging_enabled_1) node->stopJointLogging();
        if(cartesian_logging_enabled_1) node->stopCartesianLogging();
        if(torque_logging_enabled_1) node->stopTorqueLogging();
        if(controller_logging_enabled_1) node->stopControllerLogging();
    }
    
    

    // FASE 2 - approach alla pallina
    {
        if(control_execution_by_user_input_){
            node->print_and_wait("\n\nApproach alla pallina..");
        }
        else{
            RCLCPP_INFO(node->get_logger(), "\n\nApproach alla pallina..");
        }
        

        //distanza desiderata dal centro della pallina (per allontanarsi)
        double desired_distance_from_ball_center = approach_distance_from_ball_surface_ + BALL_RADIUS; // distanza posizionamento dal centro della pallina


        //uso coordinate sferiche per calcolare la posizione in 3D di dove deve andare la punta dell'asta
        Vector3d pos_pre_shot = Vector3d(desired_distance_from_ball_center * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                         desired_distance_from_ball_center * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                         desired_distance_from_ball_center * sin(impact_angle_rad) + offset_correction_center_z_
                                        );


        //logging
        if(joints_logging_enabled_2) node->startJointLogging(LOG_JOINT_PATH + "joint_log_2_approach.csv");
        if(cartesian_logging_enabled_2) node->startCartesianLogging(LOG_CARTESIAN_PATH + "cartesian_log_2_approach.csv");
        if(torque_logging_enabled_2) node->startTorqueLogging(LOG_TORQUE_PATH + "torque_log_2_approach.csv");
        if(controller_logging_enabled_2) node->startControllerLogging(LOG_CONTROLLER_PATH + "controller_log_2_approach.csv");

        perc_success = node->moveCartesianPath(pos_pre_shot, Q_shot, WHITE_SOLID_BALL_FRAME, 
                                                      success_threshold_approach_); //soglia di successo 95%, perché voglio che ci arrivi

        if(using_mujoco_simulation_){
            //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
            node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

            //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
        }

        if(joints_logging_enabled_2) node->stopJointLogging();
        if(cartesian_logging_enabled_2) node->stopCartesianLogging();
        if(torque_logging_enabled_2) node->stopTorqueLogging();
        if(controller_logging_enabled_2) node->stopControllerLogging();


        if(print_EEF_distance_and_position_) {
            //prima di procedere, stampo la distanza e la posizione relativa tra tip dell'asta e pallina bianca, utile per debug
            node->printEEFDebugInfo();
        }

        //controllo se l'approach è andato a buon fine
        if(perc_success < success_threshold_approach_) {
            RCLCPP_ERROR(node->get_logger(), "\n\nApproach alla pallina fallito: non è stato possibile raggiungere la posizione desiderata con sufficiente precisione.");
            rclcpp::shutdown();
            spinner.join();
            return -1;
        }
    }



    // FASE 3 - si allontana all'indietro per prendere velocità
    {
  
        if(control_execution_by_user_input_){
            node->print_and_wait("\n\nAllontanamento all'indietro per prendere velocità..");
        }
        else{
            RCLCPP_INFO(node->get_logger(), "\n\nAllontanamento all'indietro per prendere velocità..");
        }

        //distanza desiderata dal centro della pallina (per allontanarsi)
        double desired_distance_from_ball_center = shooting_distance_from_ball_surface_ + BALL_RADIUS; // distanza posizionamento dal centro della pallina


        //uso coordinate sferiche per calcolare la posizione in 3D di dove deve andare la punta dell'asta
        Vector3d pos_back_shot = Vector3d(desired_distance_from_ball_center * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                          desired_distance_from_ball_center * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                          desired_distance_from_ball_center * sin(impact_angle_rad) + offset_correction_center_z_
                                          );

        //logging
        if(joints_logging_enabled_3) node->startJointLogging(LOG_JOINT_PATH + "joint_log_3_back_shot.csv");
        if(cartesian_logging_enabled_3) node->startCartesianLogging(LOG_CARTESIAN_PATH + "cartesian_log_3_back_shot.csv");
        if(torque_logging_enabled_3) node->startTorqueLogging(LOG_TORQUE_PATH + "torque_log_3_back_shot.csv");
        if(controller_logging_enabled_3) node->startControllerLogging(LOG_CONTROLLER_PATH + "controller_log_3_back_shot.csv");

        perc_success = node->moveCartesianPath(pos_back_shot, Q_shot, WHITE_SOLID_BALL_FRAME, 
                                                      success_threshold_back_);            

        
        if(using_mujoco_simulation_){
            //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
            node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

            //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
        }


        if(joints_logging_enabled_3) node->stopJointLogging();
        if(cartesian_logging_enabled_3) node->stopCartesianLogging();
        if(torque_logging_enabled_3) node->stopTorqueLogging();
        if(controller_logging_enabled_3) node->stopControllerLogging();


        if(print_EEF_distance_and_position_) {
            //prima di procedere, stampo la distanza tra tip dell'asta e pallina bianca, utile per debug
            node->printEEFDebugInfo();
        }


        //controllo se l'allontanamento è andato a buon fine
        if(perc_success < success_threshold_back_) {
            RCLCPP_ERROR(node->get_logger(), "\n\nAllontanamento all'indietro fallito: non è stato possibile raggiungere la posizione desiderata con sufficiente precisione.");
            rclcpp::shutdown();
            spinner.join();
            return -1;
        }

        
    }


    // FASE 4 - eseguo tiro
    {
       
        if(control_execution_by_user_input_){
            node->print_and_wait("\n\nEsecuzione tiro..");
        }
        else{
            RCLCPP_INFO(node->get_logger(), "\n\nEsecuzione tiro..");
        }


        //parametri del tiro
        double accel_distance = node->getEEFDistance() - BALL_RADIUS;                       // distanza di accelerazione (dalla posizione all'indietro a cui sono riuscito ad arrivare, fino al contatto con la pallina)
        double decel_distance = distance_deceleration_phase_fraction_radius_ * BALL_RADIUS; // distanza di decelerazione (dal contatto al centro della pallina, scelta progettuale)

        //considerato che decel_distance è la distanza tra il tip dell'asta e il centro della pallina, per calcolare la posizione di arresto devo sottrarre il raggio della pallina
        //distanza arresto = distanza di decelerazione - raggio della pallina
        
        Vector3d pos_arresto = Vector3d(      
                                          -(decel_distance - BALL_RADIUS) * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                          -(decel_distance - BALL_RADIUS) * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                          -(decel_distance - BALL_RADIUS) * sin(impact_angle_rad) + offset_correction_center_z_
                                        );

        //disabilito collisione tra asta e pallina bianca, così la stecca può penetrare la pallina senza che MoveIt! blocchi il tiro per collisione
        node->disable_collision();

        
        //logging
        if(joints_logging_enabled_4) node->startJointLogging(LOG_JOINT_PATH + "joint_log_4_shot.csv");
        if(cartesian_logging_enabled_4) node->startCartesianLogging(LOG_CARTESIAN_PATH + "cartesian_log_4_shot.csv");
        if(torque_logging_enabled_4) node->startTorqueLogging(LOG_TORQUE_PATH + "torque_log_4_shot.csv");
        if(controller_logging_enabled_4) node->startControllerLogging(LOG_CONTROLLER_PATH + "controller_log_4_shot.csv");


        shot_success = node->ExecuteShot(pos_arresto, Q_shot, WHITE_SOLID_BALL_FRAME, 
                          impact_shot_velocity_,
                          accel_distance, decel_distance);

        
        if(using_mujoco_simulation_){
            //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
            node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

            //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
        }


        if(joints_logging_enabled_4) node->stopJointLogging();
        if(cartesian_logging_enabled_4) node->stopCartesianLogging();
        if(torque_logging_enabled_4) node->stopTorqueLogging();
        if(controller_logging_enabled_4) node->stopControllerLogging();


        if(print_EEF_distance_and_position_) {
            //prima di procedere, stampo la distanza tra tip dell'asta e pallina bianca, utile per debug
            node->printEEFDebugInfo();
        }
    }


    // FASE 5 - mi alzo un po' per liberare il campo (ma lo faccio solo se ho fatto il tiro)
    {
    
        if(shot_success) {

            if(control_execution_by_user_input_){
                node->print_and_wait("\n\nMi alzo..");
            }
            else{
                RCLCPP_INFO(node->get_logger(), "\n\nMi alzo..");
            }


            //uso coordinate sferiche per calcolare la posizione in 3D di dove deve andare la punta dell'asta
            Vector3d pos_back_shot = Vector3d(0, 0, 0 + elevation_escape_);

            //logging
            if(joints_logging_enabled_5) node->startJointLogging(LOG_JOINT_PATH + "joint_log_5_get_high.csv");
            if(cartesian_logging_enabled_5) node->startCartesianLogging(LOG_CARTESIAN_PATH + "cartesian_log_5_get_high.csv");
            if(torque_logging_enabled_5) node->startTorqueLogging(LOG_TORQUE_PATH + "torque_log_5_get_high.csv");
            if(controller_logging_enabled_5) node->startControllerLogging(LOG_CONTROLLER_PATH + "controller_log_5_get_high.csv");


            node->moveCartesianPath(pos_back_shot, Q_shot, WHITE_SOLID_BALL_FRAME);


            if(using_mujoco_simulation_){
                //questo ritardo indispensabile serve a far sincronizzare mujoco (più lento) con moveit
                node->get_clock()->sleep_for(rclcpp::Duration(std::chrono::milliseconds(mujoco_sync_pause_time_milliseconds_)));

                //ATTENZIONE: se non sto usando MuJoCo, questo sleep per qualche motivo non fa più pianificare e blocca il programma
            }

            if(joints_logging_enabled_5) node->stopJointLogging();
            if(cartesian_logging_enabled_5) node->stopCartesianLogging();
            if(torque_logging_enabled_5) node->stopTorqueLogging();
            if(controller_logging_enabled_5) node->stopControllerLogging();


            if(print_EEF_distance_and_position_) {
                //prima di procedere, stampo la distanza tra tip dell'asta e pallina bianca, utile per debug
                node->printEEFDebugInfo();
            }
        }
        else {
            RCLCPP_WARN(node->get_logger(), "\n\nTiro non eseguito, salto la fase di alzata.");
        }

        node->enable_collision() ; //riattivo collisione tra asta e pallina bianca
    }
    

    //----------------------------------------
    // Termina: shutdown sblocca lo spinner, poi join aspetta che finisca
    rclcpp::shutdown();
    spinner.join();
 

    return 0;

}