// ============================================================
//  task_node.cpp
//  Nodo ROS2 che pianifica e ESEGUE movimenti del braccio
//  usando MoveIt! MoveGroupInterface.
//
//  Eseguire con: ros2 run shot_planning task_node --ros-args --params-file $(ros2 pkg prefix shot_planning)/share/shot_planning/config/task_params.yaml
//
//  Struttura:
//    - TaskNode (classe nodo ROS2)
//        ├── moveToJointConfig    → pianifica ed esegue verso una configurazione di giunti specifica
//        ├── moveToNamedTarget()  → va a una posizione predefinita (es. "home")
//        ├── moveToPose()         → va a una posa cartesiana (posizione + orientamento)
//        └── moveCartesianPath()  → va a una posa cartesiana in linea retta (cartesian path)
// ============================================================

#include <memory>
#include <vector>


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
// #include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"



using namespace std::chrono_literals;


using joint_config          = std::vector<double>;      //globale


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


    //Other
    using TimerPtr              = rclcpp::TimerBase::SharedPtr;


  public:

    /* Builder */
    TaskNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
        : rclcpp::Node("task_node", opt)
    {
        /*PLANNING PARAMETERS from task_param.yaml*/ 

        //generic
        this->declare_parameter<double>("max_velocity_acceleration_scaling_factor", 0.3);               // default scaling factor

        //moveToJointConfig e moveToNamedTarget:    
        this->declare_parameter<double>("def_joint_planning_time", 5.0);                                // default planning time in seconds
        this->declare_parameter<std::string>("joint_planning_algorithm", "RRTConnectkConfigDefault");   // def planner                        
        this->declare_parameter<double>("goal_joint_tolerance", 0.001);                                 // default joint tolerance in radians

        //moveToPose:
        this->declare_parameter<double>("def_cartesian_planning_time", 10.0);                           // default planning time in seconds
        this->declare_parameter<std::string>("cartesian_planning_algorithm", "RRTstarkConfigDefault");  // def planner
        this->declare_parameter<double>("goal_position_tolerance", 0.002);                              // default position tolerance in meters
        this->declare_parameter<double>("goal_orientation_tolerance", 0.02);                            // default orientation tolerance in radians
        
        //moveCartesianPath:
        this->declare_parameter<double>("resolution_step", 0.01);                                       // default resolution step in meters
        
        //moveCartesianPathAsymmTriangle
        this->declare_parameter<double>("resolution_step_Ruckig", 0.005);                        // default resolution step in meters
        this->declare_parameter<double>("success_threshold_Ruckig", 0.95);                       // default success threshold
        this->declare_parameter<double>("Ruckig_dt", 0.01);                                      // default Ruckig working step in seconds
        this->declare_parameter<double>("max_jerk", 50.0);                                       // default max jerk in m/s^3



        max_velocity_acceleration_scaling_factor_ = this->get_parameter("max_velocity_acceleration_scaling_factor").as_double();

        def_joint_planning_time_ = this->get_parameter("def_joint_planning_time").as_double();
        joint_planning_algorithm_ = this->get_parameter("joint_planning_algorithm").as_string();
        goal_joint_tolerance_ = this->get_parameter("goal_joint_tolerance").as_double();
        
        def_cartesian_planning_time_ = this->get_parameter("def_cartesian_planning_time").as_double();
        cartesian_planning_algorithm_ = this->get_parameter("cartesian_planning_algorithm").as_string();
        goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
        goal_orientation_tolerance_ = this->get_parameter("goal_orientation_tolerance").as_double();
        
        resolution_step_ = this->get_parameter("resolution_step").as_double();
        
        resolution_step_Ruckig_ = this->get_parameter("resolution_step_Ruckig").as_double();
        success_threshold_Ruckig_ = this->get_parameter("success_threshold_Ruckig").as_double();
        Ruckig_dt_ = this->get_parameter("Ruckig_dt").as_double();
        max_jerk_ = this->get_parameter("max_jerk").as_double();
        


        // MoveGroupInterface ha bisogno di shared_from_this(), che non è
        // disponibile dentro il costruttore → usiamo un timer one-shot (WORKAROUND)
        start_timer_ = this->create_wall_timer(
            100ms, std::bind(&TaskNode::start, this));
    }

    //inizializzazione differita
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
        move_group_->setGoalJointTolerance(goal_joint_tolerance_);              // ~un ventesimo di grado
        move_group_->setGoalPositionTolerance(goal_position_tolerance_);        // 2 mm
        move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_);  // ~1 grado


        // Velocità e accelerazione — scaling rispetto ai limiti massimi definiti in URDF e joint_limits.yaml
        move_group_->setMaxVelocityScalingFactor(max_velocity_acceleration_scaling_factor_);
        move_group_->setMaxAccelerationScalingFactor(max_velocity_acceleration_scaling_factor_);


        //Scelta planner da usare
        //NON LO FACCIAMO QUI, MA NEI METODI APPOSITI
      

        /* CLIENT PLANNING SCENE */
        planning_scene_diff_cli_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");


        /*TF*/
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        RCLCPP_INFO(this->get_logger(), "TaskNode pronto.");
        init_done_.set_value();  // sblocca il main — init completato
    }


    void waitInit()
    {
        // Blocca il chiamante finché start() non ha completato l'inizializzazione.
        // Garantisce la sincronizzazione tra thread
        init_done_.get_future().wait();
    }
    //NOTA: waitInit() è chiamato dal main dopo la creazione del nodo, prima di qualsiasi movimento
    //      per assicurarsi che start() abbia completato l'inizializzazione di move_group_ 
    //      e altri componenti necessari, senza i quali i metodi di movimento non funzionerebbero correttamente.



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


    // ── moveToPose ───────────────────────────────────────────────────────
    // Muove l'end-effector NON IN LINEA RETTA nella posa cartesiana specificata.
    // 
    // In pratica: MoveIt! calcola la IK internamente e pianifica in spazio dei joint.
    //
    //
    // INPUT:  posizione    → Vector3d {x, y, z} rispetto a frame_id
    //         orientamento → Quaternion che descrive l'orientamento del EEF
    //         frame_id     → terna di riferimento (world di default)
    //         planning_time → tempo massimo di pianificazione (in secondi). Se <0, usa il valore di default
    // RETURN: true se pianificazione ed esecuzione hanno avuto successo
    bool moveToPose(const Vector3d& posizione, 
                    const Quaternion& orientamento,
                    const std::string& frame_id = WORLD_FRAME, 
                    double planning_time = -1.0)
    {
        //setting moveit
        move_group_->setPlannerId(cartesian_planning_algorithm_);

        // Se non è stato specificato (valore sentinella -1.0), usa il membro di classe
        double real_planning_time = (planning_time < 0.0) ? def_cartesian_planning_time_ : planning_time;
        move_group_->setPlanningTime(real_planning_time);
        

        RCLCPP_INFO(this->get_logger(),
                    "→ Planning verso posa cartesiana: [%.3f, %.3f, %.3f] con orientamento [w: %.3f, x: %.3f, y: %.3f, z: %.3f] in frame '%s'",
                    posizione.x(), posizione.y(), posizione.z(), 
                    orientamento.normalized().w(), orientamento.normalized().x(), orientamento.normalized().y(), orientamento.normalized().z(), 
                    frame_id.c_str());


        // Costruisco il messaggio ROS2 della posa target a partire da posizione e orientamento desiderati
        // NOTA: ho bisogno di stamped per specificare anche il frame di riferimento, altrimenti
        //       MoveIt! assume che la posa sia espressa nel frame di default del robot (probabilmente base_link)
        PoseStampedMsg target_pose;

        target_pose.header.frame_id = frame_id;

        target_pose.header.stamp = this->get_clock()->now();

        target_pose.pose.position.x = posizione.x();
        target_pose.pose.position.y = posizione.y();
        target_pose.pose.position.z = posizione.z();
 
        Quaternion q_norm = orientamento.normalized();
        target_pose.pose.orientation.w = q_norm.w();
        target_pose.pose.orientation.x = q_norm.x();
        target_pose.pose.orientation.y = q_norm.y();
        target_pose.pose.orientation.z = q_norm.z();
        

        // 3. Forziamo l'aggiornamento dello stato (funziona solo se c'è uno spinner attivo!)
        move_group_->setStartStateToCurrentState();
        move_group_->clearPoseTargets();


        // 4. (Opzionale ma raccomandato) Specifica esplicitamente l'end-effector
        move_group_->setEndEffectorLink(EE_LINK);


        move_group_->setPoseTarget(target_pose);
 
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
                         "Pianificazione fallita per la posa cartesiana richiesta. Codice errore MoveIt: %d", 
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
            RCLCPP_INFO(this->get_logger(), "Trasformazione coordinate: da '%s' a '%s'...", 
                        frame_id.c_str(), planning_frame_moveit.c_str());
            try {
                PoseStampedMsg pose_stamped_out;
                
                // Esegue la trasformazione (con un timeout di 100ms per aspettare che l'albero tf sia pronto)
                pose_stamped_out = tf_buffer_->transform(
                    pose_stamped_in, 
                    planning_frame_moveit, 
                    tf2::durationFromSec(0.1)
                );
                
                // -------------------------------------
                // --- AGGIUNGI QUESTO BLOCCO DI LOG ---
                RCLCPP_INFO(this->get_logger(), 
                            "Posa trasformata nel frame '%s': pos[%.3f, %.3f, %.3f]",
                            planning_frame_moveit.c_str(),
                            pose_stamped_out.pose.position.x,
                            pose_stamped_out.pose.position.y,
                            pose_stamped_out.pose.position.z);
                           
                // -------------------------------------

                // Aggiorna la target_pose con i valori trasformati
                target_pose = pose_stamped_out.pose;
                
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "Errore in tf2: impossibile trasformare la posa. %s", ex.what());
                return -1.0; // Interrompe il processo se non riesce a convertire
            }
        }


         // 3. Calcola la traiettoria cartesiana grezza (senza temporizzazione) usando MoveIt!
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

        std::string planning_frame_moveit = move_group_->getPlanningFrame();
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

        const double total_distance = (num_waypoints - 1) * eef_step;       //numero di stemp per risoluzione step = distanza totale percorsa lungo la linea retta

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

        // Generiamo i punti temporali passo-passo
        while (result == ruckig::Result::Working) {
            result = otg.update(input, output);         //stato ottimare al prossimo (attuale nel ciclo) dt

            double s = output.new_position[0];          // Posizione corrente lungo la linea (metri)
            double v = output.new_velocity[0];          // Velocità corrente lungo la linea (m/s)

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
        if(vel_impact > MAX_TRANS_VEL * max_velocity_acceleration_scaling_factor_ || 
           accel > MAX_TRANS_ACC * max_velocity_acceleration_scaling_factor_ ||
           decel > abs(MAX_TRANS_DEC) * max_velocity_acceleration_scaling_factor_) {
            RCLCPP_ERROR(this->get_logger(), "Profilo di tiro non fattibile: superati i limiti fisici del robot");
            return false;
        }


        // 3. eseguo il tiro con profilo asimmetrico
        return moveCartesianPathAsymmTriangle(posizione_arresto, orientamento, frame_id, 
                                              vel_impact, 
                                              accel, decel); 
    }


    // Disabilita la collisioni
    bool disable_collision() 
    { 
        RCLCPP_INFO(this->get_logger(), "Disattivazione collisioni...");
        ignore_cartesian_collisions_ = true;
        return true;
    }
    

    // Abilita la collisione tra asta e pallina bianca
     bool enable_collision() 
    { 
        RCLCPP_INFO(this->get_logger(), "Riattivazione controlli di collisione ambientali...");
        ignore_cartesian_collisions_ = false;
        return true;
    }

    
    /*ALTRI METODI DI UTILITIES*/

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

    // tf2_ros
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    /* PARAMETRI DI PIANIFICAZIONE */
    double max_velocity_acceleration_scaling_factor_;   // fattore di scala per velocità e accelerazione

    double def_joint_planning_time_;                    // tempo di pianificazione di default (secondi)
    std::string joint_planning_algorithm_;              // planner per pianificazione in spazio giunti
    double goal_joint_tolerance_;                       // tolleranza di giunto (radians)

    double def_cartesian_planning_time_;                // tempo di pianificazione di default (secondi)
    std::string cartesian_planning_algorithm_;          // planner per pianificazione cartesiana
    double goal_position_tolerance_;                    // tolleranza di posizione (metri)
    double goal_orientation_tolerance_;                 // tolleranza di orientamento (radians)
   
    double resolution_step_;                             // passo di risoluzione per pianificazione cartesian path (metri)
    
    double resolution_step_Ruckig_;                      // passo di risoluzione per pianificazione cartesian path con Ruckig (metri)
    double success_threshold_Ruckig_;                    // soglia di successo per pianificazione cartesian path con Ruckig (0.0 - 1.0)
    double Ruckig_dt_;                                   // passo di lavoro per Ruckig (secondi)
    double max_jerk_;                                    // limite di jerk per pianificazione cartesian path con Ruckig (m/s^3)


    //temporanee
    char c_in; // variabile per input da terminale (usata in print_and_wait)
    bool ignore_cartesian_collisions_ = false; // Flag per abilitare/disabilitare collisioni nel tiro


};



// ── main ────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    /* inizializzazione */
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskNode>();
 
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });
 
    // Aspetta che start() abbia completato l'inizializzazione — sincronizzazione
    node->waitInit();
    //adesso sono sicuro che start() ha inizializzato move_group_ e posso chiamare i metodi di movimento


    
    /* SHOT PLANNING PARAMETERS */
    node->declare_parameter<double>("approach_distance_from_ball_surface", 0.02);
    node->declare_parameter<double>("shooting_distance_from_ball_surface", 0.05);
    node->declare_parameter<double>("impact_angle_deg", 10.0);
    node->declare_parameter<double>("direction_angle_deg", 0.0);
    node->declare_parameter<double>("impact_shot_velocity", 0.1); 
    node->declare_parameter<double>("offset_correction_center", 0.002);
    node->declare_parameter<double>("elevation_escape", 0.05);
    node->declare_parameter<double>("success_threshold_approach", 0.95);
    node->declare_parameter<double>("success_threshold_shooting", 0.20);


    double approach_distance_from_ball_surface_ = node->get_parameter("approach_distance_from_ball_surface").as_double();
    double shooting_distance_from_ball_surface_ = node->get_parameter("shooting_distance_from_ball_surface").as_double();
    double impact_angle_deg_ = node->get_parameter("impact_angle_deg").as_double();
    double direction_angle_deg_ = node->get_parameter("direction_angle_deg").as_double();
    double impact_shot_velocity_ = node->get_parameter("impact_shot_velocity").as_double();
    double offset_correction_center_ = node->get_parameter("offset_correction_center").as_double();
    double elevation_escape_ = node->get_parameter("elevation_escape").as_double();
    double success_threshold_approach_ = node->get_parameter("success_threshold_approach").as_double();
    double success_threshold_shooting_ = node->get_parameter("success_threshold_shooting").as_double();


    /* VARIABILI DELL'ESECUZIONE */
    double perc_success;                // percentuale di successo della pianificazione (0.0 - 1.0)



    /* ORIENTAMENTO STECCA*/
    // orientamento è costante in molte fasi, dall'approach all'esecuzione tiro.. lo calcolo una sola volta


    //NOTA: impact_angle_deg_ e direction_angle_deg_ sono angoli in gradi che dovranno essere restituiti dal motore di gioco
    //      per ora, in assenza del motore di gioco, li abbiamo presi simulati da config yaml file


    //direzione d'impatto
    double impact_angle_rad = impact_angle_deg_ * M_PI / 180;           // inclinazione asta -> rotazione attorno asse y (latitudine)
    double direction_angle_rad = direction_angle_deg_ * M_PI / 180;     // direzione asta -> rotazione attorno asse z (longitudine)

    // matrice di rotazione di base che allinea z' -> x, y' --> -y, x' --> -z (da posa di approach a colpo)
    Matrix3d R_base;
    R_base <<  0,  0, -1,
               0, -1,  0,
              -1,  0,  0;

    Quaternion Q_base(R_base);  //converto in quaternione

    // calcolo il quaternione dell'orientamento stecca
    Quaternion Q_shot = Quaternion(
        RotationAxis(direction_angle_rad, Z_AXIS) *
        RotationAxis(-impact_angle_rad, Y_AXIS)           //- perché per alzarsi dal tavolo, l'asta deve ruotare in senso orario
    ) * Q_base;


    //Risultato: d'ora in avanti Q_shot è l'orientamento per tutte le sequenze di tiro, dall'approach all'esecuzione del tiro stesso



    /* SEQUENZA DI TASK */


    // 1 - vado in pre-approach per approcciare la pallina
    {
        node->print_and_wait("Posizionamento in 'pre_approach..");
        node->moveToNamedTarget(READY_TO_APPROACH_CONFIG);
    }
    
    

    // 2 - approach alla pallina
    {
        node->print_and_wait("Approach alla pallina..");

        //distanza
        double distance_from_ball_center = approach_distance_from_ball_surface_ + BALL_RADIUS; // distanza posizionamento dal centro della pallina


        //uso coordinate sferiche per calcolare la posizione in 3D di dove deve andare la punta dell'asta
        Vector3d pos_pre_shot = Vector3d(distance_from_ball_center * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                         distance_from_ball_center * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                         distance_from_ball_center * sin(impact_angle_rad) + offset_correction_center_
                                        );


        perc_success = node->moveCartesianPath(pos_pre_shot, Q_shot, WHITE_SOLID_BALL_FRAME, 
                                                      success_threshold_approach_); //soglia di successo 95%, perché voglio che ci arrivi

        if(perc_success < success_threshold_approach_) {
            RCLCPP_ERROR(node->get_logger(), "Approach alla pallina fallito: non è stato possibile raggiungere la posizione desiderata con sufficiente precisione.");
            rclcpp::shutdown();
            spinner.join();
            return -1;
        }
    }



    // 3 - si allontana all'indietro per prendere velocità
    {
        node->print_and_wait("Allontanamento all'indietro per prendere velocità..");

        //distanza
        double distance_from_ball_center = shooting_distance_from_ball_surface_ + BALL_RADIUS; // distanza posizionamento dal centro della pallina


        //uso coordinate sferiche per calcolare la posizione in 3D di dove deve andare la punta dell'asta
        Vector3d pos_back_shot = Vector3d(distance_from_ball_center * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                          distance_from_ball_center * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                          distance_from_ball_center * sin(impact_angle_rad) + offset_correction_center_
                                          );


        perc_success = node->moveCartesianPath(pos_back_shot, Q_shot, WHITE_SOLID_BALL_FRAME, 
                                                      success_threshold_shooting_);                 //soglia di successo 20%, perché è almeno 1 cm di allontanamento, fondamentale che ci arrivi
        if(perc_success < success_threshold_shooting_) {
            RCLCPP_ERROR(node->get_logger(), "Allontanamento all'indietro fallito: non è stato possibile raggiungere la posizione desiderata con sufficiente precisione.");
            rclcpp::shutdown();
            spinner.join();
            return -1;
        }
    }


    // 4 - eseguo tiro
    {
        node->print_and_wait("Esecuzione tiro..");

        //disabilito collisione tra asta e pallina bianca, così la stecca può penetrare la pallina senza che MoveIt! blocchi il tiro per collisione
        node->disable_collision() ;


        //parametri del tiro
        Vector3d pos_arresto = Vector3d(0, 0, 0 + offset_correction_center_);        //per semplicità finisco il tiro nel centro (corretto) della sfera
        double accel_distance = perc_success * shooting_distance_from_ball_surface_; // distanza di accelerazione (dalla posizione all'indietro a cui sono riuscito ad arrivare, fino al contatto con la pallina)
        double decel_distance = BALL_RADIUS;                                         // distanza di decelerazione (dal contatto alla frenata, centro pallina, scelta progettuale)
        

        
        node->ExecuteShot(pos_arresto, Q_shot, WHITE_SOLID_BALL_FRAME, 
                          impact_shot_velocity_,
                          accel_distance, decel_distance);
    }

    // 5 - mi alzo un po' per liberare il campo
    {
        node->print_and_wait("Torno indietro..");

        

        //uso coordinate sferiche per calcolare la posizione in 3D di dove deve andare la punta dell'asta
        Vector3d pos_back_shot = Vector3d(0, 0, 0 + elevation_escape_);


        node->moveCartesianPath(pos_back_shot, Q_shot, WHITE_SOLID_BALL_FRAME);

        node->enable_collision() ; //riattivo collisione tra asta e pallina bianca
    }
    

    // // 6 - vado in pre-approach per la prossima pallina
    // {
    //     node->print_and_wait("Ritorno in 'pre_approach per prossimo tiro");
    //     node->moveToNamedTarget(READY_TO_APPROACH_CONFIG, 10);
    // }



    // Termina: shutdown sblocca lo spinner, poi join aspetta che finisca
    rclcpp::shutdown();
    spinner.join();
 

    return 0;

}