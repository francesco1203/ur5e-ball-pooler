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
// ============================================================

#include <memory>
// #include <thread>
#include <vector>
// #include <cmath>     //moveit già la include

#include <rclcpp/rclcpp.hpp>
// #include "rclcpp_action/rclcpp_action.hpp"


// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
 #include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>


#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "geometry_msgs/msg/pose.hpp" 
#include "sensor_msgs/msg/joint_state.hpp"


//tf
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// miei headers
#include "shared_headers_pkg/eigen_utilities.hpp"
#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"



// servizio
// #include "interfaces_pkg/srv/compute_ik_clik.hpp"



using namespace std::chrono_literals;


using joint_config          = std::vector<double>;      //globale


class TaskNode : public rclcpp::Node
{
    /* alias*/

    //MoveIt
    using MoveGroupInterface    = moveit::planning_interface::MoveGroupInterface;
    using MoveGroupInterfacePtr = std::unique_ptr<MoveGroupInterface>;
    using Plan                  = MoveGroupInterface::Plan;

    //Msh
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PoseMsg = geometry_msgs::msg::Pose;
    using JointStateMsg  = sensor_msgs::msg::JointState;   
    using RobotTrajectoryMsg = moveit_msgs::msg::RobotTrajectory;

    //Service
    // using ComputeIkClikSrv = interfaces_pkg::srv::ComputeIkClik;
    // using ComputeIkClikSrvClientPtr = rclcpp::Client<ComputeIkClikSrv>::SharedPtr;

    //Other
    using TimerPtr              = rclcpp::TimerBase::SharedPtr;


  public:

    /* Builder */
    TaskNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
        : rclcpp::Node("task_node", opt)
    {
        /*PLANNING PARAMETERS from task_param.yaml*/ 
        this->declare_parameter<double>("def_cartesian_planning_time", 10.0);                           // default planning time in seconds
        this->declare_parameter<double>("def_joint_planning_time", 5.0);                                // default planning time in seconds
        this->declare_parameter<double>("goal_joint_tolerance", 0.001);                                 // default joint tolerance in radians
        this->declare_parameter<double>("goal_position_tolerance", 0.002);                              // default position tolerance in meters
        this->declare_parameter<double>("goal_orientation_tolerance", 0.02);                            // default orientation tolerance in radians
        this->declare_parameter<double>("max_velocity_acceleration_scaling_factor", 0.3);               // default scaling factor
        this->declare_parameter<std::string>("joint_planning_algorithm", "RRTConnectkConfigDefault");   // def planner
        this->declare_parameter<std::string>("cartesian_planning_algorithm", "RRTstarkConfigDefault");  // def planner

        def_cartesian_planning_time_ = this->get_parameter("def_cartesian_planning_time").as_double();
        def_joint_planning_time_ = this->get_parameter("def_joint_planning_time").as_double();
        goal_joint_tolerance_ = this->get_parameter("goal_joint_tolerance").as_double();
        goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
        goal_orientation_tolerance_ = this->get_parameter("goal_orientation_tolerance").as_double();
        max_velocity_acceleration_scaling_factor_ = this->get_parameter("max_velocity_acceleration_scaling_factor").as_double();
        joint_planning_algorithm_ = this->get_parameter("joint_planning_algorithm").as_string();
        cartesian_planning_algorithm_ = this->get_parameter("cartesian_planning_algorithm").as_string();


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
        //NON LO FACCIAMO QUI, MA NEI METODI APPOSITI, PERCHÉ DISTINGUIAMO TRA PIANIFICAZIONE CARTESIANA E PIANIFICAZIONE IN SPAZIO GIUNTI
      

        /*SERVZIO CLIK*/
        //clik_client_ = this->create_client<ComputeIkClikSrv>(CLIK_SERVICE);


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
    // INPUT:  joint_values → vettore di double contenente gli angoli dei giunti
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
    // Muove l'end-effector in una posa cartesiana specificata tramite:
    //   - un vettore Eigen per la posizione (x, y, z) in metri
    //   - un quaternione Eigen per l'orientamento
    //
    // MoveIt! calcola la IK internamente e pianifica in spazio dei joint.
    //
    // INPUT:  posizione    → Vector3d {x, y, z} rispetto a "world"
    //         orientamento → Quaternion che descrive l'orientamento del EEF
    //         terna di riferimento frame esplicito (world di default)
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


    bool moveCartesianPath(const Vector3d& posizione, 
                    const Quaternion& orientamento,
                    const std::string& frame_id =   WORLD_FRAME)
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
                
                // Aggiorna la target_pose con i valori trasformati
                target_pose = pose_stamped_out.pose;
                
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "Errore in tf2: impossibile trasformare la posa. %s", ex.what());
                return false; // Interrompe il processo se non riesce a convertire
            }
        }


         // 3. Calcola la traiettoria cartesiana grezza (senza temporizzazione) usando MoveIt!
        std::vector<PoseMsg> waypoints;
        waypoints.push_back(target_pose);

       
        RobotTrajectoryMsg raw_trajectory;
        const double eef_step = 0.01;      // 1 cm di risoluzione


        double fraction = move_group_->computeCartesianPath(waypoints, 
                                                             eef_step, 
                                                             raw_trajectory);
        RCLCPP_INFO(this->get_logger(), "Traiettoria cartesiana calcolata al %.2f%%", fraction * 100.0);

        // 4. Esecuzione se il percorso è completato (almeno al 95%)
        if (fraction >= 0.95) 
        // if (fraction >= 0.01)           //per debug, accettiamo anche frazioni basse, così vediamo se la pianificazione fallisce per collisione o limiti giunti
        {
            // --- GESTIONE DEL TEMPO E DELLA VELOCITÀ ---
            // Convertiamo il messaggio in un oggetto robot_trajectory
            robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
            rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), raw_trajectory);

            // Applichiamo la parametrizzazione del tempo (TimeOptimalTrajectoryGeneration o Ruckig)
            // Scaliamo la velocità e l'accelerazione massima al 50% per un movimento fluido
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            bool success = totg.computeTimeStamps(rt, 0.5, 0.5); 

            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Fallita la parametrizzazione temporale della traiettoria!");
                return false;
            }

            // Riconvertiamo nel messaggio da inviare a MoveIt
            Plan piano;
            rt.getRobotTrajectoryMsg(piano.trajectory);
            
            move_group_->execute(piano);
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Pianificazione cartesiana fallita o interrotta dagli ostacoli/limiti giunti.");
            return false;
        }
    }



    /*ALTRI METODI DI UTILITIES*/

    //metodi getter
    // double getDefJointPlanningTime() const { return def_joint_planning_time_; }
    // double getDefCartesianPlanningTime() const { return def_cartesian_planning_time_; }


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

    // tf2_ros
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //ComputeIkClikSrvClientPtr clik_client_;

    char c_in; // variabile per input da terminale (usata in print_and_wait)

    // Parametri di pianificazione da yaml
    double def_cartesian_planning_time_;                // tempo di pianificazione di default (secondi)
    double def_joint_planning_time_;                    // tempo di pianificazione di default (secondi)
    double goal_joint_tolerance_;                       // tolleranza di giunto (radians)
    double goal_position_tolerance_;                    // tolleranza di posizione (metri)
    double goal_orientation_tolerance_;                 // tolleranza di orientamento (radians)
    double max_velocity_acceleration_scaling_factor_;   // fattore di scala per velocità e accelerazione
    std::string joint_planning_algorithm_;              // planner per pianificazione in spazio giunti
    std::string cartesian_planning_algorithm_;          // planner per pianificazione cartesiana

    // // Parametri di tiro da yaml
    // double approach_distance_from_ball_surface_;
    // double shooting_distance_from_ball_surface_;
    // double latitude_angle_deg_;
    // double longitude_angle_deg_;
    // double angle_pre_rotation_y_;
    // double angle_pre_rotation_z_;

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

    double approach_distance_from_ball_surface_ = node->get_parameter("approach_distance_from_ball_surface").as_double();
    double shooting_distance_from_ball_surface_ = node->get_parameter("shooting_distance_from_ball_surface").as_double();
    double impact_angle_deg_ = node->get_parameter("impact_angle_deg").as_double();
    double direction_angle_deg_ = node->get_parameter("direction_angle_deg").as_double();


    /* SEQUENZA DI TASK */
 

    // 1 - vado in pre-approach per approcciare la pallina
    {
        node->print_and_wait("Posizionamento in 'pre_approach");
        node->moveToNamedTarget(READY_TO_APPROACH_CONFIG);
    }
    
    

    // 2 - approach alla pallina
    {
        node->print_and_wait("Pianificando verso approach alla pallina..");

        // direzione d'impatto
        double impact_angle_rad = impact_angle_deg_ * M_PI / 180;       // inclinazione asta -> rotazione attorno asse y
        double direction_angle_rad = direction_angle_deg_ * M_PI / 180;    // direzione asta -> rotazione attorno asse z

        double distance_from_ball_center = approach_distance_from_ball_surface_ + BALL_RADIUS; // distanza posizionamento dal centro della pallina


        //uso coordinate sferiche per calcolare la posizione in 3D della punta dell'asta
        Vector3d pos_pre_shot = Vector3d(distance_from_ball_center * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                         distance_from_ball_center * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                         distance_from_ball_center * sin(impact_angle_rad));


        // 1. Definisci la matrice di rotazione 3x3
        Matrix3d R_base;
        R_base <<  0,  0, -1,
                   0, -1,  0,
                   -1,  0,  0;

        // 2. Converti la matrice in un Quaternione Eigen
        Quaternion Q_base(R_base);

        // 3. Calcola il Quaternione finale per pre_shot
        Quaternion Q_pre_shot = Quaternion(
            RotationAxis(direction_angle_rad, Z_AXIS) *
            RotationAxis(-impact_angle_rad, Y_AXIS)                 //- perché per alzarsi dal tavolo, l'asta deve ruotare in senso orario
        ) * Q_base;

        // node->moveToPose(pos_pre_shot, Q_pre_shot, WHITE_SOLID_BALL_FRAME);
        node->moveCartesianPath(pos_pre_shot, Q_pre_shot, WHITE_SOLID_BALL_FRAME);
    }


    // 3 - si allontana all'indietro per prendere velocità
    {
        node->print_and_wait("Allontanamento all'indietro per prendere velocità..");

        // direzione d'impatto
        double impact_angle_rad = impact_angle_deg_ * M_PI / 180;       // inclinazione asta -> rotazione attorno asse y
        double direction_angle_rad = direction_angle_deg_ * M_PI / 180;    // direzione asta -> rotazione attorno asse z

        double distance_from_ball_center = shooting_distance_from_ball_surface_ + BALL_RADIUS; // distanza posizionamento dal centro della pallina


        //uso coordinate sferiche per calcolare la posizione in 3D della punta dell'asta
        Vector3d pos_back_shot = Vector3d(distance_from_ball_center * cos(impact_angle_rad) * cos(direction_angle_rad) ,
                                         distance_from_ball_center * cos(impact_angle_rad) * sin(direction_angle_rad), 
                                         distance_from_ball_center * sin(impact_angle_rad));

         // 1. Definisci la matrice di rotazione 3x3
        Matrix3d R_base;
        R_base <<  0,  0, -1,
                   0, -1,  0,
                   -1,  0,  0;

        // 2. Converti la matrice in un Quaternione Eigen
        Quaternion Q_base(R_base);

        // 3. Calcola il Quaternione finale per pre_shot
        Quaternion Q_back_shot = Quaternion(
            RotationAxis(direction_angle_rad, Z_AXIS) *
            RotationAxis(-impact_angle_rad, Y_AXIS)                 //- perché per alzarsi dal tavolo, l'asta deve ruotare in senso orario
        ) * Q_base;

        // node->moveToPose(pos_pre_shot, Q_pre_shot, WHITE_SOLID_BALL_FRAME);
        node->moveCartesianPath(pos_back_shot, Q_back_shot, WHITE_SOLID_BALL_FRAME);

    }


    // 4 - eseguo tiro
    {
        //TODO..
    }
    

    // 5 - vado in pre-approach per la prossima pallina
    // {
    //     node->print_and_wait("Ritorno in 'pre_approach");
    //     node->moveToNamedTarget(READY_TO_APPROACH_CONFIG, 10);
    // }



    // Termina: shutdown sblocca lo spinner, poi join aspetta che finisca
    rclcpp::shutdown();
    spinner.join();
 

    return 0;

}