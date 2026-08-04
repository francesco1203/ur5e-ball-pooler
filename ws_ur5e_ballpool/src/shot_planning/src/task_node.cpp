// ============================================================
//  task_node.cpp
//  Nodo ROS2 che pianifica e ESEGUE movimenti del braccio
//  usando MoveIt! MoveGroupInterface.
//
//  Struttura:
//    - TaskNode (classe nodo ROS2)
//        ├── moveToJointConfig    → pianifica ed esegue verso una configurazione di giunti specifica
//        ├── moveToPose()         → va a una posa cartesiana (posizione + orientamento)
//        ├── moveToNamedTarget()  → va a una posizione predefinita (es. "home")
// ============================================================

#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// #include "rclcpp_action/rclcpp_action.hpp"


// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "sensor_msgs/msg/joint_state.hpp"


// miei headers
#include "shared_headers_pkg/eigen_utilities.hpp"
#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"



// servizio
#include "interfaces_pkg/srv/compute_ik_clik.hpp"



using namespace std::chrono_literals;


using joint_config          = std::vector<double>;      //globale


class TaskNode : public rclcpp::Node
{
    /* alias*/

    //MoveIt
    using MoveGroupInterface    = moveit::planning_interface::MoveGroupInterface;
    using MoveGroupInterfacePtr = std::unique_ptr<MoveGroupInterface>;
    using Plan                  = MoveGroupInterface::Plan;

    //messaggi
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using JointStateMsg  = sensor_msgs::msg::JointState;   

    //servzio
    // using ComputeIkClikSrv = interfaces_pkg::srv::ComputeIkClik;
    // using ComputeIkClikSrvClientPtr = rclcpp::Client<ComputeIkClikSrv>::SharedPtr;

    //altro
    using TimerPtr              = rclcpp::TimerBase::SharedPtr;


  public:

    /* Costruttore */
    TaskNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
    : rclcpp::Node("task_node", opt)
    {
        // Parametri di pianificazione da launch file
        
        this->declare_parameter<double>("def_cartesian_planning_time", 10.0);     // default planning time in seconds
        this->declare_parameter<std::string>("joint_planning_algorithm", "RRTConnectkConfigDefault");  
        this->declare_parameter<std::string>("cartesian_planning_algorithm", "RRTstarkConfigDefault"); 

        def_cartesian_planning_time_ = this->get_parameter("def_cartesian_planning_time").as_double();
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
 
        // Velocità e accelerazione — abbassale ulteriormente in fase di test
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);


        //Scelta planner da usare
        //move_group_->setPlannerId("PLANNER");
        //NON LO FACCIAMO QUI, MA NEI METODI APPOSITI, PERCHÉ DISTINGUIAMO TRA PIANIFICAZIONE CARTESIANA E PIANIFICAZIONE IN SPAZIO GIUNTI
      

        /*SERVZIO CLIK*/
        //clik_client_ = this->create_client<ComputeIkClikSrv>(CLIK_SERVICE);

 
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
    // RETURN: true se pianificazione ed esecuzione hanno avuto successo
    bool moveToJointConfig(const joint_config& joint_values, double planning_time = 5.0)
    {
        //setting moveit
        move_group_->setPlannerId(joint_planning_algorithm_);
        move_group_->setPlanningTime(planning_time);

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
    // INPUT:  target_name → nome della configurazione (es. "home")
    // RETURN: true se pianificazione ed esecuzione hanno avuto successo
    bool moveToNamedTarget(const std::string& target_name, 
                           double planning_time = 5.0)
    {
        //setting moveit
        move_group_->setPlannerId(joint_planning_algorithm_);
        move_group_->setPlanningTime(planning_time);

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
    // RETURN: true se pianificazione ed esecuzione hanno avuto successo
    bool moveToPose(const Vector3d& posizione, 
                    const Quaternion& orientamento,
                    const std::string& frame_id = WORLD_FRAME, 
                    double planning_time = 5.0)
    {
        //setting moveit
        move_group_->setPlannerId(cartesian_planning_algorithm_);
        move_group_->setPlanningTime(planning_time);
        

        RCLCPP_INFO(this->get_logger(),
                    "→ Planning verso posa cartesiana: [%.3f, %.3f, %.3f] con orientamento [w: %.3f, x: %.3f, y: %.3f, z: %.3f] in frame '%s'",
                    posizione.x(), posizione.y(), posizione.z(), 
                    orientamento.normalized().w(), orientamento.normalized().x(), orientamento.normalized().y(), orientamento.normalized().z(), 
                    frame_id.c_str());

        
        // 1. Rilassiamo le tolleranze (adatta questi valori in base alla precisione richiesta)
        move_group_->setGoalPositionTolerance(0.002);    // 2 mm
        move_group_->setGoalOrientationTolerance(0.02);  // ~1 grado


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


    // bool moveToCartesianPoseThroughJointSpace(const Vector3d& posizione, 
    //                                           const Quaternion& orientamento, 
    //                                           const std::string& frame_id,
    //                                           double planning_time = 5.0 )
    // {
    //     // 1. Preparazione della posa target espressa in PoseStamped
    //     PoseStampedMsg target_pose;
    //     target_pose.header.stamp = this->now();
    //     target_pose.header.frame_id = frame_id;

    //     target_pose.pose.position.x = posizione.x();
    //     target_pose.pose.position.y = posizione.y();
    //     target_pose.pose.position.z = posizione.z();

    //     Quaternion q_norm = orientamento.normalized();
    //     target_pose.pose.orientation.w = q_norm.w();
    //     target_pose.pose.orientation.x = q_norm.x();
    //     target_pose.pose.orientation.y = q_norm.y();
    //     target_pose.pose.orientation.z = q_norm.z();


    //     // 2. Controllo disponibilità del servizio CLIK
    //     if (!clik_client_->wait_for_service(std::chrono::seconds(2))) {
    //         RCLCPP_ERROR(this->get_logger(), "Servizio CLIK non disponibile!");
    //         return false;
    //     }


    //     // 3. Costruzione richiesta Servizio CLIK
    //     auto request = std::make_shared<ComputeIkClikSrv::Request>();
    //     request->target_pose = target_pose;

    //     //se non passo nulla agli altri campi, il servizio usa i valori di default
    //     // request->max_iterations = 100;             // 100 iterazioni massime
    //     // request->position_tolerance = 0.001;       // 1 mm
    //     // request->orientation_tolerance = 0.01;     // ~0.5 gradi


    //     // 4. Chiamata ed attesa sincrona della risposta
    //     auto future_result = clik_client_->async_send_request(request);

    //     // Attesa massima di 1 secondi per la risposta della CPU
    //     if (future_result.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    //         RCLCPP_ERROR(this->get_logger(), "Timeout nell'attesa della risposta dal servizio CLIK!");
    //         return false;
    //     }

    //     auto response = future_result.get();


    //     // 5. Verifica dell'esito della Cinematica Inversa
    //     if (!response->success) {
    //         RCLCPP_ERROR(this->get_logger(), "Calcolo CLIK fallito: %s", response->message.c_str());
    //         return false;
    //     }


    //     // 6. Estrazione della configurazione giunti calcolata
    //     joint_config joint_target_positions = response->joint_state.position;


    //     // Log delle posizioni trovate
    //     RCLCPP_INFO(this->get_logger(), "Configurazione giunti trovata con successo via CLIK:");
    //     for (size_t i = 0; i < joint_target_positions.size(); ++i) {
    //         std::string j_name = (i < response->joint_state.name.size()) ? response->joint_state.name[i] : std::to_string(i);
    //         RCLCPP_INFO(this->get_logger(), "  - %s: %.4f rad", j_name.c_str(), joint_target_positions[i]);
    //     }

        
        // 7. Pianificazione ed esecuzione nello spazio giunti
    //     return moveToJointConfig(joint_target_positions, planning_time);
    // }


    //metodi getter
    double getDefCartesianPlanningTime() const { return def_cartesian_planning_time_; }

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

    //ComputeIkClikSrvClientPtr clik_client_;

    char c_in; // variabile per input da terminale (usata in print_and_wait)

    // Parametri di pianificazione
    double def_cartesian_planning_time_;          // tempo di pianificazione di default (secondi)
    std::string joint_planning_algorithm_;      // planner per pianificazione in spazio giunti
    std::string cartesian_planning_algorithm_;  // planner per pianificazione cartesiana

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


    /* SEQUENZA DI TASK */
 

    // 1 - vado in pre-approach per approcciare la pallina
    {
        node->print_and_wait("Posizionamento in 'pre_approach");
        node->moveToNamedTarget(READY_TO_APPROACH_CONFIG, 5);
    }
    
    

    // 2 - approach alla pallina
    {
        node->print_and_wait("Pianificando verso approach alla pallina..");

        Vector3d pos_pre_shot = Vector3d(0.02, 
                                         0, 
                                         0.02
                                        );

        // direzione d'impatto
        double alpha_latitude_rad = 10 * M_PI / 180;   //rotazione attorno asse y
        double beta_longitude_rad = 0 * M_PI / 180;    //rotazione attorno asse z

        Quaternion Q_pre_shot(
                            RotationAxis(M_PI/2, Y_AXIS) *
                            RotationAxis(alpha_latitude_rad, Y_AXIS) * 
                            RotationAxis(beta_longitude_rad, Z_AXIS)
                            );

        node->moveToPose(pos_pre_shot, Q_pre_shot, WHITE_SOLID_BALL_FRAME, node->getDefCartesianPlanningTime());

        //node->moveToCartesianPoseThroughJointSpace(pos_pre_shot, Q_pre_shot, "left_rod_tip_virtual_link", 60);
    }



    // 3 - eseguo tiro
    {
        //TODO..
    }
    

    // 4 - vado in pre-approach per la prossima pallina
    {
        node->print_and_wait("Ritorno in 'pre_approach");
        node->moveToNamedTarget(READY_TO_APPROACH_CONFIG, 10);
    }



    // Termina: shutdown sblocca lo spinner, poi join aspetta che finisca
    rclcpp::shutdown();
    spinner.join();
 

    return 0;

}