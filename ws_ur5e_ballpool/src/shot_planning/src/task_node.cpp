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


#include "geometry_msgs/msg/pose_stamped.hpp" 


// miei headers
#include "shared_headers_pkg/eigen_alias.hpp"
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

    //messaggi
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

    //altro
    using TimerPtr              = rclcpp::TimerBase::SharedPtr;


  public:

    /* Costruttore */
    TaskNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
        : rclcpp::Node("task_node", rclcpp::NodeOptions(opt).automatically_declare_parameters_from_overrides(true))
    {
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

        // Scelgo il planner da usare
        move_group_->setPlanningTime(5.0);  // tempo massimo per la pianificazione (in secondi)
        move_group_->setPlannerId("RRTkConfigDefault");
        // move_group_->setPlannerId("RRTConnectkConfigDefault");
        // move_group_->setPlannerId("PRMkConfigDefault");
        //move_group_->setPlannerId("PRMstarkConfigDefault");

 
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
    bool moveToNamedTarget(const std::string& target_name, double planning_time = 5.0)
    {

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
        
        RCLCPP_INFO(this->get_logger(),
                    "→ Planning verso posa cartesiana: [%.3f, %.3f, %.3f] con orientamento [w: %.3f, x: %.3f, y: %.3f, z: %.3f] in frame '%s'",
                    posizione.x(), posizione.y(), posizione.z(), 
                    orientamento.normalized().w(), orientamento.normalized().x(), orientamento.normalized().y(), orientamento.normalized().z(), 
                    frame_id.c_str());


        move_group_->setPlanningTime(planning_time);

        // Costruisco il messaggio ROS2 della posa target a partire da posizione e orientamento desiderati
        // NOTA: ho bisogno di stamped per specificare anche il frame di riferimento, altrimenti
        //       MoveIt! assume che la posa sia espressa nel frame di default del robot (probabilmente base_link)
        PoseStampedMsg target_pose;

        target_pose.header.frame_id = frame_id;

        target_pose.pose.position.x = posizione.x();
        target_pose.pose.position.y = posizione.y();
        target_pose.pose.position.z = posizione.z();
 
        target_pose.pose.orientation.w = orientamento.normalized().w();
        target_pose.pose.orientation.x = orientamento.normalized().x();
        target_pose.pose.orientation.y = orientamento.normalized().y();
        target_pose.pose.orientation.z = orientamento.normalized().z();
        

        // Puliamo i target precedenti prima di impostare la nuova configurazione
        //ridondanza per robustezza codice: assicuriamoci che MoveIt! conosca la configurazione attuale del robot prima di pianificare
        move_group_->setStartStateToCurrentState();
        move_group_->clearPoseTargets();


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

    char c_in; // variabile per input da terminale (usata in print_and_wait)


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
    node->print_and_wait("Posizionamento in 'pre_approach");
    node->moveToNamedTarget(READY_TO_APPROACH_CONFIG, 5);
    

    // 2 - vado in posizione di pre-shot
    node->print_and_wait("Pianificando verso la posizione di pre-shot...");

    Vector3d pos_pre_shot = Vector3d(-0.05, 
                                      0, 
                                      0.05
                                    );

    // Vector3d pos_pre_shot = Vector3d( -0.5, 
    //                                   -0.2, 
    //                                   0.3
    //                                 );

    Quaternion Q_pre_shot(RotationAxis(10 * M_PI / 180, Y_AXIS)); // rotazione di 10° attorno a Y
    Q_pre_shot = Q_pre_shot.normalized(); // normalizzo il quaternione per sicurezza

    // Quaternion Q_pre_shot(1,0,0,0); // orientamento identità (nessuna rotazione)


    node->moveToPose(pos_pre_shot, Q_pre_shot, BILLIARD_TABLE_FRAME, 30);



    // 3 - eseguo tiro
    //TODO..


    // 4 - vado in pre-approach per la prossima pallina
    node->print_and_wait("Ritorno in 'pre_approach");
    node->moveToNamedTarget(READY_TO_APPROACH_CONFIG, 10);
    


    // Termina: shutdown sblocca lo spinner, poi join aspetta che finisca
    rclcpp::shutdown();
    spinner.join();
 

    return 0;

}