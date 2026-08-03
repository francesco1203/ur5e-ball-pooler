#include <memory>
#include <vector>
#include <chrono>
#include <thread>

#include <fstream>   // <--- Aggiunto per l'I/O su file

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "shared_headers_pkg/eigen_utilities.hpp"
#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/ur5e_constants.hpp"


// servizio
#include "interfaces_pkg/srv/compute_ik_clik.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;


class ClikServiceNode : public rclcpp::Node
{
  public:
    /*ALIAS*/
    
    //msg
    using JointStateMsg = sensor_msgs::msg::JointState;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;


    //sottocomponenti ros
    using ComputeIkClikSrv = interfaces_pkg::srv::ComputeIkClik;

    using ComputeIkClikSrvPtr = rclcpp::Service<ComputeIkClikSrv>::SharedPtr;
    using JointStateSubPtr = rclcpp::Subscription<JointStateMsg>::SharedPtr;

    //alto
    using joint_config  = std::vector<double>;


    

    
    ClikServiceNode() : Node("clik_service_node"),
        q_k_received_(false),
        n_joints_(N_JOINTS),
        joint_names_(UR5e_JOINT_NAMES),
        planning_group_name_(PLANNING_GROUP),
        last_link_name_(EE_LINK),
        base_link_name_(BASE_LINK),
        joint_states_topic_name_(READING_JOINT_STATES_TOPIC),
        service_name_(CLIK_SERVICE)
    {
        /* PARAMETRI DA LAUNCH FILE*/
        this->declare_parameter<double>("Tclik", 0.001);                     // Periodo di integrazione
        this->declare_parameter<double>("gamma_on_T", 0.5);                  // Guadagno proporzionale
        this->declare_parameter<int>("default_max_iterations", 100);                // Numero massimo di iterazioni
        this->declare_parameter<double>("default_position_tolerance", 0.001);       // Tolleranza di posizione (1mm)
        this->declare_parameter<double>("default_orientation_tolerance", 0.01);    // Tolleranza di orientamento (circa 1°)
        this->declare_parameter<double>("singularity_trshld_warn", 0.01);      // Soglia di warning per singolarità
        this->declare_parameter<double>("singularity_trshld_error", 0.001);  // Soglia di errore per singolarità
        this->declare_parameter<double>("lambda_max", 1.0);                  // lambda_max per DLS (Damped Least Squares)
        this->declare_parameter<bool>("save_on", false);                        // Salvataggio dati su file
        this->declare_parameter<std::string>("file_path", "/home/francesco/Desktop/ProgettoRobotica/ws_ur5e_ballpool/src/shot_planning/temp_data/clik_data.csv");  // Percorso del file di output

        T_clik_ = this->get_parameter("Tclik").as_double();
        gamma_on_T_clik_ = this->get_parameter("gamma_on_T").as_double();
        default_max_iterations_ = this->get_parameter("default_max_iterations").as_int();
        default_position_tolerance_ = this->get_parameter("default_position_tolerance").as_double();
        default_orientation_tolerance_ = this->get_parameter("default_orientation_tolerance").as_double();
        singularity_trshld_warn_ = this->get_parameter("singularity_trshld_warn").as_double();
        singularity_trshld_error_ = this->get_parameter("singularity_trshld_error").as_double();
        lambda_max_ = this->get_parameter("lambda_max").as_double();
        save_on_ = this->get_parameter("save_on").as_bool();
        file_path_ = this->get_parameter("file_path").as_string();

        /* INIZIALIZZAZIONE MOVEIT */
        robot_loader_node_ = std::make_shared<rclcpp::Node>(
            "robot_model_loader_clik",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(robot_loader_node_);
        const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();

        joint_model_group_ = kinematic_model->getJointModelGroup(planning_group_name_);
        kinematic_state_   = std::make_shared<moveit::core::RobotState>(kinematic_model);
        last_link_         = kinematic_state_->getLinkModel(last_link_name_);


        /*SUBSCRIBER*/
        joint_states_sub_ = this->create_subscription<JointStateMsg>(
            joint_states_topic_name_, 10,
            std::bind(&ClikServiceNode::read_joint_states_callback, this, _1)
        );

        /* SERVICE SERVER*/
        clik_server_ = this->create_service<ComputeIkClikSrv>(
            service_name_,
            std::bind(&ClikServiceNode::handle_clik_request, this, _1, _2)
        );


        /* INIZIALIZZAZIONE TF2 */
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        
        q_k_.resize(n_joints_, 0.0);
        RCLCPP_INFO(this->get_logger(), "Nodo CLIK Service pronto.");
    }

  private:
    //sottocomponenti ros
    JointStateSubPtr joint_states_sub_;
    ComputeIkClikSrvPtr clik_server_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    bool q_k_received_;
    joint_config q_k_;

    int n_joints_;
    std::vector<std::string> joint_names_;
    std::string planning_group_name_;
    std::string last_link_name_;
    std::string base_link_name_;
    std::string joint_states_topic_name_;
    std::string service_name_;

    double T_clik_;
    double gamma_on_T_clik_;
    int default_max_iterations_;
    double default_position_tolerance_;
    double default_orientation_tolerance_;
    double singularity_trshld_warn_;
    double singularity_trshld_error_;
    double lambda_max_;
    bool save_on_;
    std::string file_path_;

    rclcpp::Node::SharedPtr robot_loader_node_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    const moveit::core::LinkModel* last_link_;


    /*metodi privati*/

    //ccallback per leggere la configurazione attuale del robot
    void read_joint_states_callback(const JointStateMsg::SharedPtr msg)
    {
        for (int i = 0; i < n_joints_; i++) {
        for (size_t j = 0; j < msg->name.size(); j++) {
            if (msg->name[j] == joint_names_[i]) {
            q_k_[i] = msg->position[j];
            }
        }
        }
        q_k_received_ = true;
    }


    /* Callback per gestire le richieste del servizio CLIK
    * Implementa l'algoritmo CLIK a tempo discreto:
    *
    *   qdot_k = J†(q_k) * (v_d + gamma * e_k)
    *   q_k+1  = q_k + T * qdot_k
    *
    * Con v_d = 0 (posa desiderata statica, nessuna velocità desiderata feedforward),
    * quindi si semplifica in:
    * 
    * qdot_k = J†(q_k) * gamma * e_k
    * q_k+1  = q_k + T * qdot_k
    * 
    * L'errore e_k è 6D:
    *   - prime 3 componenti: errore di posizione   e_p = p_d - p_k
    *   - ultime 3 componenti: errore di orientamento e_o = epsilon di (Q_d * Q_k^-1)
    * 
    */

    void handle_clik_request(
        const std::shared_ptr<ComputeIkClikSrv::Request> request,
        std::shared_ptr<ComputeIkClikSrv::Response> response)
    {
        // 0. Check per vedere se ho ricevuto la configurazione attuale del robot
        if (!q_k_received_) {
            response->success = false;
            response->message = "Errore: Joint states attuali non ancora ricevuti.";
            return;
        }

        // 1. Parametri di default se non specificati nella richiesta
        // NOTA*: se non passo niente dalla richiesta, vuol dire che uso quelli di default del nodo
        double pos_tol = (request->position_tolerance > 0.0) 
                        ? request->position_tolerance 
                        : default_position_tolerance_;

        double ori_tol = (request->orientation_tolerance > 0.0) 
                        ? request->orientation_tolerance 
                        : default_orientation_tolerance_;

        int max_iter = (request->max_iterations > 0) 
                    ? request->max_iterations 
                    : default_max_iterations_;
        

        // --- TRASFORMAZIONE TF2 ---
        //NOTA FONDAMENTALE: Lo jacobiano viene calcolato da base_link a last_link, quindi la target_pose deve essere trasformata nel frame di base del robot (base_link)
        PoseStampedMsg target_pose_base;
        
        // Se il frame di arrivo non è specificato, interrompo la richiesta e ritorno un errore
        std::string req_frame = request->target_pose.header.frame_id;
        if (req_frame.empty()) {
            response->success = false;
            response->message = "Errore: frame_id della target_pose vuoto.";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        if (req_frame != base_link_name_) {
            try {
                // Trasforma la posa dal frame della request al frame di base di MoveIt
                target_pose_base = tf_buffer_->transform(
                    request->target_pose, 
                    base_link_name_, 
                    tf2::durationFromSec(1.0)
                );
            } catch (const tf2::TransformException & ex) {
                response->success = false;
                response->message = "Errore TF2: " + std::string(ex.what());
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
        } else {
            target_pose_base = request->target_pose;
        }


        // ---------------------------------------------------------------------
        // SALVATAGGIO SU FILE - PER DEBUG
        // --- APERTURA FILE LOG PER LA SINGOLA ESECUZIONE CLIK ---
        std::ofstream log_file(file_path_);

        if(save_on_)
        {
            if (!log_file.is_open()) {
                RCLCPP_WARN(this->get_logger(), "Impossibile aprire il file CSV");
            } else {
                // Intestazione: iteraration, q_0..q_5, err_p, err_o
                log_file << "iteration";
                for (int i = 0; i < n_joints_; ++i) {
                    log_file << ",q_" << i;
                }
                log_file << ",err_p,err_o\n";
            }
        }
        // ---------------------------------------------------------------------


        // 2. Inizializzazione variabili di calcolo e richiese
        VectorXd q_current = Eigen::Map<VectorXd>(q_k_.data(), n_joints_);
        Quaternion Q_k_prev = Quaternion::Identity();   //serve a continuità del quaternione

        Vector3d p_d(
            target_pose_base.pose.position.x,
            target_pose_base.pose.position.y,
            target_pose_base.pose.position.z
        );

        Quaternion Q_d(
            target_pose_base.pose.orientation.w,
            target_pose_base.pose.orientation.x,
            target_pose_base.pose.orientation.y,
            target_pose_base.pose.orientation.z
        );
        Q_d.normalize();

        //gestione iterazioni 
        int iterations = 0;
        const int max_iterations = max_iter;
        
        //variabili d'errore
        double err_p_norm = 0.0;
        double err_o_norm = 0.0;
        bool converged = false;

        //pre-allocazione variabili di controllo
        double sigma_min = 0.0;                 //autovalore minimo jacob
        double lambda2 = 0.0;                   //per DLS 
        double ratio = 0.0;                     //per DLS
        double gamma = 0.0;                     //guadagno proporzionale clik


        // 2. Loop di simulazione CLIK interno
        while (iterations < max_iterations)
        {
            // Aggiorna cinematica del modello con la configurazione attuale
            for (int i = 0; i < n_joints_; i++) {
                kinematic_state_->setJointPositions(joint_names_[i], &q_current[i]);
            }


            // Recupero posizione e orientamento attuale dalla matrice di trasformazione
            const RotoTraslMatrix& b_T_e = kinematic_state_->getGlobalLinkTransform(last_link_);
            Vector3d p_k = b_T_e.translation();
            Quaternion Q_k(b_T_e.rotation());
            Q_k.normalize();

            Q_k = quaternionContinuity(Q_k, Q_k_prev);  //evita salti di segno del quaternione
            Q_k_prev = Q_k;


            // Errore 6D
            Vector6d e_k;
            Vector3d err_p = p_d - p_k;
            
            Quaternion DeltaQ = Q_d * Q_k.inverse();
            DeltaQ.normalize();
            Vector3d err_o = DeltaQ.vec();

            e_k.block<3, 1>(0, 0) = err_p;
            e_k.block<3, 1>(3, 0) = err_o;

            err_p_norm = err_p.norm();
            err_o_norm = err_o.norm();


            // ---------------------------------------------------------------------
            // SALVATAGGIO SU FILE - PER DEBUG
            if(save_on_) {
                if (log_file.is_open()) {
                    log_file << iterations;
                    for (int i = 0; i < n_joints_; ++i) {
                        log_file << "," << q_current[i];
                    }
                    log_file << "," << err_p_norm << "," << err_o_norm << "\n";
                }
            }
            // ---------------------------------------------------------------------


            // Controllo Criteri di Arresto (Raggiunta la posa)
            if (err_p_norm <= pos_tol && err_o_norm <= ori_tol) {
                converged = true;
                break;
            }


            // Jacobiano (calcolto da base_link a last_link)
            MatrixXd J(6, n_joints_);
            Vector3d reference_point(0.0, 0.0, 0.0);
            if (!kinematic_state_->getJacobian(joint_model_group_, last_link_, reference_point, J)) {
                response->success = false;
                response->message = "Fallito il calcolo dello Jacobiano.";
                return;
            }


            // Controllo Singolarità
            Eigen::BDCSVD<Eigen::MatrixXd> svd(J);
            sigma_min = svd.singularValues().minCoeff();

            // --- Calcolo dello smorzamento (Damped Least Squares, Chan & Lawrence) ---
            lambda2 = 0.0;
            if (sigma_min < singularity_trshld_warn_) {
                ratio = sigma_min / singularity_trshld_warn_;   // in [0,1)
                lambda2 = (1.0 - ratio * ratio) * (lambda_max_ * lambda_max_);

                // 1. Costruisci la stringa formattata della configurazione di giunto
                std::string q_str = "[ ";
                for (int i = 0; i < n_joints_; ++i) {
                    q_str += std::to_string(q_current[i]);
                    if (i < n_joints_ - 1) q_str += ", ";
                }
                q_str += " ]";


                // 2. Stampa un log di warn sul terminale del nodo CLIK
                RCLCPP_WARN(this->get_logger(), 
                    "Warning di singolarità  (sigma_min = %f), iterazione %d,  configurazione q: %s \n",
                     sigma_min,
                     iterations,
                     q_str.c_str());
            }


            // if (sigma_min < singularity_trshld_error_) {    //sono in singolarità
                
            //     // 1. Costruisci la stringa formattata della configurazione di giunto
            //     std::string q_str = "[ ";
            //     for (int i = 0; i < n_joints_; ++i) {
            //         q_str += std::to_string(q_current[i]);
            //         if (i < n_joints_ - 1) q_str += ", ";
            //     }
            //     q_str += " ]";

            //     // 2. Stampa un log sul terminale del nodo CLIK
            //     RCLCPP_ERROR(this->get_logger(), 
            //         "Singolarità critica (sigma_min = %f) iterazione %d in configurazione q: %s",
            //          sigma_min,
            //          iterations,
            //          q_str.c_str());

            // } 
            // if (sigma_min < singularity_trshld_warn_) {     //sono vicino alla singolarità, ma non ancora andato

            //     // 1. Costruisci la stringa formattata della configurazione di giunto
            //     std::string q_str = "[ ";
            //     for (int i = 0; i < n_joints_; ++i) {
            //         q_str += std::to_string(q_current[i]);
            //         if (i < n_joints_ - 1) q_str += ", ";
            //     }
            //     q_str += " ]";


            //     // 2. Stampa un log di warn sul terminale del nodo CLIK
            //     RCLCPP_WARN(this->get_logger(), 
            //         "Warning di avvicinamento a singolarità  (sigma_min = %f), iterazione %d,  configurazione q: %s \n",
            //          sigma_min,
            //          iterations,
            //          q_str.c_str());
            // } else {    //non sono andato in singolarità, tutto ok
            //     lambda2 = 0.0;
            // }


           //correzione Jacobiano con DLS
            MatrixXd JJt_damped = J * J.transpose() + lambda2 * MatrixXd::Identity(6, 6);

            // Integrazione Step CLIK
            gamma = gamma_on_T_clik_ / T_clik_;
            VectorXd q_dot = J.transpose() * JJt_damped.ldlt().solve(gamma * e_k);


            q_current += q_dot * T_clik_;   //aggiornamento dello stato

            iterations++;
        }


        // ---------------------------------------------------------------------
        // SALVATAGGIO SU FILE - PER DEBUG
        // Chiusura del file al termine dell'algoritmo
        if(save_on_) {
            if (log_file.is_open()) {
                log_file.close();
                RCLCPP_INFO(this->get_logger(), "Giunti ed Errori salvati con successo su clik_debug_log.csv");
            }
        }
        // ---------------------------------------------------------------------


        // 3. Risposta del Servizio
        if (converged) {
            response->success = true;
            response->message = "Posa raggiunta con successo entro la tolleranza.";
        } else {
            response->success = false;
            response->message = "Limite iterazioni raggiunto! Errore pos: " + std::to_string(err_p_norm) +
                                " m, Errore orient: " + std::to_string(err_o_norm) + " rad.";
        }

        // Imposta il JointState finale da restituire
        response->joint_state.name = joint_names_;
        response->joint_state.position.assign(q_current.data(), q_current.data() + q_current.size());
        response->joint_state.header.stamp = this->now();
    }


    Quaternion quaternionContinuity(const Quaternion& Q_k, const Quaternion& Q_k_minus_1)
    {
        double dot = Q_k.vec().transpose() * Q_k_minus_1.vec();
        if (dot < -0.01) {
        Quaternion out(Q_k);
        out.vec() = -out.vec();
        out.w()   = -out.w();
        return out;
        }
        return Q_k;
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // Utilizziamo un MultiThreadedExecutor per evitare deadlock se il nodo legge e processa il servizio contemporaneamente
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ClikServiceNode>();
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}