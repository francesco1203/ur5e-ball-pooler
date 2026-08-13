#include <rclcpp/rclcpp.hpp>

#include <memory>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>



#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

//per leggere le terne
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


//mie costanti e librerie
#include "shared_headers_pkg/scene_description.hpp"


//costanti di programma
const std::string NOME_PACCHETTO = "scene_description"; //nome del pacchetto ROS2, per recuperare la cartella share
constexpr double eps_floating = 0.0001; // piccola elevazione per evitare problemi di collision detection con il piano del tavolo


using namespace std::placeholders;
using namespace std::chrono_literals;




class SceneBuilderNode : public rclcpp::Node
{
  //using alias utili
    //moveit
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
    using MoveGroupInterfacePtr = std::unique_ptr<MoveGroupInterface>;
    using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
    using PlanningSceneInterfacePtr = std::unique_ptr<PlanningSceneInterface>;

    //altro
    using TimerPtr = rclcpp::TimerBase::SharedPtr;

    //tf2
    using TfBuffer = std::shared_ptr<tf2_ros::Buffer>;
    using TfListener = std::shared_ptr<tf2_ros::TransformListener>;
    using TransformStampedMsg = geometry_msgs::msg::TransformStamped;

    //msg
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using MeshMsg = shape_msgs::msg::Mesh;
    using Vector3Msg = geometry_msgs::msg::Vector3;

    using CollisionObjectMsg = moveit_msgs::msg::CollisionObject;
    using SolidPrimitiveMsg = shape_msgs::msg::SolidPrimitive;


  public:
    //costruttore
    SceneBuilderNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
        : rclcpp::Node("scene_builder", rclcpp::NodeOptions(opt).automatically_declare_parameters_from_overrides(true))
    {
        start_timer_ = this->create_wall_timer(
            100ms, std::bind(&SceneBuilderNode::start, this));

        /* SPIEGAZIONE WORKAROUND TIMER:
           Il timer viene utilizzato per ritardare l'inizializzazione dei componenti
           per evitare problemi di inizializzazione con MoveIt! e ROS2, che potrebbero
           causare errori se i componenti vengono inizializzati troppo presto.

           Inoltre, il MoveGroup ha bisogno di this, quindi non è possibile inizializzarlo direttamente nel costruttore.
           Pertanto, si può usare una callback in cui viene richiamato con un timer, a nodo creato, e poi
           questo si autocancella, in modo da eseguire l'inizializzazione solo una volta.

        */
    }

    //callback del servizio per costruire la scena
    void start()
    {
        start_timer_->cancel(); // cancello il timer per evitare che venga richiamato (workaround one-shot)

        // moveit_components init
        // move_group_interface_ =
        //     std::make_unique<MoveGroupInterface>(this->shared_from_this(), PLANNING_GROUP); // PLANNING_GROUP = 'left_arm' (costante in private)
        planning_scene_interface_ = std::make_unique<PlanningSceneInterface>();

        
        // Inizializzazione TF2 Buffer e Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Recupero la directory root del pacchetto ROS 2
        pkg_share_dir = ament_index_cpp::get_package_share_directory(NOME_PACCHETTO);
        billiard_mesh_path = "file://" + pkg_share_dir + "/meshes/billiard/billiard.obj";


        RCLCPP_INFO(this->get_logger(), "Scene builder pronto.");
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



    /* METODI DI MODELLAZIONE SCENA */
    void clearScene()
    {
        // RIMUOVO TUTTO GLI OGGETTI PRESENTI
        // for (auto const& element : planning_scene_interface_->getAttachedObjects())
        // {
        //     move_group_interface_->detachObject(element.first);
        //     this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(1.0));  //per visualizzazione, rimuove un oggetto 1 secondo alla volta
        // }

        auto objects_map = planning_scene_interface_->getObjects();
        std::vector<std::string> obj_keys;
        for (auto const& element : objects_map)
        {
            obj_keys.push_back(element.first);
        }

        planning_scene_interface_->removeCollisionObjects(obj_keys);
        this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(1.0));
    }


    void buildScene()
    {
        clearScene(); // pulisco la scena prima di costruirla, in modo da evitare problemi di oggetti duplicati

    

        /* --- Costruisco mini-tavolo da biliardo --- */ 
        {
            // Vettore per il fattore di scala (1.0 = dimensione originale, regola se i tuoi CAD sono in mm)
            Vector3Msg scale;
            scale.x = 0.001; scale.y = 0.001; scale.z = 0.001;  //da mm->m

            PoseStampedMsg pose;
            pose.header.frame_id = BILLIARD_TABLE_FRAME;
            pose.pose.orientation.w = 1.0;                  //terna è già orientata correttamente da telecamera, quindi non serve ruotarla
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = - POOL_TABLE_FIELD_HEIGHT + eps_floating; //la terna è sul campo, ma l'origine della mesh è sul pavimento, quindi devo abbassare la posizione dell'altezza del campo, ma elevarla di un epsilon per problemi di collision detection con il piano

            // Carico la mesh del biliardo usando il suo percorso
            addMESH(billiard_mesh_path, scale, pose, ID_MINI_POOL_TABLE); 

        }
        

        
        /* --- 2. Carico le palline dinamiche come sfere leggendo le terne da TF2 --- */
        addBallsFromTF();
        
        
    }

  private:
    /* MEMBRI PRIVATI */
    //MoveGroupInterfacePtr move_group_interface_;
    PlanningSceneInterfacePtr planning_scene_interface_;
    TimerPtr start_timer_;

    TfBuffer tf_buffer_;
    TfListener tf_listener_;

    std::promise<void> init_done_;     // segnala al main che start() è completato


    std::string pkg_share_dir;          //path del pacchetto ROS
    std::string billiard_mesh_path;     //path della mesh del biliardo



    /* METODI PRIVATI */

    /*aggiunta di oggetti alla scena*/
    void addMESH(const std::string& file_path, 
                 const Vector3Msg& scale, 
                 const PoseStampedMsg& pose, 
                 const std::string& obj_id
                )
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "#####\nAdding mesh [" << file_path
                                                                    << "]\n pose:\n"
                                                                    << geometry_msgs::msg::to_yaml(pose) << "\n#####");

        // 1. Carica la mesh usando geometric_shapes
        shapes::Mesh* m = shapes::createMeshFromResource(file_path, Eigen::Vector3d(scale.x, scale.y, scale.z));
        if (!m) {
            RCLCPP_ERROR(this->get_logger(), "Impossibile caricare la mesh: %s", file_path.c_str());
            return;
        }

        // 2. Converte la forma in shape_msgs::msg::Mesh
        MeshMsg mesh_msg;
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(m, shape_msg);
        delete m; // Liberiamo la memoria allocata da createMeshFromResource

        mesh_msg = boost::get<MeshMsg>(shape_msg);

        // 3. Prepara il CollisionObject
        CollisionObjectMsg collision_object;
        collision_object.header.frame_id = pose.header.frame_id;
        collision_object.id = obj_id;

        collision_object.meshes.push_back(mesh_msg);
        collision_object.mesh_poses.push_back(pose.pose);
        collision_object.operation = collision_object.ADD;

        // 4. Invia alla Planning Scene
        planning_scene_interface_->applyCollisionObject(collision_object);

        //this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
    }

    void addSPHERE(double radius,
                   PoseStampedMsg pose,
                   const std::string& obj_id
                  )
    {
        /*INPUT
        - radius: raggio della sfera
        - pose: posizione e orientamento dal centro della sfera
        - obj_id: id univoco dell'oggetto da inserire nella scena, usato per identificarlo in seguito
        */

        RCLCPP_INFO_STREAM(this->get_logger(), "#####\nAdding sphere [Radius: " << radius
                                                                    << "]\n pose:\n"
                                                                    << geometry_msgs::msg::to_yaml(pose) << "\n#####");

        CollisionObjectMsg collision_object;
        collision_object.header.frame_id = pose.header.frame_id;
        collision_object.id = obj_id;

        SolidPrimitiveMsg primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1); // La sfera ha solo 1 parametro geometrico (il raggio)
        primitive.dimensions[primitive.SPHERE_RADIUS] = radius;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose.pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_->applyCollisionObject(collision_object);

        //this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
    }
    
    /*individuazione palline dalle terne pubblicate dalla telecamera*/
    void addBallsFromTF()
    {
        // Mappa tra gli ID delle palline che ti aspetti da TF e i loro colori (R, G, B, A)
        const std::map<std::string, std::array<float, 4>> balls_to_find = {
            {ID_WHITE_SOLID_BALL,  {1.0f, 1.0f, 1.0f, 1.0f}}, // Bianco
            {ID_RED_SOLID_BALL,    {1.0f, 0.0f, 0.0f, 1.0f}}, // Rosso
            {ID_BLUE_SOLID_BALL,   {0.0f, 0.0f, 1.0f, 1.0f}}, // Blu
            {ID_YELLOW_SOLID_BALL, {1.0f, 1.0f, 0.0f, 1.0f}}  // Giallo
        };

        //per semplicità, è stato dato come ID il nome della terna

        // Diamo tempo al buffer TF di riempirsi di trasformate ricevute dai nodi di percezione
        this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));

        for (const auto& [ball_id, color] : balls_to_find)
        {
            try {
                // Attende fino a 1.0 secondi che la trasformata sia disponibile nel buffer TF.
                // canTransform blocca l'esecuzione consentendo allo spinner di aggiornare il buffer TF interno.
           
                if (tf_buffer_->canTransform(BILLIARD_TABLE_FRAME, ball_id, tf2::TimePointZero, tf2::durationFromSec(1.0)))
                
                {
                    // Legge la trasformata della pallina rispetto al frame del tavolo/riferimento
                    TransformStampedMsg tf_stamped = 
                        tf_buffer_->lookupTransform(BILLIARD_TABLE_FRAME, ball_id, tf2::TimePointZero);

                    PoseStampedMsg ball_pose;
                    ball_pose.header.frame_id = BILLIARD_TABLE_FRAME;
                    ball_pose.pose.position.x = tf_stamped.transform.translation.x;
                    ball_pose.pose.position.y = tf_stamped.transform.translation.y;
                    ball_pose.pose.position.z = tf_stamped.transform.translation.z + eps_floating; // Elevazione anti-collisione
                    ball_pose.pose.orientation = tf_stamped.transform.rotation;

                    // Aggiunge la sfera e ne imposta il colore
                    addSPHERE(BALL_RADIUS, ball_pose, ball_id);
                    setObjectColor(ball_id, color[0], color[1], color[2], color[3]);

                    RCLCPP_INFO(this->get_logger(), "Pallina [%s] aggiunta con successo da TF.", ball_id.c_str());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Timeout: TF per [%s] rispetto a [%s] non trovata entro 1 secondo.", 
                                ball_id.c_str(), BILLIARD_TABLE_FRAME.c_str());
                }

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Eccezione TF durante la ricerca di [%s] rispetto a [%s]: %s", 
                            ball_id.c_str(), BILLIARD_TABLE_FRAME.c_str(), ex.what());
            }
        }
    }

    //altro di utilities
    void setObjectColor(const std::string& obj_id, 
                        float r, float g, float b, 
                        float a = 1.0f)
    {
        moveit_msgs::msg::ObjectColor color_msg;
        color_msg.id = obj_id;
        color_msg.color.r = r;
        color_msg.color.g = g;
        color_msg.color.b = b;
        color_msg.color.a = a;

        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.object_colors.push_back(color_msg);
        planning_scene_msg.is_diff = true; // modifica solo il colore, non tutta la scena
        planning_scene_interface_->applyPlanningScene(planning_scene_msg);
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SceneBuilderNode>();

    // Executor su thread separato: gestisce il timer one-shot di start()
    // e le callback ROS2, mentre il main esegue la sequenza
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });


    // Aspetta che start() abbia completato l'inizializzazione — sincronizzazione
    node->waitInit();
    //adesso sono sicuro che start() ha inizializzato move_group_ e posso chiamare i metodi

    
    node->buildScene();


    // Termina: shutdown sblocca lo spinner, poi join aspetta che finisca
    rclcpp::shutdown();
    spinner.join();
    return 0;
}