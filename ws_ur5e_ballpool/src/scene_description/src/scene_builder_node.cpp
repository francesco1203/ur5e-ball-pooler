#include <rclcpp/rclcpp.hpp>

#include <memory>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>


#include <Eigen/Dense>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

//per pubblicare terne statiche
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include "scene_description/scene_description.hpp"


//costanti
const std::string PLANNING_GROUP = "left_arm";
const std::string NOME_PACCHETTO = "scene_description"; //nome del pacchetto ROS2, per recuperare la cartella share

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
    using tf2_static_broadcaster_ptr = std::shared_ptr<tf2_ros::StaticTransformBroadcaster>;
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

        
        // inizializzo il broadcaster per le terne statiche
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());


        // Recupero la directory root del pacchetto ROS 2
        pkg_share_dir = ament_index_cpp::get_package_share_directory(NOME_PACCHETTO);


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
            pose.header.frame_id = TERNA_RIFERIMENTO_BILIARDO; //world
            pose.pose.orientation.w = 1.0;
            pose.pose.position.x = mini_pool_table_x;
            pose.pose.position.y = mini_pool_table_y;
            pose.pose.position.z = mini_pool_table_z;

            // Carico la mesh del biliardo usando il percorso "package://" oppure il path assoluto file://
            std::string pool_path = "file://" + pkg_share_dir + "/meshes/pool/pool.obj";
            addMESH(pool_path, scale, pose, ID_MINI_POOL_TABLE); 


            // Pubblica la terna del biliardino
            TransformStampedMsg tf_msg;
            tf_msg.header.stamp = this->get_clock()->now();
            tf_msg.header.frame_id = TERNA_RIFERIMENTO_BILIARDO;          
            tf_msg.child_frame_id = TERNA_SOLIDALE_CENTRO_BILIARDO;       

            //la metto all'altezza del campo, in modo che le palline siano posizionate correttamente
            tf_msg.transform.translation.x = mini_pool_table_x;
            tf_msg.transform.translation.y = mini_pool_table_y;
            tf_msg.transform.translation.z = mini_pool_table_z + pool_table_field_height; 
            tf_msg.transform.rotation.w = 1.0; 

            static_broadcaster_->sendTransform(tf_msg);
        }
        

        /* --- Aggiungo le palline come semplici sfere --- (PREFERIBILE PER FACILITARE COLLISION DETECTION)*/ 
        {

            //Pallina Bianca
            PoseStampedMsg white_ball_pose;
            white_ball_pose.header.frame_id = TERNA_RIFERIMENTO_PALLINE;
            white_ball_pose.pose.orientation.w = 1.0;
            white_ball_pose.pose.position.x = w_ball_x;
            white_ball_pose.pose.position.y = w_ball_y;
            white_ball_pose.pose.position.z = w_ball_z;

            addSPHERE(ball_radius, white_ball_pose, ID_WHITE_BALL);
            setObjectColor(ID_WHITE_BALL, 1.0, 1.0, 1.0, 1.0); // Bianco

            // Pallina Rossa
            PoseStampedMsg red_ball_pose;
            red_ball_pose.header.frame_id = TERNA_RIFERIMENTO_PALLINE;
            red_ball_pose.pose.orientation.w = 1.0;
            red_ball_pose.pose.position.x = r_ball_x;
            red_ball_pose.pose.position.y = r_ball_y;
            red_ball_pose.pose.position.z = r_ball_z;

            addSPHERE(ball_radius, red_ball_pose, ID_RED_BALL);
            setObjectColor(ID_RED_BALL, 1.0, 0.0, 0.0, 1.0); // Rosso

            // Pallina Blu
            PoseStampedMsg blue_ball_pose;
            blue_ball_pose.header.frame_id = TERNA_RIFERIMENTO_PALLINE;
            blue_ball_pose.pose.orientation.w = 1.0;
            blue_ball_pose.pose.position.x = b_ball_x;
            blue_ball_pose.pose.position.y = b_ball_y;
            blue_ball_pose.pose.position.z = b_ball_z;

            addSPHERE(ball_radius, blue_ball_pose, ID_BLUE_BALL);
            setObjectColor(ID_BLUE_BALL, 0.0, 0.0, 1.0, 1.0); // Blu

            // Pallina Gialla
            PoseStampedMsg yellow_ball_pose;
            yellow_ball_pose.header.frame_id = TERNA_RIFERIMENTO_PALLINE;
            yellow_ball_pose.pose.orientation.w = 1.0;
            yellow_ball_pose.pose.position.x = y_ball_x;
            yellow_ball_pose.pose.position.y = y_ball_y;
            yellow_ball_pose.pose.position.z = y_ball_z;

            addSPHERE(ball_radius, yellow_ball_pose, ID_YELLOW_BALL);
            setObjectColor(ID_YELLOW_BALL, 1.0, 1.0, 0.0, 1.0); // Giallo
        }


        /* --- Aggiungo le palline come Mesh --- (NON PREFERIBILE, SOVRACCARICO COLLISION DETECTION)*/ 

        
            //{
            //  // Vettore per il fattore di scala (1.0 = dimensione originale, regola se i tuoi CAD sono in mm)
            // Vector3Msg scale;
            // scale.x = 0.001; scale.y = 0.001; scale.z = 0.001;  //da mm->m

            //  // Pallina Bianca
            // PoseStampedMsg white_ball_pose;
            // white_ball_pose.header.frame_id = TERNA_RIFERIMENTO_PALLINE;
            // white_ball_pose.pose.orientation.w = 1.0;
            // white_ball_pose.pose.position.x = w_ball_x;
            // white_ball_pose.pose.position.y = w_ball_y;
            // white_ball_pose.pose.position.z = w_ball_z;

            // std::string white_ball_path = "file://" + pkg_share_dir + "/meshes/balls/white_ball.obj";
            // addMESH(white_ball_path, scale, white_ball_pose, ID_WHITE_BALL);

            // // Pallina Rossa
            // PoseStampedMsg red_ball_pose;
            // red_ball_pose.header.frame_id = TERNA_RIFERIMENTO_PALLINE;
            // red_ball_pose.pose.orientation.w = 1.0;
            // red_ball_pose.pose.position.x = r_ball_x;
            // red_ball_pose.pose.position.y = r_ball_y;
            // red_ball_pose.pose.position.z = r_ball_z;

            // std::string red_ball_path = "file://" + pkg_share_dir + "/meshes/balls/red_ball.obj";
            // addMESH(red_ball_path, scale, red_ball_pose, ID_RED_BALL);

            // // Pallina Blu
            // PoseStampedMsg blue_ball_pose;
            // blue_ball_pose.header.frame_id = TERNA_RIFERIMENTO_PALLINE;
            // blue_ball_pose.pose.orientation.w = 1.0;
            // blue_ball_pose.pose.position.x = b_ball_x;
            // blue_ball_pose.pose.position.y = b_ball_y;
            // blue_ball_pose.pose.position.z = b_ball_z;

            // std::string blue_ball_path = "file://" + pkg_share_dir + "/meshes/balls/blue_ball.obj";
            // addMESH(blue_ball_path, scale, blue_ball_pose, ID_BLUE_BALL);

            // // Pallina Gialla
            // PoseStampedMsg yellow_ball_pose;
            // yellow_ball_pose.header.frame_id = TERNA_RIFERIMENTO_PALLINE;
            // yellow_ball_pose.pose.orientation.w = 1.0;
            // yellow_ball_pose.pose.position.x = y_ball_x + 0.05; // Offset d'esempio
            // yellow_ball_pose.pose.position.y = y_ball_y;
            // yellow_ball_pose.pose.position.z = y_ball_z;

            // std::string yellow_ball_path = "file://" + pkg_share_dir + "/meshes/balls/yellow_ball.obj";
            // addMESH(yellow_ball_path, scale, yellow_ball_pose, ID_YELLOW_BALL);
            //}
        
    }




  private:
    /* MEMBRI PRIVATI */
    //MoveGroupInterfacePtr move_group_interface_;
    PlanningSceneInterfacePtr planning_scene_interface_;
    tf2_static_broadcaster_ptr static_broadcaster_;
    TimerPtr start_timer_;

    std::promise<void> init_done_;     // segnala al main che start() è completato

    std::string pkg_share_dir;          //path del pacchetto ROS


    /* METODI PRIVATI */
   
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

        this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
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

        this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
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