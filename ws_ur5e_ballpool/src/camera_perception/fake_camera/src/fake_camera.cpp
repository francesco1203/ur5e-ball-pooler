#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <yaml-cpp/yaml.h>

#include "shared_headers_pkg/scene_description.hpp"

class FakeCamera : public rclcpp::Node
{
public:
  using TransformStampedMsg = geometry_msgs::msg::TransformStamped;

  FakeCamera() : Node("fake_camera")
  {
    // 1. Dichiarazione dei parametri
    this->declare_parameter<std::string>("yaml_file_path", "config/fake_camera_config.yaml");
    std::string yaml_path = this->get_parameter("yaml_file_path").as_string();

    // 2. Inizializzazione del Broadcaster STATICO (fondamentale per ONE-SHOT)
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 3. Esecuzione immediata della logica
    RCLCPP_INFO(this->get_logger(), "Fake Camera avviata in modalità ONE-SHOT.");
    RCLCPP_INFO(this->get_logger(), "Caricamento file YAML da: %s", yaml_path.c_str());

    YAML::Node config;
    try {
      config = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento del file YAML: %s", e.what());
      return;
    }

    // Vettore per raccogliere tutte le trasformate e inviarle in un unico messaggio
    std::vector<TransformStampedMsg> transforms;
    rclcpp::Time now = this->get_clock()->now();

    // --- TF 1: world -> billiard_table ---
    TransformStampedMsg t_table;
    t_table.header.stamp = now;
    t_table.header.frame_id = WORLD_FRAME;
    t_table.child_frame_id = BILLIARD_TABLE_FRAME;

    t_table.transform.translation.x = config["billiard_table"]["pos"][0].as<double>();
    t_table.transform.translation.y = config["billiard_table"]["pos"][1].as<double>();
    t_table.transform.translation.z = POOL_TABLE_FIELD_HEIGHT; // sposto la terna sul campo

    tf2::Quaternion q_table;
    q_table.setRPY(0, 0, config["billiard_table"]["yaw_angle_rad"].as<double>());
    t_table.transform.rotation.x = q_table.x();
    t_table.transform.rotation.y = q_table.y();
    t_table.transform.rotation.z = q_table.z();
    t_table.transform.rotation.w = q_table.w();

    transforms.push_back(t_table);

    // --- TF 2: billiard_table -> balls ---
    if (config["balls"]) {
      for (const auto& ball : config["balls"]) {
        TransformStampedMsg t_ball;
        t_ball.header.stamp = now;
        t_ball.header.frame_id = BILLIARD_TABLE_FRAME;

        std::string color = ball["color"].as<std::string>();
        t_ball.child_frame_id = color + "_" + SOLID_BALL_FRAME;

        t_ball.transform.translation.x = ball["pos"][0].as<double>();
        t_ball.transform.translation.y = ball["pos"][1].as<double>();
        t_ball.transform.translation.z = BALL_RADIUS;

        // Le palline sono sferiche, manteniamo la rotazione neutra
        t_ball.transform.rotation.x = 0.0;
        t_ball.transform.rotation.y = 0.0;
        t_ball.transform.rotation.z = 0.0;
        t_ball.transform.rotation.w = 1.0;

        transforms.push_back(t_ball);
      }
    }

    // Invio di TUTTE le trasformate statiche
    static_tf_broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(this->get_logger(), "Inviate %zu trasformate statiche TF con successo.", transforms.size());
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};


/*NOTA IMPORTANTE: fake_camera deve rimanere a girare, perché altrimenti le terne pubblicate verranno perse con lui durante la sua chiusura */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Istanziamo il nodo (che esegue tutto nel costruttore)
  auto node = std::make_shared<FakeCamera>();

  RCLCPP_INFO(node->get_logger(), "Nodo avviato, mantengo attivo per pubblicazione TF statica.");

  // Blocca ed esegue il nodo finché non riceve SIGINT (Ctrl+C)
  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "Chiusura nodo.");
  rclcpp::shutdown();
  return 0;
}