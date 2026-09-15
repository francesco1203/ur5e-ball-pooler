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
      this->declare_parameter<std::string>("prefix", ""); // <--- NUOVO PARAMETRO

      std::string yaml_path = this->get_parameter("yaml_file_path").as_string();
      std::string prefix = this->get_parameter("prefix").as_string(); // <--- LETTURA PARAMETRO

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

      // --- LOG BILIARDO ---
      double table_x = config["billiard_table"]["pos"][0].as<double>();
      double table_y = config["billiard_table"]["pos"][1].as<double>();
      double table_yaw = config["billiard_table"]["yaw_angle_rad"].as<double>();
      
      RCLCPP_INFO(this->get_logger(), "Biliardo letto da file -> posizione: [%.2f, %.2f], yaw: %.2f rad", 
                  table_x, table_y, table_yaw);

      // --- TF 1: world -> billiard_table ---
      TransformStampedMsg t_table;
      t_table.header.stamp = now;
      t_table.header.frame_id = WORLD_FRAME; // Il world rimane fisso senza prefisso
      // Aggiungiamo il prefisso al child
      t_table.child_frame_id = prefix + BILLIARD_TABLE_FRAME; 

      t_table.transform.translation.x = table_x;
      t_table.transform.translation.y = table_y;
      t_table.transform.translation.z = POOL_TABLE_FIELD_HEIGHT;

      tf2::Quaternion q_table;
      q_table.setRPY(0, 0, table_yaw + M_PI);
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
          // Il genitore ora ha il prefisso
          t_ball.header.frame_id = prefix + BILLIARD_TABLE_FRAME; 

          std::string color = ball["color"].as<std::string>();
          double ball_x = ball["pos"][0].as<double>();
          double ball_y = ball["pos"][1].as<double>();

          RCLCPP_INFO(this->get_logger(), "  - Pallina '%s' letta -> posizione: [%.3f, %.3f]", 
                      color.c_str(), ball_x, ball_y);

          // Aggiungiamo il prefisso anche alla pallina
          t_ball.child_frame_id = prefix + color + "_" + SOLID_BALL_FRAME;

          t_ball.transform.translation.x = ball_x;
          t_ball.transform.translation.y = ball_y;
          t_ball.transform.translation.z = BALL_RADIUS;

          t_ball.transform.rotation.x = 0.0;
          t_ball.transform.rotation.y = 0.0;
          t_ball.transform.rotation.z = 0.0;
          t_ball.transform.rotation.w = 1.0;

          transforms.push_back(t_ball);
        }
      }

      // --- TF 3: billiard_table -> holes (buche) ---
      double half_l = POOL_TABLE_FIELD_LENGTH / 2.0;
      double half_w = POOL_TABLE_FIELD_WIDTH / 2.0;

      struct HoleDef { std::string name; double x; double y; };
      std::vector<HoleDef> holes = {
          {"hole_top_left",     -half_l + 0.01, -half_w + 0.02},
          {"hole_top_right",    -half_l + 0.01,  half_w - 0.02},
          {"hole_mid_left",      0.0,    -half_w + 0.0125},
          {"hole_mid_right",     0.0,     half_w - 0.0125},
          {"hole_bottom_left",   half_l - 0.01, -half_w + 0.02},
          {"hole_bottom_right",  half_l - 0.01,  half_w - 0.02}
      };

      for (const auto& hole : holes) {
          TransformStampedMsg t_hole;
          t_hole.header.stamp = now;
          // Anche qui, il genitore ha il prefisso
          t_hole.header.frame_id = prefix + BILLIARD_TABLE_FRAME;
          // E la singola buca ha il prefisso
          t_hole.child_frame_id = prefix + hole.name;

          t_hole.transform.translation.x = hole.x;
          t_hole.transform.translation.y = hole.y;
          t_hole.transform.translation.z = 0.0; 

          t_hole.transform.rotation.x = 0.0;
          t_hole.transform.rotation.y = 0.0;
          t_hole.transform.rotation.z = 0.0;
          t_hole.transform.rotation.w = 1.0;

          transforms.push_back(t_hole);
      }
      
      RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");

      // Invio di TUTTE le trasformate statiche
      static_tf_broadcaster_->sendTransform(transforms);
      RCLCPP_INFO(this->get_logger(), "Inviate %zu trasformate statiche TF con successo.", transforms.size());
    }

  private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeCamera>();
  RCLCPP_INFO(node->get_logger(), "Nodo avviato, mantengo attivo per pubblicazione TF statica.");
  rclcpp::spin(node);
  RCLCPP_INFO(node->get_logger(), "Chiusura nodo.");
  rclcpp::shutdown();
  return 0;
}