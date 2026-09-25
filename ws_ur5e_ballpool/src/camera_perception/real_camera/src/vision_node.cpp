// ============================================================
//  vision_node.cpp - SOLUZIONE DEFINITIVA (ON-DEMAND)
//  Calibrazione Tavolo e Palline tramite Servizi ROS 2
// ============================================================
#include <new>
#include <iterator>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <chrono>
#include <vector>
#include <cmath>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h> 

// Servizi di trigger
#include "std_srvs/srv/trigger.hpp"

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"

using namespace std::chrono_literals;

enum class VisionState {
    IDLE,
    CALIBRATING_TABLE,
    DETECTING_BALLS
};

class VisionNode : public rclcpp::Node
{
public:
    VisionNode() : Node("vision_node")
    {
        this->declare_parameter<int>("required_samples", 200);
        required_samples_ = this->get_parameter("required_samples").as_int();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("perception_markers", 10);

        sub_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
            RGB_IMAGE_TOPIC, rclcpp::SensorDataQoS(), std::bind(&VisionNode::rgb_callback, this, std::placeholders::_1));
        
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            DEPTH_IMAGE_TOPIC, rclcpp::SensorDataQoS(), std::bind(&VisionNode::depth_callback, this, std::placeholders::_1));

        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            CAMERA_INFO_TOPIC, rclcpp::SensorDataQoS(), std::bind(&VisionNode::info_callback, this, std::placeholders::_1));

        // SERVIZI PER RICEVERE ORDINI DAL TASK NODE
        srv_calibrate_table_ = this->create_service<std_srvs::srv::Trigger>(
            "vision/calibrate_table", std::bind(&VisionNode::handle_calibrate_table, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_detect_balls_ = this->create_service<std_srvs::srv::Trigger>(
            "vision/detect_balls", std::bind(&VisionNode::handle_detect_balls, this, std::placeholders::_1, std::placeholders::_2));

        // Timer per mantenere vive le TF anche in stato di IDLE
        timer_tf_keepalive_ = this->create_wall_timer(
            100ms, std::bind(&VisionNode::keepalive_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Vision Node avviato in stato IDLE. In attesa di chiamate ai servizi...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_calibrate_table_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_detect_balls_;
    rclcpp::TimerBase::SharedPtr timer_tf_keepalive_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    cv::Mat current_depth_frame_;
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
    bool has_camera_info_ = false;
    
    // Variabili di Stato
    VisionState current_state_ = VisionState::IDLE;
    int required_samples_;
    
    // Buffer per calibrazione tavolo
    std::vector<double> z_measurements_;
    std::vector<double> x_measurements_;
    std::vector<double> y_measurements_;
    std::vector<double> yaw_measurements_;

    // Dati Congelati
    bool table_calibrated_ = false;
    double frozen_table_x_ = 0.0;
    double frozen_table_y_ = 0.0;
    double frozen_table_z_ = 0.0; 
    double frozen_table_yaw_ = 0.0;
    
    // Mappa per salvare le TF congelate delle palline rilevate
    std::map<std::string, geometry_msgs::msg::TransformStamped> frozen_balls_;


    // ==========================================
    // CALLBACK DEI SERVIZI
    // ==========================================
    void handle_calibrate_table(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        z_measurements_.clear(); x_measurements_.clear(); y_measurements_.clear(); yaw_measurements_.clear();
        current_state_ = VisionState::CALIBRATING_TABLE;
        RCLCPP_INFO(this->get_logger(), "Ricevuta richiesta calibrazione TAVOLO. Inizio campionamento...");
        res->success = true;
        res->message = "Calibrazione tavolo avviata asincronamente.";
    }

    void handle_detect_balls(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (!table_calibrated_) {
            res->success = false;
            res->message = "Errore: Calibrare prima il tavolo.";
            RCLCPP_WARN(this->get_logger(), "Tentativo di rilevare le palline prima del tavolo.");
            return;
        }
        
        frozen_balls_.clear(); // Svuota la memoria delle vecchie palline
        current_state_ = VisionState::DETECTING_BALLS;
        RCLCPP_INFO(this->get_logger(), "Ricevuta richiesta identificazione PALLINE. Inizio rilevamento...");
        res->success = true;
        res->message = "Rilevamento palline avviato asincronamente.";
    }


    // ==========================================
    // KEEPALIVE TF TIMER
    // ==========================================
    void keepalive_timer_callback()
    {
        rclcpp::Time now = this->get_clock()->now();
        
        if (table_calibrated_) {
            publish_table_tf(BILLIARD_TABLE_FRAME, frozen_table_x_, frozen_table_y_, frozen_table_z_, frozen_table_yaw_, now);
            publish_holes_fixed_geometry(now);
        }
        
        for (const auto& ball : frozen_balls_) {
            geometry_msgs::msg::TransformStamped t = ball.second;
            t.header.stamp = now;
            tf_broadcaster_->sendTransform(t);
        }
        
        if (table_calibrated_) {
            publish_rviz_markers(now);
        }
    }


    // ==========================================
    // CALLBACK IMMAGINI E VISIONE
    // ==========================================
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0]; cx_ = msg->k[2]; fy_ = msg->k[4]; cy_ = msg->k[5];
        has_camera_info_ = true;
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (current_state_ == VisionState::IDLE) return;

        try {
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || msg->encoding == "16UC1") {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
                cv_ptr->image.convertTo(current_depth_frame_, CV_32FC1, 0.001);
            } 
            else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 || msg->encoding == "32FC1") {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
                current_depth_frame_ = cv_ptr->image.clone(); 
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge depth: %s", e.what());
        }
    }

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (current_state_ == VisionState::IDLE || !has_camera_info_ || current_depth_frame_.empty()) return;

        cv_bridge::CvImagePtr cv_ptr;
        try { cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); } 
        catch (cv_bridge::Exception& e) { return; }

        rclcpp::Time img_stamp = msg->header.stamp;
        cv::Mat hsv_frame;
        cv::cvtColor(cv_ptr->image, hsv_frame, cv::COLOR_BGR2HSV);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

        // 1. STATO: CALIBRATING TABLE
        if (current_state_ == VisionState::CALIBRATING_TABLE) {
            cv::Mat green_mask;
            cv::inRange(hsv_frame, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), green_mask);

            std::vector<std::vector<cv::Point>> table_contours;
            cv::findContours(green_mask, table_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double max_table_area = 0.0;
            int best_table_idx = -1;

            for (size_t i = 0; i < table_contours.size(); i++) {
                double area = cv::contourArea(table_contours[i]);
                if (area > 5000.0 && area > max_table_area) { 
                    max_table_area = area; best_table_idx = i;
                }
            }

            if (best_table_idx != -1) {
                cv::RotatedRect table_rect = cv::minAreaRect(table_contours[best_table_idx]);
                cv::Point2f center_uv = table_rect.center;
                float z_table_live = get_average_depth(center_uv.x, center_uv.y);

                if (z_table_live > 0.0) {
                    double x_t_live = (center_uv.x - cx_) * z_table_live / fx_;
                    double y_t_live = (center_uv.y - cy_) * z_table_live / fy_;

                    cv::Point2f pts[4]; table_rect.points(pts);
                    double max_dist = 0.0; cv::Point2f p1, p2;
                    for (int i = 0; i < 4; i++) {
                        double dist = cv::norm(pts[i] - pts[(i + 1) % 4]);
                        if (dist > max_dist) { max_dist = dist; p1 = pts[i]; p2 = pts[(i + 1) % 4]; }
                    }
                    if (p1.x > p2.x) std::swap(p1, p2);
                    double yaw_rad_live = std::atan2(p2.y - p1.y, p2.x - p1.x);

                    z_measurements_.push_back(z_table_live);
                    x_measurements_.push_back(x_t_live);
                    y_measurements_.push_back(y_t_live);
                    yaw_measurements_.push_back(yaw_rad_live);
                    
                    if (z_measurements_.size() >= static_cast<size_t>(required_samples_)){
                        double sum_z = 0.0, sum_x = 0.0, sum_y = 0.0;
                        double sum_sin = 0.0, sum_cos = 0.0;
                        
                        for (int i = 0; i < required_samples_; i++) {
                            sum_z += z_measurements_[i]; sum_x += x_measurements_[i]; sum_y += y_measurements_[i];
                            sum_sin += std::sin(yaw_measurements_[i]); sum_cos += std::cos(yaw_measurements_[i]);
                        }
                        
                        frozen_table_z_ = sum_z / required_samples_;
                        frozen_table_x_ = sum_x / required_samples_;
                        frozen_table_y_ = sum_y / required_samples_;
                        frozen_table_yaw_ = std::atan2(sum_sin, sum_cos);
                        table_calibrated_ = true;

                        current_state_ = VisionState::IDLE;
                        RCLCPP_INFO(this->get_logger(), "Tavolo CALIBRATO e CONGELATO. Torno in IDLE.");
                    }
                }
            }
            return;
        }

        // 2. STATO: DETECTING BALLS
        if (current_state_ == VisionState::DETECTING_BALLS && table_calibrated_) {
            cv::Mat blurred_hsv;
            cv::GaussianBlur(hsv_frame, blurred_hsv, cv::Size(5, 5), 0);
            double ball_min_area = 80.0; 

            cv::Mat mask1, mask2, red_mask;
            cv::inRange(blurred_hsv, cv::Scalar(0, 80, 20), cv::Scalar(10, 255, 255), mask1);
            cv::inRange(blurred_hsv, cv::Scalar(170, 80, 20), cv::Scalar(180, 255, 255), mask2);
            red_mask = mask1 | mask2;
            cv::dilate(red_mask, red_mask, dilate_kernel); cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
            process_and_freeze_ball(red_mask, RED_SOLID_BALL_FRAME, ball_min_area, img_stamp);

            cv::Mat orange_mask, red_inv;
            cv::inRange(blurred_hsv, cv::Scalar(10, 80, 20), cv::Scalar(35, 255, 255), orange_mask);
            cv::bitwise_not(red_mask, red_inv); cv::bitwise_and(orange_mask, red_inv, orange_mask);
            cv::dilate(orange_mask, orange_mask, dilate_kernel); cv::morphologyEx(orange_mask, orange_mask, cv::MORPH_CLOSE, kernel);
            process_and_freeze_ball(orange_mask, YELLOW_SOLID_BALL_FRAME, ball_min_area, img_stamp);

            cv::Mat blue_mask;
            cv::inRange(blurred_hsv, cv::Scalar(100, 80, 20), cv::Scalar(130, 255, 255), blue_mask);
            cv::dilate(blue_mask, blue_mask, dilate_kernel); cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);
            process_and_freeze_ball(blue_mask, BLUE_SOLID_BALL_FRAME, ball_min_area, img_stamp);

            cv::Mat white_mask, all_colors_inv;
            cv::inRange(blurred_hsv, cv::Scalar(0, 0, 130), cv::Scalar(180, 50, 255), white_mask);
            cv::Mat all_colors = red_mask | orange_mask | blue_mask;
            cv::bitwise_not(all_colors, all_colors_inv); cv::bitwise_and(white_mask, all_colors_inv, white_mask);
            cv::dilate(white_mask, white_mask, dilate_kernel); cv::morphologyEx(white_mask, white_mask, cv::MORPH_CLOSE, kernel);
            process_and_freeze_ball(white_mask, WHITE_SOLID_BALL_FRAME, ball_min_area, img_stamp);

            current_state_ = VisionState::IDLE; 
            RCLCPP_INFO(this->get_logger(), "Palline RILEVATE e CONGELATE (%zu trovate). Torno in IDLE.", frozen_balls_.size());
        }
    }


    // ==========================================
    // FUNZIONI HELPER E TF
    // ==========================================
    void process_and_freeze_ball(const cv::Mat& mask, const std::string& frame_name, double min_area, rclcpp::Time stamp)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_area = 0.0;
        int best_idx = -1;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > min_area && area > max_area) {
                max_area = area;
                best_idx = i;
            }
        }

        if (best_idx == -1) return;

        cv::Moments m = cv::moments(contours[best_idx]);
        double u = m.m10 / m.m00;
        double v = m.m01 / m.m00;

        double z_ball_center = frozen_table_z_ - BALL_RADIUS; 

        double x_c = (u - cx_) * z_ball_center / fx_;
        double y_c = (v - cy_) * z_ball_center / fy_;
        
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp; 
        t.header.frame_id = CAMERA_FRAME;
        t.child_frame_id = frame_name;
        t.transform.translation.x = x_c;
        t.transform.translation.y = y_c;
        t.transform.translation.z = z_ball_center;
        
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, frozen_table_yaw_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        frozen_balls_[frame_name] = t;
    }

    float get_average_depth(int u_c, int v_c)
    {
        float z_sum = 0.0f;
        int valid_pixels = 0;
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                int px = u_c + dx;
                int py = v_c + dy;
                if (px >= 0 && px < current_depth_frame_.cols && py >= 0 && py < current_depth_frame_.rows) {
                    float val = current_depth_frame_.at<float>(py, px);
                    if (!std::isnan(val) && val > 0.1f) {
                        z_sum += val;
                        valid_pixels++;
                    }
                }
            }
        }
        return (valid_pixels > 0) ? (z_sum / valid_pixels) : -1.0f;
    }


    void publish_table_tf(const std::string& child_frame, double x, double y, double z, double yaw_rad, rclcpp::Time stamp)
    {
        geometry_msgs::msg::TransformStamped t_out;
        t_out.header.stamp = stamp;
        t_out.header.frame_id = CAMERA_FRAME;
        t_out.child_frame_id = child_frame;

        t_out.transform.translation.x = x;
        t_out.transform.translation.y = y;
        t_out.transform.translation.z = z;

        try {
            geometry_msgs::msg::TransformStamped t_world_in_cam = 
                tf_buffer_->lookupTransform(CAMERA_FRAME, "world", tf2::TimePointZero);

            tf2::Quaternion q_world_in_cam;
            tf2::fromMsg(t_world_in_cam.transform.rotation, q_world_in_cam);

            tf2::Quaternion q_table_in_world;
            q_table_in_world.setRPY(0.0, 0.0, M_PI - yaw_rad);

            tf2::Quaternion q_final = q_world_in_cam * q_table_in_world;
            q_final.normalize();

            t_out.transform.rotation.x = q_final.x();
            t_out.transform.rotation.y = q_final.y();
            t_out.transform.rotation.z = q_final.z();
            t_out.transform.rotation.w = q_final.w();

            tf_broadcaster_->sendTransform(t_out);
        } catch (const tf2::TransformException & ex) {
            // Silenzioso in attesa dell'albero TF
        }
    }

    void publish_holes_fixed_geometry(rclcpp::Time stamp)
    {
        double half_l = POOL_TABLE_FIELD_LENGTH / 2.0;  
        double half_w = POOL_TABLE_FIELD_WIDTH / 2.0;   
         
        double corner_inset_x = 0.01;  
        double corner_inset_y = 0.02;  
        double mid_inset_y = 0.0125;     

        struct HoleDef { std::string name; double x; double y; }; 
        std::vector<HoleDef> holes = { 
            {HOLE_TOP_LEFT_FRAME,     -half_l + corner_inset_x, -half_w + corner_inset_y}, 
            {HOLE_TOP_RIGHT_FRAME,    -half_l + corner_inset_x,  half_w - corner_inset_y}, 
            {HOLE_MID_LEFT_FRAME,      0.0,                     -half_w + mid_inset_y}, 
            {HOLE_MID_RIGHT_FRAME,     0.0,                      half_w - mid_inset_y}, 
            {HOLE_BOTTOM_LEFT_FRAME,   half_l - corner_inset_x, -half_w + corner_inset_y}, 
            {HOLE_BOTTOM_RIGHT_FRAME,  half_l - corner_inset_x,  half_w - corner_inset_y} 
        }; 

        for (const auto& hole : holes) { 
            geometry_msgs::msg::TransformStamped t_hole; 
            t_hole.header.stamp = stamp; 
            t_hole.header.frame_id = BILLIARD_TABLE_FRAME;  
            t_hole.child_frame_id = hole.name; 
            t_hole.transform.translation.x = hole.x; 
            t_hole.transform.translation.y = hole.y; 
            t_hole.transform.translation.z = 0.0;  
            t_hole.transform.rotation.w = 1.0; 
            tf_broadcaster_->sendTransform(t_hole);  
        }

        // VERTICI ESTERNI
        auto publish_corner = [&](const std::string& frame_id, double x, double y) {
            geometry_msgs::msg::TransformStamped t_corner;
            t_corner.header.stamp = stamp;
            t_corner.header.frame_id = BILLIARD_TABLE_FRAME; 
            t_corner.child_frame_id = frame_id;
            t_corner.transform.translation.x = x;
            t_corner.transform.translation.y = y;
            t_corner.transform.translation.z = POOL_TABLE_HEIGHT - POOL_TABLE_FIELD_HEIGHT; 
            t_corner.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t_corner);
        };

        publish_corner(CORNER_BOTTOM_RIGHT_FRAME,  POOL_TABLE_LENGTH / 2,  POOL_TABLE_WIDTH / 2);
        publish_corner(CORNER_TOP_RIGHT_FRAME,    -POOL_TABLE_LENGTH / 2,  POOL_TABLE_WIDTH / 2);
        publish_corner(CORNER_TOP_LEFT_FRAME,     -POOL_TABLE_LENGTH / 2, -POOL_TABLE_WIDTH / 2);
        publish_corner(CORNER_BOTTOM_LEFT_FRAME,   POOL_TABLE_LENGTH / 2, -POOL_TABLE_WIDTH / 2);
    }

    void publish_rviz_markers(rclcpp::Time stamp)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        auto create_ball_marker = [&](const std::string& frame_id, int id, float r, float g, float b) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = frame_id;
            marker.ns = "balls";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = 0.0; marker.pose.position.y = 0.0; marker.pose.position.z = 0.0; 
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.025; marker.scale.y = 0.025; marker.scale.z = 0.025;
            marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration(0, 500000000); 
            return marker;
        };

        if (frozen_balls_.find(WHITE_SOLID_BALL_FRAME) != frozen_balls_.end()) 
            marker_array.markers.push_back(create_ball_marker(WHITE_SOLID_BALL_FRAME, 0, 1.0, 1.0, 1.0));
        if (frozen_balls_.find(RED_SOLID_BALL_FRAME) != frozen_balls_.end()) 
            marker_array.markers.push_back(create_ball_marker(RED_SOLID_BALL_FRAME, 1, 1.0, 0.0, 0.0));
        if (frozen_balls_.find(BLUE_SOLID_BALL_FRAME) != frozen_balls_.end()) 
            marker_array.markers.push_back(create_ball_marker(BLUE_SOLID_BALL_FRAME, 2, 0.0, 0.0, 1.0));
        if (frozen_balls_.find(YELLOW_SOLID_BALL_FRAME) != frozen_balls_.end()) 
            marker_array.markers.push_back(create_ball_marker(YELLOW_SOLID_BALL_FRAME, 3, 1.0, 0.6, 0.0));

        std::vector<std::string> holes = {
            HOLE_TOP_LEFT_FRAME, HOLE_TOP_RIGHT_FRAME, HOLE_MID_LEFT_FRAME,
            HOLE_MID_RIGHT_FRAME, HOLE_BOTTOM_LEFT_FRAME, HOLE_BOTTOM_RIGHT_FRAME
        };
        for (size_t i = 0; i < holes.size(); i++) {
            visualization_msgs::msg::Marker hole;
            hole.header.stamp = stamp; hole.header.frame_id = holes[i];
            hole.ns = "holes"; hole.id = 20 + i;
            hole.type = visualization_msgs::msg::Marker::CYLINDER; hole.action = visualization_msgs::msg::Marker::ADD;
            hole.pose.position.x = 0.0; hole.pose.position.y = 0.0; hole.pose.position.z = 0.0; 
            hole.pose.orientation.w = 1.0;
            hole.scale.x = 0.04; hole.scale.y = 0.04; hole.scale.z = 0.005;
            hole.color.r = 0.1; hole.color.g = 0.1; hole.color.b = 0.1; hole.color.a = 1.0;
            hole.lifetime = rclcpp::Duration(0, 500000000);
            marker_array.markers.push_back(hole);
        }

        visualization_msgs::msg::Marker corner_marker;
        corner_marker.header.stamp = stamp; corner_marker.header.frame_id = CORNER_BOTTOM_RIGHT_FRAME;
        corner_marker.ns = "corners"; corner_marker.id = 50;
        corner_marker.type = visualization_msgs::msg::Marker::SPHERE; corner_marker.action = visualization_msgs::msg::Marker::ADD;
        corner_marker.pose.position.x = 0.0; corner_marker.pose.position.y = 0.0; corner_marker.pose.position.z = 0.0; 
        corner_marker.pose.orientation.w = 1.0;
        corner_marker.scale.x = 0.02; corner_marker.scale.y = 0.02; corner_marker.scale.z = 0.02;
        corner_marker.color.r = 1.0; corner_marker.color.g = 0.0; corner_marker.color.b = 1.0; corner_marker.color.a = 1.0;
        corner_marker.lifetime = rclcpp::Duration(0, 500000000);
        marker_array.markers.push_back(corner_marker);

        visualization_msgs::msg::Marker table_marker;
        table_marker.header.stamp = stamp; table_marker.header.frame_id = BILLIARD_TABLE_FRAME;
        table_marker.ns = "table_surface"; table_marker.id = 10;
        table_marker.type = visualization_msgs::msg::Marker::CUBE; table_marker.action = visualization_msgs::msg::Marker::ADD;
        table_marker.pose.position.x = 0.0; table_marker.pose.position.y = 0.0; table_marker.pose.position.z = -0.005; 
        table_marker.pose.orientation.w = 1.0;
        table_marker.scale.x = POOL_TABLE_FIELD_LENGTH; table_marker.scale.y = POOL_TABLE_FIELD_WIDTH; table_marker.scale.z = 0.01; 
        table_marker.color.r = 0.0; table_marker.color.g = 0.4; table_marker.color.b = 0.0; table_marker.color.a = 0.8;
        table_marker.lifetime = rclcpp::Duration(0, 500000000);
        marker_array.markers.push_back(table_marker);

        pub_markers_->publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}