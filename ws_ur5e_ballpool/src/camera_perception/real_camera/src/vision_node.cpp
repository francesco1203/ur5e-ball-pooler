// ============================================================
//  vision_node.cpp
//  Rilevamento Tavolo e Palline tramite OpenCV + cv_bridge
// ============================================================

#include <memory>
#include <chrono>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Assicurati che in questi header ci siano le macro:
#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"

class VisionNode : public rclcpp::Node
{
public:
    VisionNode() : Node("vision_node")
    {
        // Inizializza TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscriber
        sub_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10, std::bind(&VisionNode::rgb_callback, this, std::placeholders::_1));
        
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, std::bind(&VisionNode::depth_callback, this, std::placeholders::_1));

        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10, std::bind(&VisionNode::info_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Vision Node avviato! In attesa delle immagini...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    cv::Mat current_depth_frame_;
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
    bool has_camera_info_ = false;

    // --- 1. CALLBACK PARAMETRI INTRINSECI ---
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        cx_ = msg->k[2];
        fy_ = msg->k[4];
        cy_ = msg->k[5];
        has_camera_info_ = true;
    }

    // --- 2. CALLBACK PROFONDITA' (DEPTH) ---
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Converte il messaggio ROS nella matrice Depth OpenCV (Float 32-bit, metri)
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            current_depth_frame_ = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge depth: %s", e.what());
        }
    }

    // --- 3. CALLBACK IMMAGINE COLOR (RGB) ---
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!has_camera_info_ || current_depth_frame_.empty()) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            return;
        }

        cv::Mat hsv_frame;
        cv::cvtColor(cv_ptr->image, hsv_frame, cv::COLOR_BGR2HSV);

        // ========================================================
        // A. RILEVAMENTO TAVOLO (Panno Verde)
        // ========================================================
        cv::Mat green_mask;
        // Range da calibrare in base alle luci del tuo ambiente
        cv::inRange(hsv_frame, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), green_mask);

        std::vector<std::vector<cv::Point>> table_contours;
        cv::findContours(green_mask, table_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_table_area = 0.0;
        int best_table_idx = -1;

        for (size_t i = 0; i < table_contours.size(); i++) {
            double area = cv::contourArea(table_contours[i]);
            if (area > 5000.0 && area > max_table_area) { 
                max_table_area = area;
                best_table_idx = i;
            }
        }

        if (best_table_idx != -1) {
            cv::RotatedRect table_rect = cv::minAreaRect(table_contours[best_table_idx]);
            cv::Point2f center_uv = table_rect.center;

            // Prende la profondità (media 5x5 per robustezza)
            float z_table = get_average_depth(center_uv.x, center_uv.y);

            if (z_table > 0.0) {
                // Coordinate 3D
                double x_t = (center_uv.x - cx_) * z_table / fx_;
                double y_t = (center_uv.y - cy_) * z_table / fy_;

                // Calcolo Angolo Yaw per allineare gli assi X e Y del tavolo
                double angle_deg = table_rect.angle;
                if (table_rect.size.width < table_rect.size.height) {
                    angle_deg += 90.0;
                }
                double yaw_rad = angle_deg * (M_PI / 180.0);

                publish_table_tf(BILLIARD_TABLE_FRAME, x_t, y_t, z_table, yaw_rad);
            }
        }

        // ========================================================
        // B. RILEVAMENTO PALLINA ROSSA
        // ========================================================
        cv::Mat mask1, mask2, red_mask;
        cv::inRange(hsv_frame, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv_frame, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
        red_mask = mask1 | mask2;

        process_and_publish_ball(red_mask, RED_SOLID_BALL_FRAME);

        // ========================================================
        // C. RILEVAMENTO PALLINA BIANCA
        // ========================================================
        cv::Mat white_mask;
        // Bassa saturazione (vicino a 0), alta luminosità (vicino a 255)
        cv::inRange(hsv_frame, cv::Scalar(0, 0, 200), cv::Scalar(180, 50, 255), white_mask);
        
        process_and_publish_ball(white_mask, WHITE_SOLID_BALL_FRAME);
    }

    // --- FUNZIONI DI SUPPORTO ---

    void process_and_publish_ball(const cv::Mat& mask, const std::string& frame_name)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 100.0) { // Filtra il rumore piccolo
                cv::Moments m = cv::moments(contour);
                double u = m.m10 / m.m00;
                double v = m.m01 / m.m00;

                float z = get_average_depth(u, v);
                if (z > 0.0) {
                    double x_c = (u - cx_) * z / fx_;
                    double y_c = (v - cy_) * z / fy_;
                    
                    publish_ball_tf(frame_name, x_c, y_c, z);
                    return; // Trovata la palla più grande, esci dal ciclo
                }
            }
        }
    }

    // Legge la profondità calcolando la media in un quadrato 5x5 pixel
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

    void publish_ball_tf(const std::string& child_frame, double x, double y, double z)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "camera_color_optical_frame";
        t.child_frame_id = child_frame;

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;

        t.transform.rotation.w = 1.0; // Sfera: nessuna rotazione
        tf_broadcaster_->sendTransform(t);
    }

    void publish_table_tf(const std::string& child_frame, double x, double y, double z, double yaw_rad)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "camera_color_optical_frame";
        t.child_frame_id = child_frame;

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;

        // Quaternione per la rotazione attorno all'asse Z della telecamera
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = std::sin(yaw_rad / 2.0);
        t.transform.rotation.w = std::cos(yaw_rad / 2.0);

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}