// ============================================================
//  vision_node.cpp - SOLUZIONE A (HSV ROI & Morfologia)
// ============================================================
#include <new>
#include <iterator>
#include <cstdint>
#include <cstddef>

#include <memory>
#include <chrono>

#include <memory>
#include <chrono>
#include <vector>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "shared_headers_pkg/ros2_architecture.hpp"
#include "shared_headers_pkg/scene_description.hpp"

class VisionNode : public rclcpp::Node
{
public:
    VisionNode() : Node("vision_node")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

        sub_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", sensor_qos, std::bind(&VisionNode::rgb_callback, this, std::placeholders::_1));
        
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", sensor_qos, std::bind(&VisionNode::depth_callback, this, std::placeholders::_1));

        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", sensor_qos, std::bind(&VisionNode::info_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Vision Node avviato (Soluzione A). In attesa dei dati...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    cv::Mat current_depth_frame_;
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
    bool has_camera_info_ = false;

    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        cx_ = msg->k[2];
        fy_ = msg->k[4];
        cy_ = msg->k[5];
        has_camera_info_ = true;
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                cv_ptr->image.convertTo(current_depth_frame_, CV_32FC1, 0.001);
            } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
                current_depth_frame_ = cv_ptr->image;
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge depth: %s", e.what());
        }
    }

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!has_camera_info_ || current_depth_frame_.empty()) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            return;
        }

        rclcpp::Time img_stamp = msg->header.stamp;
        cv::Mat hsv_frame;
        cv::cvtColor(cv_ptr->image, hsv_frame, cv::COLOR_BGR2HSV);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

        // ========================================================
        // 1. RILEVAMENTO TAVOLO E OSCURAMENTO SFONDO
        // ========================================================
        cv::Mat green_mask;
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
            float z_table = get_average_depth(center_uv.x, center_uv.y);

            if (z_table > 0.0) {
                double x_t = (center_uv.x - cx_) * z_table / fx_;
                double y_t = (center_uv.y - cy_) * z_table / fy_;

                double angle_deg = table_rect.angle;
                if (table_rect.size.width < table_rect.size.height) {
                    angle_deg += 90.0;
                }
                double yaw_rad = angle_deg * (M_PI / 180.0);
                
                publish_table_tf("BILLIARD_TABLE_FRAME", x_t, y_t, z_table, yaw_rad, img_stamp);
                
                double physical_w = (table_rect.size.width * z_table) / fx_;
                double physical_h = (table_rect.size.height * z_table) / fy_;
                publish_holes_relative_to_table(img_stamp, std::max(physical_w, physical_h), std::min(physical_w, physical_h)); 

                // --- CREAZIONE E "RESTRINGIMENTO" DELLA ROI ---
                cv::Mat roi_mask = cv::Mat::zeros(hsv_frame.size(), CV_8U);
                cv::drawContours(roi_mask, table_contours, best_table_idx, cv::Scalar(255), cv::FILLED);
                
                // L'Erosione mangia i bordi della maschera: le buche e le sponde vengono eliminate!
                // Usiamo un kernel bello grande (25x25) per stringere il campo visivo in modo netto
                cv::Mat roi_erosion_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
                cv::erode(roi_mask, roi_mask, roi_erosion_kernel);

                cv::Mat masked_hsv;
                cv::bitwise_and(hsv_frame, hsv_frame, masked_hsv, roi_mask);
                hsv_frame = masked_hsv; 
            }
        }

        // ========================================================
        // 2. RILEVAMENTO PALLINE (HSV TUNING)
        // ========================================================
        cv::Mat blurred_hsv;
        cv::GaussianBlur(hsv_frame, blurred_hsv, cv::Size(5, 5), 0);
        double ball_min_area = 80.0; 

        // PALLINA ROSSA
        cv::Mat mask1, mask2, red_mask;
        cv::inRange(blurred_hsv, cv::Scalar(0, 80, 20), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(blurred_hsv, cv::Scalar(170, 80, 20), cv::Scalar(180, 255, 255), mask2);
        red_mask = mask1 | mask2;
        cv::dilate(red_mask, red_mask, dilate_kernel);
        cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
        process_and_publish_ball(red_mask, "RED_SOLID_BALL_FRAME", ball_min_area, img_stamp);

        // PALLINA ARANCIONE / GIALLA
        cv::Mat orange_mask;
        // Aumentato il limite H da 25 a 35 per includere le sfumature di giallo!
        cv::inRange(blurred_hsv, cv::Scalar(10, 80, 20), cv::Scalar(35, 255, 255), orange_mask);
        cv::dilate(orange_mask, orange_mask, dilate_kernel);
        cv::morphologyEx(orange_mask, orange_mask, cv::MORPH_CLOSE, kernel);
        process_and_publish_ball(orange_mask, "ORANGE_SOLID_BALL_FRAME", ball_min_area, img_stamp);

        // PALLINA BLU
        cv::Mat blue_mask;
        cv::inRange(blurred_hsv, cv::Scalar(100, 80, 20), cv::Scalar(130, 255, 255), blue_mask);
        cv::dilate(blue_mask, blue_mask, dilate_kernel);
        cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);
        process_and_publish_ball(blue_mask, "BLUE_SOLID_BALL_FRAME", ball_min_area, img_stamp);

        // PALLINA BIANCA
        cv::Mat white_mask;
        // V min alzato a 130 per ignorare le macchie grigie. S max abbassato a 50 per ignorare colori chiari.
        cv::inRange(blurred_hsv, cv::Scalar(0, 0, 130), cv::Scalar(180, 50, 255), white_mask);
        cv::dilate(white_mask, white_mask, dilate_kernel);
        cv::morphologyEx(white_mask, white_mask, cv::MORPH_CLOSE, kernel);
        process_and_publish_ball(white_mask, "WHITE_SOLID_BALL_FRAME", ball_min_area, img_stamp);
    }

    void process_and_publish_ball(const cv::Mat& mask, const std::string& frame_name, double min_area, rclcpp::Time stamp)
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

        if (best_idx != -1) {
            cv::Moments m = cv::moments(contours[best_idx]);
            double u = m.m10 / m.m00;
            double v = m.m01 / m.m00;

            float z = get_average_depth(u, v);
            if (z > 0.0) {
                double x_c = (u - cx_) * z / fx_;
                double y_c = (v - cy_) * z / fy_;
                
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = stamp; 
                t.header.frame_id = "camera_color_optical_frame";
                t.child_frame_id = frame_name;
                t.transform.translation.x = x_c;
                t.transform.translation.y = y_c;
                t.transform.translation.z = z;
                t.transform.rotation.w = 1.0; 

                tf_broadcaster_->sendTransform(t);
            }
        }
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
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "camera_color_optical_frame";
        t.child_frame_id = child_frame;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = std::sin(yaw_rad / 2.0);
        t.transform.rotation.w = std::cos(yaw_rad / 2.0);

        tf_broadcaster_->sendTransform(t);
    }

    void publish_holes_relative_to_table(rclcpp::Time stamp, double field_length, double field_width)
    {
        double half_l = field_length / 2.0;
        double half_w = field_width / 2.0;
        double offset_x = 0.01; 
        double offset_y = 0.02;

        struct HoleDef { std::string name; double x; double y; };
        std::vector<HoleDef> holes = {
            {"hole_top_left",     -half_l + offset_x, -half_w + offset_y},
            {"hole_top_right",    -half_l + offset_x,  half_w - offset_y},
            {"hole_mid_left",      0.0,               -half_w + (offset_y/2)},
            {"hole_mid_right",     0.0,                half_w - (offset_y/2)},
            {"hole_bottom_left",   half_l - offset_x, -half_w + offset_y},
            {"hole_bottom_right",  half_l - offset_x,  half_w - offset_y}
        };

        for (const auto& hole : holes) {
            geometry_msgs::msg::TransformStamped t_hole;
            t_hole.header.stamp = stamp;
            t_hole.header.frame_id = "BILLIARD_TABLE_FRAME";
            t_hole.child_frame_id = hole.name;
            
            t_hole.transform.translation.x = hole.x;
            t_hole.transform.translation.y = hole.y;
            t_hole.transform.translation.z = 0.0; 
            t_hole.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(t_hole);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}