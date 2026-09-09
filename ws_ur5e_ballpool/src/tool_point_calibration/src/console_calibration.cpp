#include <tool_point_calibration/tool_point_calibration.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>

class ToolCalibrationNode : public rclcpp::Node
{
public:
  ToolCalibrationNode()
    : rclcpp::Node("console_tool_calibration"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
  {
    // Declare parameters
    this->declare_parameter<std::string>("base_frame", "ur5e_base_link");
    this->declare_parameter<std::string>("tool0_frame", "ur5e_tool0");
    this->declare_parameter<int>("num_samples", 4);

    // Get parameters
    base_frame_ = this->get_parameter("base_frame").as_string();
    tool0_frame_ = this->get_parameter("tool0_frame").as_string();
    num_samples_ = this->get_parameter("num_samples").as_int();

    RCLCPP_INFO(this->get_logger(), "Starting tool calibration with base frame: '%s' and tool0 frame: '%s'.",
                base_frame_.c_str(), tool0_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Move the robot to '%d' different poses, each of which should touch"
                " the tool to the same position in space.\n", num_samples_);
  }

  bool run()
  {
    // Wait for transform to be available
    std::string error_msg;
    if (!waitForTransform(base_frame_, tool0_frame_, std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Unable to lookup transform between base frame: '%s'"
                  " and tool frame: '%s'. TF may not be available.",
                  base_frame_.c_str(), tool0_frame_.c_str());

      bool base_found = tf_buffer_.canTransform(base_frame_, base_frame_, tf2::TimePointZero);
      bool tool_found = tf_buffer_.canTransform(tool0_frame_, tool0_frame_, tf2::TimePointZero);

      if (!base_found && !tool_found)
      {
        RCLCPP_WARN(this->get_logger(), "Check to make sure that a robot state publisher or other node is publishing"
                   " tf frames for your robot. Also check that your base/tool frames names are"
                   " correct and not missing a prefix, for example.");
      }
      else if (!base_found)
      {
        RCLCPP_WARN(this->get_logger(), "Check to make sure that base frame '%s' actually exists.", base_frame_.c_str());
      }
      else if (!tool_found)
      {
        RCLCPP_WARN(this->get_logger(), "Check to make sure that tool0 frame '%s' actually exists.", tool0_frame_.c_str());
      }

      return false;
    }

    // Create storage for user observations
    tool_point_calibration::Affine3dVector observations;
    observations.reserve(num_samples_);

    std::string line;
    int count = 0;

    // Collect observations
    while (rclcpp::ok() && count < num_samples_)
    {
      RCLCPP_INFO(this->get_logger(), "Pose %d: Jog robot to a new location touching the shared position and"
                  " press enter.", count);

      std::getline(std::cin, line); // Blocks program until enter is pressed

      try
      {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_.lookupTransform(base_frame_, tool0_frame_, tf2::TimePointZero);

        Eigen::Affine3d eigen_pose = tf2::transformToEigen(transform.transform);

        observations.push_back(eigen_pose);

        RCLCPP_INFO_STREAM(this->get_logger(), "Pose " << count << ": captured transform:\n" << eigen_pose.matrix());
        count++;
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Calibration captured %d tool poses (out of %d requested). Computing calibration...",
                count, num_samples_);

    tool_point_calibration::TcpCalibrationResult result =
        tool_point_calibration::calibrateTcp(observations);

    RCLCPP_INFO_STREAM(this->get_logger(), "Calibrated tcp (meters in xyz): [" << result.tcp_offset.transpose() << "] from " << tool0_frame_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Touch point (meters in xyz): [" << result.touch_point.transpose() << "] in frame " << base_frame_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Average residual: " << result.average_residual);
    RCLCPP_INFO_STREAM(this->get_logger(), "Converged: " << result.converged);

    if (count < 4)
    {
      RCLCPP_WARN(this->get_logger(), "Computing a tool calibration w/ fewer than 4 points may produce an answer with good"
                 " convergence and residual error, but without a meaningful result. You are encouraged"
                 " to try with more points");
    }

    return true;
  }

private:
  bool waitForTransform(const std::string& target_frame, const std::string& source_frame, 
                        std::chrono::duration<int64_t, std::milli> timeout)
  {
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
      try
      {
        if (tf_buffer_.canTransform(target_frame, source_frame, tf2::TimePointZero))
        {
          return true;
        }
      }
      catch (const tf2::TransformException&)
      {
        // Continue trying
      }

      auto current_time = std::chrono::steady_clock::now();
      if (current_time - start_time > timeout)
      {
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_frame_;
  std::string tool0_frame_;
  int num_samples_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ToolCalibrationNode>();
  
  if (!node->run())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to run tool calibration");
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
