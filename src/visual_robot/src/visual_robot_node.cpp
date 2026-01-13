#include <ros/ros.h>
#include <ros/package.h>
#include <arm_follow/PersonKeypoints.h>
#include <arm_follow/PersonsKeypoints.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>

namespace
{
double clamp(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(max_val, value));
}
}  // namespace

class VisualRobotNode
{
public:
  VisualRobotNode()
    : nh_()
    , pnh_("~")
    , is_initialized_(false)
  {
    // Load parameters
    std::string calibration_path = pnh_.param("calibration_path", std::string(""));
    if (calibration_path.empty())
    {
      std::string pkg_path = ros::package::getPath("Position_Calibration");
      calibration_path = pkg_path + "/config/calibration.yaml";
    }
    
    table_height_ = pnh_.param("table_height", 0.746);
    object_height_ = pnh_.param("object_height", 0.085);
    conf_threshold_ = pnh_.param("conf_threshold", 0.7);
    orientation_yaw_ = pnh_.param("w", 0.0);  // yaw angle around vertical axis

    // Measured vertical-down quaternion from robot:
    // x=0.998517, y=0.052423, z=-0.003210, w=0.014341
    tf2::Quaternion q_down(0.998517, 0.052423, -0.003210, 0.014341);
    q_down.normalize();
    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, orientation_yaw_);
    orientation_quat_ = q_yaw * q_down;
    
    // Load calibration
    if (!loadCalibration(calibration_path))
    {
      ROS_ERROR("Failed to load calibration, exiting...");
      ros::shutdown();
      return;
    }
    
    // Subscribe to YOLO detections
    yolo_sub_ = nh_.subscribe("/yolo_pose/keypoints", 5, &VisualRobotNode::yoloDetectionCallback, this);
    ROS_INFO("Subscribed to YOLO topic: /yolo_pose/keypoints");
    
    // Initialize arm to known posture (same as visual_grasping)
    initializeArm();
    
    ROS_INFO("Waiting 3 seconds for arm to stabilize after initialization...");
    ros::Duration(3.0).sleep();
    is_initialized_ = true;
    
    ROS_INFO("==================================================");
    ROS_INFO("Visual Robot node initialized");
    ROS_INFO("Confidence threshold: %.3f", conf_threshold_);
    ROS_INFO("Table height: %.3f m", table_height_);
    ROS_INFO("Object height: %.3f m", object_height_);
    ROS_INFO("Using vertical-down orientation with yaw=%.3f rad "
             "(quat: x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
             orientation_yaw_, orientation_quat_.x(), orientation_quat_.y(),
             orientation_quat_.z(), orientation_quat_.w());
    ROS_INFO("Ready to convert detections to base coordinates...");
    ROS_INFO("Will print (x, y, z, w) when confidence > %.3f", conf_threshold_);
    ROS_INFO("==================================================");
  }

private:
  bool loadCalibration(const std::string& path)
  {
    std::ifstream file(path);
    if (!file.is_open())
    {
      ROS_ERROR("Failed to open calibration file: %s", path.c_str());
      return false;
    }
    
    std::string line;
    std::vector<double> matrix_values;
    matrix_values.reserve(9);
    bool in_matrix = false;
    
    while (std::getline(file, line))
    {
      if (line.find("homography_matrix:") != std::string::npos)
      {
        in_matrix = true;
        matrix_values.clear();
        continue;
      }
      
      if (in_matrix && matrix_values.size() < 9)
      {
        std::istringstream iss(line);
        std::string token;
        while (iss >> token)
        {
          if (token == "-" || token == "|" || token == "[" || token == "]")
            continue;
          
          try
          {
            double val = std::stod(token);
            matrix_values.push_back(val);
          }
          catch (...)
          {
            // skip tokens that are not numbers
          }
        }
        
        if (matrix_values.size() >= 9)
        {
          in_matrix = false;
        }
      }
      
      if (line.find("table_height:") != std::string::npos)
      {
        std::istringstream iss(line);
        std::string token;
        while (iss >> token)
        {
          if (token == "table_height:")
            continue;
          try
          {
            table_height_ = std::stod(token);
            break;
          }
          catch (...)
          {
            // skip
          }
        }
      }
    }
    
    if (matrix_values.size() != 9)
    {
      ROS_ERROR("Failed to parse homography matrix from %s", path.c_str());
      return false;
    }
    
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        homography_matrix_(i, j) = matrix_values[i * 3 + j];
      }
    }
    
    ROS_INFO("Calibration loaded successfully");
    ROS_INFO("Homography matrix:");
    ROS_INFO("  [%.6f, %.6f, %.6f]", homography_matrix_(0, 0), homography_matrix_(0, 1), homography_matrix_(0, 2));
    ROS_INFO("  [%.6f, %.6f, %.6f]", homography_matrix_(1, 0), homography_matrix_(1, 1), homography_matrix_(1, 2));
    ROS_INFO("  [%.6f, %.6f, %.6f]", homography_matrix_(2, 0), homography_matrix_(2, 1), homography_matrix_(2, 2));
    ROS_INFO("Table height: %.3f m", table_height_);
    
    return true;
  }
  
  Eigen::Vector3d pixelToArmCoordinate(double u, double v)
  {
    // Homography transformation: [x, y, w] = H * [u, v, 1]
    Eigen::Vector3d pixel(u, v, 1.0);
    Eigen::Vector3d result = homography_matrix_ * pixel;
    
    double x = result(0) / result(2);
    double y = result(1) / result(2);
    
    // Apply same global calibration offset as visual_grasping:
    // move 0.025m in negative X (towards the robot base).
    x -= 0.025;
    double z = table_height_ + object_height_ / 2.0;
    
    return Eigen::Vector3d(x, y, z);
  }
  
  void yoloDetectionCallback(const arm_follow::PersonsKeypoints::ConstPtr& msg)
  {
    if (!is_initialized_)
    {
      return;
    }
    
    // Parse YOLO detection message
    if (msg->persons.empty())
    {
      return;
    }
    
    // Select the detection with largest bbox_area (usually the main object)
    const arm_follow::PersonKeypoints* best_detection = nullptr;
    float max_area = 0.0;
    
    for (const auto& person : msg->persons)
    {
      if (person.bbox_area > max_area && person.bbox_area > 0)
      {
        max_area = person.bbox_area;
        best_detection = &person;
      }
    }
    
    if (best_detection == nullptr)
    {
      return;
    }
    
    // Check confidence threshold
    if (best_detection->bbox_confidence < conf_threshold_)
    {
      return;
    }
    
    // Extract center point (u, v) from bounding box
    double u = best_detection->bbox_x + best_detection->bbox_width / 2.0;
    double v = best_detection->bbox_y + best_detection->bbox_height / 2.0;
    
    // Convert pixel to arm coordinate
    Eigen::Vector3d arm_coord = pixelToArmCoordinate(u, v);
    
    // Print in real-time
    ROS_INFO("========================================");
    ROS_INFO("Detection: confidence=%.3f", best_detection->bbox_confidence);
    ROS_INFO("Pixel center: (u=%.1f, v=%.1f)", u, v);
    ROS_INFO("Base coordinate & orientation -> x=%.3f m, y=%.3f m, z=%.3f m, w=%.3f", 
             arm_coord(0), arm_coord(1), arm_coord(2), orientation_quat_.w());
    ROS_INFO("========================================");
  }
  
  void initializeArm()
  {
    ROS_INFO("Running joint initialization commands (visual_robot)...");
    
    const std::vector<std::string> cmds = {
      "rosrun arm_follow j3_set.py --angle 0.0",
      "rosrun arm_follow j1_set.py --angle 0.0",
      "rosrun arm_follow j2_set.py --angle 0.0",
      "rosrun arm_follow j4_set.py --angle 0.0",
      "rosrun arm_follow j5_set.py --angle 1.6",
      "rosrun arm_follow j6_set.py --angle 0.0",
      "rosrun arm_follow gnd_set.py --angle 1.5",
      "rosrun arm_follow head_set.py --angle -1.5"
    };
    
    for (const auto& cmd : cmds)
    {
      ROS_INFO("Executing init command: %s", cmd.c_str());
      int ret = std::system(cmd.c_str());
      if (ret != 0)
      {
        ROS_WARN("Command failed (return=%d): %s", ret, cmd.c_str());
      }
      ros::Duration(0.5).sleep();
    }
    
    ROS_INFO("Arm initialization complete (visual_robot)");
  }
  
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber yolo_sub_;
  
  Eigen::Matrix3d homography_matrix_;
  double table_height_;
  double object_height_;
  double conf_threshold_;
  double orientation_yaw_;
  tf2::Quaternion orientation_quat_;
  bool is_initialized_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_robot_node");
  
  VisualRobotNode node;
  
  ros::spin();
  
  return 0;
}

