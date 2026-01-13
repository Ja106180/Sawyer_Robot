#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <intera_core_msgs/JointCommand.h>
#include <intera_core_msgs/SolvePositionIK.h>
#include <intera_core_msgs/SolvePositionFK.h>
#include <intera_core_msgs/EndpointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <arm_follow/PersonKeypoints.h>
#include <arm_follow/PersonsKeypoints.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace
{
double clamp(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(max_val, value));
}
}  // namespace

struct Detection
{
  double center_x;
  double center_y;
  double confidence;
  ros::Time timestamp;
};

struct JointTarget
{
  std::unordered_map<std::string, double> positions;
  ros::Time start_time;
  double timeout;
  double threshold;
};

struct GraspSpringProfile
{
  double omega;           // natural frequency
  double zeta;            // damping ratio
  double max_velocity;    // maximum joint velocity (rad/s)
  double max_delta;       // maximum joint delta per cycle (rad)
  double snap_threshold;  // snap-to-target window (rad)
  double pos_tolerance;   // end-effector positional tolerance (m)
  double hold_time;       // hold duration once within tolerance (s)
};

namespace
{
const GraspSpringProfile kGraspApproachProfile = {
  5.5,    // omega
  2.50,   // zeta
  1.6,    // max_velocity
  0.035,  // max_delta
  0.010,  // snap_threshold
  0.005,  // pos_tolerance (0.5 cm)
  0.12    // hold_time
};

const GraspSpringProfile kGraspDescendProfile = {
  6.0,    // omega
  3.05,   // zeta
  1.2,    // max_velocity
  0.020,  // max_delta
  0.008,  // snap_threshold
  0.005,  // pos_tolerance
  0.18    // hold_time
};

const GraspSpringProfile kGraspRetreatProfile = {
  6.0,    // omega
  3.05,   // zeta
  2.0,    // max_velocity
  0.030,  // max_delta
  0.010,  // snap_threshold
  0.005,  // pos_tolerance
  0.12    // hold_time
};
}  // namespace

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

class VisualGraspingNode
{
public:
  VisualGraspingNode()
    : nh_()
    , pnh_("~")
    , has_joint_state_(false)
    , is_grasping_(false)
    , is_initialized_(false)
    , grasp_count_(0)
    , control_loop_count_(0)
    , suspend_control_(false)
    , has_fixed_grasp_position_(false)
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
    conf_threshold_ = pnh_.param("conf_threshold", 0.7);  // use 0.7 as grab threshold
    stable_time_ = pnh_.param("stable_time", 5.0);  // legacy parameter for compatibility
    position_threshold_ = pnh_.param("position_threshold", 10.0);  // legacy parameter
    min_detections_ = pnh_.param("min_detections", 5);  // legacy parameter
    
    // Smoothing parameters - all joints use spring-damper dynamics
    // Speed increased: all joints use same high-speed parameters for consistent movement
    omega_j0_ = 18.0;  // Unified high speed
    omega_j1_ = 14.0;  // Unified high speed
    omega_j2_ = 14.0;  // Unified high speed
    omega_j3_ = 14.0;  // Unified high speed
    omega_j4_ = 14.0;  // Unified high speed
    omega_j5_ = 14.0;  // Unified high speed
    omega_j6_ = 14.0;  // Unified high speed
    
    zeta_j0_ = 0.1;   // Normal damping (will be temporarily increased during initialization)
    zeta_j1_ = 0.5;   // Unified with others
    zeta_j2_ = 0.5;
    zeta_j3_ = 0.5;
    zeta_j4_ = 0.5;
    zeta_j5_ = 0.5;
    zeta_j6_ = 0.5;
    
    // Max delta: unified high speed for all joints
    j0_max_delta_ = 2.0;  // Unified high speed
    j1_max_delta_ = 2.0;  // Unified high speed
    j2_max_delta_ = 2.0;  // Unified high speed
    j3_max_delta_ = 2.0;  // Unified high speed
    j4_max_delta_ = 2.0;  // Unified high speed
    j5_max_delta_ = 2.0;  // Unified high speed
    j6_max_delta_ = 2.0;  // Unified high speed
    
    j0_snap_threshold_ = 0.03;
    j1_snap_threshold_ = 0.05;
    j2_snap_threshold_ = 0.05;
    j3_snap_threshold_ = 0.05;
    j4_snap_threshold_ = 0.05;
    j5_snap_threshold_ = 0.05;
    j6_snap_threshold_ = 0.05;
    smoothing_base_ = 0.6;
    smoothing_min_ = 0.2;
    smoothing_max_ = 0.9;
    speed_gain_ = 0.15;  // From arm_follow
    max_delta_per_cycle_ = 0.25;
    
    // Movement parameters
    move_timeout_ = 15.0;
    move_threshold_ = 0.03;  // Keep precision at 0.03 (1.7 degrees) for accurate grasping
    pre_grasp_offset_ = 0.08;  // 8cm above grasp position
    lift_height_ = 0.15;  // 15cm lift height
    
    // Load calibration
    if (!loadCalibration(calibration_path))
    {
      ROS_ERROR("Failed to load calibration, exiting...");
      ros::shutdown();
      return;
    }
    
    // Initialize joint names
    joint_names_ = { "right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6" };
    initJointLimits();
    
    // Publishers and subscribers
    joint_cmd_pub_ = nh_.advertise<intera_core_msgs::JointCommand>("/robot/limb/right/joint_command", 10);
    joint_state_sub_ = nh_.subscribe("/robot/joint_states", 5, &VisualGraspingNode::jointStateCallback, this);
    // No need to subscribe to images; YOLO node already shows them
    // image_sub_ = nh_.subscribe(pnh_.param("camera_topic", std::string("/io/internal_camera/head_camera/image_raw")), 
    //                            5, &VisualGraspingNode::imageCallback, this);
    yolo_sub_ = nh_.subscribe("/yolo_pose/keypoints", 5, &VisualGraspingNode::yoloDetectionCallback, this);
    endpoint_state_sub_ = nh_.subscribe("/robot/limb/right/endpoint_state", 10,
                                        &VisualGraspingNode::endpointStateCallback, this);
    ROS_INFO("Subscribed to YOLO topic: /yolo_pose/keypoints");
    
    // Service clients
    ik_client_ = nh_.serviceClient<intera_core_msgs::SolvePositionIK>(
        "/ExternalTools/right/PositionKinematicsNode/IKService");
    fk_client_ = nh_.serviceClient<intera_core_msgs::SolvePositionFK>(
        "/ExternalTools/right/PositionKinematicsNode/FKService");
    
    // Timers
    control_timer_ = nh_.createTimer(ros::Duration(0.01), &VisualGraspingNode::controlLoop, this);  // 100Hz
    detection_timer_ = nh_.createTimer(ros::Duration(1.0), &VisualGraspingNode::detectionCheckLoop, this);  // 1Hz
    
    last_update_time_ = ros::Time::now();
    
    ROS_INFO("Visual Grasping node initialized");
    ROS_INFO("Confidence threshold: %.3f", conf_threshold_);
    ROS_INFO("Initializing arm to default position...");
    
    // Initialize arm
    initializeArm();
    
    // Wait for arm to stabilize after initialization
    ROS_INFO("Waiting 3 seconds for arm to stabilize after initialization...");
    ros::Duration(3.0).sleep();
    
    // Mark as initialized - now ready to process detections
    is_initialized_ = true;
    
    ROS_INFO("==================================================");
    ROS_INFO("Visual Grasping system is ready!");
    ROS_INFO("Waiting for YOLO detections (/yolo_pose/keypoints)...");
    ROS_INFO("Objects with confidence > %.3f will be grasped automatically", conf_threshold_);
    ROS_INFO("Press Ctrl+C to exit");
    ROS_INFO("==================================================");
  }

private:
  void initJointLimits()
  {
    joint_limits_["right_j0"] = { -3.05, 3.05 };
    joint_limits_["right_j1"] = { -1.2, 1.4 };
    joint_limits_["right_j2"] = { -1.5, 1.5 };
    joint_limits_["right_j3"] = { -1.5, 1.5 };
    joint_limits_["right_j4"] = { -2.0, 2.0 };
    joint_limits_["right_j5"] = { -2.0, 1.7 };
    joint_limits_["right_j6"] = { -3.0, 3.0 };
  }
  
  bool loadCalibration(const std::string& path)
  {
    std::ifstream file(path);
    if (!file.is_open())
    {
      ROS_ERROR("Failed to open calibration file: %s", path.c_str());
      return false;
    }
    
    std::string line;
    bool in_matrix = false;
    std::vector<double> matrix_values;
    matrix_values.reserve(9);
    
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
    ROS_INFO("Table height: %.3f m", table_height_);
    ROS_INFO("Homography matrix:");
    ROS_INFO("  [%.6f, %.6f, %.6f]", homography_matrix_(0, 0), homography_matrix_(0, 1), homography_matrix_(0, 2));
    ROS_INFO("  [%.6f, %.6f, %.6f]", homography_matrix_(1, 0), homography_matrix_(1, 1), homography_matrix_(1, 2));
    ROS_INFO("  [%.6f, %.6f, %.6f]", homography_matrix_(2, 0), homography_matrix_(2, 1), homography_matrix_(2, 2));
    
    return true;
  }
  
  // Initialize joints according to user-provided sequence
  // New order (for safety): j3 -> j1 -> j2 -> j4 -> j5 -> j6 -> j0(gnd_set) -> head_set
  // All joint angles are set using smooth spring-damper dynamics (like arm_follow)
  void initializeArm()
  {
    ROS_INFO("Initializing arm using spring-damper dynamics (smooth motion)...");
    
    // Wait for joint states to be available (required for smooth motion)
    if (!has_joint_state_)
    {
      ROS_INFO("Waiting for joint states to be available...");
      ros::Rate wait_rate(10);
      while (ros::ok() && !has_joint_state_)
      {
        ros::spinOnce();
        wait_rate.sleep();
      }
      ROS_INFO("Joint states available, starting initialization...");
    }
    
    // Initialize joint velocities to zero (ensure smooth start)
    for (const auto& name : joint_names_)
    {
      joint_velocity_[name] = 0.0;
    }
    ROS_INFO("Joint velocities initialized to zero");
    
    // Move joints in two phases: other joints in parallel, then j0 separately
    // Phase 1: j5, j1, j2, j4, j3, j6 in parallel (j0 excluded to avoid oscillation)
    std::unordered_map<std::string, double> init_targets_parallel = {
      {"right_j5", 1.6},
      {"right_j1", 0.0},
      {"right_j2", 0.0},
      {"right_j4", 0.0},
      {"right_j3", 0.0},
      {"right_j6", 0.0}
    };
    
    // Initialize last_command_ to current positions for smooth spring-damper start
    for (const auto& name : joint_names_)
    {
      if (has_joint_state_ && current_joint_positions_.find(name) != current_joint_positions_.end())
      {
        last_command_[name] = current_joint_positions_[name];
      }
    }
    
    // Reset last_update_time_ for proper dt calculation in spring-damper dynamics
    last_update_time_ = ros::Time::now();
    
    // Set all targets for parallel movement
    for (const auto& target : init_targets_parallel)
    {
      ROS_INFO("Setting target for %s to %.3f (parallel initialization with spring-damper)...", 
               target.first.c_str(), target.second);
    }
    
    // Phase 1: Move other joints in parallel using spring-damper dynamics
    ros::Rate rate(100);
    ros::Time start_time = ros::Time::now();
    const double init_timeout = 10.0;  // Overall timeout for initialization
    
    ROS_INFO("Phase 1: Starting parallel initialization (j5, j1, j2, j4, j3, j6) with spring-damper dynamics...");
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < init_timeout)
    {
      bool all_reached = true;
      
      for (const auto& target : init_targets_parallel)
      {
        if (has_joint_state_ && current_joint_positions_.find(target.first) != current_joint_positions_.end())
        {
          double error = std::abs(current_joint_positions_[target.first] - target.second);
          if (error >= move_threshold_)
          {
            all_reached = false;
          }
        }
        else
        {
          all_reached = false;
        }
      }
      
      if (all_reached)
      {
        ROS_INFO("Phase 1 complete: All parallel joints reached initialization targets");
        break;
      }
      
      // Apply smoothing and publish for parallel joints only
      applySmoothingAndPublish(init_targets_parallel);
      ros::spinOnce();
      rate.sleep();
      }
    
    // Phase 2: Move j0 separately (to avoid oscillation with other joints)
    ROS_INFO("Phase 2: Moving j0 separately to %.3f...", 1.5);
    moveJointToPosition("right_j0", 1.5);  // Keep base joint lifted during idle to avoid collision with table
    
    // Head still uses external script (not in joint_names_)
    ROS_INFO("Setting head angle (using external script)...");
    system("rosrun arm_follow head_set.py --angle -1.5");
    
    ROS_INFO("Arm initialization finished (smooth spring-damper motion: j5=1.6->j1->j2->j4->j3->j6->j0->head_set)");
  }
  
  bool moveJointToPosition(const std::string& joint_name, double target_angle)
  {
    // Create target
    JointTarget target;
    target.positions[joint_name] = target_angle;
    target.start_time = ros::Time::now();
    target.timeout = move_timeout_;
    target.threshold = move_threshold_;
    
    // Set current command to current position
    if (has_joint_state_)
    {
      for (const auto& name : joint_names_)
      {
        if (current_joint_positions_.find(name) != current_joint_positions_.end())
        {
          last_command_[name] = current_joint_positions_[name];
        }
        else
        {
          last_command_[name] = 0.0;
        }
      }
    }
    
    // Reset velocity for the target joint to ensure smooth spring-damper start
    joint_velocity_[joint_name] = 0.0;
    
    // Reset last_update_time_ for proper dt calculation
    last_update_time_ = ros::Time::now();
    
    // Update target
    target.positions[joint_name] = clamp(target_angle, 
                                         joint_limits_[joint_name].first,
                                         joint_limits_[joint_name].second);
    
    // Wait until reached (with early exit for faster initialization)
    ros::Rate rate(100);
    bool reached_once = false;
    ros::Time first_reach_time;
    
    while (ros::ok())
    {
      if ((ros::Time::now() - target.start_time).toSec() > target.timeout)
      {
        ROS_WARN("Timeout moving %s to %.3f", joint_name.c_str(), target_angle);
        return false;
      }
      
      // Check if reached
      if (has_joint_state_)
      {
        if (current_joint_positions_.find(joint_name) != current_joint_positions_.end())
        {
          double error = std::abs(current_joint_positions_[joint_name] - target.positions[joint_name]);
          if (error < target.threshold)
          {
            if (!reached_once)
            {
              reached_once = true;
              first_reach_time = ros::Time::now();
            }
            // Exit immediately after reaching threshold (no extra hold time for initialization)
            if ((ros::Time::now() - first_reach_time).toSec() > 0.05)  // Very short hold (50ms)
          {
            ROS_INFO("%s reached target (error: %.4f)", joint_name.c_str(), error);
            return true;
            }
          }
          else
          {
            reached_once = false;  // Reset if left tolerance
          }
        }
      }
      
      // Apply smoothing and publish
      applySmoothingAndPublish(target.positions);
      rate.sleep();
      ros::spinOnce();
    }
    
    return false;
  }
  
  void applySmoothingAndPublish(const std::unordered_map<std::string, double>& targets, double hand_speed = 0.0)
  {
    ros::Time now = ros::Time::now();
    double dt = (now - last_update_time_).toSec();
    if (dt < 1e-3)
    {
      dt = 1e-3;
    }
    else if (dt > 0.05)
    {
      dt = 0.05;
    }
    last_update_time_ = now;
    
    intera_core_msgs::JointCommand cmd;
    cmd.mode = intera_core_msgs::JointCommand::POSITION_MODE;
    cmd.header.stamp = now;
    
    for (const auto& name : joint_names_)
    {
      const auto limit = joint_limits_[name];
      double q_prev = last_command_[name];
      double q_ref = (targets.find(name) != targets.end()) ? 
                     clamp(targets.at(name), limit.first, limit.second) : q_prev;
      double q_new = q_prev;
      
      // All joints use spring-damper dynamics (3x speed)
      {
        // Virtual spring-damper dynamics for all joints
        double& v = joint_velocity_[name];
        double omega, zeta, snap_threshold, delta_limit;
        
        // Get parameters for each joint
        if (name == "right_j0")
        {
          omega = omega_j0_;
          zeta = zeta_j0_;
          snap_threshold = j0_snap_threshold_;
          delta_limit = j0_max_delta_;
        }
        else if (name == "right_j1")
        {
          omega = omega_j1_;
          zeta = zeta_j1_;
          snap_threshold = j1_snap_threshold_;
          delta_limit = j1_max_delta_;
        }
        else if (name == "right_j2")
        {
          omega = omega_j2_;
          zeta = zeta_j2_;
          snap_threshold = j2_snap_threshold_;
          delta_limit = j2_max_delta_;
        }
        else if (name == "right_j3")
        {
          omega = omega_j3_;
          zeta = zeta_j3_;
          snap_threshold = j3_snap_threshold_;
          delta_limit = j3_max_delta_;
        }
        else if (name == "right_j4")
        {
          omega = omega_j4_;
          zeta = zeta_j4_;
          snap_threshold = j4_snap_threshold_;
          delta_limit = j4_max_delta_;
        }
        else if (name == "right_j5")
        {
          omega = omega_j5_;
          zeta = zeta_j5_;
          snap_threshold = j5_snap_threshold_;
          delta_limit = j5_max_delta_;
        }
        else if (name == "right_j6")
        {
          omega = omega_j6_;
          zeta = zeta_j6_;
          snap_threshold = j6_snap_threshold_;
          delta_limit = j6_max_delta_;
        }
        else
        {
          // Fallback for unknown joints
          omega = 4.5 * 3.0;
          zeta = 0.9;
          snap_threshold = 0.05;
          delta_limit = 0.60 * 3.0;
        }
        
        double e = q_ref - q_prev;
        double a = omega * omega * e - 2.0 * zeta * omega * v;
        v += a * dt;
        q_new = q_prev + v * dt;
        
        // Avoid overshoot around target when very close
        double err_before = q_ref - q_prev;
        double err_after = q_ref - q_new;
        if (std::fabs(err_before) < snap_threshold && std::fabs(err_after) < snap_threshold &&
            err_before * err_after <= 0.0)
        {
          q_new = q_ref;
          v = 0.0;
        }
        
        // For j1 specifically, avoid overshoot around neutral during downward motion
        if (name == "right_j1")
        {
          double err_before = q_ref - q_prev;
          double err_after = q_ref - q_new;
          if (std::fabs(err_before) < j1_snap_threshold_ && std::fabs(err_after) < j1_snap_threshold_ &&
              err_before * err_after <= 0.0)
          {
            q_new = q_ref;
            v = 0.0;
          }
        }

        double delta = q_new - q_prev;
        if (std::abs(delta) > delta_limit)
        {
          delta = std::copysign(delta_limit, delta);
          q_new = q_prev + delta;
          v = delta / dt;
        }
        q_new = clamp(q_new, limit.first, limit.second);
      }
      
      last_command_[name] = q_new;
      cmd.names.push_back(name);
      cmd.position.push_back(q_new);
    }
    
    joint_cmd_pub_.publish(cmd);
  }
  
  bool waitForTargetReached(const sensor_msgs::JointState& target_state, 
                            double timeout, double threshold)
  {
    intera_core_msgs::JointCommand cmd;
    cmd.mode = intera_core_msgs::JointCommand::POSITION_MODE;
    cmd.names = target_state.name;
    cmd.position = target_state.position;

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(50);  // Increased from 20 to 50 Hz for faster response
    
    while (ros::ok())
    {
      joint_cmd_pub_.publish(cmd);
      ros::spinOnce();

      if ((ros::Time::now() - start_time).toSec() > timeout)
      {
        ROS_WARN("Timeout waiting for target reached");
        return false;
      }
      
      if (has_joint_state_)
      {
        bool all_reached = true;
        double max_error = 0.0;
        for (size_t i = 0; i < target_state.name.size(); ++i)
        {
          const auto& joint_name = target_state.name[i];
          if (current_joint_positions_.find(joint_name) != current_joint_positions_.end())
          {
            double error = std::abs(current_joint_positions_[joint_name] - target_state.position[i]);
            max_error = std::max(max_error, error);
            if (error > threshold)
            {
              all_reached = false;
            }
          }
          else
          {
            all_reached = false;
          }
        }
        
        if (all_reached)
        {
          ROS_INFO("Target reached (max error: %.4f)", max_error);
          return true;
        }
      }
      
      rate.sleep();
    }
    
    return false;
  }
  
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]) != joint_names_.end())
      {
        current_joint_positions_[msg->name[i]] = msg->position[i];
      }
    }
    has_joint_state_ = true;
  }
  
  void endpointStateCallback(const intera_core_msgs::EndpointState::ConstPtr& msg)
  {
    current_endpoint_pose_ = msg->pose;
    has_endpoint_state_ = true;
  }
  
  // Image callback removed; YOLO node already provides display
  // void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  // {
  //   // Visualization handled by YOLO node
  // }
  
  void detectionCheckLoop(const ros::TimerEvent&)
  {
    // Skip checks during grasping
    if (is_grasping_)
      return;
    
    // Check whether confidence threshold is satisfied
    if (isReadyToGrasp())
    {
    ROS_INFO("Detected object (confidence > %.2f), executing grasp...", conf_threshold_);
      executeGrasp();
    }
  }
  
  void yoloDetectionCallback(const arm_follow::PersonsKeypoints::ConstPtr& msg)
  {
    // Ignore detections if not initialized yet
    if (!is_initialized_)
    {
      ROS_DEBUG("System not initialized yet, ignoring detection");
      return;
    }
    
    // Ignore detections while grasping
    if (is_grasping_)
    {
      return;
    }
    
    // Parse YOLO detection message
    // Find the best detection (largest bbox_area with confidence > threshold)
    if (msg->persons.empty())
    {
      return;
    }
    
    ROS_INFO("Received %zu YOLO detections", msg->persons.size());
    
    // Select the person with largest bbox_area
    const arm_follow::PersonKeypoints* best_person = nullptr;
    float max_area = 0.0;
    
    for (const auto& person : msg->persons)
    {
      // Use bbox_area as fallback indicator
      // Keypoint confidences can be checked if needed
      if (person.bbox_area > max_area && person.bbox_area > 0)
      {
        max_area = person.bbox_area;
        best_person = &person;
      }
    }
    
    if (best_person != nullptr)
    {
      Detection det;
      // Calculate center point from bounding box
      det.center_x = best_person->bbox_x + best_person->bbox_width / 2.0;
      det.center_y = best_person->bbox_y + best_person->bbox_height / 2.0;
      
      // Use bbox_confidence from YOLO (same value shown in the UI)
      det.confidence = best_person->bbox_confidence;
      det.timestamp = ros::Time::now();
      
      ROS_INFO("Detection pixel (u=%.1f, v=%.1f), confidence: %.3f (threshold: %.3f)",
                det.center_x, det.center_y, det.confidence, conf_threshold_);
      
      // Keep only the latest detection
      detection_history_.clear();  // keep only the latest entry
      detection_history_.push_back(det);
      
      // Trigger grasp immediately if confidence is high enough
      if (det.confidence > conf_threshold_)
      {
        ROS_INFO("Confidence %.3f > %.3f, fixing grasp position and starting grasp sequence", 
                 det.confidence, conf_threshold_);
        
        // Fix grasp position immediately: convert pixel to arm coordinate and save
        Eigen::Vector3d grasp_pos = pixelToArmCoordinate(det.center_x, det.center_y);
        
        fixed_grasp_position_ = grasp_pos;
        has_fixed_grasp_position_ = true;
        
        ROS_INFO("Fixed grasp position (raw from calibration): (x=%.3f, y=%.3f, z=%.3f) - will not update until grasp completes",
                 fixed_grasp_position_(0), fixed_grasp_position_(1), fixed_grasp_position_(2));
        
        // Set flag to stop receiving new detections
        is_grasping_ = true;
        
        // Start grasp sequence
        ROS_INFO("Starting grasp sequence...");
        executeGrasp();
      }
      else
      {
        ROS_DEBUG("Confidence %.3f <= %.3f, not triggering grasp", det.confidence, conf_threshold_);
      }
    }
    else
    {
      ROS_DEBUG("No valid detection found");
    }
  }
  
  bool isReadyToGrasp()
  {
    // Skip checks during grasp
    if (is_grasping_)
      return false;
    
    // Simple rule: latest detection confidence must exceed threshold
    if (detection_history_.empty())
      return false;
    
    // Use latest detection
    const Detection& latest = detection_history_.back();
    
    // Check confidence threshold
    if (latest.confidence > conf_threshold_)
    {
      ROS_INFO("Object ready: confidence %.3f > %.3f, preparing grasp", latest.confidence, conf_threshold_);
      return true;
    }
    else
    {
      ROS_DEBUG("Confidence too low: %.3f <= %.3f", latest.confidence, conf_threshold_);
    }
    
    return false;
  }
  
  Eigen::Vector3d pixelToArmCoordinate(double u, double v)
  {
    // Homography transformation: [x, y, w] = H * [u, v, 1]
    Eigen::Vector3d pixel(u, v, 1.0);
    Eigen::Vector3d result = homography_matrix_ * pixel;
    
    double x = result(0) / result(2);
    double y = result(1) / result(2);
    
    // Apply global calibration offset: experimentally determined that
    // moving 0.015m in negative X (towards the robot) and
    // 0.01m in positive Y makes the grasp accurate.
    x -= 0.0140;
    y -= 0.0200;
    double z = table_height_ + object_height_ / 2.0;
    
    ROS_DEBUG("Pixel(%.1f, %.1f) -> arm(%.3f, %.3f, %.3f)", u, v, x, y, z);
    
    return Eigen::Vector3d(x, y, z);
  }
  
  bool getCurrentEndEffectorPose(geometry_msgs::Pose& pose)
  {
    if (!has_joint_state_)
      return false;
    
    intera_core_msgs::SolvePositionFK fk_request;
    sensor_msgs::JointState seed_state;
    seed_state.name = joint_names_;
    for (const auto& name : joint_names_)
    {
      if (current_joint_positions_.find(name) != current_joint_positions_.end())
      {
        seed_state.position.push_back(current_joint_positions_[name]);
      }
      else
      {
        seed_state.position.push_back(0.0);
      }
    }
    fk_request.request.configuration.push_back(seed_state);
    fk_request.request.tip_names.push_back("right_hand");
    
    if (!fk_client_.call(fk_request))
    {
      ROS_ERROR("FK service call failed");
      return false;
    }
    
    if (fk_request.response.isValid.size() > 0 && fk_request.response.isValid[0])
    {
      pose = fk_request.response.pose_stamp[0].pose;
      return true;
    }
    
    return false;
  }
  
  bool moveToPosition(double x, double y, double z, bool /*use_current_orientation*/ = true)
  {
    // Always use the measured vertical-down orientation for grasping.
    // This quaternion was measured on the real robot when the gripper was
    // visually aligned to point straight down towards the table:
    //   x=0.998517, y=0.052423, z=-0.003210, w=0.014341
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "base";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    
    target_pose.pose.orientation.x = 0.998517;
    target_pose.pose.orientation.y = 0.052423;
    target_pose.pose.orientation.z = -0.003210;
    target_pose.pose.orientation.w = 0.014341;
    
    intera_core_msgs::SolvePositionIK ik_request;
    ik_request.request.pose_stamp.push_back(target_pose);
    ik_request.request.tip_names.push_back("right_hand");
    
    // Use current joint positions as seed
    if (has_joint_state_)
    {
      sensor_msgs::JointState seed_state;
      seed_state.name = joint_names_;
      for (const auto& name : joint_names_)
      {
        if (current_joint_positions_.find(name) != current_joint_positions_.end())
        {
          seed_state.position.push_back(current_joint_positions_[name]);
        }
        else
        {
          seed_state.position.push_back(0.0);
        }
      }
      ik_request.request.seed_mode = ik_request.request.SEED_USER;
      ik_request.request.seed_angles.push_back(seed_state);
    }
    
    if (!ik_client_.call(ik_request))
    {
      ROS_ERROR("IK service call failed");
      return false;
    }
    
    if (ik_request.response.result_type.size() > 0 && 
        ik_request.response.result_type[0] > 0)
    {
      // Valid solution found
      sensor_msgs::JointState solution = ik_request.response.joints[0];
      
      // j5 is automatically solved by IK to maintain vertical-down orientation
      // No need to fix it - let IK adjust j5 as needed

      ROS_INFO("IK succeeded for position (%.3f, %.3f, %.3f) with vertical-down orientation",
               x, y, z);
      int j5_index = -1;
      for (size_t i = 0; i < solution.name.size(); ++i)
      {
        if (solution.name[i] == "right_j5")
        {
          j5_index = static_cast<int>(i);
          break;
        }
      }
      if (j5_index >= 0)
      {
        ROS_DEBUG("IK solved j5 to %.3f for vertical-down pose", solution.position[j5_index]);
      }
      // Wait until reached using the same approach as test_robot/go_to_xyz
      return waitForTargetReached(solution, move_timeout_, move_threshold_);
    }
    else
    {
      int result_type = (ik_request.response.result_type.size() > 0)
                        ? ik_request.response.result_type[0] : 0;
      ROS_ERROR("INVALID POSE - No valid joint solution found. "
                "position=(%.3f, %.3f, %.3f), result_type=%d",
                x, y, z, result_type);
      return false;
    }
  }
  
  bool moveToPoseGoToStyle(double target_x, double target_y, double target_z, double target_w)
  {
    ROS_INFO("GoTo-style move to -> x=%.3f, y=%.3f, z=%.3f, yaw(w)=%.3f rad",
             target_x, target_y, target_z, target_w);

    // Wait for first endpoint state if needed (like go_to_xyz)
    if (!has_endpoint_state_)
    {
      ROS_INFO("Waiting for first endpoint_state message...");
      ros::Time wait_start = ros::Time::now();
      ros::Rate wait_rate(50.0);
      while (ros::ok() && !has_endpoint_state_ &&
             (ros::Time::now() - wait_start).toSec() < 5.0)
      {
        wait_rate.sleep();
      }
      if (!has_endpoint_state_)
      {
        ROS_WARN("No endpoint_state received within 5 seconds. Will proceed anyway.");
    }
    else
    {
        ROS_INFO("First endpoint_state received.");
      }
    }

    // Build target pose (base frame, vertical-down orientation + yaw)
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "base";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = target_x;
    target_pose.pose.position.y = target_y;
    target_pose.pose.position.z = target_z;

    // Measured vertical-down quaternion from robot
    tf2::Quaternion q_down(0.998517, 0.052423, -0.003210, 0.014341);
    q_down.normalize();
    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, target_w);
    tf2::Quaternion q_target = q_yaw * q_down;
    target_pose.pose.orientation.x = q_target.x();
    target_pose.pose.orientation.y = q_target.y();
    target_pose.pose.orientation.z = q_target.z();
    target_pose.pose.orientation.w = q_target.w();
    ROS_INFO("Target orientation quaternion (go_to style): [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
             q_target.x(), q_target.y(), q_target.z(), q_target.w());

    // IK request (reuse class ik_client_)
    intera_core_msgs::SolvePositionIK ik_srv;
    ik_srv.request.pose_stamp.push_back(target_pose);
    ik_srv.request.tip_names.push_back("right_hand");

    ROS_INFO("Calling IK for target pose (go_to style)...");
    if (!ik_client_.call(ik_srv))
    {
      ROS_ERROR("IK service call failed.");
      return false;
    }

    if (ik_srv.response.result_type.empty() || ik_srv.response.result_type[0] <= 0)
    {
      int rt = ik_srv.response.result_type.empty() ? 0 : ik_srv.response.result_type[0];
      ROS_ERROR("No valid IK solution (go_to style). result_type = %d", rt);
      return false;
    }

    sensor_msgs::JointState joint_solution = ik_srv.response.joints[0];
    ROS_INFO("IK succeeded (go_to style). Got %zu joints.", joint_solution.name.size());

    // Build joint command (reuse joint_cmd_pub_)
    intera_core_msgs::JointCommand cmd;
    cmd.mode = intera_core_msgs::JointCommand::POSITION_MODE;
    cmd.names = joint_solution.name;
    cmd.position = joint_solution.position;

    // Go_to style loop: keep publishing until reach tolerance
    ros::Rate rate(100.0);  // Increased to 100 Hz for faster response
    const double pos_tolerance = 0.050;   // 5 cm (relaxed for faster completion)
    bool target_within_tolerance = false;
    ros::Time tolerance_enter_time;

    ROS_INFO("Sending joint command towards target (go_to style).");

    while (ros::ok())
    {
      // 发布关节命令
      joint_cmd_pub_.publish(cmd);

      // 处理回调，更新 current_endpoint_pose_
      ros::spinOnce();

      if (has_endpoint_state_)
      {
        const auto& p = current_endpoint_pose_.position;
        const auto& o = current_endpoint_pose_.orientation;
        tf2::Quaternion q_cur(o.x, o.y, o.z, o.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_cur).getRPY(roll, pitch, yaw);

        double dx = p.x - target_x;
        double dy = p.y - target_y;
        double dz = p.z - target_z;
        double pos_error = std::sqrt(dx * dx + dy * dy + dz * dz);
        double yaw_error = normalizeAngle(yaw - target_w);

        ROS_INFO_THROTTLE(0.1,
                          "[GoTo] Target: (x=%.3f, y=%.3f, z=%.3f, w=%.3f) | "
                          "Current: (x=%.3f, y=%.3f, z=%.3f, yaw=%.3f) | "
                          "Error: pos=%.3f m, yaw=%.3f rad",
                          target_x, target_y, target_z, target_w,
                          p.x, p.y, p.z, yaw,
                          pos_error, yaw_error);

        // 位置到位即可，yaw 只用于打印，不作为收敛条件
        if (pos_error < pos_tolerance)
        {
          if (!target_within_tolerance)
          {
            target_within_tolerance = true;
            tolerance_enter_time = ros::Time::now();
            ROS_INFO("[GoTo] Within tolerance (pos=%.3f m, yaw=%.3f rad). Holding...",
                     pos_error, yaw_error);
          }
          else if ((ros::Time::now() - tolerance_enter_time).toSec() > 0.1)  // Reduced to 0.1s
          {
            ROS_INFO("[GoTo] Hold complete. Reached target pose.");
            break;
          }
        }
        else if (target_within_tolerance)
        {
          target_within_tolerance = false;
        }
      }

      rate.sleep();
    }

    return true;
  }
  
  bool moveToPoseGraspStyle(double target_x, double target_y, double target_z, double target_w,
                            const GraspSpringProfile& profile, const std::string& profile_name)
  {
    ROS_INFO("Grasp-style move (%s) -> x=%.3f, y=%.3f, z=%.3f, w=%.3f",
             profile_name.c_str(), target_x, target_y, target_z, target_w);

    if (!has_endpoint_state_)
    {
      ROS_INFO("Waiting for first endpoint_state message...");
      ros::Time wait_start = ros::Time::now();
      ros::Rate wait_rate(50.0);
      while (ros::ok() && !has_endpoint_state_ &&
             (ros::Time::now() - wait_start).toSec() < 5.0)
      {
        wait_rate.sleep();
      }
      if (!has_endpoint_state_)
      {
        ROS_WARN("No endpoint_state received within 5 seconds. Proceeding anyway.");
      }
      else
      {
        ROS_INFO("First endpoint_state received.");
      }
    }

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "base";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = target_x;
    target_pose.pose.position.y = target_y;
    target_pose.pose.position.z = target_z;

    tf2::Quaternion q_down(0.998517, 0.052423, -0.003210, 0.014341);
    q_down.normalize();
    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, target_w);
    tf2::Quaternion q_target = q_yaw * q_down;
    target_pose.pose.orientation.x = q_target.x();
    target_pose.pose.orientation.y = q_target.y();
    target_pose.pose.orientation.z = q_target.z();
    target_pose.pose.orientation.w = q_target.w();
    ROS_INFO("Target quaternion (grasp-style): [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
             q_target.x(), q_target.y(), q_target.z(), q_target.w());

    // 计算目标姿态的 RPY，用于后续仅对 j5 做一个简单的“姿态 P 闭环”（主要管 pitch）
    double des_roll = 0.0, des_pitch = 0.0, des_yaw = 0.0;
    tf2::Matrix3x3(q_target).getRPY(des_roll, des_pitch, des_yaw);

    intera_core_msgs::SolvePositionIK ik_srv;
    ik_srv.request.pose_stamp.push_back(target_pose);
    ik_srv.request.tip_names.push_back("right_hand");

    ROS_INFO("Calling IK for grasp-style target...");
    if (!ik_client_.call(ik_srv))
    {
      ROS_ERROR("IK service call failed (grasp-style).");
      return false;
    }

    if (ik_srv.response.result_type.empty() || ik_srv.response.result_type[0] <= 0)
    {
      int rt = ik_srv.response.result_type.empty() ? 0 : ik_srv.response.result_type[0];
      ROS_ERROR("No valid IK solution (grasp-style). result_type = %d", rt);
      return false;
    }

    sensor_msgs::JointState joint_solution = ik_srv.response.joints[0];
    ROS_INFO("IK succeeded (grasp-style). Got %zu joints.", joint_solution.name.size());

    // Build local command and velocity maps (independent from initialization controller)
    std::unordered_map<std::string, double> joint_targets;
    std::unordered_map<std::string, double> joint_commands;
    std::unordered_map<std::string, double> joint_velocities;

    for (size_t i = 0; i < joint_solution.name.size(); ++i)
    {
      const std::string& joint_name = joint_solution.name[i];
      joint_targets[joint_name] = joint_solution.position[i];

      double start_position = joint_solution.position[i];
      if (has_joint_state_ && current_joint_positions_.find(joint_name) != current_joint_positions_.end())
      {
        start_position = current_joint_positions_[joint_name];
      }

      joint_commands[joint_name] = start_position;
      joint_velocities[joint_name] = 0.0;
    }

    // 记录 j5 的初始 IK 参考角度，后续姿态闭环始终在这个“基准”上做小修正，
    // 避免在 joint_targets 上不断累加导致目标越修越偏。
    double j5_ik_base = 0.0;
    bool has_j5_ik_base = false;
    auto it_j5_init = joint_targets.find("right_j5");
    if (it_j5_init != joint_targets.end())
    {
      j5_ik_base = it_j5_init->second;
      has_j5_ik_base = true;
    }

    ros::Rate rate(200.0);
    ros::Time last_update_time = ros::Time::now();
    bool target_within_tolerance = false;
    ros::Time tolerance_enter_time;

    // === 回退到“单次 IK + 关节空间弹簧-阻尼”的方案 ===
    // 不再在循环里做步进 IK / 笛卡尔 z 闭环，只用初始 IK 解作为关节参考。
    while (ros::ok())
    {
      ros::Time now = ros::Time::now();
      double dt = (now - last_update_time).toSec();
      if (dt < 1e-3)
      {
        dt = 1e-3;
      }
      else if (dt > 0.02)
      {
        dt = 0.02;
      }
      last_update_time = now;

      if (has_endpoint_state_)
      {
        const auto& p = current_endpoint_pose_.position;
        const auto& o = current_endpoint_pose_.orientation;
        tf2::Quaternion q_cur(o.x, o.y, o.z, o.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_cur).getRPY(roll, pitch, yaw);

        double dx = p.x - target_x;
        double dy = p.y - target_y;
        double dz = p.z - target_z;
        double pos_error = std::sqrt(dx * dx + dy * dy + dz * dz);
        double yaw_error = normalizeAngle(yaw - target_w);

        ROS_INFO_THROTTLE(0.1,
                          "[GraspSpring:%s] Target(x=%.3f,y=%.3f,z=%.3f,w=%.3f) "
                          "Current(x=%.3f,y=%.3f,z=%.3f,yaw=%.3f) "
                          "Error: pos=%.4f m, yaw=%.4f rad",
                          profile_name.c_str(),
                          target_x, target_y, target_z, target_w,
                          p.x, p.y, p.z, yaw,
                          pos_error, yaw_error);

        if (pos_error < profile.pos_tolerance)
        {
          if (!target_within_tolerance)
          {
            target_within_tolerance = true;
            tolerance_enter_time = now;
            ROS_INFO("[GraspSpring:%s] Within %.1f mm tolerance (pos=%.4f m). Holding...",
                     profile_name.c_str(), profile.pos_tolerance * 1000.0, pos_error);
          }
          else if ((now - tolerance_enter_time).toSec() > profile.hold_time)
          {
            ROS_INFO("[GraspSpring:%s] Hold complete. Reached target pose.", profile_name.c_str());
            break;
          }
        }
        else if (target_within_tolerance)
        {
          target_within_tolerance = false;
        }
      }

      // === 在关节空间执行弹簧-阻尼，其中 j5 的参考角度叠加一个基于姿态误差的 P 修正 ===
      intera_core_msgs::JointCommand cmd;
      cmd.mode = intera_core_msgs::JointCommand::POSITION_MODE;
      cmd.header.stamp = now;

      for (const auto& target_pair : joint_targets)
      {
        const std::string& joint_name = target_pair.first;
        double q_ref = target_pair.second;
        double q_new = q_ref;

        // 对 j5：使用“初始 IK 角 + 姿态 P 修正”作为本周期参考角，
        // 而不是在 joint_targets 上累计，避免目标越修越偏。
        if (joint_name == "right_j5" && has_endpoint_state_ && has_j5_ik_base)
        {
          const double k_p_pitch_to_j5 = 0.35;   // 姿态误差 → j5 角度的小增益，可再微调

          // 当前姿态误差：目标 pitch（竖直朝下）减当前 pitch，不做周期归一化，避免大跳变
          const auto& o = current_endpoint_pose_.orientation;
          tf2::Quaternion q_cur(o.x, o.y, o.z, o.w);
          double roll_cur, pitch_cur, yaw_cur;
          tf2::Matrix3x3(q_cur).getRPY(roll_cur, pitch_cur, yaw_cur);

          double pitch_error = des_pitch - pitch_cur;
          const double max_pitch_err = 0.35;  // 限制在约 20 度内，避免修正过大
          if (pitch_error > max_pitch_err) pitch_error = max_pitch_err;
          if (pitch_error < -max_pitch_err) pitch_error = -max_pitch_err;

          q_ref = j5_ik_base + k_p_pitch_to_j5 * pitch_error;
        }

        // 所有关节（包括 j5/j6）统一走弹簧-阻尼动力学，避免硬命令导致腕部抽搐
        double q_prev = joint_commands[joint_name];
        double& v = joint_velocities[joint_name];

        double a = profile.omega * profile.omega * (q_ref - q_prev)
                   - 2.0 * profile.zeta * profile.omega * v;
        v += a * dt;

        if (std::abs(v) > profile.max_velocity)
        {
          v = std::copysign(profile.max_velocity, v);
        }

        double delta = v * dt;
        if (std::abs(delta) > profile.max_delta)
        {
          delta = std::copysign(profile.max_delta, delta);
          v = delta / dt;
        }

        q_new = q_prev + delta;
        double err_before = q_ref - q_prev;
        double err_after = q_ref - q_new;
        if (std::fabs(err_before) < profile.snap_threshold &&
            std::fabs(err_after) < profile.snap_threshold &&
            err_before * err_after <= 0.0)
        {
          q_new = q_ref;
          v = 0.0;
        }

        if (joint_limits_.find(joint_name) != joint_limits_.end())
        {
          const auto& limit = joint_limits_[joint_name];
          q_new = clamp(q_new, limit.first, limit.second);
        }

        joint_commands[joint_name] = q_new;

        cmd.names.push_back(joint_name);
        cmd.position.push_back(joint_commands[joint_name]);
      }

      joint_cmd_pub_.publish(cmd);
      ros::spinOnce();

      rate.sleep();
    }

    // Ensure final command equals IK target for a clean hand-off
    intera_core_msgs::JointCommand final_cmd;
    final_cmd.mode = intera_core_msgs::JointCommand::POSITION_MODE;
    final_cmd.header.stamp = ros::Time::now();
    final_cmd.names = joint_solution.name;
    final_cmd.position = joint_solution.position;
    joint_cmd_pub_.publish(final_cmd);

    return true;
  }
  
  void controlGripper(bool open)
  {
    // j_zha.py 的用法是：--open / --close / --position，所以这里必须加上参数前缀
    std::string cmd = open ? "--open" : "--close";
    std::string command = "rosrun arm_follow j_zha.py " + cmd;
    system(command.c_str());
    ros::Duration(1.0).sleep();
  }
  
  void executeGrasp()
  {
    // Check if we have a fixed grasp position
    if (!has_fixed_grasp_position_)
    {
      ROS_ERROR("No fixed grasp position available, aborting grasp");
      is_grasping_ = false;
      return;
    }
    
    grasp_count_++;
    
    ROS_INFO("==================================================");
    ROS_INFO("Executing grasp sequence #%d", grasp_count_);
    ROS_INFO("==================================================");
    
    // Use the fixed grasp position (already converted and clamped).
    // We only use x and y from vision; z is controlled explicitly by the
    // grasp sequence (high and low heights).
    Eigen::Vector3d grasp_pos = fixed_grasp_position_;
    double target_x = grasp_pos(0);
    double target_y = grasp_pos(1);
    double vision_z = grasp_pos(2);
    ROS_INFO("Using fixed grasp position (vision): (x=%.3f, y=%.3f, z=%.3f)",
             target_x, target_y, vision_z);
    
    // Define explicit grasp heights (in base frame, meters).
    // z_high: safe height above the table to approach and retreat.
    // z_low : grasp height near the object (table height + object thickness).
    const double z_high = 0.40;  // approach / retreat height
    const double z_low  = 0.03;  // grasp height near the table surface

    // Fixed w value (yaw angle)
    const double target_w = 1.5708;  // Fixed value (π/2)

    // Step 0: Rotate base joint (j0) to 0.0 before starting actual grasp
    // This moves the arm to the object position without blocking the camera
    // Use smooth spring-damper dynamics instead of external script
    ROS_INFO("Step 0: Rotating base joint (j0) to 0.0 using smooth spring-damper...");
    bool j0_success = moveJointToPosition("right_j0", 0.0);
    if (!j0_success)
    {
      ROS_WARN("Failed to rotate j0 to 0.0, but continuing with grasp...");
    }
    
    // Step 1: Move above target using grasp-style spring-damper motion
    ROS_INFO("Step 1: grasp-style move to high position (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
             target_x, target_y, z_high, target_w);
    suspend_control_ = true;
    bool goto_success = moveToPoseGraspStyle(target_x, target_y, z_high, target_w,
                                             kGraspApproachProfile, "approach_high");
    suspend_control_ = false;
    if (!goto_success)
    {
      ROS_ERROR("Grasp-style move to high position failed, aborting grasp.");
      is_grasping_ = false;
      return;
    }
	    // No delay - move immediately
    
    // Step 2: Open gripper在高位
    ROS_INFO("Step 2: open gripper at high position...");
    controlGripper(true);
    ros::Duration(0.1).sleep();  // Minimal delay for gripper command
    
    // Step 3: 使用抓取专用弹簧阻尼下降到抓取高度
    ROS_INFO("Step 3: grasp-style move to low position (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
             target_x, target_y, z_low, target_w);
    suspend_control_ = true;
    goto_success = moveToPoseGraspStyle(target_x, target_y, z_low, target_w,
                                        kGraspDescendProfile, "descend_low");
    suspend_control_ = false;
    if (!goto_success)
    {
      ROS_ERROR("Grasp-style move to low position failed, aborting grasp.");
      is_grasping_ = false;
      return;
    }
    
    // Step 4: Close gripper
    ROS_INFO("Step 4: close gripper (grasp)...");
    controlGripper(false);
    ros::Duration(0.2).sleep();  // Minimal delay for gripper
    
    // Step 5: 使用抓取专用弹簧阻尼抬回到高位
    ROS_INFO("Step 5: grasp-style move back to high position (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
             target_x, target_y, z_high, target_w);
    suspend_control_ = true;
    goto_success = moveToPoseGraspStyle(target_x, target_y, z_high, target_w,
                                        kGraspRetreatProfile, "retreat_high");
    suspend_control_ = false;
    if (!goto_success)
    {
      ROS_ERROR("Grasp-style move back to high position failed, aborting grasp.");
      is_grasping_ = false;
      return;
    }
    
    // Step 6: Open gripper to release
    ROS_INFO("Step 6: open gripper (release)...");
    controlGripper(true);
    ros::Duration(0.1).sleep();  // Minimal delay
    
    // Step 8: Return to initial position
    ROS_INFO("Step 8: return to initial posture...");
    initializeArm();
    
    // Clear detection history and fixed grasp position after returning to base posture
    detection_history_.clear();
    has_fixed_grasp_position_ = false;
    
    ROS_INFO("Grasp sequence finished, arm reset, waiting for next detection...");
    is_grasping_ = false;
  }
  
  void controlLoop(const ros::TimerEvent&)
  {
    if (suspend_control_)
      return;

    // Keep publishing current command to maintain position
    if (has_joint_state_)
    {
      // Maintain current position by using current joint positions as targets
      std::unordered_map<std::string, double> current_targets;
      for (const auto& name : joint_names_)
      {
        if (current_joint_positions_.find(name) != current_joint_positions_.end())
        {
          current_targets[name] = current_joint_positions_[name];
        }
        else
        {
          // If joint state not available, use last command
          if (last_command_.find(name) != last_command_.end())
          {
            current_targets[name] = last_command_[name];
          }
        }
      }
      applySmoothingAndPublish(current_targets);
    }
    
    control_loop_count_++;
    if (control_loop_count_ % 1000 == 0)
    {
      ROS_INFO_THROTTLE(10.0, "System running... (loop: %d)", control_loop_count_);
      ROS_INFO_THROTTLE(10.0, "  Grasp count: %d", grasp_count_);
      ROS_INFO_THROTTLE(10.0, "  Is grasping: %s", is_grasping_ ? "Yes" : "No");
      ROS_INFO_THROTTLE(10.0, "  Detection history size: %zu", detection_history_.size());
    }
  }
  
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  
  // Publishers and subscribers
  ros::Publisher joint_cmd_pub_;
  ros::Subscriber joint_state_sub_;
  // ros::Subscriber image_sub_;  // not needed
  ros::Subscriber yolo_sub_;
  ros::Subscriber endpoint_state_sub_;
  
  // Service clients
  ros::ServiceClient ik_client_;
  ros::ServiceClient fk_client_;
  
  // Timers
  ros::Timer control_timer_;
  ros::Timer detection_timer_;
  
  // State
  bool has_joint_state_;
  bool is_grasping_;
  bool is_initialized_;
  int grasp_count_;
  int control_loop_count_;
  bool suspend_control_;
  bool has_endpoint_state_;
  
  // Joint state
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, double> current_joint_positions_;
  std::unordered_map<std::string, double> last_command_;
  std::unordered_map<std::string, double> joint_velocity_;
  std::unordered_map<std::string, std::pair<double, double>> joint_limits_;
  geometry_msgs::Pose current_endpoint_pose_;
  
  // Calibration
  Eigen::Matrix3d homography_matrix_;
  double table_height_;
  double object_height_;
  
  // Detection
  std::deque<Detection> detection_history_;
  double conf_threshold_;
  double stable_time_;
  double position_threshold_;
  int min_detections_;
  
  // Fixed grasp position (saved when object is detected)
  Eigen::Vector3d fixed_grasp_position_;
  bool has_fixed_grasp_position_;
  
  // Smoothing parameters - all joints use spring-damper dynamics
  double omega_j0_, omega_j1_, omega_j2_, omega_j3_, omega_j4_, omega_j5_, omega_j6_;
  double zeta_j0_, zeta_j1_, zeta_j2_, zeta_j3_, zeta_j4_, zeta_j5_, zeta_j6_;
  double j0_max_delta_, j1_max_delta_, j2_max_delta_, j3_max_delta_, j4_max_delta_, j5_max_delta_, j6_max_delta_;
  double j0_snap_threshold_, j1_snap_threshold_, j2_snap_threshold_, j3_snap_threshold_, j4_snap_threshold_, j5_snap_threshold_, j6_snap_threshold_;
  double smoothing_base_, smoothing_min_, smoothing_max_;
  double speed_gain_;  // From arm_follow (legacy, not used for spring-damper)
  double max_delta_per_cycle_;  // Legacy, not used for spring-damper
  
  // Movement parameters
  double move_timeout_;
  double move_threshold_;
  double pre_grasp_offset_;
  double lift_height_;
  
  // Image buffer not required (handled by YOLO node)
  // cv::Mat current_image_;
  
  // Time
  ros::Time last_update_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_grasping");
  
  VisualGraspingNode node;
  
  ros::spin();
  
  return 0;
}

