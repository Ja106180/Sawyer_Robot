#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <intera_core_msgs/EndpointState.h>
#include <intera_core_msgs/SolvePositionIK.h>
#include <intera_core_msgs/JointCommand.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <cstdlib>

geometry_msgs::Pose g_current_endpoint;
bool g_has_endpoint = false;

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

void endpointCallback(const intera_core_msgs::EndpointState::ConstPtr& msg)
{
  g_current_endpoint = msg->pose;
  g_has_endpoint = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go_to_xyz");
  ros::NodeHandle nh("~");

  // Parameters: target position in base frame
  double target_x, target_y, target_z, target_w;
  nh.param("x", target_x, 0.5);
  nh.param("y", target_y, 0.0);
  nh.param("z", target_z, 0.7);
  nh.param("w", target_w, 0.0);  // yaw angle around vertical axis (radians)

  ROS_INFO("Requested target pose -> x=%.3f, y=%.3f, z=%.3f, yaw(w)=%.3f rad",
           target_x, target_y, target_z, target_w);

  // Initialize joints: set j1..j6 and j0 (gnd_set) to 0 in sequence
  ROS_INFO("Running joint zeroing commands (arm_follow scripts) before moving to target...");
  // Initialization order (for safety): j3 -> j1 -> j2 -> j4 -> j5 -> j6 -> j0(gnd_set)
  const std::vector<std::string> init_cmds = {
    "rosrun arm_follow j3_set.py --angle 0.0",
    "rosrun arm_follow j1_set.py --angle 0.0",
    "rosrun arm_follow j2_set.py --angle 0.0",
    "rosrun arm_follow j4_set.py --angle 0.0",
    "rosrun arm_follow j5_set.py --angle 0.0",
    "rosrun arm_follow j6_set.py --angle 0.0",
    "rosrun arm_follow gnd_set.py --angle 0.0"
  };

  for (const auto& cmd : init_cmds)
  {
    ROS_INFO("Executing init command: %s", cmd.c_str());
    int ret = std::system(cmd.c_str());
    if (ret != 0)
    {
      ROS_WARN("Command failed (return=%d): %s", ret, cmd.c_str());
    }
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("Joint zeroing sequence finished.");

  // Subscribe endpoint state to monitor current pose
  ros::Subscriber endpoint_sub = nh.subscribe<intera_core_msgs::EndpointState>(
      "/robot/limb/right/endpoint_state", 10, endpointCallback);

  // Wait for first endpoint state so we know current orientation
  ROS_INFO("Waiting for first endpoint_state message...");
  ros::Time wait_start = ros::Time::now();
  ros::Rate wait_rate(50.0);
  while (ros::ok() && !g_has_endpoint && (ros::Time::now() - wait_start).toSec() < 5.0)
  {
    ros::spinOnce();
    wait_rate.sleep();
  }
  if (!g_has_endpoint)
  {
    ROS_WARN("No endpoint_state received within 5 seconds. Will fall back to vertical-down orientation.");
  }
  else
  {
    ROS_INFO("First endpoint_state received. Will reuse current end-effector orientation.");
  }

  // IK service client
  ros::ServiceClient ik_client = nh.serviceClient<intera_core_msgs::SolvePositionIK>(
      "/ExternalTools/right/PositionKinematicsNode/IKService");

  ROS_INFO("Waiting for IK service...");
  if (!ik_client.waitForExistence(ros::Duration(5.0)))
  {
    ROS_ERROR("IK service not available.");
    return 1;
  }
  ROS_INFO("IK service is ready.");

  // Build target pose (base frame, vertical-down orientation + yaw)
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = target_x;
  target_pose.pose.position.y = target_y;
  target_pose.pose.position.z = target_z;

  // Measured vertical-down quaternion from robot:
  // x=0.998517, y=0.052423, z=-0.003210, w=0.014341
  tf2::Quaternion q_down(0.998517, 0.052423, -0.003210, 0.014341);
  q_down.normalize();
  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, target_w);
  tf2::Quaternion q_target = q_yaw * q_down;
  target_pose.pose.orientation.x = q_target.x();
  target_pose.pose.orientation.y = q_target.y();
  target_pose.pose.orientation.z = q_target.z();
  target_pose.pose.orientation.w = q_target.w();
  ROS_INFO("Target orientation quaternion: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
           q_target.x(), q_target.y(), q_target.z(), q_target.w());

  intera_core_msgs::SolvePositionIK ik_srv;
  ik_srv.request.pose_stamp.push_back(target_pose);
  ik_srv.request.tip_names.push_back("right_hand");

  ROS_INFO("Calling IK for target pose...");
  if (!ik_client.call(ik_srv))
  {
    ROS_ERROR("IK service call failed.");
    return 1;
  }

  if (ik_srv.response.result_type.empty() || ik_srv.response.result_type[0] <= 0)
  {
    int rt = ik_srv.response.result_type.empty() ? 0 : ik_srv.response.result_type[0];
    ROS_ERROR("No valid IK solution. result_type = %d", rt);
    return 1;
  }

  sensor_msgs::JointState joint_solution = ik_srv.response.joints[0];
  ROS_INFO("IK succeeded. Got %zu joints.", joint_solution.name.size());

  // Joint command publisher
  ros::Publisher joint_cmd_pub = nh.advertise<intera_core_msgs::JointCommand>(
      "/robot/limb/right/joint_command", 10);

  // Wait a bit for publisher/subscriber connections
  ros::Duration(0.5).sleep();

  intera_core_msgs::JointCommand cmd;
  cmd.mode = intera_core_msgs::JointCommand::POSITION_MODE;
  cmd.names = joint_solution.name;
  cmd.position = joint_solution.position;

  ROS_INFO("Sending joint command towards target. Node will keep printing current position.");

  ros::Rate rate(20.0);
  ros::Time start_time = ros::Time::now();
  const double pos_tolerance = 0.010;   // 1 cm
  const double yaw_tolerance = 0.05;    // ~3 degrees
  bool target_within_tolerance = false;
  ros::Time tolerance_enter_time;

  while (ros::ok())
  {
    // Keep publishing the same command so the controller holds the target
    joint_cmd_pub.publish(cmd);

    ros::spinOnce();

    if (g_has_endpoint)
    {
      const auto& p = g_current_endpoint.position;
      const auto& o = g_current_endpoint.orientation;
      tf2::Quaternion q_cur(o.x, o.y, o.z, o.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_cur).getRPY(roll, pitch, yaw);

      double dx = p.x - target_x;
      double dy = p.y - target_y;
      double dz = p.z - target_z;
      double pos_error = std::sqrt(dx * dx + dy * dy + dz * dz);
      double yaw_error = normalizeAngle(yaw - target_w);

      ROS_INFO_THROTTLE(0.2,
                        "Target: (x=%.3f, y=%.3f, z=%.3f, w=%.3f) | "
                        "Current: (x=%.3f, y=%.3f, z=%.3f, yaw=%.3f) | "
                        "Error: pos=%.3f m, yaw=%.3f rad",
                        target_x, target_y, target_z, target_w,
                        p.x, p.y, p.z, yaw,
                        pos_error, yaw_error);

      if (pos_error < pos_tolerance && std::fabs(yaw_error) < yaw_tolerance)
      {
        if (!target_within_tolerance)
        {
          target_within_tolerance = true;
          tolerance_enter_time = ros::Time::now();
          ROS_INFO("Within tolerance (pos=%.3f m, yaw=%.3f rad). Holding for 1s before exit...",
                   pos_error, yaw_error);
        }
        else if ((ros::Time::now() - tolerance_enter_time).toSec() > 1.0)
        {
          ROS_INFO("Hold complete. Reached target pose, exiting go_to_xyz.");
          break;
        }
      }
      else if (target_within_tolerance)
      {
        ROS_INFO("Left tolerance band (pos=%.3f m, yaw=%.3f rad). Continuing to command target.",
                 pos_error, yaw_error);
        target_within_tolerance = false;
      }
    }

    rate.sleep();
  }

  return 0;
}


