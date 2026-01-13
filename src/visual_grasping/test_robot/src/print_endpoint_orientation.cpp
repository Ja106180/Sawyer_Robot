#include <ros/ros.h>
#include <intera_core_msgs/EndpointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cstdlib>
#include <vector>
#include <string>

void initializeArm()
{
  ROS_INFO("Running joint initialization commands (print_endpoint_orientation)...");

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

  ROS_INFO("Arm initialization complete (print_endpoint_orientation)");
}

void endpointCallback(const intera_core_msgs::EndpointState::ConstPtr& msg)
{
  const auto& p = msg->pose.position;
  const auto& o = msg->pose.orientation;
  tf2::Quaternion q(o.x, o.y, o.z, o.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  ROS_INFO("Endpoint pose:");
  ROS_INFO("  position:  x=%.3f, y=%.3f, z=%.3f", p.x, p.y, p.z);
  ROS_INFO("  orientation (quat): x=%.6f, y=%.6f, z=%.6f, w=%.6f",
           o.x, o.y, o.z, o.w);
  ROS_INFO("  orientation (rpy):  roll=%.3f, pitch=%.3f, yaw=%.3f (rad)",
           roll, pitch, yaw);
  ROS_INFO("------------------------------------------------------------");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "print_endpoint_orientation");
  ros::NodeHandle nh;

  // Initialize arm first
  initializeArm();
  ROS_INFO("Waiting 3 seconds for arm to stabilize...");
  ros::Duration(3.0).sleep();

  ros::Subscriber sub = nh.subscribe<intera_core_msgs::EndpointState>(
      "/robot/limb/right/endpoint_state", 10, endpointCallback);

  ROS_INFO("Subscribed to /robot/limb/right/endpoint_state.");
  ROS_INFO("Move the arm to the pose you think is vertically down.");
  ROS_INFO("Endpoint pose (position + quaternion + RPY) will be printed continuously.");

  ros::spin();
  return 0;
}


