#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <arm_follow/PersonKeypoints.h>
#include <arm_follow/PersonsKeypoints.h>
#include <intera_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>

namespace
{
double clamp(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(max_val, value));
}

double mapRange(double value, double in_min, double in_max, double out_min, double out_max)
{
  if (std::abs(in_max - in_min) < 1e-6)
  {
    return out_min;
  }
  double ratio = (value - in_min) / (in_max - in_min);
  ratio = clamp(ratio, 0.0, 1.0);
  return out_min + ratio * (out_max - out_min);
}

double angleBetween(double ax, double ay, double bx, double by)
{
  double dot = ax * bx + ay * by;
  double mag_a = std::hypot(ax, ay);
  double mag_b = std::hypot(bx, by);
  if (mag_a < 1e-6 || mag_b < 1e-6)
  {
    return 0.0;
  }
  double cos_angle = clamp(dot / (mag_a * mag_b), -1.0, 1.0);
  return std::acos(cos_angle);
}
}  // namespace

class ArmFollowNode
{
public:
  ArmFollowNode()
    : nh_()
    , pnh_("~")
    , has_joint_state_(false)
    , last_command_initialized_(false)
    , conf_threshold_(pnh_.param("confidence_threshold", 0.5))
    , smoothing_base_(pnh_.param("smoothing_factor", 0.6))
    , smoothing_min_(pnh_.param("smoothing_min", 0.2))
    , smoothing_max_(pnh_.param("smoothing_max", 0.9))
    , speed_gain_(pnh_.param("speed_adapt_gain", 0.15))
    , prediction_horizon_(pnh_.param("prediction_horizon", 0.08))
    , max_delta_per_cycle_(pnh_.param("max_delta_per_cycle", 0.25))
    , j1_smoothing_factor_(pnh_.param("j1_smoothing_factor", 0.18))
    , j1_max_delta_(pnh_.param("j1_max_delta", 0.40))
    , j1_snap_threshold_(pnh_.param("j1_snap_threshold", 0.05))
    , j1_down_scale_(pnh_.param("j1_down_scale", 0.6))
    , j3_smoothing_factor_(pnh_.param("j3_smoothing_factor", 0.12))
    , j3_max_delta_(pnh_.param("j3_max_delta", 0.60))
    , j3_snap_threshold_(pnh_.param("j3_snap_threshold", 0.05))
    , wait_posture_ready_(pnh_.param("wait_posture_ready", true))
    , warn_joint_state_missing_(false)
    , j3_scale_(pnh_.param("j3_scale", 0.8))
    , j3_offset_(pnh_.param("j3_offset", 0.0))
    , neutral_j3_(0.0)
    , neutral_j3_ready_(false)
    , shoulder_rel_min_(pnh_.param("shoulder_rel_min", -1.5))
    , shoulder_rel_max_(pnh_.param("shoulder_rel_max", 1.5))
    , shoulder_gain_(pnh_.param("shoulder_gain", 1.5))
    , shoulder_metric_alpha_(pnh_.param("shoulder_metric_alpha", 0.3))
    , last_elbow_dir_(1.0)
    , wrist_metric_initialized_(false)
    , filtered_wrist_metric_(0.0)
  {
    joint_names_ = { "right_j1", "right_j3" };

    initJointLimits();
    waitForPostureReady();

    joint_cmd_pub_ = nh_.advertise<intera_core_msgs::JointCommand>("/robot/limb/right/joint_command", 10);
    joint_state_sub_ = nh_.subscribe("/robot/joint_states", 5, &ArmFollowNode::jointStateCallback, this);
    keypoints_sub_ = nh_.subscribe("/yolo_pose/keypoints", 5, &ArmFollowNode::keypointsCallback, this);
    control_timer_ = nh_.createTimer(ros::Duration(0.01), &ArmFollowNode::controlLoop, this);

    ROS_INFO("Arm follow C++ node initialized.");
    last_update_time_ = ros::Time::now();
  }

private:
  struct HandState
  {
    ros::Time stamp;
    double wrist_x = 0.0;
    double wrist_y = 0.0;
    double elbow_x = 0.0;
    double elbow_y = 0.0;
  };

  struct JointLimits
  {
    double min;
    double max;
  };

  void initJointLimits()
  {
    joint_limits_["right_j1"] = { pnh_.param("j1_min", -1.7), pnh_.param("j1_max", 1.4) };// -1.2 - 1.4
    joint_limits_["right_j3"] = { pnh_.param("j3_min", -2.0), pnh_.param("j3_max", 1.5) };// -1.5 - 1.5
  }

  void waitForPostureReady()
  {
    if (!wait_posture_ready_)
    {
      return;
    }

    ros::Rate rate(10.0);
    bool ready = false;
    while (ros::ok())
    {
      if (ros::param::get("/arm_follow/posture_ready", ready) && ready)
      {
        ROS_INFO("Detected posture ready flag, proceeding.");
        break;
      }
      ROS_INFO_THROTTLE(5.0, "Waiting for posture initialization...");
      rate.sleep();
    }
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      joint_positions_[msg->name[i]] = msg->position[i];
    }

    bool has_all = true;
    for (const auto& name : joint_names_)
    {
      if (joint_positions_.find(name) == joint_positions_.end())
      {
        has_all = false;
        break;
      }
    }

    if (has_all)
    {
      has_joint_state_ = true;
      if (!last_command_initialized_)
      {
        for (const auto& name : joint_names_)
        {
          last_command_[name] = joint_positions_[name];
        }
        last_command_initialized_ = true;
        auto it = joint_positions_.find("right_j3");
        if (it != joint_positions_.end())
        {
          neutral_j3_ = it->second;
          neutral_j3_ready_ = true;
          ROS_INFO("Captured neutral right_j3=%.3f rad", neutral_j3_);
        }
        ROS_INFO("Captured initial joint positions for controlled joints.");
      }
    }
  }

  void keypointsCallback(const arm_follow::PersonsKeypoints::ConstPtr& msg)
  {
    if (!has_joint_state_ || !last_command_initialized_)
    {
      if (!warn_joint_state_missing_)
      {
        ROS_WARN("Waiting for joint states before issuing commands...");
        warn_joint_state_missing_ = true;
      }
      return;
    }

    arm_follow::PersonKeypoints person;
    if (!selectTrackedPerson(*msg, &person))
    {
      holdPosition();
      return;
    }

    HandState state = buildHandState(person);
    HandState predicted = applyPrediction(state);

    double hand_speed = computeHandSpeed(state);
    updateHistory(state);

    latest_targets_ = mapToJointTargets(person, predicted);
    last_hand_speed_ = hand_speed;
    has_target_ = true;
  }

  void controlLoop(const ros::TimerEvent&)
  {
    if (!has_joint_state_ || !last_command_initialized_ || !has_target_)
    {
      return;
    }
    applySmoothingAndPublish(latest_targets_, last_hand_speed_);
  }

  bool selectTrackedPerson(const arm_follow::PersonsKeypoints& msg, arm_follow::PersonKeypoints* out_person)
  {
    double best_area = 0.0;
    bool found = false;
    for (const auto& person : msg.persons)
    {
      if (person.right_wrist_conf < conf_threshold_ || person.right_elbow_conf < conf_threshold_ ||
          person.right_shoulder_conf < conf_threshold_)
      {
        continue;
      }

      if (person.bbox_area <= 0.0)
      {
        continue;
      }

      if (!found || person.bbox_area > best_area)
      {
        *out_person = person;
        best_area = person.bbox_area;
        found = true;
      }
    }
    return found;
  }

  HandState buildHandState(const arm_follow::PersonKeypoints& person)
  {
    HandState state;
    state.stamp = ros::Time::now();

    const double width = static_cast<double>(person.image_width);
    const double height = static_cast<double>(person.image_height);

    auto normX = [width](double v) { return (v / width) - 0.5; };      // [-0.5, 0.5]
    auto normY = [height](double v) { return 1.0 - (v / height); };    // [0, 1], bottom->0 top->1

    state.wrist_x = normX(person.right_wrist_x);
    state.wrist_y = normY(person.right_wrist_y);
    state.elbow_x = normX(person.right_elbow_x);
    state.elbow_y = normY(person.right_elbow_y);

    latest_should_x_ = normX(person.right_shoulder_x);
    latest_should_y_ = normY(person.right_shoulder_y);
    return state;
  }

  HandState applyPrediction(const HandState& current)
  {
    if (hand_history_.empty())
    {
      return current;
    }

    const HandState& prev = hand_history_.back();
    double dt = (current.stamp - prev.stamp).toSec();
    if (dt < 1e-3)
    {
      return current;
    }

    double vx = (current.wrist_x - prev.wrist_x) / dt;
    double vy = (current.wrist_y - prev.wrist_y) / dt;

    HandState predicted = current;
    predicted.wrist_x = clamp(current.wrist_x + vx * prediction_horizon_, -0.6, 0.6);
    predicted.wrist_y = clamp(current.wrist_y + vy * prediction_horizon_, 0.0, 1.2);
    predicted.elbow_x = current.elbow_x;
    predicted.elbow_y = current.elbow_y;
    return predicted;
  }

  double computeHandSpeed(const HandState& current) const
  {
    if (hand_history_.empty())
    {
      return 0.0;
    }

    const HandState& prev = hand_history_.back();
    double dt = (current.stamp - prev.stamp).toSec();
    if (dt < 1e-3)
    {
      return 0.0;
    }

    double vx = (current.wrist_x - prev.wrist_x) / dt;
    double vy = (current.wrist_y - prev.wrist_y) / dt;
    return std::hypot(vx, vy);
  }

  void updateHistory(const HandState& state)
  {
    hand_history_.push_back(state);
    while (hand_history_.size() > 10)
    {
      hand_history_.pop_front();
    }
  }

  std::unordered_map<std::string, double> mapToJointTargets(const arm_follow::PersonKeypoints& person,
                                                            const HandState& predicted)
  {
    std::unordered_map<std::string, double> targets;

    double sx = predicted.elbow_x - latest_should_x_;
    double sy = predicted.elbow_y - latest_should_y_;
    double upper_len = std::hypot(sx, sy);
    double shoulder_metric = 0.0;
    if (upper_len > 1e-3)
    {
      shoulder_metric = (sy / upper_len) * shoulder_gain_;
    }
    shoulder_metric = clamp(shoulder_metric, shoulder_rel_min_, shoulder_rel_max_);

    if (!wrist_metric_initialized_)
    {
      filtered_wrist_metric_ = shoulder_metric;
      wrist_metric_initialized_ = true;
    }
    else
    {
      filtered_wrist_metric_ =
          shoulder_metric_alpha_ * filtered_wrist_metric_ + (1.0 - shoulder_metric_alpha_) * shoulder_metric;
    }

    // Shoulder pitch (J1) derived from normalized vertical component of shoulder->elbow vector
    const auto& j1_limit = joint_limits_.at("right_j1");
    double metric_for_j1 = filtered_wrist_metric_;
    if (metric_for_j1 < 0.0)
    {
      metric_for_j1 *= j1_down_scale_;
    }
    double j1 = mapRange(metric_for_j1, shoulder_rel_min_, shoulder_rel_max_, j1_limit.max, j1_limit.min);

    // Elbow bend angle (J3)
    const double upper_x = predicted.elbow_x - latest_should_x_;
    const double upper_y = predicted.elbow_y - latest_should_y_;
    const double lower_x = predicted.wrist_x - predicted.elbow_x;
    const double lower_y = predicted.wrist_y - predicted.elbow_y;
    double elbow_angle = angleBetween(upper_x, upper_y, lower_x, lower_y);  // 0 straight, pi folded
    double elbow_norm = clamp(elbow_angle / M_PI, 0.0, 1.0);
    const auto& j3_limit = joint_limits_.at("right_j3");
    double neutral = neutral_j3_ready_ ? neutral_j3_ : (j3_limit.max + j3_limit.min) * 0.5;
    double dir = 0.0;
    double rel = predicted.wrist_y - predicted.elbow_y;
    if (rel > 0.02)
    {
      dir = 1.0;
    }
    else if (rel < -0.02)
    {
      dir = -1.0;
    }
    else
    {
      dir = last_elbow_dir_;
    }
    last_elbow_dir_ = dir;

    double j3 = neutral + j3_offset_ - dir * j3_scale_ * elbow_angle;
    j3 = clamp(j3, j3_limit.min, j3_limit.max);

    targets["right_j1"] = j1;
    targets["right_j3"] = j3;
    return targets;
  }

  void applySmoothingAndPublish(const std::unordered_map<std::string, double>& targets, double hand_speed)
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
      const auto limit = joint_limits_.at(name);
      double q_prev = last_command_[name];
      double q_ref = clamp(targets.at(name), limit.min, limit.max);
      double q_new = q_prev;

      if (name == "right_j1" || name == "right_j3")
      {
        // Virtual spring-damper dynamics for smoother tracking on j1/j3
        double& v = joint_velocity_[name];
        double omega = (name == "right_j1") ? 3.5 : 4.5;
        double zeta = (name == "right_j1") ? 0.95 : 0.9;

        double e = q_ref - q_prev;
        double a = omega * omega * e - 2.0 * zeta * omega * v;
        v += a * dt;
        q_new = q_prev + v * dt;

        // For j1/j3 specifically, avoid overshoot around target when very close
        double err_before = q_ref - q_prev;
        double err_after = q_ref - q_new;
        double snap_threshold = (name == "right_j1") ? j1_snap_threshold_ : j3_snap_threshold_;
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
          // If we are very close to target and moving past it in downward direction, snap to target
          if (std::fabs(err_before) < j1_snap_threshold_ && std::fabs(err_after) < j1_snap_threshold_ &&
              err_before * err_after <= 0.0)
          {
            q_new = q_ref;
            v = 0.0;
          }
        }

        double delta_limit = (name == "right_j1") ? j1_max_delta_ : j3_max_delta_;
        double delta = q_new - q_prev;
        if (std::abs(delta) > delta_limit)
        {
          delta = std::copysign(delta_limit, delta);
          q_new = q_prev + delta;
          v = delta / dt;
        }
        q_new = clamp(q_new, limit.min, limit.max);
      }
      else
      {
        // Fallback to original smoothing (currently unused)
        double smoothing = clamp(smoothing_base_ - speed_gain_ * hand_speed, smoothing_min_, smoothing_max_);
        double delta_limit = max_delta_per_cycle_;

        double desired = q_ref;
        double delta = desired - q_prev;
        if (std::abs(delta) > delta_limit)
        {
          desired = q_prev + std::copysign(delta_limit, delta);
        }

        q_new = smoothing * q_prev + (1.0 - smoothing) * desired;
      }

      last_command_[name] = q_new;
      cmd.names.push_back(name);
      cmd.position.push_back(q_new);
    }

    joint_cmd_pub_.publish(cmd);
  }

  void holdPosition()
  {
    // keep last_command_; control loop will continue holding
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber keypoints_sub_;
  ros::Publisher joint_cmd_pub_;
  ros::Timer control_timer_;

  bool has_joint_state_;
  bool last_command_initialized_;
  bool warn_joint_state_missing_;

  double conf_threshold_;
  double smoothing_base_;
  double smoothing_min_;
  double smoothing_max_;
  double speed_gain_;
  double prediction_horizon_;
  double max_delta_per_cycle_;
  double j1_smoothing_factor_;
  double j1_max_delta_;
  double j1_down_scale_;
  double j1_snap_threshold_;
  double j3_smoothing_factor_;
  double j3_max_delta_;
  double j3_snap_threshold_;

  double latest_should_x_ = 0.0;
  double latest_should_y_ = 0.0;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, JointLimits> joint_limits_;
  std::unordered_map<std::string, double> joint_positions_;
  std::unordered_map<std::string, double> last_command_;

  bool wait_posture_ready_;
  double j3_scale_;
  double j3_offset_;
  double neutral_j3_;
  bool neutral_j3_ready_;
  double shoulder_rel_min_;
  double shoulder_rel_max_;
  double shoulder_gain_;
  double shoulder_metric_alpha_;
  double last_elbow_dir_;
  bool wrist_metric_initialized_;
  double filtered_wrist_metric_;
  ros::Time last_update_time_;
  std::unordered_map<std::string, double> joint_velocity_;
  std::unordered_map<std::string, double> latest_targets_;
  double last_hand_speed_ = 0.0;
  bool has_target_ = false;

  std::deque<HandState> hand_history_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_follow_node_cpp");
  ArmFollowNode node;
  ros::spin();
  return 0;
}

