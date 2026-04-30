//
// Created by cqincat on 10/31/25.
//

#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <memory>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sentry_chassis_controller/pivot_wheel_tuning.h>
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

namespace sentry_chassis_controller {

class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  SentryChassisController() = default ;
  ~SentryChassisController() override = default;

 private:
  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
          ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;
  void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
  void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void computeWheelEfforts(const ros::Time& time, const ros::Duration& period);
  void odom_update(const ros::Time& time, const ros::Duration& period);
  void applyTimeoutAndLock(const ros::Time& time);
  double rampCommand(double current, double target, double acc_limit, double dt) const;

  hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
  hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_, back_left_wheel_joint_, back_right_wheel_joint_;
  control_toolbox::Pid front_left_pivot_pos_pid_, front_right_pivot_pos_pid_, back_left_pivot_pos_pid_, back_right_pivot_pos_pid_;
  control_toolbox::Pid front_left_pivot_vel_pid_, front_right_pivot_vel_pid_, back_left_pivot_vel_pid_, back_right_pivot_vel_pid_;
  control_toolbox::Pid front_left_wheel_vel_pid_, front_right_wheel_vel_pid_, back_left_wheel_vel_pid_, back_right_wheel_vel_pid_;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber odom_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  tf2_ros::TransformBroadcaster odom_broadcaster;
  tf::TransformListener tf_listener_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> odom_buffer_;

  ros::Time last_cmd_vel_time_;
  ros::Time last_cmd_input_time_;
  double wheel_track_;
  double wheel_base_;
  double wheel_radius_;
  double cmd_vel_timeout_;
  double cmd_accel_x_;
  double cmd_accel_y_;
  double cmd_accel_w_;
  double filtered_cmd_x_ = 0.0;
  double filtered_cmd_y_ = 0.0;
  double filtered_cmd_w_ = 0.0;
  double rx, ry;
  bool odomMode;
  bool spinMode;
  bool lockMode;

//  double left_front_pivot_offset_, right_front_pivot_offset_, left_back_pivot_offset_, right_back_pivot_offset_;
  double desired_left_front_vel = 0.0;
  double desired_right_front_vel = 0.0;
  double desired_left_back_vel = 0.0;
  double desired_right_back_vel =0.0;
  double desired_left_front_angle = 0.0;
  double desired_right_front_angle = 0.0;
  double desired_left_back_angle = 0.0;
  double desired_right_back_angle =0.0;
  double actual_front_left_vel = 0;
  double actual_front_right_vel = 0;
  double actual_back_left_vel = 0;
  double actual_back_right_vel = 0;
  double actual_front_left_pos = 0;
  double actual_front_right_pos = 0;
  double actual_back_left_pos = 0;
  double actual_back_right_pos = 0;
  double actual_front_left_pivot_vel = 0;
  double actual_front_right_pivot_vel = 0;
  double actual_back_left_pivot_vel = 0;
  double actual_back_right_pivot_vel = 0;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double dt = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dth = 0.0;

  double yaw = 0.0;
  std::string gimbal_frame_;  // 云台坐标系
  PivotAndWheelTuning pivot_tuning_;
};
}// namespace sentry_chassis_controller

#endif //SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
