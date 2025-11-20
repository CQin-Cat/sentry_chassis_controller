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
#include <dynamic_reconfigure/server.h>
#include <sentry_chassis_controller/PIDConfig.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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
  void cb(const PIDConfig& config, uint32_t level);
  void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
  void computeWheelEfforts(const ros::Time& time, const ros::Duration& period);
  void odom_update(const ros::Time& time, const ros::Duration& period);

  hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
  hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_, back_left_wheel_joint_, back_right_wheel_joint_;
  ros::Subscriber cmd_vel_sub;
  std::shared_ptr<dynamic_reconfigure::Server<PIDConfig>> server;
  dynamic_reconfigure::Server<PIDConfig>::CallbackType cbType;
  ros::Publisher lf_error_vel_pub,rf_error_vel_pub, lb_error_vel_pub, rb_error_vel_pub;
  ros::Publisher odom_pub;
  tf2_ros::TransformBroadcaster odom_broadcaster;
  tf::TransformListener tf_listener_;
    tf2_ros::Buffer buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
//    tf2_ros::Buffer tf_buffer_;
//    tf2_ros::TransformListener tf_listener_;

  ros::Time last_change_;
  double wheel_track_;
  double wheel_base_;
  double wheel_radius_;
  double rx, ry;
  bool odomMode;

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

  control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
  control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double dt = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dth = 0.0;
};
}// namespace sentry_chassis_controller

#endif //SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
