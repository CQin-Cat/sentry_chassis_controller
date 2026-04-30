#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <memory>

namespace sentry_chassis_controller {

class PivotAndWheelTuning {
public:
  bool init(ros::NodeHandle& controller_nh);

  // 返回 true 表示已启用正弦激励，并已写入目标角/速度
  bool applySinePivotTuning(const ros::Time& time,
                          double& des_fl_ang, double& des_fr_ang, double& des_bl_ang, double& des_br_ang,
                          double& des_fl_vel, double& des_fr_vel, double& des_bl_vel, double& des_br_vel,
                          ros::Time& last_cmd_vel_time);
  bool applySineWheelTuning(const ros::Time& time,
                          double& des_fl_ang, double& des_fr_ang, double& des_bl_ang, double& des_br_ang,
                          double& des_fl_vel, double& des_fr_vel, double& des_bl_vel, double& des_br_vel,
                          ros::Time& last_cmd_vel_time);

  void publishState(const ros::Time& time,
                    double pos_des_fl, double pos_act_fl, double pos_err_fl, double pos_cmd_fl,
                    double vel_des_fl, double vel_act_fl, double vel_err_fl, double vel_cmd_fl,
                    double pos_des_fr, double pos_act_fr, double pos_err_fr, double pos_cmd_fr,
                    double vel_des_fr, double vel_act_fr, double vel_err_fr, double vel_cmd_fr,
                    double pos_des_bl, double pos_act_bl, double pos_err_bl, double pos_cmd_bl,
                    double vel_des_bl, double vel_act_bl, double vel_err_bl, double vel_cmd_bl,
                    double pos_des_br, double pos_act_br, double pos_err_br, double pos_cmd_br,
                    double vel_des_br, double vel_act_br, double vel_err_br, double vel_cmd_br,
                    // wheel velocity states (for publishing wheel PID state)
                    double wheel_vel_des_fl, double wheel_vel_act_fl, double wheel_vel_err_fl, double wheel_vel_cmd_fl,
                    double wheel_vel_des_fr, double wheel_vel_act_fr, double wheel_vel_err_fr, double wheel_vel_cmd_fr,
                    double wheel_vel_des_bl, double wheel_vel_act_bl, double wheel_vel_err_bl, double wheel_vel_cmd_bl,
                    double wheel_vel_des_br, double wheel_vel_act_br, double wheel_vel_err_br, double wheel_vel_cmd_br);

private:
  void sineEnableCb(const std_msgs::Bool::ConstPtr& msg);
  void sineParamCb(const geometry_msgs::Vector3::ConstPtr& msg);

private:
  bool pivot_sine_enable_{false};
  double pivot_sine_amp_{0.2};
  double pivot_sine_freq_{0.1};
  double pivot_sine_bias_{0.0};
  ros::Time pivot_sine_start_time_;

  bool wheel_sine_enable_{false};
  double wheel_sine_amp_{0.2};  
  double wheel_sine_freq_{0.1};
  double wheel_sine_bias_{0.0};
  ros::Time wheel_sine_start_time_;

  double state_pub_rate_{100.0};

  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> fl_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> fr_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> bl_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> br_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> fl_pos_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> fr_pos_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> bl_pos_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> br_pos_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> wheel_fr_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> wheel_fl_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> wheel_bl_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> wheel_br_state_pub_;
};

}  // namespace sentry_chassis_controller