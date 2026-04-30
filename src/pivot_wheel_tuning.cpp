#include <sentry_chassis_controller/pivot_wheel_tuning.h>
#include <cmath>

namespace sentry_chassis_controller {

bool PivotAndWheelTuning::init(ros::NodeHandle& controller_nh) {
    if (!controller_nh.getParam("debug_state_pub_rate", state_pub_rate_) ||
          !controller_nh.getParam("pivot_sine/enabled", pivot_sine_enable_) ||
                !controller_nh.getParam("pivot_sine/amplitude", pivot_sine_amp_) ||
                    !controller_nh.getParam("pivot_sine/frequency", pivot_sine_freq_) ||
                        !controller_nh.getParam("pivot_sine/bias", pivot_sine_bias_) ||
                            !controller_nh.getParam("wheel_sine/enabled", wheel_sine_enable_) ||
                                !controller_nh.getParam("wheel_sine/amplitude", wheel_sine_amp_) ||
                                    !controller_nh.getParam("wheel_sine/frequency", wheel_sine_freq_) ||
                                        !controller_nh.getParam("wheel_sine/bias", wheel_sine_bias_)) {                       
        ROS_ERROR("Missing tuning required parameters in %s", controller_nh.getNamespace().c_str());
        return false;
    }

  fl_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "left_front_pivot/state", 100));
  fr_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "right_front_pivot/state", 100));
  bl_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "left_back_pivot/state", 100));
  br_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "right_back_pivot/state", 100));

  fl_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "left_front_pivot/pos_state", 100));
  fr_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "right_front_pivot/pos_state", 100));
  bl_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "left_back_pivot/pos_state", 100));
  br_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "right_back_pivot/pos_state", 100));

  wheel_fr_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "right_front_wheel/state", 100));
  wheel_fl_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "left_front_wheel/state", 100));
  wheel_bl_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "left_back_wheel/state", 100));
  wheel_br_state_pub_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(
    controller_nh, "right_back_wheel/state", 100));

  return true;
}

void PivotAndWheelTuning::sineEnableCb(const std_msgs::Bool::ConstPtr& msg) {
  const bool old = pivot_sine_enable_;
  pivot_sine_enable_ = msg->data;
  if (!old && pivot_sine_enable_) {
    pivot_sine_start_time_ = ros::Time::now();
  }
}

void PivotAndWheelTuning::sineParamCb(const geometry_msgs::Vector3::ConstPtr& msg) {
  pivot_sine_amp_ = msg->x;
  pivot_sine_freq_ = msg->y;
  pivot_sine_bias_ = msg->z; 
}

bool PivotAndWheelTuning::applySinePivotTuning(const ros::Time& time,
                                     double& des_fl_ang, double& des_fr_ang, double& des_bl_ang, double& des_br_ang,
                                     double& des_fl_vel, double& des_fr_vel, double& des_bl_vel, double& des_br_vel,
                                     ros::Time& last_cmd_vel_time) {
  if (!pivot_sine_enable_) return false;

  const double t = (time - pivot_sine_start_time_).toSec();
  const double ref = pivot_sine_bias_ + pivot_sine_amp_ * std::sin(2.0 * M_PI * pivot_sine_freq_ * t);

  des_fl_ang = ref;
  des_fr_ang = ref;
  des_bl_ang = ref;
  des_br_ang = ref;

  des_fl_vel = 0.0;
  des_fr_vel = 0.0;
  des_bl_vel = 0.0;
  des_br_vel = 0.0;

  last_cmd_vel_time = time;
  return true;
}

bool PivotAndWheelTuning::applySineWheelTuning(const ros::Time& time,
                                     double& des_fl_ang, double& des_fr_ang, double& des_bl_ang, double& des_br_ang,
                                     double& des_fl_vel, double& des_fr_vel, double& des_bl_vel, double& des_br_vel,
                                     ros::Time& last_cmd_vel_time) {
  if (!wheel_sine_enable_) return false;

  const double t = (time - wheel_sine_start_time_).toSec();
  const double ref = wheel_sine_bias_ + wheel_sine_amp_ * std::sin(2.0 * M_PI * wheel_sine_freq_ * t);

  des_fl_vel = ref;
  des_fr_vel = ref;
  des_bl_vel = ref;
  des_br_vel = ref;

  des_fl_ang = 0.0;
  des_fr_ang = 0.0;
  des_bl_ang = 0.0;
  des_br_ang = 0.0;

  last_cmd_vel_time = time;
  return true;
}

void PivotAndWheelTuning::publishState(const ros::Time& time,
                               double pos_des_fl, double pos_act_fl, double pos_err_fl, double pos_cmd_fl,
                               double vel_des_fl, double vel_act_fl, double vel_err_fl, double vel_cmd_fl,
                               double pos_des_fr, double pos_act_fr, double pos_err_fr, double pos_cmd_fr,
                               double vel_des_fr, double vel_act_fr, double vel_err_fr, double vel_cmd_fr,
                               double pos_des_bl, double pos_act_bl, double pos_err_bl, double pos_cmd_bl,
                               double vel_des_bl, double vel_act_bl, double vel_err_bl, double vel_cmd_bl,
                               double pos_des_br, double pos_act_br, double pos_err_br, double pos_cmd_br,
                               double vel_des_br, double vel_act_br, double vel_err_br, double vel_cmd_br,
                               double wheel_vel_des_fl, double wheel_vel_act_fl, double wheel_vel_err_fl, double wheel_vel_cmd_fl,
                               double wheel_vel_des_fr, double wheel_vel_act_fr, double wheel_vel_err_fr, double wheel_vel_cmd_fr,
                               double wheel_vel_des_bl, double wheel_vel_act_bl, double wheel_vel_err_bl, double wheel_vel_cmd_bl,
                               double wheel_vel_des_br, double wheel_vel_act_br, double wheel_vel_err_br, double wheel_vel_cmd_br) {
  if (state_pub_rate_ <= 0.0) return;

  if (fl_state_pub_ && fl_state_pub_->trylock()) {
    fl_state_pub_->msg_.header.stamp = time;
    fl_state_pub_->msg_.set_point = vel_des_fl;
    fl_state_pub_->msg_.process_value = vel_act_fl;
    fl_state_pub_->msg_.error = vel_err_fl;
    fl_state_pub_->msg_.command = vel_cmd_fl;
    fl_state_pub_->unlockAndPublish();
  }
  if (fl_pos_state_pub_ && fl_pos_state_pub_->trylock()) {
    fl_pos_state_pub_->msg_.header.stamp = time;
    fl_pos_state_pub_->msg_.set_point = pos_des_fl;
    fl_pos_state_pub_->msg_.process_value = pos_act_fl;
    fl_pos_state_pub_->msg_.error = pos_err_fl;
    fl_pos_state_pub_->msg_.command = pos_cmd_fl;
    fl_pos_state_pub_->unlockAndPublish();
  }

  if (fr_state_pub_ && fr_state_pub_->trylock()) {
    fr_state_pub_->msg_.header.stamp = time;
    fr_state_pub_->msg_.set_point = vel_des_fr;
    fr_state_pub_->msg_.process_value = vel_act_fr;
    fr_state_pub_->msg_.error = vel_err_fr;
    fr_state_pub_->msg_.command = vel_cmd_fr;
    fr_state_pub_->unlockAndPublish();
  }
  if (fr_pos_state_pub_ && fr_pos_state_pub_->trylock()) {
    fr_pos_state_pub_->msg_.header.stamp = time;
    fr_pos_state_pub_->msg_.set_point = pos_des_fr;
    fr_pos_state_pub_->msg_.process_value = pos_act_fr;
    fr_pos_state_pub_->msg_.error = pos_err_fr;
    fr_pos_state_pub_->msg_.command = pos_cmd_fr;
    fr_pos_state_pub_->unlockAndPublish();
  }

  if (bl_state_pub_ && bl_state_pub_->trylock()) {
    bl_state_pub_->msg_.header.stamp = time;
    bl_state_pub_->msg_.set_point = vel_des_bl;
    bl_state_pub_->msg_.process_value = vel_act_bl;
    bl_state_pub_->msg_.error = vel_err_bl;
    bl_state_pub_->msg_.command = vel_cmd_bl;
    bl_state_pub_->unlockAndPublish();
  }
  if (bl_pos_state_pub_ && bl_pos_state_pub_->trylock()) {
    bl_pos_state_pub_->msg_.header.stamp = time;
    bl_pos_state_pub_->msg_.set_point = pos_des_bl;
    bl_pos_state_pub_->msg_.process_value = pos_act_bl;
    bl_pos_state_pub_->msg_.error = pos_err_bl;
    bl_pos_state_pub_->msg_.command = pos_cmd_bl;
    bl_pos_state_pub_->unlockAndPublish();
  }

  if (br_state_pub_ && br_state_pub_->trylock()) {
    br_state_pub_->msg_.header.stamp = time;
    br_state_pub_->msg_.set_point = vel_des_br;
    br_state_pub_->msg_.process_value = vel_act_br;
    br_state_pub_->msg_.error = vel_err_br;
    br_state_pub_->msg_.command = vel_cmd_br;
    br_state_pub_->unlockAndPublish();
  }
  if (br_pos_state_pub_ && br_pos_state_pub_->trylock()) {
    br_pos_state_pub_->msg_.header.stamp = time;
    br_pos_state_pub_->msg_.set_point = pos_des_br;
    br_pos_state_pub_->msg_.process_value = pos_act_br;
    br_pos_state_pub_->msg_.error = pos_err_br;
    br_pos_state_pub_->msg_.command = pos_cmd_br;
    br_pos_state_pub_->unlockAndPublish();
  }
  if(wheel_fl_state_pub_ && wheel_fl_state_pub_->trylock()) {
    wheel_fl_state_pub_->msg_.header.stamp = time;
    wheel_fl_state_pub_->msg_.set_point = wheel_vel_des_fl;
    wheel_fl_state_pub_->msg_.process_value = wheel_vel_act_fl;
    wheel_fl_state_pub_->msg_.error = wheel_vel_err_fl;
    wheel_fl_state_pub_->msg_.command = wheel_vel_cmd_fl;
    wheel_fl_state_pub_->unlockAndPublish();
  }
  if(wheel_fr_state_pub_ && wheel_fr_state_pub_->trylock()) {
    wheel_fr_state_pub_->msg_.header.stamp = time;
    wheel_fr_state_pub_->msg_.set_point = wheel_vel_des_fr;
    wheel_fr_state_pub_->msg_.process_value = wheel_vel_act_fr;
    wheel_fr_state_pub_->msg_.error = wheel_vel_err_fr;
    wheel_fr_state_pub_->msg_.command = wheel_vel_cmd_fr;
    wheel_fr_state_pub_->unlockAndPublish();
  }
  if(wheel_bl_state_pub_ && wheel_bl_state_pub_->trylock()) {
    wheel_bl_state_pub_->msg_.header.stamp = time;
    wheel_bl_state_pub_->msg_.set_point = wheel_vel_des_bl;
    wheel_bl_state_pub_->msg_.process_value = wheel_vel_act_bl;
    wheel_bl_state_pub_->msg_.error = wheel_vel_err_bl;
    wheel_bl_state_pub_->msg_.command = wheel_vel_cmd_bl;
    wheel_bl_state_pub_->unlockAndPublish();
  }
  if(wheel_br_state_pub_ && wheel_br_state_pub_->trylock()) {
    wheel_br_state_pub_->msg_.header.stamp = time;
    wheel_br_state_pub_->msg_.set_point = wheel_vel_des_br;
    wheel_br_state_pub_->msg_.process_value = wheel_vel_act_br;
    wheel_br_state_pub_->msg_.error = wheel_vel_err_br;
    wheel_br_state_pub_->msg_.command = wheel_vel_cmd_br;
    wheel_br_state_pub_->unlockAndPublish();
  } 
}
}  // namespace sentry_chassis_controller