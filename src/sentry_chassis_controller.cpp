//
// Created by cqincat on 2/6/21.
//

#include <hardware_interface/joint_command_interface.h>
#include <sentry_chassis_controller/sentry_chassis_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

namespace sentry_chassis_controller {

void SentryChassisController::applyTimeoutAndLock(const ros::Time& time){
    if ((time - last_cmd_vel_time_).toSec() > cmd_vel_timeout_) {
        desired_left_front_vel = 0.0;
        desired_right_front_vel = 0.0;
        desired_left_back_vel = 0.0;
        desired_right_back_vel = 0.0;

        if (lockMode) {
            desired_left_front_angle = -0.785;
            desired_right_front_angle = 0.785;
            desired_left_back_angle = 0.785;
            desired_right_back_angle = -0.785;
        }
    }
}

double SentryChassisController::rampCommand(double current, double target, double acc_limit, double dt) const
{
    if (acc_limit <= 0.0 || dt <= 0.0) {
        return target;
    }
    const double max_delta = acc_limit * dt;
    const double delta = std::max(-max_delta, std::min(max_delta, target - current));
    return current + delta;
}

//Inverse kinematics solution
void SentryChassisController::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    const ros::Time now = ros::Time::now();
    double dt = 0.0;
    if (!last_cmd_input_time_.isZero()) {
        dt = std::max(0.0, std::min(0.1, (now - last_cmd_input_time_).toSec()));
    }
    last_cmd_input_time_ = now;

    filtered_cmd_x_ = rampCommand(filtered_cmd_x_, msg->linear.x, cmd_accel_x_, dt);
    filtered_cmd_y_ = rampCommand(filtered_cmd_y_, msg->linear.y, cmd_accel_y_, dt);
    filtered_cmd_w_ = rampCommand(filtered_cmd_w_, msg->angular.z, cmd_accel_w_, dt);

    const double cmd_x = filtered_cmd_x_;
    const double cmd_y = filtered_cmd_y_;
    const double cmd_w = filtered_cmd_w_;

    Vec2<double> left_front_vel, right_front_vel, left_back_vel, right_back_vel;

    std::vector<Vec2<double>> module_positions = {
        Vec2<double>(rx, ry),    //left_front
        Vec2<double>(rx, -ry),   //right_front
        Vec2<double>(-rx, ry),   //left_back
        Vec2<double>(-rx, -ry)   //right_back
    };

    if (!odomMode) {
        //小陀螺模式
        if(spinMode) {
            double act[3] = {cmd_x, cmd_y, cmd_w};
            double base[3] = {0};
            base[0] = act[0] * cos(yaw) - act[1] * sin(yaw);  //x方向
            base[1] = act[0] * sin(yaw) + act[1] * cos(yaw);  //y方向
            base[2] = act[2] + (abs(act[0]) + abs(act[1])) * 2.0;  //角速度
            left_front_vel = Vec2<double>(base[0], base[1]) +
                            base[2] * Vec2<double>(-module_positions[0].y(), module_positions[0].x());
            right_front_vel = Vec2<double>(base[0], base[1]) +
                            base[2] * Vec2<double>(-module_positions[1].y(), module_positions[1].x());
            left_back_vel = Vec2<double>(base[0], base[1]) +
                            base[2] * Vec2<double>(-module_positions[2].y(), module_positions[2].x());
            right_back_vel = Vec2<double>(base[0], base[1]) +
                            base[2] * Vec2<double>(-module_positions[3].y(), module_positions[3].x());

        }else {
            left_front_vel = Vec2<double>(cmd_x, cmd_y) +
                             cmd_w * Vec2<double>(-module_positions[0].y(), module_positions[0].x());
            right_front_vel = Vec2<double>(cmd_x, cmd_y) +
                              cmd_w * Vec2<double>(-module_positions[1].y(), module_positions[1].x());
            left_back_vel = Vec2<double>(cmd_x, cmd_y) +
                            cmd_w * Vec2<double>(-module_positions[2].y(), module_positions[2].x());
            right_back_vel = Vec2<double>(cmd_x, cmd_y) +
                             cmd_w * Vec2<double>(-module_positions[3].y(), module_positions[3].x());
        }
    } else{
        geometry_msgs::Vector3Stamped odom_vel;
        odom_vel.header.stamp = ros::Time(0);
        odom_vel.header.frame_id = "odom";
        odom_vel.vector = msg->linear;
        geometry_msgs::Vector3Stamped base_vel;
        try {
//            tf_listener_.waitForTransform("base_link","odom",ros::Time(0),ros::Duration(0.5));
            tf_listener_.transformVector("base_link", odom_vel, base_vel);
//            tf_listener_.transformVector("odom", ros::Time(0), odom_vel, "base_link", base_vel);
        } catch (tf::TransformException& ex) {
            ROS_WARN("tf error: %s", ex.what());
            return;
        }
        left_front_vel = Vec2<double>(base_vel.vector.x, base_vel.vector.y) +
                                 cmd_w * Vec2<double>(-module_positions[0].y(), module_positions[0].x());
        right_front_vel = Vec2<double>(base_vel.vector.x, base_vel.vector.y) +
                                 cmd_w * Vec2<double>(-module_positions[1].y(), module_positions[1].x());
        left_back_vel = Vec2<double>(base_vel.vector.x, base_vel.vector.y) +
                                 cmd_w * Vec2<double>(-module_positions[2].y(), module_positions[2].x());
        right_back_vel = Vec2<double>(base_vel.vector.x, base_vel.vector.y) +
                                 cmd_w * Vec2<double>(-module_positions[3].y(), module_positions[3].x());
    }

    double left_front_vel_angle = std::atan2(left_front_vel.y(), left_front_vel.x());
    double right_front_vel_angle = std::atan2(right_front_vel.y(), right_front_vel.x());
    double left_back_vel_angle = std::atan2(left_back_vel.y(), left_back_vel.x());
    double right_back_vel_angle = std::atan2(right_back_vel.y(), right_back_vel.x());

    double a = angles::shortest_angular_distance(front_left_pivot_joint_.getPosition(), left_front_vel_angle);
    double b = angles::shortest_angular_distance(front_left_pivot_joint_.getPosition(), left_front_vel_angle + M_PI);
    desired_left_front_angle = std::abs(a) < std::abs(b) ? left_front_vel_angle : left_front_vel_angle + M_PI;

    double c = angles::shortest_angular_distance(front_right_pivot_joint_.getPosition(), right_front_vel_angle);
    double d = angles::shortest_angular_distance(front_right_pivot_joint_.getPosition(), right_front_vel_angle + M_PI);
    desired_right_front_angle = std::abs(c) < std::abs(d) ? right_front_vel_angle : right_front_vel_angle + M_PI;

    double e = angles::shortest_angular_distance(back_left_pivot_joint_.getPosition(), left_back_vel_angle);
    double f = angles::shortest_angular_distance(back_left_pivot_joint_.getPosition(), left_back_vel_angle + M_PI);
    desired_left_back_angle = std::abs(e) < std::abs(f) ? left_back_vel_angle : left_back_vel_angle + M_PI;

    double g = angles::shortest_angular_distance(back_right_pivot_joint_.getPosition(), right_back_vel_angle);
    double h = angles::shortest_angular_distance(back_right_pivot_joint_.getPosition(), right_back_vel_angle + M_PI);
    desired_right_back_angle = std::abs(g) < std::abs(h) ? right_back_vel_angle : right_back_vel_angle + M_PI;

    desired_left_front_vel = left_front_vel.norm() * std::cos(a) / wheel_radius_;
    desired_right_front_vel = right_front_vel.norm() * std::cos(c) / wheel_radius_;
    desired_left_back_vel = left_back_vel.norm() * std::cos(e) / wheel_radius_;
    desired_right_back_vel = right_back_vel.norm()  * std::cos(g) / wheel_radius_;

    last_cmd_vel_time_ = ros::Time::now();
}

void SentryChassisController::odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_buffer_.writeFromNonRT(*msg);
}


void SentryChassisController::odom_update(const ros::Time& time, const ros::Duration& period)
{
    //Forward kinematics calculation
    actual_front_left_vel = front_left_wheel_joint_.getVelocity();
    actual_front_right_vel = front_right_wheel_joint_.getVelocity();
    actual_back_left_vel = back_left_wheel_joint_.getVelocity();
    actual_back_right_vel = back_right_wheel_joint_.getVelocity();
    actual_front_left_pos = front_left_pivot_joint_.getPosition();
    actual_front_right_pos = front_right_pivot_joint_.getPosition();
    actual_back_left_pos = back_left_pivot_joint_.getPosition();
    actual_back_right_pos = back_right_pivot_joint_.getPosition();

    double x_velocity = (actual_front_left_vel * cos(actual_front_left_pos) +
                        actual_front_right_vel * cos(actual_front_right_pos)+
                        actual_back_left_vel * cos(actual_back_left_pos) +
                        actual_back_right_vel * cos(actual_back_right_pos)) * wheel_radius_ / 4;
    double y_velocity = (actual_front_left_vel * sin(actual_front_left_pos) +
                        actual_front_right_vel * sin(actual_front_right_pos)+
                        actual_back_left_vel * sin(actual_back_left_pos) +
                        actual_back_right_vel * sin(actual_back_right_pos)) * wheel_radius_ / 4;
    double th_velocity = (- actual_front_left_vel * cos(actual_front_left_pos) + actual_front_left_vel * sin(actual_front_left_pos)
                        + actual_front_right_vel * cos(actual_front_right_pos) + actual_front_right_vel * sin(actual_front_right_pos)
                        - actual_back_left_vel * cos(actual_back_left_pos) - actual_back_left_vel * sin(actual_back_left_pos)
                        + actual_back_right_vel * cos(actual_back_right_pos) - actual_back_right_vel * sin(actual_back_right_pos))
                        * sqrt(2) /2 / sqrt(ry*ry + rx*rx) * wheel_radius_ / 4;

    dt = period.toSec();
    dx = (x_velocity * cos(th) - y_velocity * sin(th)) * dt;
    dy = (x_velocity * sin(th) + y_velocity * cos(th)) * dt;
    dth = th_velocity * dt;

    x += dx;
    y += dy;
    th += dth;

    if(spinMode){
        geometry_msgs::TransformStamped gimbal_to_base_trans_;
        gimbal_to_base_trans_.header.stamp = time;
        gimbal_to_base_trans_.header.frame_id = gimbal_frame_;
        gimbal_to_base_trans_.child_frame_id = "base_link";
        gimbal_to_base_trans_.transform.translation.x = x;
        gimbal_to_base_trans_.transform.translation.y = y;
        gimbal_to_base_trans_.transform.translation.z = 0.0;
        tf2::Quaternion qtn;
        //暂定以odom作为云台坐标系
        qtn.setRPY(0,0,th);
        yaw = th;
        gimbal_to_base_trans_.transform.rotation.x = qtn.getX();
        gimbal_to_base_trans_.transform.rotation.y = qtn.getY();
        gimbal_to_base_trans_.transform.rotation.z = qtn.getZ();
        gimbal_to_base_trans_.transform.rotation.w = qtn.getW();

        odom_broadcaster.sendTransform(gimbal_to_base_trans_);
    } else {
        //创建坐标转换
        geometry_msgs::TransformStamped odom2base;
        //设置头信息
        //odom_ts.header.seq = 100;
        odom2base.header.stamp = time;
        odom2base.header.frame_id = "odom";
        //设置子级坐标系
        odom2base.child_frame_id = "base_link";
        //设置子级相对于父级的偏移量
        odom2base.transform.translation.x = x;
        odom2base.transform.translation.y = y;
        odom2base.transform.translation.z = 0.0;
        //设置四元数:将 欧拉角数据转换成四元数
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, th);
        odom2base.transform.rotation.x = qtn.getX();
        odom2base.transform.rotation.y = qtn.getY();
        odom2base.transform.rotation.z = qtn.getZ();
        odom2base.transform.rotation.w = qtn.getW();

        odom_broadcaster.sendTransform(odom2base);
    }
    if ( odom_pub_->trylock())
    {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.header.frame_id = "odom";
        odom_pub_->msg_.child_frame_id = "base_link";
        odom_pub_->msg_.pose.pose.position.x = x;
        odom_pub_->msg_.pose.pose.position.y = y;
        odom_pub_->msg_.pose.pose.position.z = 0.0;
        odom_pub_->msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
        odom_pub_->msg_.twist.twist.linear.x = x_velocity;
        odom_pub_->msg_.twist.twist.linear.y = y_velocity;
        odom_pub_->msg_.twist.twist.angular.z = th_velocity;
        odom_pub_->unlockAndPublish();
    }
}

bool SentryChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    front_left_wheel_joint_ =
        effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_wheel_joint_ =
        effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_wheel_joint_ = 
        effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_wheel_joint_ =
        effort_joint_interface->getHandle("right_back_wheel_joint");


    front_left_pivot_joint_ =
        effort_joint_interface->getHandle("left_front_pivot_joint");
    front_right_pivot_joint_ =
        effort_joint_interface->getHandle("right_front_pivot_joint");
    back_left_pivot_joint_ = 
        effort_joint_interface->getHandle("left_back_pivot_joint");
    back_right_pivot_joint_ =
        effort_joint_interface->getHandle("right_back_pivot_joint");

    front_left_pivot_joint_.setCommand(0.);
    front_right_pivot_joint_.setCommand(0.);
    back_left_pivot_joint_.setCommand(0.);
    back_right_pivot_joint_.setCommand(0.);

//  wheel_track_ = controller_nh.param("wheel_track", 0.362);
//  wheel_base_ = controller_nh.param("wheel_base", 0.362);
    if (!controller_nh.getParam("wheel_track", wheel_track_) ||
            !controller_nh.getParam("wheel_base", wheel_base_) ||
                !controller_nh.getParam("wheel_radius", wheel_radius_) ||
                    !controller_nh.getParam("cmd_vel_timeout", cmd_vel_timeout_) ||
                        !controller_nh.getParam("odomMode", odomMode) ||
                            !controller_nh.getParam("spinMode", spinMode) ||
                                !controller_nh.getParam("lockMode", lockMode)) {
        ROS_ERROR("Missing required parameters in %s", controller_nh.getNamespace().c_str());
        return false;
    }

    cmd_accel_x_ = controller_nh.param("cmd_accel_x", 100.0);
    cmd_accel_y_ = controller_nh.param("cmd_accel_y", 100.0);
    cmd_accel_w_ = controller_nh.param("cmd_accel_w", 100.0);

    rx = wheel_base_ / 2.0;
    ry = wheel_track_ / 2.0;

    ros::NodeHandle nh_left_front_pivot(controller_nh, "left_front_pivot");
    ros::NodeHandle nh_right_front_pivot(controller_nh, "right_front_pivot");
    ros::NodeHandle nh_left_back_pivot(controller_nh, "left_back_pivot");
    ros::NodeHandle nh_right_back_pivot(controller_nh, "right_back_pivot");
    ros::NodeHandle nh_left_front_wheel(controller_nh, "left_front_wheel");
    ros::NodeHandle nh_right_front_wheel(controller_nh, "right_front_wheel");
    ros::NodeHandle nh_left_back_wheel(controller_nh, "left_back_wheel");
    ros::NodeHandle nh_right_back_wheel(controller_nh, "right_back_wheel");

    ros::NodeHandle nh_left_front_pivot_pos(nh_left_front_pivot, "pid_pos");
    ros::NodeHandle nh_right_front_pivot_pos(nh_right_front_pivot, "pid_pos");
    ros::NodeHandle nh_left_back_pivot_pos(nh_left_back_pivot, "pid_pos");
    ros::NodeHandle nh_right_back_pivot_pos(nh_right_back_pivot, "pid_pos");
    ros::NodeHandle nh_left_front_pivot_vel(nh_left_front_pivot, "pid_vel");
    ros::NodeHandle nh_right_front_pivot_vel(nh_right_front_pivot, "pid_vel");
    ros::NodeHandle nh_left_back_pivot_vel(nh_left_back_pivot, "pid_vel");
    ros::NodeHandle nh_right_back_pivot_vel(nh_right_back_pivot, "pid_vel");
    ros::NodeHandle nh_left_front_wheel_pid(nh_left_front_wheel, "pid");
    ros::NodeHandle nh_right_front_wheel_pid(nh_right_front_wheel, "pid");
    ros::NodeHandle nh_left_back_wheel_pid(nh_left_back_wheel, "pid");
    ros::NodeHandle nh_right_back_wheel_pid(nh_right_back_wheel, "pid");

    if (!front_left_pivot_pos_pid_.init(nh_left_front_pivot_pos) ||
            !front_right_pivot_pos_pid_.init(nh_right_front_pivot_pos) ||
                !back_left_pivot_pos_pid_.init(nh_left_back_pivot_pos) ||
                    !back_right_pivot_pos_pid_.init(nh_right_back_pivot_pos)) {
        ROS_ERROR("Failed to initialize one or more pivot position PIDs in %s", controller_nh.getNamespace().c_str());
        return false;
    }

    if (!front_left_pivot_vel_pid_.init(nh_left_front_pivot_vel) ||
            !front_right_pivot_vel_pid_.init(nh_right_front_pivot_vel) ||
                !back_left_pivot_vel_pid_.init(nh_left_back_pivot_vel) ||
                    !back_right_pivot_vel_pid_.init(nh_right_back_pivot_vel) ||
                        !front_left_wheel_vel_pid_.init(nh_left_front_wheel_pid) ||
                                !front_right_wheel_vel_pid_.init(nh_right_front_wheel_pid) ||
                                    !back_left_wheel_vel_pid_.init(nh_left_back_wheel_pid) ||
                                        !back_right_wheel_vel_pid_.init(nh_right_back_wheel_pid)) {
        ROS_ERROR("Failed to initialize one or more velocity PIDs in %s", controller_nh.getNamespace().c_str());
        return false;
    }

    //订阅/cmd_vel和odom话题
    cmd_vel_sub = root_nh.subscribe("/cmd_vel", 1, &SentryChassisController::cmd_vel_cb, this);
    odom_sub_ = root_nh.subscribe("odom", 1, &SentryChassisController::odom_cb, this);
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = "base_link";

    //暂时将云台坐标系设置为odom，后续可以改为独立的云台坐标系
    gimbal_frame_ = "odom";
    if (!pivot_tuning_.init(controller_nh)) {
        ROS_ERROR("Failed to initialize pivot tuning in %s", controller_nh.getNamespace().c_str());
        return false;
    }
    last_cmd_vel_time_ = ros::Time::now();

  return true;
}

void SentryChassisController::computeWheelEfforts(const ros::Time& time, const ros::Duration& period)
{
    actual_front_left_vel = front_left_wheel_joint_.getVelocity();
    actual_front_right_vel = front_right_wheel_joint_.getVelocity();
    actual_back_left_vel = back_left_wheel_joint_.getVelocity();
    actual_back_right_vel = back_right_wheel_joint_.getVelocity();
    actual_front_left_pos = front_left_pivot_joint_.getPosition();
    actual_front_right_pos = front_right_pivot_joint_.getPosition();
    actual_back_left_pos = back_left_pivot_joint_.getPosition();
    actual_back_right_pos = back_right_pivot_joint_.getPosition();
    actual_front_left_pivot_vel = front_left_pivot_joint_.getVelocity();
    actual_front_right_pivot_vel = front_right_pivot_joint_.getVelocity();
    actual_back_left_pivot_vel = back_left_pivot_joint_.getVelocity();
    actual_back_right_pivot_vel = back_right_pivot_joint_.getVelocity();

    const double left_front_angle_err =
        angles::shortest_angular_distance(actual_front_left_pos, desired_left_front_angle);
    const double right_front_angle_err =
        angles::shortest_angular_distance(actual_front_right_pos, desired_right_front_angle);
    const double left_back_angle_err =
        angles::shortest_angular_distance(actual_back_left_pos, desired_left_back_angle);
    const double right_back_angle_err =
        angles::shortest_angular_distance(actual_back_right_pos, desired_right_back_angle);

    front_left_pivot_pos_pid_.computeCommand(left_front_angle_err, period);
    front_right_pivot_pos_pid_.computeCommand(right_front_angle_err, period);
    back_left_pivot_pos_pid_.computeCommand(left_back_angle_err, period);
    back_right_pivot_pos_pid_.computeCommand(right_back_angle_err, period);

    const double left_front_pivot_vel_err = front_left_pivot_pos_pid_.getCurrentCmd() - actual_front_left_pivot_vel;
    const double right_front_pivot_vel_err = front_right_pivot_pos_pid_.getCurrentCmd() - actual_front_right_pivot_vel;
    const double left_back_pivot_vel_err = back_left_pivot_pos_pid_.getCurrentCmd() - actual_back_left_pivot_vel;
    const double right_back_pivot_vel_err = back_right_pivot_pos_pid_.getCurrentCmd() - actual_back_right_pivot_vel;
    const double left_front_wheel_vel_err = desired_left_front_vel - actual_front_left_vel;
    const double right_front_wheel_vel_err = desired_right_front_vel - actual_front_right_vel;
    const double left_back_wheel_vel_err = desired_left_back_vel - actual_back_left_vel;
    const double right_back_wheel_vel_err = desired_right_back_vel - actual_back_right_vel;

    front_left_pivot_vel_pid_.computeCommand(left_front_pivot_vel_err, period);
    front_right_pivot_vel_pid_.computeCommand(right_front_pivot_vel_err, period);
    back_left_pivot_vel_pid_.computeCommand(left_back_pivot_vel_err, period);
    back_right_pivot_vel_pid_.computeCommand(right_back_pivot_vel_err, period);
    front_left_wheel_vel_pid_.computeCommand(left_front_wheel_vel_err, period);
    front_right_wheel_vel_pid_.computeCommand(right_front_wheel_vel_err, period);
    back_left_wheel_vel_pid_.computeCommand(left_back_wheel_vel_err, period);
    back_right_wheel_vel_pid_.computeCommand(right_back_wheel_vel_err, period);

    front_left_pivot_joint_.setCommand(front_left_pivot_vel_pid_.getCurrentCmd());
    front_right_pivot_joint_.setCommand(front_right_pivot_vel_pid_.getCurrentCmd());
    back_left_pivot_joint_.setCommand(back_left_pivot_vel_pid_.getCurrentCmd());
    back_right_pivot_joint_.setCommand(back_right_pivot_vel_pid_.getCurrentCmd());
    front_left_wheel_joint_.setCommand(front_left_wheel_vel_pid_.getCurrentCmd());
    front_right_wheel_joint_.setCommand(front_right_wheel_vel_pid_.getCurrentCmd());
    back_left_wheel_joint_.setCommand(back_left_wheel_vel_pid_.getCurrentCmd());
    back_right_wheel_joint_.setCommand(back_right_wheel_vel_pid_.getCurrentCmd());

    pivot_tuning_.publishState(
        time,
        desired_left_front_angle, actual_front_left_pos, left_front_angle_err, front_left_pivot_pos_pid_.getCurrentCmd(),
        front_left_pivot_pos_pid_.getCurrentCmd(), actual_front_left_pivot_vel, left_front_pivot_vel_err, front_left_pivot_vel_pid_.getCurrentCmd(),
        desired_right_front_angle, actual_front_right_pos, right_front_angle_err, front_right_pivot_pos_pid_.getCurrentCmd(),
        front_right_pivot_pos_pid_.getCurrentCmd(), actual_front_right_pivot_vel, right_front_pivot_vel_err, front_right_pivot_vel_pid_.getCurrentCmd(),
        desired_left_back_angle, actual_back_left_pos, left_back_angle_err, back_left_pivot_pos_pid_.getCurrentCmd(),
        back_left_pivot_pos_pid_.getCurrentCmd(), actual_back_left_pivot_vel, left_back_pivot_vel_err, back_left_pivot_vel_pid_.getCurrentCmd(),
        desired_right_back_angle, actual_back_right_pos, right_back_angle_err, back_right_pivot_pos_pid_.getCurrentCmd(),
        back_right_pivot_pos_pid_.getCurrentCmd(), actual_back_right_pivot_vel, right_back_pivot_vel_err, back_right_pivot_vel_pid_.getCurrentCmd(),
        desired_left_front_vel, actual_front_left_vel, left_front_wheel_vel_err, front_left_wheel_vel_pid_.getCurrentCmd(),
        desired_right_front_vel, actual_front_right_vel, right_front_wheel_vel_err, front_right_wheel_vel_pid_.getCurrentCmd(),
        desired_left_back_vel, actual_back_left_vel, left_back_wheel_vel_err, back_left_wheel_vel_pid_.getCurrentCmd(),
        desired_right_back_vel, actual_back_right_vel, right_back_wheel_vel_err, back_right_wheel_vel_pid_.getCurrentCmd());

}

void SentryChassisController::update(const ros::Time &time, const ros::Duration &period) {
    // 判断是否应用正弦波调试信号
    bool pivot_sine_applied = pivot_tuning_.applySinePivotTuning(
        time,
        desired_left_front_angle, desired_right_front_angle, desired_left_back_angle, desired_right_back_angle,
        desired_left_front_vel, desired_right_front_vel, desired_left_back_vel, desired_right_back_vel,
        last_cmd_vel_time_);
    
    bool wheel_sine_applied = pivot_tuning_.applySineWheelTuning(
        time,
        desired_left_front_angle, desired_right_front_angle, desired_left_back_angle, desired_right_back_angle,
        desired_left_front_vel, desired_right_front_vel, desired_left_back_vel, desired_right_back_vel,
        last_cmd_vel_time_);
    
    if (!pivot_sine_applied && !wheel_sine_applied) {
        // 如果启用正弦波调试，则关闭自锁功能，避免干扰
        applyTimeoutAndLock(time);
    }
    
    // 在得到期望角度和线速度后，计算每个轮子的期望转速和转向角，并通过PID控制器计算出每个关节的努力值
    computeWheelEfforts(time, period);
    odom_update(time, period);
}

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}
