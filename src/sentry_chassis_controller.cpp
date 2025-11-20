//
// Created by cqincat on 2/6/21.
//

#include <hardware_interface/joint_command_interface.h>
#include <sentry_chassis_controller/sentry_chassis_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace sentry_chassis_controller {
//SentryChassisController::SentryChassisController() :
//    tf_buffer_(ros::Duration(10.0)),      // 初始化缓冲区（10秒缓存）
//    tf_listener_(tf_buffer_)              // 初始化监听器，传入缓冲区引用
//{
//}

void SentryChassisController::cb(const PIDConfig& config, uint32_t level)
{
    pid_lf_.setGains(config.front_left_p, config.front_left_i, config.front_left_d, 0, 0);
    pid_rf_.setGains(config.front_right_p, config.front_right_i, config.front_right_d, 0, 0);
    pid_lb_.setGains(config.back_left_p, config.back_left_i, config.back_left_d, 0, 0);
    pid_rb_.setGains(config.back_right_p, config.back_right_i, config.back_right_d, 0, 0);
    pid_lf_wheel_.setGains(config.front_left_wheel_p, config.front_left_wheel_i, config.front_left_wheel_d, 0, 0);
    pid_rf_wheel_.setGains(config.front_right_wheel_p, config.front_right_wheel_i, config.front_right_wheel_d, 0, 0);
    pid_lb_wheel_.setGains(config.back_left_wheel_p, config.back_left_wheel_i, config.back_left_wheel_d, 0, 0);
    pid_rb_wheel_.setGains(config.back_right_wheel_p, config.back_right_wheel_i, config.back_right_wheel_d, 0, 0);
    ROS_INFO("Update PID gains:");
    ROS_INFO("Front Left Pivot: P=%.2f, I=%.2f, D=%.2f", config.front_left_p, config.front_left_i, config.front_left_d);
    ROS_INFO("Front Right Pivot: P=%.2f, I=%.2f, D=%.2f", config.front_right_p, config.front_right_i, config.front_right_d);
    ROS_INFO("Back Left Pivot: P=%.2f, I=%.2f, D=%.2f", config.back_left_p, config.back_left_i, config.back_left_d);
    ROS_INFO("Back Right Pivot: P=%.2f, I=%.2f, D=%.2f", config.back_right_p, config.back_right_i, config.back_right_d);
    ROS_INFO("Front Left Wheel: P=%.2f, I=%.2f, D=%.2f", config.front_left_wheel_p, config.front_left_wheel_i, config.front_left_wheel_d);
    ROS_INFO("Front Right Wheel: P=%.2f, I=%.2f, D=%.2f", config.front_right_wheel_p, config.front_right_wheel_i, config.front_right_wheel_d);
    ROS_INFO("Back Left Wheel: P=%.2f, I=%.2f, D=%.2f", config.back_left_wheel_p, config.back_left_wheel_i, config.back_left_wheel_d);
    ROS_INFO("Back Right Wheel: P=%.2f, I=%.2f, D=%.2f", config.back_right_wheel_p, config.back_right_wheel_i, config.back_right_wheel_d);
}

//Inverse kinematics solution
void SentryChassisController::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    Vec2<double> left_front_vel, right_front_vel, left_back_vel, right_back_vel;

    std::vector<Vec2<double>> module_positions = {
        Vec2<double>(rx, ry),    // left_front
        Vec2<double>(rx, -ry),   // right_front
        Vec2<double>(-rx, ry),   // left_back
        Vec2<double>(-rx, -ry)   // right_back
    };

    if (!odomMode) {
        left_front_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
                                      msg->angular.z * Vec2<double>(-module_positions[0].y(), module_positions[0].x());
        right_front_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
                                       msg->angular.z * Vec2<double>(-module_positions[1].y(), module_positions[1].x());
        left_back_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
                                     msg->angular.z * Vec2<double>(-module_positions[2].y(), module_positions[2].x());
        right_back_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
                                      msg->angular.z * Vec2<double>(-module_positions[3].y(), module_positions[3].x());
    } else{
        geometry_msgs::Vector3Stamped world_vel;
        world_vel.header.stamp = ros::Time::now();
        world_vel.header.frame_id = "odom";
        world_vel.vector = msg->linear;

        geometry_msgs::Vector3Stamped base_vel;
        try {
            tf_listener_.waitForTransform("base_link","odom",ros::Time::now(),ros::Duration(1.0));
            tf_listener_.transformVector("base_link", world_vel, base_vel); // 使用成员 tf listener
        } catch (tf::TransformException& ex) {
            ROS_WARN("tf error: %s", ex.what());
            return;
        }

//        geometry_msgs::TwistStamped world_vel, base_vel;
//        world_vel.header.frame_id = "odom";
//        world_vel.header.stamp = ros::Time(0);
//        world_vel.twist.linear = msg->linear;
//        world_vel.twist.angular = msg->angular;
//        try
//        {
//            auto transform = buffer.lookupTransform("base_link", "odom", ros::Time(0), ros::Duration(0.5));
//            tf2::doTransform(world_vel, base_vel, transform);
//        }
//        catch (tf2::TransformException& ex)
//        {
//            ROS_WARN("%s", ex.what());
//            return;
//        }
        left_front_vel = Vec2<double>(base_vel.vector.x, base_vel.vector.y) +
                                      msg->angular.z * Vec2<double>(-module_positions[0].y(), module_positions[0].x());
        right_front_vel = Vec2<double>(base_vel.vector.x, base_vel.vector.y) +
                                       msg->angular.z * Vec2<double>(-module_positions[1].y(), module_positions[1].x());
        left_back_vel = Vec2<double>(base_vel.vector.x, base_vel.vector.y) +
                                     msg->angular.z * Vec2<double>(-module_positions[2].y(), module_positions[2].x());
        right_back_vel = Vec2<double>(base_vel.vector.x, base_vel.vector.y) +
                                      msg->angular.z * Vec2<double>(-module_positions[3].y(), module_positions[3].x());
//        left_front_vel = Vec2<double>(base_vel.twist.linear.x, base_vel.twist.linear.y) +
//                                        base_vel.twist.angular.z * Vec2<double>(-module_positions[0].y(), module_positions[0].x());
//        right_front_vel = Vec2<double>(base_vel.twist.linear.x, base_vel.twist.linear.y) +
//                                        base_vel.twist.angular.z * Vec2<double>(-module_positions[1].y(), module_positions[1].x());
//        left_back_vel = Vec2<double>(base_vel.twist.linear.x, base_vel.twist.linear.y) +
//                                        base_vel.twist.angular.z * Vec2<double>(-module_positions[2].y(), module_positions[2].x());
//        right_back_vel = Vec2<double>(base_vel.twist.linear.x, base_vel.twist.linear.y) +
//                                        base_vel.twist.angular.z * Vec2<double>(-module_positions[3].y(), module_positions[3].x());
    }

    double left_front_vel_angle = std::atan2(left_front_vel.y(), left_front_vel.x());
    double right_front_vel_angle = std::atan2(right_front_vel.y(), right_front_vel.x());
    double left_back_vel_angle = std::atan2(left_back_vel.y(), left_back_vel.x());
    double right_back_vel_angle = std::atan2(right_back_vel.y(), right_back_vel.x());

    double a = angles::shortest_angular_distance(front_left_pivot_joint_.getPosition(), left_front_vel_angle);
//    double b = angles::shortest_angular_distance(front_left_pivot_joint_.getPosition(), left_front_vel_angle + M_PI);
//    desired_left_front_angle = std::abs(a) < std::abs(b) ? left_front_vel_angle : left_front_vel_angle + M_PI;
    desired_left_front_angle = left_front_vel_angle;

    double c = angles::shortest_angular_distance(front_right_pivot_joint_.getPosition(), right_front_vel_angle);
//    double d = angles::shortest_angular_distance(front_right_pivot_joint_.getPosition(), right_front_vel_angle + M_PI);
//    desired_right_front_angle = std::abs(c) < std::abs(d) ? right_front_vel_angle : right_front_vel_angle + M_PI;
    desired_right_front_angle = right_front_vel_angle;

    double e = angles::shortest_angular_distance(back_left_pivot_joint_.getPosition(), left_back_vel_angle);
//    double f = angles::shortest_angular_distance(back_left_pivot_joint_.getPosition(), left_back_vel_angle + M_PI);
//    desired_left_back_angle = std::abs(e) < std::abs(f) ? left_back_vel_angle : left_back_vel_angle + M_PI;
    desired_left_back_angle = left_back_vel_angle;

    double g = angles::shortest_angular_distance(back_right_pivot_joint_.getPosition(), right_back_vel_angle);
//    double h = angles::shortest_angular_distance(back_right_pivot_joint_.getPosition(), right_back_vel_angle + M_PI);
//    desired_right_back_angle = std::abs(g) < std::abs(h) ? right_back_vel_angle : right_back_vel_angle + M_PI;
    desired_right_back_angle = right_back_vel_angle;

    desired_left_front_vel = left_front_vel.norm() * std::cos(a) / wheel_radius_;
    desired_right_front_vel = right_front_vel.norm() * std::cos(c) / wheel_radius_;
    desired_left_back_vel = left_back_vel.norm() * std::cos(e) / wheel_radius_;
    desired_right_back_vel = right_back_vel.norm()  * std::cos(g) / wheel_radius_;

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

//    ROS_INFO("desired front left angle: %.2f", desired_left_front_angle);
//    ROS_INFO("desired front right angle: %.2f", desired_right_front_angle);
//    ROS_INFO("desired back left angle: %.2f", desired_left_back_angle);
//    ROS_INFO("desired back right angle: %.2f", desired_right_back_angle);
//    ROS_INFO("actual front left angle: %.2f", actual_front_left_pos);
//    ROS_INFO("actual front right angle: %.2f", actual_front_right_pos);
//    ROS_INFO("actual back left angle: %.2f", actual_back_left_pos);
//    ROS_INFO("actual back right angle: %.2f", actual_back_right_pos);

    double error_front_left_vel =desired_left_front_vel -  actual_front_left_vel;
    double error_front_right_vel =desired_right_front_vel -  actual_front_right_vel;
    double error_back_left_vel =desired_left_back_vel -  actual_back_left_vel;
    double error_back_right_vel =desired_right_back_vel -  actual_back_right_vel;
    double error_front_left_angle = desired_left_front_angle - actual_front_left_pos;
    double error_front_right_angle = desired_right_front_angle - actual_front_right_pos;
    double error_back_left_angle = desired_left_back_angle - actual_back_left_pos;
    double error_back_right_angle = desired_right_back_angle - actual_back_right_pos;

    double cmd_effort_front_left = pid_lf_wheel_.computeCommand(error_front_left_vel, period);
    double cmd_effort_front_right = pid_rf_wheel_.computeCommand(error_front_right_vel, period);
    double cmd_effort_back_left = pid_lb_wheel_.computeCommand(error_back_left_vel, period);
    double cmd_effort_back_right = pid_rb_wheel_.computeCommand(error_back_right_vel, period);

    double cmd_pos_front_left = pid_lf_.computeCommand(error_front_left_angle, period);
    double cmd_pos_front_right = pid_rf_.computeCommand(error_front_right_angle, period);
    double cmd_pos_back_left = pid_lb_.computeCommand(error_back_left_angle, period);
    double cmd_pos_back_right = pid_rb_.computeCommand(error_back_right_angle, period);
    double cmd_vel_front_left = pid_lf_wheel_.computeCommand(cmd_pos_front_left, period);
    double cmd_vel_front_right = pid_rf_wheel_.computeCommand(cmd_pos_front_right, period);
    double cmd_vel_back_left = pid_lb_wheel_.computeCommand(cmd_pos_back_left, period);
    double cmd_vel_back_right = pid_rb_wheel_.computeCommand(cmd_pos_back_right, period);

    front_left_pivot_joint_.setCommand(cmd_vel_front_left);
    front_right_pivot_joint_.setCommand(cmd_vel_front_right);
    back_left_pivot_joint_.setCommand(cmd_vel_back_left);
    back_right_pivot_joint_.setCommand(cmd_vel_back_right);
    front_left_wheel_joint_.setCommand(cmd_effort_front_left);
    front_right_wheel_joint_.setCommand(cmd_effort_front_right);
    back_left_wheel_joint_.setCommand(cmd_effort_back_left);
    back_right_wheel_joint_.setCommand(cmd_effort_back_right);


//    double a = 0., b = 0., c = 0.;
//    a = square(cmd_effort_front_left_)+square(cmd_effort_front_right_)+square(cmd_effort_back_left_)+square(cmd_effort_back_right_);
//    b = std::abs(cmd_effort_front_left_ * actual_front_left_velocity_)+std::abs(cmd_effort_front_right_ * actual_front_right_velocity_)+
//        std::abs(cmd_effort_back_left_ * actual_back_left_velocity_)+std::abs(cmd_effort_back_right_ * actual_back_right_velocity_);
//    c = square(actual_front_left_velocity_)+square(actual_front_right_velocity_)+
//        square(actual_back_left_velocity_)+square(actual_back_right_velocity_);
//    a *= effort_coeff_;
//    c = c * velocity_coeff_ - power_offset_ - power_limit;
//    // Root formula for quadratic equation in one variable
//    double zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
//    for (auto joint : joint_handles_)
//        joint.setCommand(zoom_coeff > 1 ? joint.getCommand() : joint.getCommand() * zoom_coeff);
}
//Forward kinematics calculation
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
        // 里程计累加
        x += dx;
        y += dy;
        th += dth;
        while (th > M_PI) th -= 2.0 * M_PI;
        while (th < -M_PI) th += 2.0 * M_PI;

        // 创建坐标转换
        geometry_msgs::TransformStamped odom2base_link;
        //----设置头信息
        //odom_ts.header.seq = 100;
        odom2base_link.header.stamp = time;
        odom2base_link.header.frame_id = "odom";
        //----设置子级坐标系
        odom2base_link.child_frame_id = "base_link";
        //----设置子级相对于父级的偏移量
        odom2base_link.transform.translation.x = x;
        odom2base_link.transform.translation.y = y;
        odom2base_link.transform.translation.z = 0.0;
        //----设置四元数:将 欧拉角数据转换成四元数
        tf2::Quaternion qtn;
        qtn.setRPY(0,0,th);
        odom2base_link.transform.rotation.x = qtn.getX();
        odom2base_link.transform.rotation.y = qtn.getY();
        odom2base_link.transform.rotation.z = qtn.getZ();
        odom2base_link.transform.rotation.w = qtn.getW();

        odom_broadcaster.sendTransform(odom2base_link);

        // 发布 Odometry 消息
        nav_msgs::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = x_velocity;
        odom.twist.twist.linear.y = y_velocity;
        odom.twist.twist.angular.z = th_velocity;

        odom_pub.publish(odom);
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
  controller_nh.getParam("wheel_track", wheel_track_);
  controller_nh.getParam("wheel_base", wheel_base_);
  controller_nh.getParam("wheel_radius", wheel_radius_);
  controller_nh.getParam("odomMode", odomMode);
//  controller_nh.getParam("left_front_pivot_offset", left_front_pivot_offset_);
//  controller_nh.getParam("right_front_pivot_offset", right_front_pivot_offset_);
//  controller_nh.getParam("left_back_pivot_offset", left_back_pivot_offset_);
//  controller_nh.getParam("right_back_pivot_offset", right_back_pivot_offset_);

  rx = wheel_base_ / 2.0;
  ry = wheel_track_ / 2.0;

  pid_lf_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
  pid_rf_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
  pid_lb_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
  pid_rb_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);

  // PID for wheel velocity control
  pid_lf_wheel_.initPid(1.0, 0.1, 0.0, 0.0, 0.0);
  pid_rf_wheel_.initPid(1.0, 0.1, 0.0, 0.0, 0.0);
  pid_lb_wheel_.initPid(1.0, 0.1, 0.0, 0.0, 0.0);
  pid_rb_wheel_.initPid(1.0, 0.1, 0.0, 0.0, 0.0);

  //subscribe the velocity command
  cmd_vel_sub = root_nh.subscribe("/cmd_vel", 1, &SentryChassisController::cmd_vel_cb, this);
  odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 10);

  //dynamic reconfigure
  server = std::make_shared<dynamic_reconfigure::Server<sentry_chassis_controller::PIDConfig>>(controller_nh);
  cbType = boost::bind(&SentryChassisController::cb,this,_1,_2);
  server->setCallback(cbType);

//  listener = std::make_shared<tf2_ros::TransformListener>(buffer);


  return true;
}

void SentryChassisController::update(const ros::Time &time, const ros::Duration &period) {
    computeWheelEfforts(time, period);
    odom_update(time, period);
}

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}
