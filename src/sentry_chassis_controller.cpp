//
// Created by cqincat on 2/6/21.
//

#include <hardware_interface/joint_command_interface.h>
#include <sentry_chassis_controller/sentry_chassis_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace sentry_chassis_controller {
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
    front_left_pivot_joint_.setCommand(0.);
    front_right_pivot_joint_.setCommand(0.);
    back_left_pivot_joint_.setCommand(0.);
    back_right_pivot_joint_.setCommand(0.);

    if (!odomMode) {
        Vec2<double> vel_center(msg->linear.x, msg->linear.y);
        std::vector<Vec2<double>> module_positions = {
            Vec2<double>(rx, ry),    // left_front
            Vec2<double>(rx, -ry),   // right_front
            Vec2<double>(-rx, ry),   // left_back
            Vec2<double>(-rx, -ry)   // right_back
        };
//
//        Vec2<double> left_front_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
//                           msg->angular.z * Vec2<double>(-module_positions[0].y(), module_positions[1].x());
//        Vec2<double> right_front_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
//                           msg->angular.z * Vec2<double>(-module_positions[1].y(), module_positions[1].x());
//        Vec2<double> left_back_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
//                           msg->angular.z * Vec2<double>(-module_positions[2].y(), module_positions[2].x());
//        Vec2<double> right_back_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
//                           msg->angular.z * Vec2<double>(-module_positions[3].y(), module_positions[3].x());
//        Vec2<double> vel = vel_center + msg->angular.z * Vec2<double>(-ry, rx);
        Vec2<double> left_front_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
                                      msg->angular.z * Vec2<double>(-module_positions[0].y(), module_positions[0].x());
        Vec2<double> right_front_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
                                       msg->angular.z * Vec2<double>(-module_positions[1].y(), module_positions[1].x());
        Vec2<double> left_back_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
                                     msg->angular.z * Vec2<double>(-module_positions[2].y(), module_positions[2].x());
        Vec2<double> right_back_vel = Vec2<double>(msg->linear.x, msg->linear.y) +
                                      msg->angular.z * Vec2<double>(-module_positions[3].y(), module_positions[3].x());

        double left_front_vel_angle = std::atan2(left_front_vel.y(), left_front_vel.x()) + left_front_pivot_offset_;
        double right_front_vel_angle = std::atan2(right_front_vel.y(), right_front_vel.x()) + right_front_pivot_offset_;
        double left_back_vel_angle = std::atan2(left_back_vel.y(), left_back_vel.x()) + left_back_pivot_offset_;
        double right_back_vel_angle = std::atan2(right_back_vel.y(), right_back_vel.x()) + right_back_pivot_offset_;
        ROS_INFO("lf_vel_angle = %.2f",left_front_vel_angle);
        ROS_INFO("rf_vel_angle = %.2f",right_front_vel_angle);
        ROS_INFO("lb_vel_angle = %.2f",left_back_vel_angle);
        ROS_INFO("rb_vel_angle = %.2f",right_back_vel_angle);

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

//        desired_left_front_vel = left_front_vel.norm() * std::cos(a);
//        desired_right_front_vel = right_front_vel.norm() * std::cos(c);
//        desired_left_back_vel = left_back_vel.norm() * std::cos(e);
//        desired_right_back_vel = right_back_vel.norm()  * std::cos(g);

        desired_left_front_vel = (left_front_vel.x() * cos(desired_left_front_angle) +
                                  left_front_vel.y() * sin(desired_left_front_angle)) / wheel_radius_;
        desired_right_front_vel = (right_front_vel.x() * cos(desired_right_front_angle) +
                                   right_front_vel.y() * sin(desired_right_front_angle)) / wheel_radius_;
        desired_left_back_vel = (left_back_vel.x() * cos(desired_left_back_angle) +
                                 left_back_vel.y() * sin(desired_left_back_angle)) / wheel_radius_;
        desired_right_back_vel = (right_back_vel.x() * cos(desired_right_back_angle) +
                                  right_back_vel.y() * sin(desired_right_back_angle)) / wheel_radius_;

    } else{
//        geometry_msgs::Vector3Stamped world_vel;
//        world_vel.header.stamp = ros::Time(0);
//        world_vel.header.frame_id = "odom";
//        world_vel.vector = msg->linear;
//
//        geometry_msgs::Vector3Stamped base_vel;
//        try {
//            tf_listener_.waitForTransform("base_link","odom",ros::Time(0),ros::Duration(1.0));
//            tf_listener_.transformVector("base_link", world_vel, base_vel); // 使用成员 tf listener
//        } catch (tf::TransformException& ex) {
//            ROS_WARN("tf error: %s", ex.what());
//            return;
//        }
//        desired_front_left_velocity_  = base_vel.vector.x - base_vel.vector.y - (rx + ry) * msg->angular.z;
//        desired_front_right_velocity_  = base_vel.vector.x + base_vel.vector.y + (rx + ry) * msg->angular.z;
//        desired_back_left_velocity_  = base_vel.vector.x + base_vel.vector.y - (rx + ry) * msg->angular.z;
//        desired_back_right_velocity_  = base_vel.vector.x - base_vel.vector.y + (rx + ry) * msg->angular.z;
    }
}

void SentryChassisController::computeWheelEfforts(const ros::Time& time, const ros::Duration& period)
{
    double actual_front_left_vel = front_left_wheel_joint_.getVelocity();
    double actual_front_right_vel = front_right_wheel_joint_.getVelocity();
    double actual_back_left_vel = back_left_wheel_joint_.getVelocity();
    double actual_back_right_vel = back_right_wheel_joint_.getVelocity();
    double actual_front_left_pos = front_left_pivot_joint_.getPosition();
    double actual_front_right_pos = front_right_pivot_joint_.getPosition();
    double actual_back_left_pos = back_left_pivot_joint_.getPosition();
    double actual_back_right_pos = back_right_pivot_joint_.getPosition();

    double error_front_left_vel =desired_left_front_vel - wheel_radius_ * actual_front_left_vel;
    double error_front_right_vel =desired_right_front_vel - wheel_radius_ * actual_front_right_vel;
    double error_back_left_vel =desired_left_back_vel - wheel_radius_ * actual_back_left_vel;
    double error_back_right_vel =desired_right_back_vel - wheel_radius_ * actual_back_right_vel;
    double error_front_left_angle = desired_left_front_angle - actual_front_left_pos;
    double error_front_right_angle = desired_right_front_angle - actual_front_right_pos;
    double error_back_left_angle = desired_left_back_angle - actual_back_left_pos;
    double error_back_right_angle = desired_right_back_angle - actual_back_right_pos;
    if (lf_error_vel_pub) {
        std_msgs::Float64 m;
        m.data = error_front_left_vel;
        lf_error_vel_pub.publish(m);
    }
    if (rf_error_vel_pub) {
        std_msgs::Float64 m;
        m.data = error_front_right_vel;
        rf_error_vel_pub.publish(m);
    }
    if (lb_error_vel_pub) {
        std_msgs::Float64 m;
        m.data = error_back_left_vel;
        lb_error_vel_pub.publish(m);
    }
    if (rb_error_vel_pub) {
        std_msgs::Float64 m;
        m.data = error_back_right_vel;
        rb_error_vel_pub.publish(m);
    }

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

//  wheel_track_ = controller_nh.param("wheel_track", 0.362);
//  wheel_base_ = controller_nh.param("wheel_base", 0.362);
  controller_nh.getParam("wheel_track", wheel_track_);
  controller_nh.getParam("wheel_base", wheel_base_);
  controller_nh.getParam("wheel_radius", wheel_radius_);
  controller_nh.getParam("odomMode", odomMode);
  controller_nh.getParam("left_front_pivot_offset", left_front_pivot_offset_);
  controller_nh.getParam("right_front_pivot_offset", right_front_pivot_offset_);
  controller_nh.getParam("left_back_pivot_offset", left_back_pivot_offset_);
  controller_nh.getParam("right_back_pivot_offset", right_back_pivot_offset_);

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

  lf_error_vel_pub = root_nh.advertise<std_msgs::Float64>("/error_vel/lf", 10);
  rf_error_vel_pub = root_nh.advertise<std_msgs::Float64>("/error_vel/rf", 10);
  lb_error_vel_pub = root_nh.advertise<std_msgs::Float64>("/error_vel/lb", 10);
  rb_error_vel_pub = root_nh.advertise<std_msgs::Float64>("/error_vel/rb", 10);
  //dynamic reconfigure
  server = std::make_shared<dynamic_reconfigure::Server<sentry_chassis_controller::PIDConfig>>(controller_nh);
  cbType = boost::bind(&SentryChassisController::cb,this,_1,_2);
  server->setCallback(cbType);



  return true;
}

void SentryChassisController::update(const ros::Time &time, const ros::Duration &period) {
    computeWheelEfforts(time, period);
}

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}
