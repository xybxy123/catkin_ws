#include <ros/ros.h>
#include "chassis_controller/chassis_controller.h"

namespace controller_ns {

bool ChassisController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
    left_front_wheel_ = hw->getHandle("left_front_wheel_joint");
    right_front_wheel_ = hw->getHandle("right_front_wheel_joint");
    left_back_wheel_ = hw->getHandle("left_back_wheel_joint");
    right_back_wheel_ = hw->getHandle("right_back_wheel_joint");
    cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &ChassisController::cmd_vel_cb, this);
    
    left_front_pid_.init(ros::NodeHandle(nh,"pid_front_left_"));
    right_front_pid_.init(ros::NodeHandle(nh,"pid_front_right_"));
    left_back_pid_.init(ros::NodeHandle(nh,"pid_back_left_"));
    right_back_pid_.init(ros::NodeHandle(nh,"pid_back_right_"));
    server_.setCallback(boost::bind(&ChassisController::reconfigureCallback,this,_1,_2));
    return true;
}


void ChassisController::starting(const ros::Time& time) {
    // 重置PID控制器和时间
    left_front_pid_.reset();
    right_front_pid_.reset();
    left_back_pid_.reset();
    right_back_pid_.reset();
    
    last_time_ = time;
    last_publish_time_ = time;
    // 获取当前速度作为初始值
    left_front_vel_ = left_front_wheel_.getVelocity();
    right_front_vel_ = right_front_wheel_.getVelocity();
    left_back_vel_ = left_back_wheel_.getVelocity();
    right_back_vel_ = right_back_wheel_.getVelocity();
}


void ChassisController::update(const ros::Time& time, const ros::Duration& period) {
    // 计算时间间隔
    ros::Duration dt = time - last_time_;
    last_time_ = time;
    
    double R = wheel_radius;
    double L = wheel_track;
    double W = wheel_base;
    
    // 计算四个轮子的目标速度 (rad/s)
    double front_left_target = (linear_x_ - linear_y_ - angular_z_ * (L/2 + W/2)) / R;
    double front_right_target = (linear_x_ + linear_y_ + angular_z_ * (L/2 + W/2)) / R;
    double back_left_target = (linear_x_ + linear_y_ - angular_z_ * (L/2 + W/2)) / R;
    double back_right_target = (linear_x_ - linear_y_ + angular_z_ * (L/2 + W/2)) / R;
    
    // 获取当前速度
    double current_left_front = left_front_wheel_.getVelocity();
    double current_right_front = right_front_wheel_.getVelocity();
    double current_left_back = left_back_wheel_.getVelocity();
    double current_right_back = right_back_wheel_.getVelocity();
    
    // 使用PID控制器计算控制量
    double left_front_effort = left_front_pid_.computeCommand(
        front_left_target - current_left_front, dt);
        
    double right_front_effort = right_front_pid_.computeCommand(
        front_right_target - current_right_front, dt);
        
    double left_back_effort = left_back_pid_.computeCommand(
        back_left_target - current_left_back, dt);
        
    double right_back_effort = right_back_pid_.computeCommand(
        back_right_target - current_right_back, dt);

    cmd_left_front_effort.data = front_left_target;
    cmd_right_front_effort.data = front_right_target;
    cmd_left_back_effort.data = back_left_target;
    cmd_right_back_effort.data = back_right_target;

    local_current_left_front.data = current_left_front;
    local_current_right_front.data = current_right_front;
    local_current_left_back.data = current_left_back;
    local_current_right_back.data = current_right_back;

    // 设置四个轮子的力矩命令
    cmd_left_front_vel_pub_.publish(cmd_left_front_effort);
    cmd_right_front_vel_pub_.publish(cmd_right_front_effort);
    cmd_left_back_vel_pub_.publish(cmd_left_back_effort);
    cmd_right_back_vel_pub_.publish(cmd_right_back_effort);



    local_left_front_vel_pub_.publish(local_current_left_front);
    local_right_front_vel_pub_.publish(local_current_right_front);
    local_left_back_vel_pub_.publish(local_current_left_back);
    local_right_back_vel_pub_.publish(local_current_right_back);


    left_front_wheel_.setCommand(left_front_effort);
    right_front_wheel_.setCommand(right_front_effort);
    left_back_wheel_.setCommand(left_back_effort);
    right_back_wheel_.setCommand(right_back_effort);
    
    // 保存当前速度
    left_front_vel_ = current_left_front;
    right_front_vel_ = current_right_front;
    left_back_vel_ = current_left_back;
    right_back_vel_ = current_right_back;

    computeBaseVelocity(time);
}

void ChassisController::stopping(const ros::Time& time) {
    // 停止时重置PID控制器
    left_front_pid_.reset();
    right_front_pid_.reset();
    left_back_pid_.reset();
    right_back_pid_.reset();
}


void ChassisController::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    linear_x_ = msg->linear.x;
    linear_y_ = msg->linear.y;
    angular_z_ = msg->angular.z;
}

void ChassisController::computeBaseVelocity(const ros::Time& time) {
    // 限制发布频率
    if ((time - last_publish_time_).toSec() < 0.05) {  // 20Hz
        return;
    }
    last_publish_time_ = time;
    
    double R = wheel_radius;
    
    double w1 = left_front_vel_;    
    double w2 = right_front_vel_;  
    double w3 = left_back_vel_;     
    double w4 = right_back_vel_;   
    
    // 计算底盘速度
    geometry_msgs::Twist base_vel;
    base_vel.linear.x = R * (w1 + w2 + w3 + w4) / 4.0;
    base_vel.linear.y = R * (-w1 + w2 + w3 - w4) / 4.0;
    base_vel.angular.z = R * (-w1 + w2 - w3 + w4) / 2 / ( 0.4 + 0.4 );
    
    // 发布速度
    base_vel_pub_.publish(base_vel);
}
void ChassisController::reconfigureCallback(chassis_controller::PIDConfig &config, uint32_t level)
{
    left_front_pid_.setGains(config.pid_front_left_p,config.pid_front_left_i,config.pid_front_left_d,config.pid_i_max,config.pid_i_min,config.pid_antiwindup);
    right_front_pid_.setGains(config.pid_front_right_p,config.pid_front_right_i,config.pid_front_right_d,config.pid_i_max,config.pid_i_min,config.pid_antiwindup);
    left_back_pid_.setGains(config.pid_back_left_p,config.pid_back_left_i,config.pid_back_left_d,config.pid_i_max,config.pid_i_min,config.pid_antiwindup);
    right_back_pid_.setGains(config.pid_back_right_p,config.pid_back_right_i,config.pid_back_right_d,config.pid_i_max,config.pid_i_min,config.pid_antiwindup);
}

} // namespace controller_ns

PLUGINLIB_EXPORT_CLASS(controller_ns::ChassisController, controller_interface::ControllerBase)
