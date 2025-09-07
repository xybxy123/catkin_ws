#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <chassis_controller/PIDConfig.h> 
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace controller_ns{

class ChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  ChassisController() = default;
  ~ChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void starting(const ros::Time& time) override ;

  void stopping(const ros::Time& time) override ;

  void reconfigureCallback(chassis_controller::PIDConfig &config, uint32_t level);
  dynamic_reconfigure::Server<chassis_controller::PIDConfig>::CallbackType f;
  dynamic_reconfigure::Server<chassis_controller::PIDConfig> server_;
  control_toolbox::Pid left_front_pid_, right_front_pid_, left_back_pid_, right_back_pid_;


private:
  hardware_interface::JointHandle left_front_wheel_, right_front_wheel_, left_back_wheel_, right_back_wheel_;
  ros::NodeHandle nh;

  // 订阅速度话题信息
  ros::Subscriber cmd_vel_sub;
  double linear_x_ = 0.0;
  double linear_y_ = 0.0;
  double angular_z_ = 0.0;
  void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);

  // 车辆属性
  double wheel_radius = 0.7625;
  double wheel_track = 0.4;
  double wheel_base = 0.4;

  //pid 设置
  ros::Time last_time_;
  double left_front_vel_ = 0.0, right_front_vel_ = 0.0, left_back_vel_ = 0.0, right_back_vel_ = 0.0;


  // 发布底盘实际速度
  ros::Publisher base_vel_pub_ = nh.advertise<geometry_msgs::Twist>("base_vel", 10);
  void computeBaseVelocity(const ros::Time& time) ;
  ros::Time last_publish_time_;

  ros::Publisher cmd_left_front_vel_pub_ = nh.advertise<std_msgs::Float64>("cmd/cmd_left_front_vel_pub_",10);
  ros::Publisher cmd_left_back_vel_pub_ = nh.advertise<std_msgs::Float64>("cmd/cmd_left_back_vel_pub_",10);
  ros::Publisher cmd_right_front_vel_pub_ = nh.advertise<std_msgs::Float64>("cmd/cmd_right_front_vel_pub_",10);
  ros::Publisher cmd_right_back_vel_pub_ = nh.advertise<std_msgs::Float64>("cmd/cmd_right_back_vel_pub_",10);

  ros::Publisher local_left_front_vel_pub_ = nh.advertise<std_msgs::Float64>("local/local_left_front_vel_pub_",10);
  ros::Publisher local_left_back_vel_pub_ = nh.advertise<std_msgs::Float64>("local/local_left_back_vel_pub_",10);
  ros::Publisher local_right_front_vel_pub_ = nh.advertise<std_msgs::Float64>("local/local_right_front_vel_pub_",10);
  ros::Publisher local_right_back_vel_pub_ = nh.advertise<std_msgs::Float64>("local/local_right_back_vel_pub_",10);

  std_msgs::Float64 cmd_left_front_effort;
  std_msgs::Float64 cmd_right_front_effort;
  std_msgs::Float64 cmd_left_back_effort;
  std_msgs::Float64 cmd_right_back_effort;

  std_msgs::Float64 local_current_left_front;
  std_msgs::Float64 local_current_right_front;
  std_msgs::Float64 local_current_left_back;
  std_msgs::Float64 local_current_right_back;

};

} // namespace controller_ns

#endif // CHASSIS_CONTROLLER_H
