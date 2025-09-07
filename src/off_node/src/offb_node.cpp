//首选
#include<ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

using namespace std;

mavros_msgs::State current_state_;
void state_cb_(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

geometry_msgs::PoseStamped curr_pose_;
void curr_pose_cb_(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pose_ = *msg;
}

enum State {
    TAKEOFF, 
    HOVER_1, 
    MOVE_FORWARD,
    POS_FORWARD,
    HOVER_2, 
    ORIENT_1,
    ORIENT_2,
    ORIENT_3,
    HOVER_3,
    MOVE_LEFT, 
    POS_LEFT,
    HOVER_4,
    MOVE_BACK, 
    POS_BACK,
    HOVER_5,
    MOVE_RIGHT,
    POS_RIGHT,
    HOVER_6,
    MOVE_DOWN,
    LANDING, 
    AUTOLAND,
    DONE,
    HOVER_7,
};

double target_tolerance_linear = 0.2;
double target_tolerance_angular = 0.05;

int main(int argc, char **argv){

    ros::init(argc,argv,"offboard_vel");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);  // Publish rate > 2Hz

    //连接上无人机
    ros::Subscriber state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb_);
    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected!");

    //发送初步目标位置  
    geometry_msgs::PoseStamped pose,yaw_pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.5;

    ros::Publisher pos_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    for(int i = 0;i<100;i++){
        pos_publisher.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //切换offboard模式的服务
    ros::ServiceClient set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    mavros_msgs::SetMode offboard_set_mode_;
    offboard_set_mode_.request.custom_mode = "OFFBOARD";

    // 解锁与取消解锁的服务
    ros::ServiceClient arm_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_;
    arm_.request.value = true;

    ros::ServiceClient disarm_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool disarm_;
    disarm_.request.value = false;

    //循环里需要的数据
    ros::Time last_request = ros::Time::now();
    State STATE = TAKEOFF;
    ros::Subscriber pose_subscriber = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose",10,curr_pose_cb_);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
	    ("/mavros/setpoint_velocity/cmd_vel_unstamped",10);//坐标系为local 上电点
    geometry_msgs::Twist vel_msg;//设置速度的消息
    ros::Time state_start_time = ros::Time::now();
    ros::Time hover_start_time;  // 悬停开始时间
    ros::Time vel_control_start_time;
    double hover_duration = 3.0;  // 悬停5秒
    while(ros::ok()){
        //切换offboard模式与解锁
        if(current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(set_mode_client_.call(offboard_set_mode_) && offboard_set_mode_.response.mode_sent){
                ROS_INFO("Offboard enabled!");
            }
            last_request = ros::Time::now();
        }
        else{
            if(!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(arm_client_.call(arm_) && arm_.response.success){
                    ROS_INFO("Vehicle armed!");
                }
                last_request = ros::Time::now();
            }
        }

        //进入状态机 通过速度控制发位置
        switch(STATE){
            case TAKEOFF:
                pos_publisher.publish(pose);
                // 检查是否到达目标高度（误差<0.1m）
                if(fabs(pose.pose.position.z - curr_pose_.pose.position.z) <= target_tolerance_linear){
                    ROS_INFO("Reached takeoff height, starting hover...");
                    STATE = HOVER_1;
                    hover_start_time = ros::Time::now();  // 记录悬停开始时间
                }
                break;

            case HOVER_1:
                pos_publisher.publish(pose);  // 保持目标位置
                // 悬停5秒
                if(ros::Time::now() - hover_start_time > ros::Duration(hover_duration)){
                    ROS_INFO("Hover complete, starting move forward...");
                    vel_msg.linear.x = 0.3;
                    vel_control_start_time = ros::Time::now();
                    // pose.pose.position.x = curr_pose_.pose.position.x + 1;
                    STATE = MOVE_FORWARD;
                }
                break;

            case MOVE_FORWARD:
                vel_pub.publish(vel_msg);
                if(ros::Time::now() - vel_control_start_time > ros::Duration(5.0)){
                    ROS_INFO("Reached forward position, starting hover...");
                    vel_msg.linear.x = 0;
                    vel_pub.publish(vel_msg);
                    pose = curr_pose_;
                    STATE = HOVER_2;
                    hover_start_time = ros::Time::now();  // 记录悬停开始时间
                }
                break;

            case HOVER_2:
                pos_publisher.publish(pose);  // 保持目标位置 
                // 悬停5秒
                if(ros::Time::now() - hover_start_time > ros::Duration(hover_duration)){
                    ROS_INFO("Hover complete, starting orient1...");
                    yaw_pose = curr_pose_;
                    pose = curr_pose_;
                    pose.pose.orientation.z -= 0.707;
                    yaw_pose.pose.orientation.z -= 0.235;
                    STATE = ORIENT_1;
                    hover_start_time = ros::Time::now();
                }
                break;

            case ORIENT_1:
                pos_publisher.publish(yaw_pose);
                if(fabs(yaw_pose.pose.orientation.z - curr_pose_.pose.orientation.z) <= target_tolerance_angular ){
                    ROS_INFO("Orient 2 start");
                    yaw_pose.pose.orientation.z -= 0.235;
                    STATE = ORIENT_2;
                    hover_start_time = ros::Time::now();
                }
                break;

            case ORIENT_2:
                pos_publisher.publish(yaw_pose);
                if(fabs(yaw_pose.pose.orientation.z - curr_pose_.pose.orientation.z) <= target_tolerance_angular ){
                    ROS_INFO("Orient 3 start");
                    yaw_pose.pose.orientation.z -= 0.237;
                    STATE = ORIENT_3;
                    hover_start_time = ros::Time::now();
                }

            case ORIENT_3:
                pos_publisher.publish(yaw_pose);
                if(fabs(yaw_pose.pose.orientation.z - curr_pose_.pose.orientation.z) <= target_tolerance_angular ){
                    ROS_INFO("Orient 3 end start hover");
                    yaw_pose.pose.orientation.z -= 0.237;
                    STATE = HOVER_7;
                    hover_start_time = ros::Time::now();
                }

            case HOVER_7:
                pos_publisher.publish(pose);
                if(ros::Time::now() - hover_start_time > ros::Duration(hover_duration)){
                    ROS_INFO("Hover complete, starting orient...");
                    pose = curr_pose_;
                    pose.pose.position.y = curr_pose_.pose.position.y + 1;
                    vel_msg.linear.y = 0.3;
                    STATE = MOVE_LEFT;
                }
                break;
                
            case MOVE_LEFT:
                vel_pub.publish(vel_msg);
                if(fabs(pose.pose.position.y - curr_pose_.pose.position.y) <= target_tolerance_linear){
                    ROS_INFO("Reached left position, starting hover...");
                    vel_msg.linear.y = 0;
                    vel_pub.publish(vel_msg);
                    STATE = HOVER_3;
                    hover_start_time = ros::Time::now();
                }
                break;

            case HOVER_3:
                pos_publisher.publish(pose);
                if(ros::Time::now() - hover_start_time > ros::Duration(hover_duration)){
                    ROS_INFO("Hover complete, starting move back...");
                    vel_msg.linear.x = -0.3;
                    pose = curr_pose_;
                    pose.pose.position.x = curr_pose_.pose.position.x - 1;
                    STATE = MOVE_BACK;
                }
                break;

            case MOVE_BACK:
                vel_pub.publish(vel_msg);
                if(fabs(pose.pose.position.x - curr_pose_.pose.position.x) <= target_tolerance_linear){
                    ROS_INFO("Reached back position, starting hover...");
                    vel_msg.linear.x = 0;
                    vel_pub.publish(vel_msg);
                    STATE = HOVER_4;
                    hover_start_time = ros::Time::now();
                }
                break;

            case HOVER_4:
                pos_publisher.publish(pose);
                if(ros::Time::now() - hover_start_time > ros::Duration(hover_duration)){
                    ROS_INFO("Hover complete, starting move right...");
                    vel_msg.linear.y = -0.3;
                    pose = curr_pose_;
                    pose.pose.position.y = curr_pose_.pose.position.y - 1;
                    STATE = MOVE_RIGHT;
                }
                break;

            case MOVE_RIGHT:
                vel_pub.publish(vel_msg);
                if(fabs(pose.pose.position.y - curr_pose_.pose.position.y) <= target_tolerance_linear){
                    ROS_INFO("Reached right position, starting hover...");
                    vel_msg.linear.y = 0;
                    vel_pub.publish(vel_msg);
                    STATE = HOVER_5;
                    hover_start_time = ros::Time::now();
                }
                break;

            case HOVER_5:
                pos_publisher.publish(pose);
                if(ros::Time::now() - hover_start_time > ros::Duration(hover_duration)){
                    ROS_INFO("Hover complete, starting move down...");
                    vel_msg.linear.z = -0.3;
                    pose.pose.position.z = 0.09;
                    STATE = MOVE_DOWN;
                }
                break;

            case MOVE_DOWN:
                vel_pub.publish(vel_msg);
                if(fabs(pose.pose.position.z - curr_pose_.pose.position.z) <= target_tolerance_linear){
                    ROS_INFO("Reached down position, starting hover...");
                    vel_msg.linear.z = 0;
                    vel_pub.publish(vel_msg);
                    STATE = HOVER_6;
                    hover_start_time = ros::Time::now();
                }
                break;

            case HOVER_6:
                pos_publisher.publish(pose);
                if(ros::Time::now() - hover_start_time > ros::Duration(hover_duration)){
                    ROS_INFO("Hover complete, starting landing...");
                    pose.pose.position.z = 0.01;
                    STATE = LANDING;
                }
                break;

            case LANDING:
                pos_publisher.publish(pose);
                if(fabs(pose.pose.position.z - curr_pose_.pose.position.z) <= target_tolerance_linear){
                    ROS_INFO("Reached land height, starting auto land...");
                    STATE = AUTOLAND;
                    state_start_time = ros::Time::now();
                }
                break;

            case AUTOLAND: {
                mavros_msgs::SetMode land_set_mode_;
                land_set_mode_.request.custom_mode = "AUTO.LAND";  
                bool mode_sent = set_mode_client_.call(land_set_mode_) && land_set_mode_.response.mode_sent;

                // 超时时间从进入AUTOLAND开始计算
                if ((mode_sent && current_state_.mode == "AUTO.LAND" && !current_state_.armed) ||
                    (ros::Time::now() - state_start_time > ros::Duration(0.01))) {
                    if (current_state_.armed) {
                        disarm_client_.call(disarm_);  // 上锁
                    }
                    ROS_INFO("Drone has landed");
                    ros::shutdown();
                }
                break;
            }

            default:
                ROS_ERROR("Unknown state");
                STATE = DONE;
                break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
