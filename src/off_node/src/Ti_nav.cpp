#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <string>

using namespace std;
    geometry_msgs::Twist vel_msg;  // 设置速度的消息
    ros::Publisher pos_publisher ;
    geometry_msgs::PoseStamped pose, yaw_pose;

double t_ = 0.3;

mavros_msgs::State current_state_;
void state_cb_(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

geometry_msgs::PoseStamped curr_pose_;
void curr_pose_cb_(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    curr_pose_ = *msg;
}

// 状态常量定义（添加引号并修正命名格式）
string State[10][8] = {
    {"STATE00", "STATE01", "STATE02", "STATE03", "STATE04", "STATE05", "STATE06", "STATE07"},
    {"STATE10", "STATE11", "STATE12", "STATE13", "STATE14", "STATE15", "STATE16", "STATE17"},
    {"STATE20", "STATE21", "STATE22", "STATE23", "STATE24", "STATE25", "STATE26", "STATE27"},
    {"STATE30", "STATE31", "STATE32", "STATE33", "STATE34", "STATE35", "STATE36", "STATE37"},
    {"STATE40", "STATE41", "STATE42", "STATE43", "STATE44", "STATE45", "STATE46", "STATE47"},
    {"STATE50", "STATE51", "STATE52", "STATE53", "STATE54", "STATE55", "STATE56", "STATE57"},
    {"STATE60", "STATE61", "STATE62", "STATE63", "STATE64", "STATE65", "STATE66", "STATE67"},
    {"STATE70", "STATE71", "STATE72", "STATE73", "STATE74", "STATE75", "STATE76", "STATE77"},
    {"STATE80", "STATE81", "STATE82", "STATE83", "STATE84", "STATE85", "STATE86", "STATE87"},
    {"STATE90", "STATE91", "STATE92", "STATE93", "STATE94", "STATE95", "STATE96", "STATE97"}
};

string Move[10][8] = {
    {"MOVE00", "MOVE01", "MOVE02", "MOVE03", "MOVE04", "MOVE05", "MOVE06", "MOVE07"},
    {"MOVE10", "MOVE11", "MOVE12", "MOVE13", "MOVE14", "MOVE15", "MOVE16", "MOVE17"},
    {"MOVE20", "MOVE21", "MOVE22", "MOVE23", "MOVE24", "MOVE25", "MOVE26", "MOVE27"},
    {"MOVE30", "MOVE31", "MOVE32", "MOVE33", "MOVE34", "MOVE35", "MOVE36", "MOVE37"},
    {"MOVE40", "MOVE41", "MOVE42", "MOVE43", "MOVE44", "MOVE45", "MOVE46", "MOVE47"},
    {"MOVE50", "MOVE51", "MOVE52", "MOVE53", "MOVE54", "MOVE55", "MOVE56", "MOVE57"},
    {"MOVE60", "MOVE61", "MOVE62", "MOVE63", "MOVE64", "MOVE65", "MOVE66", "MOVE67"},
    {"MOVE70", "MOVE71", "MOVE72", "MOVE73", "MOVE74", "MOVE75", "MOVE76", "MOVE77"},
    {"MOVE80", "MOVE81", "MOVE82", "MOVE83", "MOVE84", "MOVE85", "MOVE86", "MOVE87"},
    {"MOVE90", "MOVE91", "MOVE92", "MOVE93", "MOVE94", "MOVE95", "MOVE96", "MOVE97"}
};

string Hover[10][8] = {
    {"HOVER00", "HOVER01", "HOVER02", "HOVER03", "HOVER04", "HOVER05", "HOVER06", "HOVER07"},
    {"HOVER10", "HOVER11", "HOVER12", "HOVER13", "HOVER14", "HOVER15", "HOVER16", "HOVER17"},
    {"HOVER20", "HOVER21", "HOVER22", "HOVER23", "HOVER24", "HOVER25", "HOVER26", "HOVER27"},
    {"HOVER30", "HOVER31", "HOVER32", "HOVER33", "HOVER34", "HOVER35", "HOVER36", "HOVER37"},
    {"HOVER40", "HOVER41", "HOVER42", "HOVER43", "HOVER44", "HOVER45", "HOVER46", "HOVER47"},
    {"HOVER50", "HOVER51", "HOVER52", "HOVER53", "HOVER54", "HOVER55", "HOVER56", "HOVER57"},
    {"HOVER60", "HOVER61", "HOVER62", "HOVER63", "HOVER64", "HOVER65", "HOVER66", "HOVER67"},
    {"HOVER70", "HOVER71", "HOVER72", "HOVER73", "HOVER74", "HOVER75", "HOVER76", "HOVER77"},
    {"HOVER80", "HOVER81", "HOVER82", "HOVER83", "HOVER84", "HOVER85", "HOVER86", "HOVER87"},
    {"HOVER90", "HOVER91", "HOVER92", "HOVER93", "HOVER94", "HOVER95", "HOVER96", "HOVER97"}
};

// 补充缺失的状态常量定义
const string MOVE_DOWN = "MOVE_DOWN";
const string MOVE_DOWN_1 = "MOVE_DOWN_1";
const string HOVER_6 = "HOVER_6";
const string LANDING = "LANDING";
const string AUTOLAND = "AUTOLAND";
const string DONE = "DONE";

double target_tolerance_linear = 0.2;
double target_tolerance_angular = 0.05;
string STATE = "STATE11";  // 初始状态设置为字符串

int arr[11][11];
int flag_;
int x1,x2,x3,y_1,y2,y3;
void avoid_gray(int i,int j,int flag);

int main(int argc, char **argv) {
    for(int p = 0;p<11;p++)
    {
        for(int q = 0;q<11;q++)
        {
            arr[p][q] = 0;
        }
    }
    cin>>x1>>y_1>>x2>>y2>>x3>>y3;
    if(x1 == x2 && x2 == x3) flag_ = 2;
    if(y_1 == y2 && y2 == y3) flag_ = 1;
    arr[x1][y_1] = 1;
    arr[x2][y2] = 1;
    arr[x3][y3] = 1;

    ros::init(argc, argv, "offboard_vel");
    ros::NodeHandle nh;
    pos_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Rate rate(20.0);  // Publish rate > 2Hz

    // 连接上无人机
    ros::Subscriber state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb_);
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected!");

    // 发送初步目标位置  
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.2;

    for (int i = 0; i < 100; i++) {
        pos_publisher.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 切换offboard模式的服务
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

    // 循环里需要的数据
    ros::Time last_request = ros::Time::now();
    ros::Subscriber pose_subscriber = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, curr_pose_cb_);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);  // 坐标系为local 上电点
    ros::Time state_start_time = ros::Time::now();
    ros::Time hover_start_time;  // 悬停开始时间
    ros::Time vel_control_start_time;
    double hover_duration = 3.0;  // 悬停3秒
    int i = 1;
    int j = 1;

    while (ros::ok()) {
        // 切换offboard模式与解锁
        if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client_.call(offboard_set_mode_) && offboard_set_mode_.response.mode_sent) {
                ROS_INFO("Offboard enabled!");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arm_client_.call(arm_) && arm_.response.success) {
                    ROS_INFO("Vehicle armed!");
                }
                last_request = ros::Time::now();
            }
        }

        // 进入状态机 通过速度控制发位置
// 进入状态机 通过速度控制发位置
if (STATE == State[1][1]) {
    pos_publisher.publish(pose);
    // 检查是否到达目标高度（误差<0.2m）
    if (fabs(pose.pose.position.z - curr_pose_.pose.position.z) <= target_tolerance_linear) {
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();  // 记录悬停开始时间
    }
} 
else if (STATE == Hover[1][1]) {
    pos_publisher.publish(pose);  // 保持目标位置
    // 悬停2秒
    if (ros::Time::now() - hover_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[1][1]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0;
        pose.pose.position.y = 0.5;
        i = 2;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[2][1]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[2][1]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0;
        pose.pose.position.y = 1.0;
        i = 3;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[3][1]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[3][1]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0;
        pose.pose.position.y = 1.5;
        i = 4;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[4][1]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[4][1]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0;
        pose.pose.position.y = 2.0;
        i = 5;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[5][1]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[5][1]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0;
        pose.pose.position.y = 2.5;
        i = 6;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[6][1]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[6][1]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0;
        pose.pose.position.y = 3.0;
        i = 7;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[7][1]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[7][1]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0;
        pose.pose.position.y = 3.5;
        i = 8;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[8][1]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[8][1]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0;
        pose.pose.position.y = 4.0;
        i = 9;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[9][1]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.x = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[9][1]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.x = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 4.0;
        STATE = Hover[1][2];
        i = 1;
        j = 2;
    }
} 
else if (STATE == Hover[1][2]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = - 0.25;  
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[1][2]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 3.5;
        STATE = Hover[2][2];
        i = 2;
        j = 2;
    }
} 
else if (STATE == Hover[2][2]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[2][2]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 3.0;
        i = 3;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[3][2]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[3][2]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 2.5;
        i = 4;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[4][2]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[4][2]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 2.0;
        i = 5;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[5][2]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[5][2]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 1.5;
        i = 6;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[6][2]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[6][2]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 1.0;
        i = 7;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[7][2]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[7][2]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 0.5;
        i = 8;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[8][2]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[8][2]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 0.0;
        i = 9;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[9][2]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.x = 0.25;  
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[9][2]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.x = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 0.0;
        i = 1;
        j++;
        STATE = Hover[i][j];
    }
}
else if (STATE == Hover[1][3]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;  
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[1][3]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 0.5;
        i = 2;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[2][3]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[2][3]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 1.0;
        i = 3;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[3][3]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[3][3]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 1.5;
        i = 4;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[4][3]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[4][3]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 2.0;
        i = 5;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[5][3]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[5][3]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 2.5;
        i = 6;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[6][3]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[6][3]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 3.0;
        i = 7;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[7][3]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[7][3]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 3.5;
        i = 8;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[8][3]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[8][3]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.0;
        pose.pose.position.y = 4.0;
        i = 9;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[9][3]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.x = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[9][3]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.x = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.5;
        pose.pose.position.y = 4.0;
        i = 1;
        j++;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
}
else if (STATE == Hover[1][4]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;  // y方向负向移动
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[1][4]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.5;
        pose.pose.position.y = 3.5;
        i = 2;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[2][4]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[2][4]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.5;
        pose.pose.position.y = 3.0;
        i = 3;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[3][4]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[3][4]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.5;
        pose.pose.position.y = 2.5;
        i = 4;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[4][4]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[4][4]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.5;
        pose.pose.position.y = 2.0;
        i = 5;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[5][4]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[5][4]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.5;
        pose.pose.position.y = 1.5;
        i = 6;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[6][4]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[6][4]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.5;
        pose.pose.position.y = 1.0;
        i = 7;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[7][4]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[7][4]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.5;
        pose.pose.position.y = 0.5;
        i = 8;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[8][4]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[8][4]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 1.5;
        pose.pose.position.y = 0.0;
        i = 9;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[9][4]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.x = 0.25;  // x方向负向移动
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[9][4]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.x = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.0;
        pose.pose.position.y = 0.0;
        i = 1;
        j++; 
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
}
else if (STATE == Hover[1][5]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;  // y正向移动（奇数列规律）
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[1][5]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.0;
        pose.pose.position.y = 0.5;
        i = 2;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[2][5]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[2][5]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.0;
        pose.pose.position.y = 1.0;
        i = 3;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[3][5]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[3][5]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.0;
        pose.pose.position.y = 1.5;
        i = 4;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[4][5]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[4][5]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.0;
        pose.pose.position.y = 2.0;
        i = 5;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[5][5]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[5][5]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.0;
        pose.pose.position.y = 2.5;
        i = 6;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[6][5]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[6][5]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.0;
        pose.pose.position.y = 3.0;
        i = 7;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[7][5]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[7][5]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.0;
        pose.pose.position.y = 3.5;
        i = 8;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[8][5]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[8][5]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.0;
        pose.pose.position.y = 4.0;
        i = 9;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[9][5]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.x = 0.25;  // x正向过渡到下一列
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[9][5]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.x = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.5;
        pose.pose.position.y = 4.0;
        i = 1;
        j++;  // 进入第6列
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
}
else if (STATE == Hover[1][6]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;  // y负向移动（偶数列规律）
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[1][6]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.5;
        pose.pose.position.y = 3.5;
        i = 2;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[2][6]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[2][6]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.5;
        pose.pose.position.y = 3.0;
        i = 3;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[3][6]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[3][6]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.5;
        pose.pose.position.y = 2.5;
        i = 4;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[4][6]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[4][6]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.5;
        pose.pose.position.y = 2.0;
        i = 5;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[5][6]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[5][6]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.5;
        pose.pose.position.y = 1.5;
        i = 6;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[6][6]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[6][6]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.5;
        pose.pose.position.y = 1.0;
        i = 7;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[7][6]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[7][6]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.5;
        pose.pose.position.y = 0.5;
        i = 8;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[8][6]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = -0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[8][6]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 2.5;
        pose.pose.position.y = 0.0;
        i = 9;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[9][6]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.x = 0.25;  // x正向过渡到下一列
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[9][6]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.x = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 3.0;
        pose.pose.position.y = 0.0;
        i = 1;
        j++;  // 进入第7列
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
}
else if (STATE == Hover[1][7]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;  // y正向移动（奇数列规律）
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[1][7]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 3.0;
        pose.pose.position.y = 0.5;
        i = 2;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[2][7]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[2][7]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 3.0;
        pose.pose.position.y = 1.0;
        i = 3;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[3][7]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[3][7]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 3.0;
        pose.pose.position.y = 1.5;
        i = 4;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[4][7]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[4][7]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 3.0;
        pose.pose.position.y = 2.0;
        i = 5;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[5][7]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[5][7]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 3.0;
        pose.pose.position.y = 2.5;
        i = 6;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[6][7]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[6][7]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 3.0;
        pose.pose.position.y = 3.0;
        i = 7;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[7][7]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[7][7]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 3.0;
        pose.pose.position.y = 3.5;
        i = 8;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[8][7]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        vel_msg.linear.y = 0.25;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 
else if (STATE == Move[8][7]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose.pose.position.x = 3.0;
        pose.pose.position.y = 4.0;
        i = 9;
        STATE = Hover[i][j];
        hover_start_time = ros::Time::now();
    }
} 
else if (STATE == Hover[9][7]) {
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(1)) {
        // 最后一列结束，可添加停止或返航逻辑（示例：x方向停止）
        vel_msg.linear.x = 0;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = Move[i][j];
        avoid_gray(i,j,flag_);
    }
} 



else if (STATE == Move[9][7]) {
    vel_pub.publish(vel_msg);
    if (ros::Time::now() - vel_control_start_time > ros::Duration(2)) {
        // 7×9遍历完成，停止所有运动
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_pub.publish(vel_msg);
        pose = curr_pose_;
        // 可添加最终状态（如降落、返航）
        STATE = MOVE_DOWN_1;  // 假设Finish为结束状态
    }
}

else if (STATE == MOVE_DOWN_1) {
    pose.pose.position.y = -0.9;
    pose.pose.position.x = 0.7;
    pos_publisher.publish(pose);
    if (ros::Time::now() - hover_start_time > ros::Duration(9)) {
        // 最后一列结束，可添加停止或返航逻辑（示例：x方向停止）
        vel_msg.linear.x = 0;
        vel_pub.publish(vel_msg);
        vel_control_start_time = ros::Time::now();
        STATE = MOVE_DOWN;
        avoid_gray(i,j,flag_);
    }
} 

        else if (STATE == MOVE_DOWN) {
            vel_pub.publish(vel_msg);
            if (fabs(pose.pose.position.z - curr_pose_.pose.position.z) <= target_tolerance_linear) {
                ROS_INFO("Reached down position, starting hover...");
                vel_msg.linear.z = 0;
                vel_pub.publish(vel_msg);
                STATE = HOVER_6;
                hover_start_time = ros::Time::now();
            }
        } 
        else if (STATE == HOVER_6) {
            pos_publisher.publish(pose);
            if (ros::Time::now() - hover_start_time > ros::Duration(hover_duration)) {
                ROS_INFO("Hover complete, starting landing...");
                pose.pose.position.x = 0;
                pose.pose.position.y = 0;
                pose.pose.position.z = 0.01;
                STATE = LANDING;
            }
        } 
        else if (STATE == LANDING) {
            pos_publisher.publish(pose);
            if (fabs(pose.pose.position.z - curr_pose_.pose.position.z) <= target_tolerance_linear) {
                ROS_INFO("Reached land height, starting auto land...");
                STATE = AUTOLAND;
                state_start_time = ros::Time::now();
            }
        } 
        else if (STATE == AUTOLAND) {
            mavros_msgs::SetMode land_set_mode_;
            land_set_mode_.request.custom_mode = "AUTO.LAND";  
            bool mode_sent = set_mode_client_.call(land_set_mode_) && land_set_mode_.response.mode_sent;

            // 超时判断（修正为合理的超时时间15秒）
            if ((mode_sent && current_state_.mode == "AUTO.LAND" && !current_state_.armed) ||
                (ros::Time::now() - state_start_time > ros::Duration(0.01))) {
                if (current_state_.armed) {
                    disarm_client_.call(disarm_);  // 上锁
                }
                ROS_INFO("Drone has landed");
                ros::shutdown();
            }
        } 
        else if (STATE == DONE) {
            ROS_INFO("Mission completed");
            ros::shutdown();
        } 
        else {
            ROS_ERROR("Unknown state: %s", STATE.c_str());
            STATE = DONE;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
void avoid_gray(int i, int j, int flag) {
    // 保存当前位置用于后续计算
    auto original_pose = curr_pose_;  // 记录初始位置用于计算
    ros::Rate rate(20.0);  // Publish rate > 2Hz

    // 分奇偶行中的ij
    if (j == 1) {
        if (i == 9 && arr[9][2] && flag == 1) { // 横
            // 向右飞 3 * 0.5  上飞 0.5 STATE -->[6][2]
            pose = curr_pose_;
            pose.pose.position.y -= 1.5; // 向右飞3*0.5
            pos_publisher.publish(pose);
            
            // 等待移动到位（添加spin和sleep，以及退出条件）
            ros::Time start_time = ros::Time::now();
            while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();  // 处理回调
                rate.sleep();     // 控制循环频率
                
                // 超时退出（5秒超时保护）
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving right in j=1, i=9, flag=1");
                    return;
                }
            }
            
            pose.pose.position.x += 0.5; // 上飞0.5
            pos_publisher.publish(pose);
            start_time = ros::Time::now();  // 重置超时计时器
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving up in j=1, i=9, flag=1");
                    return;
                }
            }
            STATE = Hover[6][2];
        }
        
        if (i == 9 && arr[9][2] && flag == 2) { // 竖
            // 向右飞 0.5  上飞 0.5 STATE -->[8][2]
            pose = curr_pose_;
            pose.pose.position.y -= 0.5; // 向右飞0.5
            pos_publisher.publish(pose);
            
            ros::Time start_time = ros::Time::now();
            while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving right in j=1, i=9, flag=2");
                    return;
                }
            }
            
            pose.pose.position.x += 0.5; // 上飞0.5
            pos_publisher.publish(pose);
            start_time = ros::Time::now();
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving up in j=1, i=9, flag=2");
                    return;
                }
            }
            STATE = Hover[8][2];
        }
        
        if (flag == 1 && arr[i + 1][j]) { // 横
            if (x3 < 9) {
                // 向上飞0.5  左飞  4 * 0.5  下飞0.5  STATE -->[i+4][j]
                pose = curr_pose_;
                pose.pose.position.x += 0.5; // 向上飞0.5
                pos_publisher.publish(pose);
                
                ros::Time start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving up in j=1, flag=1 (x3<9)");
                        return;
                    }
                }
                
                pose.pose.position.y += 2.0; // 左飞4*0.5
                pos_publisher.publish(pose);
                start_time = ros::Time::now();
                while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving left in j=1, flag=1 (x3<9)");
                        return;
                    }
                }
                
                pose.pose.position.x -= 0.5; // 下飞0.5
                pos_publisher.publish(pose);
                start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving down in j=1, flag=1 (x3<9)");
                        return;
                    }
                }
                STATE = Hover[i + 4][j];
            } else {
                // 向上飞0.5  左飞  3 * 0.5   STATE -->[i+3][j+1]
                pose = curr_pose_;
                pose.pose.position.x += 0.5; // 向上飞0.5
                pos_publisher.publish(pose);
                
                ros::Time start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving up in j=1, flag=1 (x3>=9)");
                        return;
                    }
                }
                
                pose.pose.position.y += 1.5; // 左飞3*0.5
                pos_publisher.publish(pose);
                start_time = ros::Time::now();
                while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving left in j=1, flag=1 (x3>=9)");
                        return;
                    }
                }
                STATE = Hover[i + 3][j + 1];
            }
        }
        
        if (flag == 2 && arr[i + 1][j]) { // 竖
            if ((i + 1) == 9) {
                // 向上飞 0.5   STATE-->[8][2]
                pose = curr_pose_;
                pose.pose.position.x += 0.5; // 向上飞0.5
                pos_publisher.publish(pose);
                
                ros::Time start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving up in j=1, flag=2 (i+1=9)");
                        return;
                    }
                }
                STATE = Hover[8][2];
            } else {
                // 向上飞 3 * 0.5 向左飞 2 * 0.5 向下飞 3 * 0.5  -->STATE[i+2][j]
                pose = curr_pose_;
                pose.pose.position.x += 1.5; // 向上飞3*0.5
                pos_publisher.publish(pose);
                
                ros::Time start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving up in j=1, flag=2 (i+1!=9)");
                        return;
                    }
                }
                
                pose.pose.position.y += 1.0; // 向左飞2*0.5
                pos_publisher.publish(pose);
                start_time = ros::Time::now();
                while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving left in j=1, flag=2 (i+1!=9)");
                        return;
                    }
                }
                
                pose.pose.position.x -= 1.5; // 向下飞3*0.5
                pos_publisher.publish(pose);
                start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving down in j=1, flag=2 (i+1!=9)");
                        return;
                    }
                }
                STATE = Hover[i + 2][j];
            }
        }
    } 
    else if (j == 3 || j == 5) {
        if (i == 9 && arr[9][j + 1] && flag == 1) { // 横
            // 向右飞 3 * 0.5  上飞 0.5 STATE -->[6][j+1]
            pose = curr_pose_;
            pose.pose.position.y -= 1.5; // 向右飞3*0.5
            pos_publisher.publish(pose);
            
            ros::Time start_time = ros::Time::now();
            while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving right in j=3/5, i=9, flag=1");
                    return;
                }
            }
            
            pose.pose.position.x += 0.5; // 上飞0.5
            pos_publisher.publish(pose);
            start_time = ros::Time::now();
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving up in j=3/5, i=9, flag=1");
                    return;
                }
            }
            STATE = Hover[6][j + 1];
        }
        
        if (i == 9 && arr[9][j + 1] && flag == 2) { // 竖
            // 向右飞 0.5  上飞 0.5 STATE -->[8][j+1]
            pose = curr_pose_;
            pose.pose.position.y -= 0.5; // 向右飞0.5
            pos_publisher.publish(pose);
            
            ros::Time start_time = ros::Time::now();
            while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving right in j=3/5, i=9, flag=2");
                    return;
                }
            }
            
            pose.pose.position.x += 0.5; // 上飞0.5
            pos_publisher.publish(pose);
            start_time = ros::Time::now();
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving up in j=3/5, i=9, flag=2");
                    return;
                }
            }
            STATE = Hover[8][j + 1];
        }
        
        if (flag == 1 && arr[i + 1][j]) { // 横
            if (x3 < 9) {
                // 向上飞0.5  左飞  4 * 0.5  下飞0.5  STATE -->[i+4][j]
                pose = curr_pose_;
                pose.pose.position.x += 0.5; // 向上飞0.5
                pos_publisher.publish(pose);
                
                ros::Time start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving up in j=3/5, flag=1 (x3<9)");
                        return;
                    }
                }
                
                pose.pose.position.y += 2.0; // 左飞4*0.5
                pos_publisher.publish(pose);
                start_time = ros::Time::now();
                while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving left in j=3/5, flag=1 (x3<9)");
                        return;
                    }
                }
                
                pose.pose.position.x -= 0.5; // 下飞0.5
                pos_publisher.publish(pose);
                start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving down in j=3/5, flag=1 (x3<9)");
                        return;
                    }
                }
                STATE = Hover[i + 4][j];
            } else {
                // 向上飞0.5  左飞  3 * 0.5   STATE -->[i+3][j+1]
                pose = curr_pose_;
                pose.pose.position.x += 0.5; // 向上飞0.5
                pos_publisher.publish(pose);
                
                ros::Time start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving up in j=3/5, flag=1 (x3>=9)");
                        return;
                    }
                }
                
                pose.pose.position.y += 1.5; // 左飞3*0.5
                pos_publisher.publish(pose);
                start_time = ros::Time::now();
                while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving left in j=3/5, flag=1 (x3>=9)");
                        return;
                    }
                }
                STATE = Hover[i + 3][j + 1];
            }
        }
        
        if (flag == 2 && arr[i][j]) { // 竖
            if ((i + 1) == 9) {
                // 向上飞 0.5   STATE-->[8][j+1]
                pose = curr_pose_;
                pose.pose.position.x += 0.5; // 向上飞0.5
                pos_publisher.publish(pose);
                
                ros::Time start_time = ros::Time::now();
                while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                    pos_publisher.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                    
                    if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                        ROS_WARN("Timeout while moving up in j=3/5, flag=2 (i+1=9)");
                        return;
                    }
                }
                STATE = Hover[8][j + 1];
            } else {
                if (y_1 == 1) { // 只能从上边绕
                    // 向上飞 (y3 - j + 1) * 0.5 向左飞 2 * 0.5 向下飞 (y3 - j + 1) * 0.5 STATE-->[i+2][j]
                    float up_distance = (y3 - j + 1) * 0.5;
                    
                    pose = curr_pose_;
                    pose.pose.position.x += up_distance; // 向上飞
                    pos_publisher.publish(pose);
                    
                    ros::Time start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving up (top绕) in j=3/5, flag=2");
                            return;
                        }
                    }
                    
                    pose.pose.position.y += 1.0; // 向左飞2*0.5
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving left (top绕) in j=3/5, flag=2");
                            return;
                        }
                    }
                    
                    pose.pose.position.x -= up_distance; // 向下飞
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving down (top绕) in j=3/5, flag=2");
                            return;
                        }
                    }
                    STATE = Hover[i + 2][j];
                } else { // 可以从下边绕
                    // 向下飞 (j - y_1 + 1) * 0.5 向左飞 2 * 0.5 向上飞 (j - y_1 + 1) * 0.5 STATE-->[i+2][j]
                    float down_distance = (j - y_1 + 1) * 0.5;
                    
                    pose = curr_pose_;
                    pose.pose.position.x -= down_distance; // 向下飞
                    pos_publisher.publish(pose);
                    
                    ros::Time start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving down (bottom绕) in j=3/5, flag=2");
                            return;
                        }
                    }
                    
                    pose.pose.position.y += 1.0; // 向左飞2*0.5
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving left (bottom绕) in j=3/5, flag=2");
                            return;
                        }
                    }
                    
                    pose.pose.position.x += down_distance; // 向上飞
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving up (bottom绕) in j=3/5, flag=2");
                            return;
                        }
                    }
                    STATE = Hover[i + 2][j];
                }
            }
        }
    } 
    else if (j == 7) {
        // 横
        if (x3 == 9 && flag == 1 && arr[i + 1][j]) {
            // 回去降落
            pose = original_pose; // 返回初始位置
            pos_publisher.publish(pose);
            
            ros::Time start_time = ros::Time::now();
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_ ||
                   fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(10.0)) { // 降落超时时间更长
                    ROS_WARN("Timeout while returning to land in j=7, flag=1 (x3=9)");
                    return;
                }
            }
            STATE = MOVE_DOWN;
        } 
        else if (x3 != 9 && flag == 1 && arr[i + 1][j]) {
            // 向下飞 0.5 向左飞 4 * 0.5 向上飞 0.5 STATE -->[i+4][j]
            pose = curr_pose_;
            pose.pose.position.x -= 0.5; // 向下飞0.5
            pos_publisher.publish(pose);
            
            ros::Time start_time = ros::Time::now();
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving down in j=7, flag=1 (x3!=9)");
                    return;
                }
            }
            
            pose.pose.position.y += 2.0; // 向左飞4*0.5
            pos_publisher.publish(pose);
            start_time = ros::Time::now();
            while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving left in j=7, flag=1 (x3!=9)");
                    return;
                }
            }
            
            pose.pose.position.x += 0.5; // 向上飞0.5
            pos_publisher.publish(pose);
            start_time = ros::Time::now();
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving up in j=7, flag=1 (x3!=9)");
                    return;
                }
            }
            STATE = Hover[i + 4][j];
        }
        // 竖
        else if (x3 == 9 && flag == 2 && arr[i + 1][j]) {
            // 回去降落
            pose = original_pose; // 返回初始位置
            pos_publisher.publish(pose);
            
            ros::Time start_time = ros::Time::now();
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_ ||
                   fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(10.0)) {
                    ROS_WARN("Timeout while returning to land in j=7, flag=2 (x3=9)");
                    return;
                }
            }
            STATE = MOVE_DOWN;
        } 
        else {
            // 向下飞 3 * 0.5 向左飞2 * 0.5 向上飞 3 * 0.5 STATE -->[i+2][j]
            pose = curr_pose_;
            pose.pose.position.x -= 1.5; // 向下飞3*0.5
            pos_publisher.publish(pose);
            
            ros::Time start_time = ros::Time::now();
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving down in j=7, else case");
                    return;
                }
            }
            
            pose.pose.position.y += 1.0; // 向左飞2*0.5
            pos_publisher.publish(pose);
            start_time = ros::Time::now();
            while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving left in j=7, else case");
                    return;
                }
            }
            
            pose.pose.position.x += 1.5; // 向上飞3*0.5
            pos_publisher.publish(pose);
            start_time = ros::Time::now();
            while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                pos_publisher.publish(pose);
                ros::spinOnce();
                rate.sleep();
                
                if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                    ROS_WARN("Timeout while moving up in j=7, else case");
                    return;
                }
            }
            STATE = Hover[i + 2][j];
        }
    } 
    else { // j == 2 || j == 4 || j == 6
        if (i == 9) {
            if (arr[10 - i][j + 1]) {
                if (flag == 1) { // 横
                    // 向左飞 3 * 0.5 向上飞 0.5  STATE --> [6][j+1]
                    pose = curr_pose_;
                    pose.pose.position.y += 1.5; // 向左飞3*0.5
                    pos_publisher.publish(pose);
                    
                    ros::Time start_time = ros::Time::now();
                    while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving left in j=2/4/6, i=9, flag=1");
                            return;
                        }
                    }
                    
                    pose.pose.position.x += 0.5; // 向上飞0.5
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving up in j=2/4/6, i=9, flag=1");
                            return;
                        }
                    }
                    STATE = Hover[6][j + 1];
                } else {
                    // 向左飞0.5 向上飞0.5 STATE --> [8][j+1]
                    pose = curr_pose_;
                    pose.pose.position.y += 0.5; // 向左飞0.5
                    pos_publisher.publish(pose);
                    
                    ros::Time start_time = ros::Time::now();
                    while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving left in j=2/4/6, i=9, flag=2");
                            return;
                        }
                    }
                    
                    pose.pose.position.x += 0.5; // 向上飞0.5
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving up in j=2/4/6, i=9, flag=2");
                            return;
                        }
                    }
                    STATE = Hover[8][j + 1];
                }
            }
        } else {
            if (arr[9 - i][j] && (!arr[1][j])) { // 需要绕
                if (flag == 1) { // 横
                    // 向下0.5 向右4*0.5 向上0.5 STATE --> [i+4][j]
                    pose = curr_pose_;
                    pose.pose.position.x -= 0.5; // 向下0.5
                    pos_publisher.publish(pose);
                    
                    ros::Time start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving down (需要绕) in j=2/4/6, flag=1");
                            return;
                        }
                    }
                    
                    pose.pose.position.y -= 2.0; // 向右4*0.5
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving right (需要绕) in j=2/4/6, flag=1");
                            return;
                        }
                    }
                    
                    pose.pose.position.x += 0.5; // 向上0.5
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving up (需要绕) in j=2/4/6, flag=1");
                            return;
                        }
                    }
                    STATE = Hover[i + 4][j];
                }
                if (flag == 2) { // 竖
                    // 向下 (j - y_1 + 1) * 0.5 向右2 * 0.5 向上(j - y_1 + 1) * 0.5
                    float down_distance = (j - y_1 + 1) * 0.5;
                    
                    pose = curr_pose_;
                    pose.pose.position.x -= down_distance; // 向下飞
                    pos_publisher.publish(pose);
                    
                    ros::Time start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving down (需要绕) in j=2/4/6, flag=2");
                            return;
                        }
                    }
                    
                    pose.pose.position.y -= 1.0; // 向右2*0.5
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving right (需要绕) in j=2/4/6, flag=2");
                            return;
                        }
                    }
                    
                    pose.pose.position.x += down_distance; // 向上飞
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving up (需要绕) in j=2/4/6, flag=2");
                            return;
                        }
                    }
                }
            }
            if (arr[9 - 1][j] && arr[1][j]) { // 不需要绕 只向上走
                if (flag == 1) {
                    // 向上0.5 向右3*0.5 STATE --> [9][j+1]
                    pose = curr_pose_;
                    pose.pose.position.x += 0.5; // 向上0.5
                    pos_publisher.publish(pose);
                    
                    ros::Time start_time = ros::Time::now();
                    while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving up (不绕) in j=2/4/6, flag=1");
                            return;
                        }
                    }
                    
                    pose.pose.position.y -= 1.5; // 向右3*0.5
                    pos_publisher.publish(pose);
                    start_time = ros::Time::now();
                    while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                        pos_publisher.publish(pose);
                        ros::spinOnce();
                        rate.sleep();
                        
                        if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                            ROS_WARN("Timeout while moving right (不绕) in j=2/4/6, flag=1");
                            return;
                        }
                    }
                    STATE = Hover[9][j + 1];
                }
                if (flag == 2) {
                    if (arr[1][j + 1]) {
                        // 向上走0.5 STATE --> [i][j+1]
                        pose = curr_pose_;
                        pose.pose.position.x += 0.5; // 向上走0.5
                        pos_publisher.publish(pose);
                        
                        ros::Time start_time = ros::Time::now();
                        while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                            pos_publisher.publish(pose);
                            ros::spinOnce();
                            rate.sleep();
                            
                            if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                                ROS_WARN("Timeout while moving up (不绕) in j=2/4/6, flag=2 (arr[1][j+1])");
                                return;
                            }
                        }
                        STATE = Hover[i][j + 1];
                    } else {
                        // 向上走0.5 向右走0.5 STATE --> [9][j+1]
                        pose = curr_pose_;
                        pose.pose.position.x += 0.5; // 向上走0.5
                        pos_publisher.publish(pose);
                        
                        ros::Time start_time = ros::Time::now();
                        while (fabs(pose.pose.position.x - original_pose.pose.position.x) >= t_) {
                            pos_publisher.publish(pose);
                            ros::spinOnce();
                            rate.sleep();
                            
                            if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                                ROS_WARN("Timeout while moving up (不绕) in j=2/4/6, flag=2 (!arr[1][j+1])");
                                return;
                            }
                        }
                        
                        pose.pose.position.y -= 0.5; // 向右走0.5
                        pos_publisher.publish(pose);
                        start_time = ros::Time::now();
                        while (fabs(pose.pose.position.y - original_pose.pose.position.y) >= t_) {
                            pos_publisher.publish(pose);
                            ros::spinOnce();
                            rate.sleep();
                            
                            if (ros::Time::now() - start_time > ros::Duration(5.0)) {
                                ROS_WARN("Timeout while moving right (不绕) in j=2/4/6, flag=2 (!arr[1][j+1])");
                                return;
                            }
                        }
                        STATE = Hover[9][j + 1];
                    }
                }
            }
        }
    }
    return;
}
