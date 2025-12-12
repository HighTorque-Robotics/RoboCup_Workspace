#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <cmath>
#include "dmsgs/ActionCommand.h"
#include "dmsgs/VisionInfo.h"
#include "dmsgs/MotionInfo.h"
#include "dmsgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "algorithm"
#include <cmath>
#include <utility>
// #include "sim2real_msg/Joy.h"

using namespace std;

struct Parameters {
    struct Stp {
        bool see_ball= false;
        vector<double> ball_field = {1,0,0};
        vector<double> ball_global = {0,0,0};
        vector<double> ball_velocity = {0,0,0}; // 看起来应该是球在机器人坐标系下的速度
        vector<double> robot_global = {0,0,0};
        vector<double> action_global = {0,0,0};
        vector<double> vel_cmd = {0,0,0}; // 规划出来，用于发布的速度
        std::vector<std::vector<double>> obstacles;  // 障碍物信息，每个障碍物包含 x, y, radius，存储多个障碍物
        int gait_type = 0;
        double ball_field_distance;
        double ball_field_angle;
        // double dist_ball_to_target = 0; // 球到目标点的距离
        double robot_angle_error = 0; // 朝向目标点 - 现在的朝向
        // bool support_ok = false;
    } stp;
} parameters;

// sim2real_msg::Joy joy_msg; // 模拟遥控器发布的/joy_msg
dmsgs::Joy joy_msg;

double AdjustDegRange2(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

double AdjustRadRange2(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double Rad2Deg(double rad) {
    return rad * 180.0 / M_PI;
}
double Deg2Rad(double deg) {
    return deg * M_PI / 180.0;
}

double ball_field_angle() {
    return AdjustRadRange2(
        atan2(parameters.stp.ball_field[1], parameters.stp.ball_field[0]));
}

double ball_field_distance() {
    return sqrt(parameters.stp.ball_field[0] * parameters.stp.ball_field[0] +
                parameters.stp.ball_field[1] * parameters.stp.ball_field[1]);
}


double distanceToBallTargetLine() { // 计算机器人到“球-目标点”直线的垂直距离
    // 提取坐标（均为全局坐标系下的x, y）
    double x0 = parameters.stp.robot_global[0];  // 机器人x
    double y0 = parameters.stp.robot_global[1];  // 机器人y
    double x1 = parameters.stp.ball_global[0];   // 球x
    double y1 = parameters.stp.ball_global[1];   // 球y
    double x2 = parameters.stp.action_global[0]; // 目标点x
    double y2 = parameters.stp.action_global[1]; // 目标点y

    // 计算向量AB（球到目标点）和AP（球到机器人）
    double ABx = x2 - x1;  // 直线方向向量x分量
    double ABy = y2 - y1;  // 直线方向向量y分量
    double APx = x0 - x1;  // 机器人到球的向量x分量
    double APy = y0 - y1;  // 机器人到球的向量y分量

    // 若球和目标点重合，返回0（避免除以0）
    double AB_length_sq = ABx * ABx + ABy * ABy;  // AB长度的平方
    if (AB_length_sq < 1e-4) {  // 阈值处理浮点数精度问题
        return 0;
    }

    // 点到直线距离公式：|AB × AP| / |AB|（叉积的绝对值除以AB长度）
    double cross_product = ABx * APy - ABy * APx;  // 二维向量叉积的模长
    double distance = std::fabs(cross_product) / std::sqrt(AB_length_sq);

    return distance;
}


void VisionCallBack(const dmsgs::VisionInfo::ConstPtr &msg) {
    parameters.stp.robot_global.clear();
    parameters.stp.robot_global.push_back(msg->robot_pos.x / 100.0);
    parameters.stp.robot_global.push_back(msg->robot_pos.y / 100.0);
    parameters.stp.robot_global.push_back(Deg2Rad(msg->robot_pos.z));
    parameters.stp.robot_angle_error = atan2(parameters.stp.action_global[1] - parameters.stp.robot_global[1], parameters.stp.action_global[0] - parameters.stp.robot_global[0]) - parameters.stp.robot_global[2];
    // ROS_INFO("vision msg: robot_pos.x, y, z =  %.2f, %.2f, %.2f", msg->robot_pos.x, msg->robot_pos.y, msg->robot_pos.z);

    parameters.stp.see_ball = msg->see_ball;
    if (parameters.stp.see_ball) {
        parameters.stp.ball_global.clear();
        parameters.stp.ball_global.push_back(msg->ball_global.x / 100.0);
        parameters.stp.ball_global.push_back(msg->ball_global.y / 100.0);

        parameters.stp.ball_field.clear();
        parameters.stp.ball_field.push_back(msg->ball_field.x / 100.0);
        parameters.stp.ball_field.push_back(msg->ball_field.y / 100.0);

        parameters.stp.ball_velocity.clear();
        parameters.stp.ball_velocity.push_back(msg->ball_velocity.x / 100.0);
        parameters.stp.ball_velocity.push_back(msg->ball_velocity.y / 100.0);
        
        parameters.stp.ball_field_distance = ball_field_distance();
        parameters.stp.ball_field_angle = ball_field_angle();
        // parameters.stp.dist_ball_to_target = sqrt(pow(parameters.stp.ball_field[0] - parameters.stp.action_global[0], 2) + pow(parameters.stp.ball_field[1] - parameters.stp.action_global[1], 2));
        ROS_INFO("vision msg: ball_velocity.x, y =  %.2f, %.2f", msg->ball_global.x, msg->ball_global.y);

    }
}


// void MotionCallBack(const dmsgs::MotionInfo::ConstPtr &msg) {
//     if (msg->lower_board_connected) {
//         parameters.stp.support_ok = msg->support_stablizer_flag;
//     }
// }


void ActionCallBack(const dmsgs::ActionCommand::ConstPtr &msg) {
    parameters.stp.action_global.clear();
    parameters.stp.action_global.push_back(msg->bodyCmd.x / 100.0);
    parameters.stp.action_global.push_back(msg->bodyCmd.y / 100.0);
    parameters.stp.action_global.push_back(Deg2Rad(msg->bodyCmd.t));
    parameters.stp.gait_type = msg->bodyCmd.gait_type;
    parameters.stp.robot_angle_error = atan2(parameters.stp.action_global[1] - parameters.stp.robot_global[1], parameters.stp.action_global[0] - parameters.stp.robot_global[0]) - parameters.stp.robot_global[2];
    // parameters.stp.dist_ball_to_target = sqrt(pow(parameters.stp.ball_field[0] - parameters.stp.action_global[0], 2) + pow(parameters.stp.ball_field[1] - parameters.stp.action_global[1], 2));
    ROS_INFO("action msg: bodycmd.x, y, t =  %.2f, %.2f, %.2f", msg->bodyCmd.x, msg->bodyCmd.y, msg->bodyCmd.t);
}

double vectorMagnitude(const std::vector<double>& vec) {
    double sum_of_squares = 0.0;
    
    for (double element : vec) {
        sum_of_squares += element * element;  // 计算平方和
    }
    
    return sqrt(sum_of_squares);  // 返回平方根
}

double distance(double current_x, double current_y, double target_x, double target_y){
    double dx = current_x - target_x;
    double dy = current_y - target_y;
    return sqrt(dx*dx+dy*dy);
}


void generate_velocity(const std::vector<double>& current_pose, 
                      const std::vector<double>& target_pose) {
    
    // 参数验证
    if (current_pose.size() < 3 || target_pose.size() < 3) {
        // 错误处理：参数不足
        parameters.stp.vel_cmd[0] = 0.0;
        parameters.stp.vel_cmd[1] = 0.0;
        parameters.stp.vel_cmd[2] = 0.0;
        return;
    }
    
    // 提取当前位置和目标位置
    double current_x = current_pose[0];
    double current_y = current_pose[1];
    double current_yaw = current_pose[2];
    double target_x = target_pose[0];
    double target_y = target_pose[1];
    double target_yaw = target_pose[2];
    
    // 参数设置
    const double deadband = 0.2;      // 死区阈值
    const double deadband_YAW = 0.3; // YAW死区阈值   
    const double Kp = 0.5;            // 比例增益
    const double Kp_YAW = 0.57;       // YAW比例增益
    const double Vx_min = 0.4;        // X方向速度最小值
    const double Vx_max = 1.5;        // X方向速度最大值
    const double Vy_min = 0.4;        // Y方向速度最小值
    const double Vy_max = 1.0;        // Y方向速度最大值
    const double Wz_min = 0.5;        // YAW方向速度最小值
    const double Wz_max = 1.0;        // YAW方向速度最大值

    // 计算全局坐标系中的误差向量
    double e_x_global = target_x - current_x;
    double e_y_global = target_y - current_y;
    double angle_error = target_yaw - current_yaw;
    angle_error = AdjustRadRange2(angle_error);

    // 将全局误差转换到机器人局部坐标系
    double cos_yaw = std::cos(current_yaw);
    double sin_yaw = std::sin(current_yaw);
    double e_x_local = e_x_global * cos_yaw + e_y_global * sin_yaw;
    double e_y_local = -e_x_global * sin_yaw + e_y_global * cos_yaw;

    // 计算X方向速度
    double Vx = 0.0;
    if (std::abs(e_x_local) >= deadband) {
        Vx = Kp * e_x_local;
        
        // 限制X速度范围
        if (Vx > 0) {
            Vx = std::clamp(Vx, Vx_min, Vx_max);
        } else {
            Vx = std::clamp(Vx, -Vx_max, -Vx_min);
        }
    }

    // 计算Y方向速度
    double Vy = 0.0;
    if (std::abs(e_y_local) >= deadband) {
        Vy = Kp * e_y_local;
        
        // 限制Y速度范围
        if (Vy > 0) {
            Vy = std::clamp(Vy, Vy_min, Vy_max);
        } else {
            Vy = std::clamp(Vy, -Vy_max, -Vy_min);
        }
    }

    // 计算YAW速度
    double Wz = 0.0;
    if (std::abs(angle_error) >= deadband_YAW) {
        Wz = Kp_YAW * angle_error;
        
        // 限制YAW速度范围
        if (Wz > 0) {
            Wz = std::clamp(Wz, Wz_min, Wz_max);
        } else {
            Wz = std::clamp(Wz, -Wz_max, -Wz_min);
        }
    }
    
    parameters.stp.vel_cmd[0] = Vx;
    parameters.stp.vel_cmd[1] = Vy;
    parameters.stp.vel_cmd[2] = Wz;
}

void initJoyMsg() { //初始化Joy消息为默认值
    // 左摇杆
    joy_msg.l_horizontal = 0.0;
    joy_msg.l_vertical = 0.0;
    // LT/RT键
    joy_msg.lt = 1.0;
    joy_msg.rt = 0.0;
    // 右摇杆
    joy_msg.r_horizontal = 0.0;
    joy_msg.r_vertical = 0.0;
    // 方向键
    joy_msg.dpad_horizontal = 0.0;
    joy_msg.dpad_vertical = 0.0;
    // ABXY按键
    joy_msg.a = 0.0;
    joy_msg.b = 0.0;
    joy_msg.x = 0.0;
    joy_msg.y = 0.0;
    // LB/RB键
    joy_msg.lb = 0.0;
    joy_msg.rb = 0.0;
    // 回退/开始/中键
    joy_msg.back = 0.0;
    joy_msg.start = 0.0;
    joy_msg.center = 0.0;
    // 摇杆按下（L/R）
    joy_msg.L = 0.0;
    joy_msg.R = 0.0;
}

int main(int argc, char **argv) {
    string str = getenv("ZJUDANCER_ROBOTID");


    ros::init(argc, argv, "motion_hub_communicator");
    ros::NodeHandle n;
    
    ros::Subscriber VisionInfo_sub =
        n.subscribe("dvision_" + str + "/VisionInfo", 1, VisionCallBack);
    // ros::Subscriber MotionInfo_sub =
    //     n.subscribe("dmotion_" + str + "/MotionInfo", 1, MotionCallBack);
    ros::Subscriber ActionInfo_sub =
        n.subscribe("dbehavior_" + str + "/ActionCommand", 1, ActionCallBack);     
    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) // 高擎的Publisher
    // ros::Publisher joy_pub = n.advertise<sim2real_msg::Joy>("/joy_msg", 10);// 模拟遥控器发布小脚踢球等策略指令
    ros::Publisher joy_pub = n.advertise<dmsgs::Joy>("/joy_msg", 10);// 模拟遥控器发布小脚踢球等策略指令
    geometry_msgs::Twist velocity_msg;
    ros::Rate loop_rate(10);
    
    // 控制参数
    const double LINEAR_TOLERANCE = 0.2;  // 位置容差 (米)
    const double ANGULAR_TOLERANCE = 0.03; // 角度容差 (弧度)
    const double MIN_LINEAR_SPEED = 0.4;   // 最xiao线速度 (米/秒)
    const double MAX_LINEAR_SPEED_X = 1.5;   // 最大线速度VX (米/秒)
    const double MAX_LINEAR_SPEED_Y = 1.0;   // 最大线速度VY (米/秒)
    const double MIN_ANGULAR_SPEED = 0.5;  // 最xiao角速度 (弧度/秒)
    const double MAX_ANGULAR_SPEED = 1.0;  // 最大角速度 (弧度/秒)
    const double K_LINEAR = 1.0;           // 线速度增益
    const double K_ANGULAR = 1.0;          // 角速度增益

    // 盘球用到的参数
    double close_to_ball = 1.0; // 在盘球逻辑中，如果机器人和球距离比这个小，就进入接近球的调整逻辑
    vector<double> current_goal = {0, 0, 0}; // 盘球时的临时目标，x y yaw   
    // double dribble_rad_angle_tolerance = 0.174; // 约10度误差，认为已经朝向目标点
    double dribble_dist_to_line_tolerance = 0.1; // 机器人到“球-目标点”直线的垂直距离允差
    double dribble_face_ball_tolerance = Deg2Rad(20);// 机器人面向球的允差
    double dribble_face_goal_tolerance = Deg2Rad(10);// 机器人面向目标的允差
    double ball_to_target_tolerance = 0.2; // 球到达目标点的允差
    double dribble_kp_angle = 1.5;
    double dribble_kp_linear = 1;
    double dist_to_ball_target_line = 0; // 机器人到球和目标点直线的垂直距离
    double face_ball_angle = 0; // 面向球的目标角度
    double face_ball_angle_error = 0; // 面向球的角度误差
    double face_goal_angle_error = 0;
    double turn_angle_error = 0;// turn 时的角度差，目标-当前
    double turn_angle_tolerance = 0.174;// turn时的允差
    double rotate_Vy = 0.5;    // 旋转时的Y方向速度
    double x_add = 0;
    double yaw_add = 0;
    double dx = 0.3;
    double dyaw = Deg2Rad(10);

    // 踢球用到的参数
    double lateral_offset_left = 0.15;     // 左方横向偏移量
    double longitudinal_offset_back = 0.3;   // 后方纵向偏移量
    double face_current_goal_angle = 0;
    double face_current_goal_angle_error = 0;

    while (ros::ok()) {
        ros::spinOnce();
        ROS_ERROR("the gait type is %d", parameters.stp.gait_type);
        initJoyMsg(); // 初始化joy_msg为默认值
        switch(parameters.stp.gait_type){

            case 0: // 站着
                /* code */
                parameters.stp.vel_cmd = {0,0,0};
                parameters.stp.gait_type = 0;
                break;

            case 1: // 走过去
                /* code */

                // simplewalk();
                generate_velocity(parameters.stp.robot_global,parameters.stp.action_global);
                // parameters.stp.vel_cmd = dwa_planner.computeVelocity(parameters.stp.robot_global, parameters.stp.action_global, parameters.stp.obstacles);
                // 计算向量差值的模长
                {
                double dx = parameters.stp.robot_global[0] - parameters.stp.action_global[0];
                double dy = parameters.stp.robot_global[1] - parameters.stp.action_global[1];
                double dz = parameters.stp.robot_global[2] - parameters.stp.action_global[2];
                dz = AdjustRadRange2(dz);
                double distance = sqrt(dx*dx + dy*dy);
                cout << "distance is " << distance << endl;
                if (distance < 0.3) {
                    parameters.stp.vel_cmd[0] = 0;
                    parameters.stp.vel_cmd[1] = 0;
                }
                if (std::abs(dz) < 0.2) {
                    parameters.stp.vel_cmd[2] = 0;
                }
                if (distance < 0.3 && (std::abs(dz)<0.2)) {
                cout << "enter walk to pos gait!!!\n\n" << endl;
                    parameters.stp.gait_type = 0;
                }

                }
            break;


            case 2: // 走过去+踢球
                /* code */
                face_ball_angle = atan2(parameters.stp.ball_global[1] - parameters.stp.robot_global[1],parameters.stp.ball_global[0] - parameters.stp.robot_global[0]);
                face_ball_angle_error = AdjustRadRange2(face_ball_angle - parameters.stp.robot_global[2]);
                face_goal_angle_error = AdjustRadRange2(parameters.stp.action_global[2]- parameters.stp.robot_global[2]);

                if(parameters.stp.ball_field_distance > close_to_ball){ // 距离球较远，接近球，同时尽量面向球
                    current_goal.clear();
                    // 目标位置为球的位置，朝向为面向球的方向
                    current_goal.push_back(parameters.stp.ball_global[0]);
                    current_goal.push_back(parameters.stp.ball_global[1]);
                    current_goal.push_back(face_ball_angle);

                    generate_velocity(parameters.stp.robot_global,current_goal);
                    ROS_INFO("closing to ball......");
                }
                else if(abs(face_ball_angle_error) > dribble_face_ball_tolerance){ // 已接近球，调整朝向，使得面向球
                    parameters.stp.vel_cmd = {0, 0, 0};
                    parameters.stp.vel_cmd[2] = max(min(abs(face_ball_angle_error) * dribble_kp_angle, MAX_ANGULAR_SPEED), MIN_ANGULAR_SPEED);
                    // 根据误差方向调整旋转方向
                    if(face_ball_angle_error < 0){
                        parameters.stp.vel_cmd[2] = -parameters.stp.vel_cmd[2];
                    }
                    ROS_INFO("facing to ball......");
                }
                else if(abs(face_goal_angle_error) > dribble_face_goal_tolerance){ // 已面向球，但未面向目标方向，调整y使得面向目标方向
                    parameters.stp.vel_cmd = {0, 0, 0};
                    
                    parameters.stp.vel_cmd[0] = 0;
                    parameters.stp.vel_cmd[1] = - abs(face_goal_angle_error) / face_goal_angle_error * rotate_Vy;
                    parameters.stp.vel_cmd[2] = - parameters.stp.vel_cmd[1] / (close_to_ball * 0.75) ;

                    parameters.stp.vel_cmd[0] += x_add;
                    parameters.stp.vel_cmd[2] += yaw_add;

                    ROS_INFO("facing to goal......");
                }
                else if(parameters.stp.ball_field[0] > 0.3){
                    parameters.stp.vel_cmd = {0, 0, 0};
                    parameters.stp.vel_cmd[0] = MIN_LINEAR_SPEED;
                }
                else { 
                    parameters.stp.vel_cmd = {0, 0, 0};
                    joy_msg.x = 1.0; // 小脚踢球
                    ROS_WARN("joy_msg set: x = 1.0");
                }
            break;      

            case 4:// turn
                ROS_INFO("turning");
                turn_angle_error = AdjustRadRange2(parameters.stp.action_global[2]- parameters.stp.robot_global[2]);
                parameters.stp.vel_cmd = {0, 0, 0};
                parameters.stp.vel_cmd[2] = max(min(abs(turn_angle_error) * dribble_kp_angle, MAX_ANGULAR_SPEED), MIN_ANGULAR_SPEED);
                // 根据误差方向调整旋转方向
                if(turn_angle_error < -turn_angle_tolerance){
                    parameters.stp.vel_cmd[2] = -parameters.stp.vel_cmd[2];
                }else if(turn_angle_error < turn_angle_tolerance){
                    parameters.stp.vel_cmd[2] = 0;
                    parameters.stp.gait_type = 0;
                    ROS_INFO("stop turning, go to crouch case...");
                }            
                break;

            case 3: // 盘球
                /* code */
                face_ball_angle = atan2(parameters.stp.ball_global[1] - parameters.stp.robot_global[1],parameters.stp.ball_global[0] - parameters.stp.robot_global[0]);
                face_ball_angle_error = AdjustRadRange2(face_ball_angle - parameters.stp.robot_global[2]);
                face_goal_angle_error = AdjustRadRange2(parameters.stp.action_global[2]- parameters.stp.robot_global[2]);

                if(parameters.stp.ball_field_distance > close_to_ball){ // 距离球较远，接近球，同时尽量面向球
                    current_goal.clear();
                    // 目标位置为球的位置，朝向为面向球的方向
                    current_goal.push_back(parameters.stp.ball_global[0]);
                    current_goal.push_back(parameters.stp.ball_global[1]);
                    current_goal.push_back(face_ball_angle);

                    generate_velocity(parameters.stp.robot_global,current_goal);
                    ROS_INFO("closing to ball......");
                }
                else if(abs(face_ball_angle_error) > dribble_face_ball_tolerance){ // 已接近球，调整朝向，使得面向球
                    parameters.stp.vel_cmd = {0, 0, 0};
                    parameters.stp.vel_cmd[2] = max(min(abs(face_ball_angle_error) * dribble_kp_angle, MAX_ANGULAR_SPEED), MIN_ANGULAR_SPEED);
                    // 根据误差方向调整旋转方向
                    if(face_ball_angle_error < 0){
                        parameters.stp.vel_cmd[2] = -parameters.stp.vel_cmd[2];
                    }
                    ROS_INFO("facing to ball......");
                }
                else if(abs(face_goal_angle_error) > dribble_face_goal_tolerance){ // 已面向球，但未面向目标方向，调整y使得面向目标方向
                    parameters.stp.vel_cmd = {0, 0, 0};
                    
                    parameters.stp.vel_cmd[0] = 0;
                    parameters.stp.vel_cmd[1] = - abs(face_goal_angle_error) / face_goal_angle_error * rotate_Vy;
                    parameters.stp.vel_cmd[2] = - parameters.stp.vel_cmd[1] / close_to_ball;


                    parameters.stp.vel_cmd[0] += x_add;
                    parameters.stp.vel_cmd[2] += yaw_add;

                    ROS_INFO("facing to goal......");
                }
                else { // 面向球且面向目标方向，带球前进
                    parameters.stp.vel_cmd = {0, 0, 0};
                    parameters.stp.vel_cmd[0] = MIN_LINEAR_SPEED * 3;
                    ROS_ERROR("adjust completed, going ahead.....\n\n\n");
                }
                break;        

            default:
                break;
        }

        // ROS_INFO("Current robot_global: %.2f, %.2f, %.2f",
        //          parameters.stp.robot_global[0],
        //          parameters.stp.robot_global[1],
        //          parameters.stp.robot_global[2]);

        // ROS_INFO("Target action_global: %.2f, %.2f, %.2f",
        //          parameters.stp.action_global[0],
        //          parameters.stp.action_global[1],
        //          parameters.stp.action_global[2]);

        velocity_msg.linear.x = parameters.stp.vel_cmd[0];
        velocity_msg.linear.y = parameters.stp.vel_cmd[1];
        velocity_msg.angular.z = parameters.stp.vel_cmd[2];
        // ROS_INFO("Publishing velocity - Linear: (%.3f, %.3f), Angular: %.3f", 
                // velocity_msg.linear.x, velocity_msg.linear.y, velocity_msg.angular.z);
        velocity_pub.publish(velocity_msg);
        ROS_INFO("velocity published: %.2f, %.2f, %.2f", velocity_msg.linear.x, velocity_msg.linear.y, velocity_msg.angular.z);
        
        joy_pub.publish(joy_msg);
        ROS_INFO("joy_msg published");

        loop_rate.sleep();
    }
    
    return 0;
}