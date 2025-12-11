#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

// ------------- DWA算法参数配置-------------
struct DWAConfig {
    // 速度限制
    double max_vx = 1.5;  // 最大x方向速度（m/s）
    double min_vx = 0.4;  // 最小x方向速度（m/s）
    double max_vy = 1.0;  // 最大y方向速度（m/s）
    double min_vy = 0.4;  // 最小y方向速度（m/s）
    double max_vw = 1.0;  // 最大角速度
    double min_vw = 0.5;  // 最小角速度
    
    // 加速度限制
    double max_acc_vx = 0.8;  // x方向最大加速度（m/s²）
    double max_acc_vy = 0.6;  // y方向最大加速度（m/s²）
    double max_acc_vw = 0.7;  // 角加速度限制（rad/s²）
    
    // 采样参数
    int vx_samples = 10;    // x方向速度采样数量
    int vy_samples = 10;    // y方向速度采样数量
    int vw_samples = 10;    // 角速度采样数量
    double dt = 0.1;         // 时间步长（s）
    double predict_time = 0.5;  // 轨迹预测时间（s）
    
    // 评价函数权重（拆分为4个独立代价：目标距离+朝目标点+目标朝向+避障+速度）
    double weight_goal = 2.2;    // 目标距离权重
    double weight_dir = 1.6;     // 朝着目标点走的权重
    double weight_theta = 1.6;   // 目标朝向对齐的权重
    double weight_vel = 1.0;     // 速度大小权重
    double weight_obs = 0;     // 障碍物距离权重(因为现在没有障碍物，所以暂时先设置成0)

    // 安全距离（含机器人自身半径，m）
    double safe_dist = 0.5;

    double deceleration_dist = 3;
    double deceleration_angle_error = 0.52;
};

// 轨迹点结构体
struct TrajectoryPoint {
    Eigen::Vector3d pose;  // 位姿（x,y,theta）- 全局坐标系
};

// ------------- DWA规划器类-------------
class DWAPlanner {
private:
    DWAConfig config;                // DWA参数配置
    std::vector<Eigen::Vector3d> obstacles;  // 障碍物列表（x,y,radius）
    Eigen::Vector3d last_vel = Eigen::Vector3d::Zero(); // 存储上一次的速度，默认为0

    // 角度归一化到[-π, π]
    double normalizeAngle(double angle);

    // 计算动态窗口（可行速度空间）：输入当前速度，输出候选速度列表
    std::vector<Eigen::Vector3d> calculateDynamicWindow();

    // 预测轨迹：输入当前位姿、候选速度，输出预测轨迹
    std::vector<TrajectoryPoint> predictTrajectory(
        const Eigen::Vector3d& robot_pose,  // 机器人当前位姿（x,y,theta）
        const Eigen::Vector3d& candidate_vel  // 候选速度（vx,vy,vw）
    );

    // 碰撞检测：输入轨迹，判断是否碰撞
    bool isCollisionFree(const std::vector<TrajectoryPoint>& trajectory);

    // 评价轨迹：综合目标、朝目标点、目标朝向、速度、避障评分（拆分为独立代价）
    double evaluateTrajectory(
        const std::vector<TrajectoryPoint>& trajectory,
        const Eigen::Vector3d& goal_pose,  // 目标位姿（x,y,theta）
        const Eigen::Vector3d& candidate_vel  // 候选速度（vx,vy,vw）
    );

public:
    // 构造函数（传入参数配置）
    DWAPlanner(const DWAConfig& cfg);

    // 接口
    std::vector<double> computeVelocity(
        const std::vector<double>& robot_pose_vec,    // 输入：机器人位姿向量 [x, y, theta]
        const std::vector<double>& goal_pose_vec,     // 输入：目标位姿向量 [x, y, theta]
        const std::vector<std::vector<double>>& obstacles_vec  // 输入：障碍物列表，每个障碍物为 [x, y, radius]
    );
};

#endif // DWA_PLANNER_H
