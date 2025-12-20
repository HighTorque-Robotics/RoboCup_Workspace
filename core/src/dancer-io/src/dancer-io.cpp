#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <dmsgs/HeadCommand.h>
#include <dmsgs/ActionCommand.h>
#include <dmsgs/BodyCommand.h>
#include <dmsgs/MotionInfo.h>
#include <dmsgs/MotorState.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <string>

class IntegratedNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // 订阅者
    ros::Subscriber rx_pitch_sub_;
    ros::Subscriber rx_yaw_sub_;
    ros::Subscriber imu_sub_;   
    ros::Subscriber tx_sub_;
    ros::Subscriber cmd_vel_sub_;
    
    // 发布者 - 移除 odom_pub_，只保留 motion_pub_
    ros::Publisher motion_pub_;
    ros::Publisher tx_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 状态变量
    double pitch_;
    double pitch_speed_;
    double yaw_;
    double yaw_speed_;
    double vx_;
    double vy_;
    
    // 里程计状态
    double x_;
    double y_;
    double theta_rel_;
    
    // IMU相关
    double imu_roll_;
    double imu_pitch_;
    double imu_yaw_;
    
    // 稳定状态检测
    bool stable_;
    int unstable_count_;
    int stable_threshold_;
    double angle_threshold_;
    ros::Time last_unstable_time_;
    double re_stable_timeout_;

    // 里程计相关
    bool have_yaw0_;
    double yaw0_unwrapped_;
    double yaw_unwrapped_;
    double last_raw_yaw_;
    ros::Time last_time_;
    
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;

    double last_vx_ = 0, last_vy_ = 0;     //梯形法积分

    int robotId;

    dmsgs::MotionInfo motion_info_;

public:
    IntegratedNode() : nh_(), pnh_("~"), stable_(true), unstable_count_(0), have_yaw0_(false) {
        // 参数设置
        pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");
        pnh_.param<bool>("publish_tf", publish_tf_, true);
        pnh_.param<int>("stable_threshold", stable_threshold_, 2);
        pnh_.param<double>("angle_threshold", angle_threshold_, 0.5);
        pnh_.param<double>("re_stable_timeout", re_stable_timeout_, 5);

        // 订阅话题
        if(!pnh_.getParam("RobotId", robotId)) {
            std::cout << "\n\nNo robot ID!!!\n\n"<< std::endl;
            throw std::runtime_error("Can't get robot id");
        }

        rx_pitch_sub_ = nh_.subscribe("/livelybot_real_real/Head_Pitch_controller/state", 1, &IntegratedNode::pitchCallback, this);
        rx_yaw_sub_ = nh_.subscribe("/livelybot_real_real/Head_Yaw_controller/state", 1, &IntegratedNode::yawCallback, this);
        imu_sub_ = nh_.subscribe("/imu/data", 1, &IntegratedNode::imuCallback, this);
        tx_sub_ = nh_.subscribe("/dbehavior_" + std::to_string(robotId) + "/ActionCommand", 1, &IntegratedNode::actionCallback, this);
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &IntegratedNode::cmdVelCallback, this);

        // 发布话题 - 移除 odom_pub_
        motion_pub_ = nh_.advertise<dmsgs::MotionInfo>("/dmotion_" + std::to_string(robotId) + "/MotionInfo", 1);

        tx_pub_ = nh_.advertise<sensor_msgs::JointState>("/pi_plus_absolute", 10);
        
        // 初始化数据
        initializeVariables();
        
        ROS_INFO("Integrated node initialized");
    }

    // 公共方法
    void updateOdom() {
        if (!have_yaw0_) {
            last_time_ = ros::Time::now();
            return;
        }

        const ros::Time now = ros::Time::now();
        const double dt = (now - last_time_).toSec();
        last_time_ = now;
        
        if (dt <= 0.0 || !std::isfinite(dt)) return;

        // 仅在稳定状态下更新里程计
        if (stable_) {
            // 使用梯形法：平均速度
            double avg_vx = (last_vx_ + vx_) / 2.0;
            double avg_vy = (last_vy_ + vy_) / 2.0;

            // 将机体速度转换到世界坐标系
            const double c = std::cos(theta_rel_ + M_PI / 2.0);
            const double s = std::sin(theta_rel_ + M_PI / 2.0);
            const double x_dot = c * vx_ - s * vy_;
            const double y_dot = s * vx_ + c * vy_;

            x_ += x_dot * dt;
            y_ += y_dot * dt;

            last_vx_ = vx_;
            last_vy_ = vy_;
        }

        // 更新 MotionInfo 中的里程计数据
        updateMotionInfoOdometry();
        
        // 发布TF
        if (publish_tf_) {
            publishTF(now);
        }
    }
    
    void publishMotionInfo() {
        motion_info_.timestamp = ros::Time::now();
        // motion_info_.lower_board_connected = true; // 假设下位机连接
        motion_pub_.publish(motion_info_);
    }

private:
    void initializeVariables() {
        pitch_ = 0.0;
        pitch_speed_ = 0.0;
        yaw_ = 0.0;
        yaw_speed_ = 0.0;
        vx_ = 0.0;
        vy_ = 0.0;
        x_ = 0.0;
        y_ = 0.0;
        theta_rel_ = 0.0;
        imu_roll_ = 0.0;
        imu_pitch_ = 0.0;
        imu_yaw_ = 0.0;
        
        stable_ = true;
        unstable_count_ = 0;
        last_unstable_time_ = ros::Time::now();
        last_time_ = ros::Time::now();
        
        yaw0_unwrapped_ = 0.0;
        yaw_unwrapped_ = 0.0;
        last_raw_yaw_ = 0.0;
        
        // 初始化 MotionInfo 的默认值
        motion_info_.stable = true;
        motion_info_.forward_or_backward = false;
        motion_info_.lower_board_connected = true;
        motion_info_.support_stablizer_flag = false;
        motion_info_.status = 0;  // STANDBY状态
        
        // 初始化里程计数据
        motion_info_.odometry.x = 0.0;
        motion_info_.odometry.y = 0.0;
        motion_info_.odometry.z = 0.0;
    }

    void pitchCallback(const dmsgs::MotorState::ConstPtr& msg) {
        pitch_ = msg->pos;
        pitch_speed_ = msg->vel;
        // motion_info_.lower_board_connected = true;

        // 更新MotionInfo的头部姿态
        const double rad2deg = 180.0 / M_PI;
        motion_info_.curPlat.pitch = pitch_ * rad2deg;
        motion_info_.curPlat.pitchSpeed = pitch_speed_ * rad2deg;
    }

    void yawCallback(const dmsgs::MotorState::ConstPtr& msg) {
        yaw_ = msg->pos;
        yaw_speed_ = msg->vel;
        
        // 更新MotionInfo的头部姿态
        const double rad2deg = 180.0 / M_PI;
        motion_info_.curPlat.yaw = yaw_ * rad2deg;
        motion_info_.curPlat.yawSpeed = yaw_speed_ * rad2deg;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 从IMU数据提取欧拉角
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3(q).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
        
        // 处理yaw用于里程计
        processYawForOdom(imu_yaw_);
        
        // 检查稳定性
        checkStability();
        
        // 更新MotionInfo
        updateMotionInfo();
    }
    
    void processYawForOdom(double raw_yaw) {
        if (!std::isfinite(raw_yaw)) return;

        if (!have_yaw0_) {
            // raw_yaw += M_PI / 2.0;  // 初始偏移90度
            yaw0_unwrapped_ = raw_yaw;
            last_raw_yaw_ = raw_yaw;
            yaw_unwrapped_ = raw_yaw;
            have_yaw0_ = true;
            ROS_INFO("Received first IMU yaw. Zeroed heading.");
            return;
        }

        // 计算角度差（处理角度环绕）
        double diff = raw_yaw - last_raw_yaw_;
        if (diff > M_PI) diff -= 2 * M_PI;
        if (diff < -M_PI) diff += 2 * M_PI;
        
        yaw_unwrapped_ += diff;
        last_raw_yaw_ = raw_yaw;
        theta_rel_ = yaw_unwrapped_ - yaw0_unwrapped_;
    }
    
    void checkStability() {
        // 计算角度绝对值
        double roll_abs = std::abs(imu_roll_);
        double pitch_abs = std::abs(imu_pitch_);
        std::cout << "Roll: " << imu_roll_ << ", Pitch: " << imu_pitch_ << std::endl;
        // 如果角度超过阈值
        if (roll_abs > angle_threshold_ || pitch_abs > angle_threshold_) {
            unstable_count_++;
            
            // 如果连续超过阈值达到设定次数
            if (unstable_count_ >= stable_threshold_) {
                stable_ = false;
                // stable_ = true; // 临时设置为true以避免影响里程计
                last_unstable_time_ = ros::Time::now();
                // ROS_WARN("Robot is unstable! Roll: %.2f, Pitch: %.2f", imu_roll_, imu_pitch_);
                if (imu_pitch_ > angle_threshold_) {
                    motion_info_.forward_or_backward = true; // 前倾
                    ROS_WARN("Robot is Forward!\n\n\n");
                } else if (imu_pitch_ < -angle_threshold_) {
                    motion_info_.forward_or_backward = false; // 后倾
                    ROS_WARN("Robot is Backward!\n\n\n");
                }
            }
        } else {
            // 角度恢复正常，重置计数器
            unstable_count_ = 0;
            
            // 检查是否应该重新设置为稳定状态
            if (!stable_) {
                ros::Time current_time = ros::Time::now();
                double time_since_unstable = (current_time - last_unstable_time_).toSec();
                
                if (time_since_unstable > re_stable_timeout_) {
                    stable_ = true;
                    ROS_INFO("Robot has stabilized.");
                }
            }
        }
    }
    
    void updateMotionInfo() {
        motion_info_.timestamp = ros::Time::now();
        
        // 设置稳定状态
        motion_info_.stable = stable_;
        
        // 设置IMU RPY数据
        motion_info_.imuRPY.x = imu_roll_ * 180.0 / M_PI;
        motion_info_.imuRPY.y = imu_pitch_ * 180.0 / M_PI;
        motion_info_.imuRPY.z = imu_yaw_ * 180.0 / M_PI;
        
        // 设置状态（根据机器人行为）
        if (std::abs(vx_) > 0.1 || std::abs(vy_) > 0.1) {
            motion_info_.status = 1;  // WALKING
        } else {
            motion_info_.status = 0;  // STANDBY
        }
    }
    
    void updateMotionInfoOdometry() {
        // 更新MotionInfo中的里程计数据
        motion_info_.odometry.x = 30 * x_;
        motion_info_.odometry.y = 40 * y_;
        motion_info_.odometry.z = 0.0;  // 2D平面，z=0
    }

    void actionCallback(const dmsgs::ActionCommand::ConstPtr& msg) {
        // if (!stable_) {
        //     ROS_WARN_THROTTLE(1.0, "Robot is unstable, ignoring head commands");
        //     return;
        // }
        motion_info_.lower_board_connected = true;

        dmsgs::HeadCommand head_cmd = msg->headCmd;
        const double deg2rad = M_PI / 180.0;
        double pitch_rad = head_cmd.pitch * deg2rad;
        double yaw_rad = head_cmd.yaw * deg2rad;
        double pitchSpeed_deg = head_cmd.pitchSpeed;
        double yawSpeed_deg   = head_cmd.yawSpeed;

        if (std::abs(pitchSpeed_deg) < 0.1)
            return;

        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "head_joints";
        joint_state.name.push_back("head_pitch_joint");
        joint_state.name.push_back("head_yaw_joint");

        if (stable_) {
            joint_state.position.push_back(pitch_rad);
            joint_state.position.push_back(yaw_rad);
        } else if (motion_info_.forward_or_backward) {
            joint_state.position.push_back(-1.2);
            joint_state.position.push_back(0);
        } else {
            joint_state.position.push_back(0.5);
            joint_state.position.push_back(0);   
        }
        
        // 发布 JointState 消息
        tx_pub_.publish(joint_state);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
    }

    void publishTF(const ros::Time& stamp) {
        geometry_msgs::TransformStamped ts;
        ts.header.stamp = stamp;
        ts.header.frame_id = odom_frame_;
        ts.child_frame_id = base_frame_;

        ts.transform.translation.x = x_;
        ts.transform.translation.y = y_;
        ts.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_rel_);
        ts.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_.sendTransform(ts);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "integrated_node");
    
    // 创建节点
    IntegratedNode converter;
    
    // 设置循环频率
    ros::Rate loop_rate(50); // 50Hz
    
    while (ros::ok()) {
        // 处理回调
        ros::spinOnce();
        
        // 更新里程计
        converter.updateOdom();
        
        // 发布MotionInfo
        converter.publishMotionInfo();
        
        // 控制频率
        loop_rate.sleep();
    }
    
    return 0;
}