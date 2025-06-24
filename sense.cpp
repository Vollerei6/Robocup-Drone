#include <ros/ros.h>
#include <mavros_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include "Drone/poseposition.h"
//work_status机制：
//0 表示等待起飞
//1 表示正在起飞
//2 表示正在寻找标靶
//3 表示正在前往标靶处
//4 表示正在投弹
//5 表示投弹完毕，正在前往障碍区域
//6 表示正在避障
//7 表示正在降落

//以下部分使用参数服务器，设置一个全局的work_status变量
ros::param::set("work_status",0);//初始为0，表示未起飞

//以下部分提供服务，用于提供位姿信息
ros::ServiceServer poseposition_server = nh.advertiseService("poseposition",poseposition_serve);

//以下部分负责获取px4飞控信息
mavros_msgs::PoseStamped px4_pose_data;//位姿信息
ros::Subscriber px4_pose_sub = nh.subscribe<mavros_msgs::PoseStamped>("/mavros/local_position/odom",10,save_px4_pose_data);
mavros_msgs::State px4_state_data;//飞行模式信息
ros::Subscriber px4_mode_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,save_px4_mode_data);


void save_px4_data(const mavros_msgs::PoseStamped::ConstPtr& data_p){
    px4_pose_data = *data_p;//直接采用拷贝构造，把消息类全部复制到px4_data
    if(nh.getParam("work_status",-1) == 1 && px4_pose_data.pose.position.z >=3) ros::param::set("work_status",2);//1表示正在起飞，2表示起飞成功
}

void save_px4_mode_data(const mavros_msgs::PoseStamped::ConstPtr& data_p){
    px4_mode_data = *data_p;
    if(nh.getParam("work_status",-1) == 0 && px4_mode_data.current_state.mode=="OFFBOARD") ros::param::set("work_status",1);//0表示等待起飞，1表示开始起飞
}

void poseposition_serve(Drone::poseposition::Request& req,
                        Drone::poseposition::Response& resp){
    resp.x = px4_pose_data.pose.position.x;
    resp.y = px4_pose_data.pose.position.y;
    resp.z = px4_pose_data.pose.position.z;
    double Q[4] = {
                px4_pose_data.pose.orientation.w;
                px4_pose_data.pose.orientation.x,
                px4_pose_data.pose.orientation.y,
                px4_pose_data.pose.orientation.z
                };
    resp.yaw = atan2(2*(Q[0]*Q[3]+Q[1]*Q[2]),1-2*(Q[2]*Q[2]+Q[3]*Q[3]));//四元数转化为yaw；
}//用于返回x/y/z/yaw

int main(int argc,char *argv[])
{
    //初始化与基本设置
    ros::init(argc,argv,"control");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    //不断读取回调函数
    ros::spin();
}