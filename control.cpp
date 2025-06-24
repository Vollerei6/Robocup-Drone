#include <ros/ros.h>
#include <mavros_msgs/PoseStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include "drone/target.h"
//引入一种自定义信息target
//例如——
//float64 coordX
//float64 coordY
//int target_type //表示目标点的种类
//理论上只需要平面相对坐标就够了
#include "drone/poseposition.h"
//该服务用于获取位姿信息，包括x/y/z/yaw

//以下部分负责向飞控发送移动信息
ros::Publisher move_pub = nh.advertise<mavros_msgs::PoseStamped>("/mavros/setpoint_position/local",10);//用于发送移动指令
mavros_msgs::PoseStamped move_msg;

//以下部分负责向飞控发送投放货物信息
ros::Publisher throw_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",10);//用于发送投掷指令
mavros_msgs::OverrideRCIn throw_msg;

//该部分负责接收视觉发来的定位消息
ros::Subscriber destination_sub = nh.subscribe<drone::target>("target",10,doTarget);

//该部分负责向感知结点发出服务请求，以获取local坐标系的坐标值&yaw航偏角等数据
ros::ServiceClient poseposition_client = nh.serviceClient<drone::poseposition>("poseposition");

bool isThrowed[4] = {false,false,false,false};//表示是否被投放过，依次对应1.0 - 1.5 - 2.0 - 2.5系数

void doTarget(const drone::target::ConstPtr& msg_p){
    /*
    if(targetType == 0) return;
    if(distance(msg_p->x_local,msg_p->y_local,msg_p->z_local) < 阈值) throw();
    else 移动到该点
    */
}

void takeoff()
{
    move_msg.header.stamp = ros::Time::now();
    move_msg.header.frame_id = "map";
    move_msg.pose.position.x = 1;
    move_msg.pose.position.y = 1;
    move_msg.pose.position.z = 1;
    move_pub.publish(move_msg);
}//用于起飞

void _throw()//throw是c++关键字，所以只好在前面加个_了
{
    for (int i = 0; i < 18; i++) {
        throw_msg.channels[i] = 0;
    }//初始化
    //由于我们暂时不知道无人机设计，因此我们假设AUX1控制投放货物，PWM值设置为1900/20000
    throw_msg.channels[4] = 1900;
    throw_msg.header.stamp = ros::Time::now();
    throw_pub.publish(throw_msg);//发送
}//用于投放货物

int main(int argc,char *argv[]){
    //初始化与基本设置
    ros::init(argc,argv,"control");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    while(ros::ok()){
        switch(nh.getParam("work_status",-1)){//work_status表示工作状态
        case 0://初始状态，等待起飞
            break;
        case 1://起飞
            takeoff();
            break;
        case 2://起飞成功之后
            //TODO:设计这一部分的代码
        }
        rate.sleep();//注意——在offboard模式下，必须持续提供MAVLink信息，才能让飞控保持Offboard模式，否则会自动退出！
    }
    return 0;
}

/*编辑msg信息的示例——
    move_msg.header.stamp = ros::Time::now();
    move_msg.header.frame_id = "map";
    move_msg.pose.position.x = 1;
    move_msg.pose.position.y = 1;
    move_msg.pose.position.z = 1;
*/