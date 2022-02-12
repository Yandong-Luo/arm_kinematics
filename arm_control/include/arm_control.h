#ifndef ARM_CONTROL
#define ARM_CONTROL
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
using namespace std;

class ControlNode
{
    public:
    ControlNode(ros::NodeHandle& nh);

    bool receive_data;

    void Update();

    private:
    // 发布节点
    ros::Publisher joint1_pub;
    ros::Publisher joint2_pub;
    ros::Publisher joint3_pub;
    ros::Publisher joint4_pub;
    ros::Publisher joint5_pub;
    ros::Publisher joint6_pub;

    ros::Publisher *joint_pub[6];

    // 订阅节点
    ros::Subscriber joints_subscriber;

    

    // 订阅的回调函数
    void JointsStateCallBack(const sensor_msgs::JointState& joint_state_msg);

    sensor_msgs::JointState publish_position;
};

#endif