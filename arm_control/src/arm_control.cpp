#include <iostream>
#include <arm_control.h>


ControlNode::ControlNode(ros::NodeHandle& nh)
{
    double init_theta1;
    nh.getParam("init_theta1",init_theta1);
    cout<<"init_theta1:"<<init_theta1<<endl;

    // cout<<"Hello!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! !!!!!"<<endl;

    // 订阅
    joints_subscriber = nh.subscribe("/joint_states",5,&ControlNode::JointsStateCallBack, this); 
    
    // 控制关节的发布节点
    joint1_pub = nh.advertise<std_msgs::Float64>("/kuka_arm/joint1_arm_controller/command",5);
    joint2_pub = nh.advertise<std_msgs::Float64>("/kuka_arm/joint2_arm_controller/command",5);
    joint3_pub = nh.advertise<std_msgs::Float64>("/kuka_arm/joint3_arm_controller/command",5);
    joint4_pub = nh.advertise<std_msgs::Float64>("/kuka_arm/joint4_arm_controller/command",5);
    joint5_pub = nh.advertise<std_msgs::Float64>("/kuka_arm/joint5_arm_controller/command",5);
    joint6_pub = nh.advertise<std_msgs::Float64>("/kuka_arm/joint6_arm_controller/command",5);

    joint_pub[0] = &joint1_pub;
    joint_pub[1] = &joint2_pub;
    joint_pub[2] = &joint3_pub;
    joint_pub[3] = &joint4_pub;
    joint_pub[4] = &joint5_pub;
    joint_pub[5] = &joint6_pub;

    receive_data = false;

    

    // joint_commands_1_pub_ = nh.advertise<std_msgs::Float64>("/kuka/link_1_controller/command", 5);

}

void ControlNode::JointsStateCallBack(const sensor_msgs::JointState& joint_state_msg)
{   
    receive_data = true;
    publish_position = joint_state_msg;
    // for(int i=0;i<6;i++){
    //     cout<<"publishing the"<< i+1 <<"joint position:"<<publish_position.position[i]<<endl;
    // }
}

void ControlNode::Update()
{
    std_msgs::Float64 joint_position;
    ros::Rate loop_rate(30);
    for(int i=0;i<6;i++)
    {
        joint_position.data = publish_position.position[i+2];
        joint_pub[i]->publish(joint_position);
        loop_rate.sleep();
        // cout<<"pub:"<<joint_position.data<<endl;
    }
    

    // std_msgs::Float64 joint2_position;
    // joint1_position.data = publish_position.position[0];
    // joint2_pub.publish(publish_position.position[1]);
    // joint3_pub.publish(publish_position.position[2]);
    // joint4_pub.publish(publish_position.position[3]);
    // joint5_pub.publish(publish_position.position[4]);
    // joint6_pub.publish(publish_position.position[5]);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"arm_controller");

    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    ControlNode control_node(nh);

    while (ros::ok())
    {
        ros::spinOnce();
        if(control_node.receive_data)
        {
            control_node.Update();
        }
        loop_rate.sleep();
    }
    
}