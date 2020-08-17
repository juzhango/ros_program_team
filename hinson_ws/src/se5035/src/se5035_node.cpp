#include"ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <se5035/se5035.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv,"se5035_node");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");
    ros::Rate rate(30);
    SE5035 m_SE5035(&n);
   
    ROS_INFO("Start se5035_node");
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
