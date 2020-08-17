#ifndef _SE5035_SE5035_SE5035_H_
#define _SE5035_SE5035_SE5035_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include "boost/thread.hpp"
#include <fcntl.h>

#include"ros/ros.h"
#include <sensor_msgs/LaserScan.h>



class SE5035
{
public:
    union H_to_O
    {
    int o_value;
    unsigned char h_value[4];
    };
    enum Mode {
        TCP=0,
        UDP
    };
public:
    SE5035(ros::NodeHandle* nh);
    ~SE5035();
private:
    void recvThread_function();
    void connectThread_function();
    void timerCallback(const ros::TimerEvent&);

    bool startConnectThread();
    bool stopConnectThread();
    int findString(char* buf,int bufN,char* ch,int chN);
    bool Start();
    bool Stop();
    int Convert(unsigned char low_1,unsigned char high_1,unsigned char low_2,unsigned char high_2);
private:
    ros::NodeHandle m_nh;

    ros::Publisher m_laserPub;
    ros::Timer m_timer;

    //Params
    std::string m_ip;
    std::string m_port;
    std::string m_laserFrame;
    std::string m_laserTopic;
    float m_angleMin;
    float m_angleIncrement;
    float m_angleRange;
    Mode m_mode;

    int m_fd;
    struct sockaddr_in m_udpAddrServ;  
    int m_udpAddrServLength;
    bool m_connectFlag;
    bool m_heart;

    boost::thread* m_recvThread;
    bool m_recvThreadFlag;
    boost::thread* m_connectThread;
    bool m_connectThreadFlag;
    unsigned char m_laserData[1024*20];
};


#endif