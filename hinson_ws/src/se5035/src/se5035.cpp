#include <se5035/se5035.h>

SE5035::SE5035(ros::NodeHandle* nh)
{
    m_nh=(*nh);

    m_fd=-1;
    m_connectFlag=false;
    m_heart=false;
    m_connectThreadFlag=false;
    m_connectThread=NULL;

    //加载参数
    std::string mode;
    m_nh.param("SE5035_IP",m_ip,std::string("192.168.1.88"));
    m_nh.param("SE5035_PORT",m_port,std::string("8080"));
    m_nh.param("SE5035_Frame",m_laserFrame,std::string("laser"));
    m_nh.param("SE5035_Topic",m_laserTopic,std::string("scan"));

    m_nh.param("SE5035_Mode",mode,std::string("TCP"));

    std::cout<<"SE5035_IP:"<<m_ip<<"\n";
    std::cout<<"SE5035_PORT:"<<m_port<<"\n";
    std::cout<<"SE5035_Frame:"<<m_laserFrame<<"\n";
    std::cout<<"SE5035_Topic:"<<m_laserTopic<<"\n";
    std::cout<<"SE5035_Mode:"<<mode<<"\n";

    
    if(std::string("UDP")==mode)
    {
        m_mode=UDP;
    }
    else
    {
        m_mode=TCP;
    }
    m_angleMin=45.0/180*3.14;
    m_angleRange=270.0/180*3.14;//角度范围位270
    m_angleIncrement=0.1/180*3.14;//角度分辨率默认0.1

    m_laserPub=m_nh.advertise<sensor_msgs::LaserScan>("scan",10);

    //启动接收线程
    m_recvThreadFlag=true;
    m_recvThread=new boost::thread(boost::bind(&SE5035::recvThread_function,this));
    m_timer = m_nh.createTimer(ros::Duration(2), &SE5035::timerCallback,this);

}
SE5035::~SE5035()
{
    if(-1!= m_fd)
    {
        close(m_fd);
        m_fd=-1;
    } 
    stopConnectThread();

    m_recvThreadFlag=false;
    if(NULL!=m_recvThread)
    {
        m_recvThread->join();
        delete m_recvThread;
        m_recvThread=NULL;
    }
}


void SE5035::recvThread_function()
{
    int dataNum=0;
    unsigned char buf[1024*20];
    char head[5]={0x43,0x6c,0x6f,0x75,0x64};
    double start_angle,end_angle,pointCount;
    unsigned char laser_buf[1024*20];
    int laser_Num=0;
    int laserPointCount=2700;
    double angleIncrement=0.1;
 
    while(m_recvThreadFlag)
    {
        if(true==m_connectFlag)
        {
            //读取数据
            if(TCP==m_mode)//TCP模式
            {   
                dataNum=::read(m_fd,buf,1024*20);
            }
            else if(UDP==m_mode)//UDP模式
            {
                dataNum = recvfrom(m_fd, buf, 1024*20, 0, (struct sockaddr *)&m_udpAddrServ, (socklen_t *)&m_udpAddrServLength);  
            }

            if(dataNum >0 ) //提取数据
            {
               
                int n=findString((char*)buf,dataNum,head,5);
                for(int i=0;i<dataNum;i++)
                {
                    if(i==n)
                    {
                        start_angle=Convert(buf[n+7],buf[n+6],0,0);
                        end_angle=Convert(buf[n+9],buf[n+8],0,0);
                        int cout=Convert(buf[n+11],buf[n+10],0,0);
                        // printf("start_angle:%f   end_angle:%f  cout:%d\n",start_angle,end_angle,cout);
                        if(45==start_angle)
                        {
                            angleIncrement=(end_angle-start_angle)/((double)cout);
                            laserPointCount=2700 / ((int)(angleIncrement*10));
                            laser_Num=0;
                        }

                        i+=12;
                        if(i>=dataNum)
                        {
                            break;
                        }
                    }
                    // std::cout<<"laserPointCount:"<<laserPointCount<<"\n";
                    laser_buf[laser_Num]=buf[i];
                    laser_Num++;
                    if(laserPointCount*4<=laser_Num)
                    {
                        m_heart=true;

                        sensor_msgs::LaserScan msg;
                        msg.header.frame_id = m_laserFrame;
                        msg.header.stamp = ros::Time::now();

                        msg.angle_min = m_angleRange/(-2.0);
                        msg.angle_max = m_angleRange/2.0;

                        msg.angle_increment = angleIncrement/180.0*3.14;
                        msg.range_min = 0;
                        msg.range_max = 50000;
                        msg.ranges.resize(laserPointCount);
                        msg.intensities.resize(laserPointCount);
                        // std::cout<<"msg.angle_min:"<<msg.angle_min<<"\n";
                        // std::cout<<"msg.angle_max:"<<msg.angle_max<<"\n";
                        // std::cout<<"msg.angle_increment:"<<angleIncrement<<"\n";
                        // std::cout<<"laserPointCount:"<<laserPointCount<<"\n";
                        // std::cout<<"..................\n";
                        ROS_INFO("increment:%f   laserPointCount:%d",angleIncrement,laserPointCount);
                        if(0.1!=angleIncrement || 2700 != laserPointCount)
                        {
                            ROS_ERROR("angleIncrement:%f  laserPointCount:%d",angleIncrement,laserPointCount);
                            //sleep(10000);
                        }
                        for(int i=0;i<laserPointCount;i++)
                        {
                            float dis=Convert(laser_buf[i*4],laser_buf[i*4+1],0,0)/1000.0f;
                            float str=Convert(laser_buf[i*4+2],laser_buf[i*4+3],0,0);
                            msg.ranges[i] = dis;
                            msg.intensities[i] = str;
                            // if(i==laserPointCount/2)
                            // {
                            //     ROS_INFO("%f %f",dis,str);
                            // }
                        }   
                        m_laserPub.publish(msg);
                        laser_Num=0;    
                    }
                    
                }
            }
        }
        else
        {
            laser_Num=0;
        }
        sleep(0.01);
        
    }
}

void SE5035::connectThread_function()
{
    char* IP=(char*)m_ip.c_str();
    unsigned short port=std::atoi(m_port.c_str());
    ROS_INFO("Socket Connect %s %d!\n",IP,port);
    if(TCP==m_mode)
    {
        int fd=0;
        struct sockaddr_in s_add;
        fd=socket(AF_INET,SOCK_STREAM,0);
        if(-1==fd)
        {
            ROS_ERROR("Socket Connect Error:%s %d",m_ip.c_str(),std::atoi(m_port.c_str()));
            return ;
        }

        bzero(&s_add,sizeof(struct sockaddr_in));
        s_add.sin_family=AF_INET;
        s_add.sin_addr.s_addr=inet_addr(IP);
        s_add.sin_port=htons(port);

        //非阻塞
        int flags=fcntl(fd,F_GETFL,0);
        fcntl(fd, F_SETFL, flags|O_NONBLOCK);
        int n=-1;
        while(m_connectThreadFlag)
        {
            n=connect(fd,(struct sockaddr*)(&s_add),sizeof(struct sockaddr));
            sleep(0.1);
            if(0==n)
            {
                m_fd=fd;
                Start();
                m_connectFlag=true;
                ROS_INFO("Socket Connect Success!\n");
                break;
            }
        }
        
    }
    else if(UDP==m_mode)
    {
        int fd=0;
        /* 建立udp socket */  
        fd = socket(AF_INET, SOCK_DGRAM, 0);  
        if(fd < 0)  
        {  
            return ;  
        } 
        memset(&m_udpAddrServ, 0, sizeof(m_udpAddrServ));  
        m_udpAddrServ.sin_family = AF_INET;  
        // m_udpAddrServ.sin_addr.s_addr = inet_addr(IP);  
        m_udpAddrServ.sin_addr.s_addr = htonl(INADDR_ANY);  
        m_udpAddrServ.sin_port = htons(port);  
        m_udpAddrServLength=sizeof(m_udpAddrServ);

         //非阻塞
        int flags=fcntl(fd,F_GETFL,0);
        fcntl(fd, F_SETFL, flags|O_NONBLOCK);

        int n=-1;

        while(m_connectThreadFlag)
        {
            n=bind(fd, (struct sockaddr *)&m_udpAddrServ, sizeof(m_udpAddrServ));
            sleep(0.1);
            if(0==n)
            {
                m_fd=fd;
                Start();
                m_connectFlag=true;
                // ROS_INFO("Socket Connect Success !\n");
                break;
            }
        }
        
    }

}

void SE5035::timerCallback(const ros::TimerEvent&)
{
   
    if(true==m_heart)
    {
        m_heart=false;
    }
    else
    {
        m_connectFlag=false;
        if(-1!= m_fd)
        {
            close(m_fd);
            m_fd=-1;
        }

        stopConnectThread();
        startConnectThread();
    }

}

bool SE5035::startConnectThread()
{
    m_connectThreadFlag=true;
    m_connectThread=new boost::thread(boost::bind(&SE5035::connectThread_function,this));
    return true;
}
bool SE5035::stopConnectThread()
{
    m_connectThreadFlag=false;
    if(NULL!=m_connectThread)
    {
        m_connectThread->join();
        delete m_connectThread;
        m_connectThread=NULL;
    }
    return true;
}

int SE5035::findString(char* buf,int bufN,char* ch,int chN)
{
    int dst=-1;
    if(bufN>=chN)
    {
        for(int i=0;i<bufN-chN;i++)
        {
            int j=0;
            for(j=0;j<chN;j++)
            {
                if(buf[i+j]!=ch[j])
                {
                    break;
                }
            }
            if(j>=chN)
            {
               dst=i; 
            }
        }
    }
    return dst;
}


bool SE5035::Start()
{
    if(-1!=m_fd)
    {
        unsigned char startFrame[8]={0x52,0x41,0x75,0x74,0x6F,0x01,0x87,0x80}; 
        //发送开始帧
        if(TCP==m_mode)
        {
            send(m_fd,startFrame,8,0);
        }
        else if(UDP==m_mode)
        {
            unsigned int port=std::atoi(m_port.c_str());
            struct sockaddr_in udpAddrServ;
            udpAddrServ.sin_family = AF_INET;  
            udpAddrServ.sin_addr.s_addr = inet_addr(m_ip.c_str());  
            udpAddrServ.sin_port = htons(port);  
            sendto(m_fd, startFrame,8, 0, (struct sockaddr *)&udpAddrServ, sizeof(udpAddrServ));  
        } 
    }
    else
    {
        return false;
    }
    return true;
}
bool SE5035::Stop()
{
    if(-1!=m_fd)
    {
       //发送停止帧 
        unsigned char stopFrame[8]={0x52,0x41,0x75,0x74,0x6F,0x00,0x46,0x40}; 
        if(TCP==m_mode)
        {
            send(m_fd,stopFrame,8,0);
        }
        else if(UDP==m_mode)
        {
            unsigned int port=std::atoi(m_port.c_str());
            struct sockaddr_in udpAddrServ;
            udpAddrServ.sin_family = AF_INET;  
            udpAddrServ.sin_addr.s_addr = inet_addr(m_ip.c_str());  
            udpAddrServ.sin_port = htons(port);  
            sendto(m_fd, stopFrame,8, 0, (struct sockaddr *)&udpAddrServ,sizeof(udpAddrServ));  
        } 
       
    }
    else
    {
        return false;
    }
    return true;
}

int SE5035::Convert(unsigned char low_1,unsigned char high_1,unsigned char low_2,unsigned char high_2)
{
    H_to_O h_to_o;
    h_to_o.h_value[0]=low_1;
    h_to_o.h_value[1]=high_1;
    h_to_o.h_value[2]=low_2;
    h_to_o.h_value[3]=high_2;
    return h_to_o.o_value;
}
