/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/syscore_rqt_regions/qnode.hpp"

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "XmlRpc.h"

#include "../include/syscore_rqt_regions/input_format_yaml.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace syscore_rqt_regions {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"syscore_rqt_regions");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.

  scan_sub = n.subscribe(this->str_scanTopic.data(),1000,&QNode::scan_callBack,this);

  // input avoid config
  XmlRpc::XmlRpcValue config;
  bool found_avoid_config = n.getParam(get_inputParamName(),config);
  if(found_avoid_config){
    ROS_INFO("I GET CONFIG");
    if(checkAvoidConfigsValidate(config, n.getNamespace())){
      ROS_INFO("Validate CONFIG");
      this->scenes_input = getVectorScene(config);
    }
  }
  else{
    ROS_INFO("NONE CONFIG");
  }

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"syscore_rqt_regions");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.

  scan_sub = n.subscribe(this->str_scanTopic.data(),1000,&QNode::scan_callBack,this);

  // input avoid config
  XmlRpc::XmlRpcValue config;
  qDebug()<< "enter getParam()";
  bool found_avoid_config = n.getParam(get_inputParamName(),config);
  if(found_avoid_config){
    ROS_INFO("I GET CONFIG");
    if(checkAvoidConfigsValidate(config, n.getNamespace())){
      ROS_INFO("Validate CONFIG");
      this->scenes_input = getVectorScene(config);
    }
  }
  else{
    ROS_INFO("NONE CONFIG");
  }

	start();
	return true;
}

void QNode::run() {
  log(Info,std::string("ros node running"));
	ros::Rate loop_rate(1);
	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();

	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


/**
  * @brief  topic callBack
  * @param  &msg
  * @retval None
  */
void QNode::scan_callBack(const sensor_msgs::LaserScan &msg)
{
  std::vector <float> vectorPolar = msg.ranges;
  QPolygon polygon;
  float interval = (msg.angle_max-msg.angle_min)/vectorPolar.size();    //=== 每个极点弧度间隔
  QPoint cartesianPoint;           // 直角坐标点

  for(int i=0; i<vectorPolar.size(); i++)
  {
    // 过滤掉msg.angle< 0.01的值
    if(vectorPolar.at(i) < 0.01){
      vectorPolar.at(i) = 10;
    }
    // 极坐标数组-》笛卡尔直角坐标数组
    cartesianPoint = QPoint(vectorPolar.at(i)*cos(msg.angle_min+(i*interval))*100, vectorPolar.at(i)*sin(msg.angle_min+(i*interval))*100);
    polygon.push_back(cartesianPoint);
  }
  Q_EMIT get_scan(polygon);
}




}  // namespace syscore_rqt_regions
