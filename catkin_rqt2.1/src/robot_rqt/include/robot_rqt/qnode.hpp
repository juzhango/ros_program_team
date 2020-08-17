/**
 * @file /include/robot_rqt/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_rqt_QNODE_HPP_
#define robot_rqt_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include "scenetable.h"
#include "sensor_msgs/LaserScan.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_rqt {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();


  std::vector<ST_SCENE> getScenesInput(){return this->scenes_input;}
  void setScanTopic(std::string topic){this->str_scanTopic = topic;}
  std::string get_inputParamName(){return this->inputParamName;}
  void set_inputParamName(std::string  str){this->inputParamName = str;}

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
  void get_scan(QPolygon);

private:
	int init_argc;
	char** init_argv;
  std::string str_scanTopic;
	ros::Publisher chatter_publisher;
  ros::Subscriber scan_sub;
    QStringListModel logging_model;
  std::string inputParamName;

  std::vector<ST_SCENE> scenes_input;


  void scan_callBack(const sensor_msgs::LaserScan& msg);

};

}  // namespace robot_rqt

#endif /* robot_rqt_QNODE_HPP_ */
