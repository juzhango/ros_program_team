#include "../include/robot_rqt/pointwidget.h"
#include <QDebug>
#include "../include/robot_rqt/common.h"
#include "ros/ros.h"

PointWidget::PointWidget(QWidget *parent) : QWidget(parent)
{
  setMinimumSize(1000,720);
  setMaximumSize(1000,720);

  originPos = QPoint(this->width()*0.5,this->height()*0.7);

  ROS_INFO("Origin_qt: %d, %d", originPos.x(), originPos.y());


}
QPoint PointWidget::pointToPoint_qt(QPoint point)
{
  QPoint point_qt;
  point_qt = QPoint(originPos.x()-point.y(),originPos.y()-point.x());
  return point_qt;
}
QPoint PointWidget::point_qtToPoint(QPoint point_qt)
{
  QPoint point;
  point_qt = QPoint(originPos.y()-point.y(),originPos.x()-point.x());
  return point;
}
/**
  * @brief  turn polygon to polygon_qt
  * @param  polygon:
  * @retval polygon_qt
  */
QPolygon PointWidget::polygonToPolygon_qt(QPolygon polygon)
{
  QPolygon polygon_qt;
  for(int i = 0; i < polygon.size(); i++){
    QPoint point_qt = pointToPoint_qt(polygon.at(i));
    polygon_qt.push_back(point_qt);
  }
  return polygon_qt;
}
QPolygon PointWidget::polygon_qtToPolygon(QPolygon polygon_qt)
{
  QPolygon polygon;
  for(int i = 0; i < polygon_qt.size(); i++){
    QPoint point = point_qtToPoint(polygon_qt.at(i));
    polygon.push_back(point);
  }
  return polygon;
}

void PointWidget::mousePressEvent(QMouseEvent *ev)        //单击
{
  QPoint point = QPoint(originPos.y() - ev->y(), originPos.x()-ev->x());
  setPoint(point,QPoint(ev->x(),ev->y()));
  update();
}

void PointWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);

  this->drawBox(&painter);
  this->drawClickPoint(&painter);
  this->drawMarkPoint(&painter);
  this->drawMarkPolygon(&painter);
  this->drawLaserDev(&painter);
  this->drawLaserPolygon(&painter,pointCloud);
  this->drawInnerPolygons(&painter,inner_area);
  this->drawMidPolygons(&painter,mid_area);
  this->drawOutPolygons(&painter,out_area);
}
void PointWidget::drawBox(QPainter *painter)
{
  painter->setPen(Qt::black);
  painter->drawLine(QPoint(1,1),QPoint(1,this->height()-1));
  painter->drawLine(QPoint(1,1),QPoint(this->width()-1,1));
  painter->drawLine(QPoint(this->width()-1,this->height()-1),QPoint(1,this->height()-1));
  painter->drawLine(QPoint(this->width()-1,this->height()-1),QPoint(this->width()-1,1));
}
void PointWidget::drawClickPoint(QPainter *painter)
{
  painter->setPen(Qt::red);
  painter->drawLine(QPoint(clickPoint_qt.x()-3,clickPoint_qt.y()), QPoint(clickPoint_qt.x()+3,clickPoint_qt.y()));
  painter->drawLine(QPoint(clickPoint_qt.x(),clickPoint_qt.y()-3), QPoint(clickPoint_qt.x(),clickPoint_qt.y()+3));
  painter->drawEllipse(clickPoint_qt,6,6);
}
void PointWidget::drawMarkPoint(QPainter *painter)
{
  painter->setPen(Qt::black);
  painter->drawLine(QPoint(markPoint_qt.x()-2,markPoint_qt.y()), QPoint(markPoint_qt.x()+2,markPoint_qt.y()));
  painter->drawLine(QPoint(markPoint_qt.x(),markPoint_qt.y()-2), QPoint(markPoint_qt.x(),markPoint_qt.y()+2));
  painter->drawEllipse(markPoint_qt,5,5);
}
void PointWidget::drawMarkPolygon(QPainter *painter)
{
  painter->setPen(QPen(Qt::black,2));
  painter->drawPolygon(markPolygon_qt);
}

void PointWidget::drawLaserDev(QPainter *painter)
{
  painter->setPen(Qt::black);
  painter->drawEllipse(originPos,8,8);
}
/**
  * @brief
  * @param  *painter:
  * @param  polygon:
  * @retval None
  */
void PointWidget::drawLaserPolygon(QPainter *painter, QPolygon polygon)
{
  QPolygon po = polygon;
  po.push_back(this->originPos);
  painter->setPen(Qt::black);
  painter->drawPolygon(po);
}

void PointWidget::drawInnerPolygons(QPainter *painter,std::vector<QPolygon> inner_area)
{
  painter->setPen(Qt::red);
  for(int i=0; i < inner_area.size(); i++){
    QPolygon polygon = inner_area.at(i);
    painter->drawPolygon(polygon);
  }
}
void PointWidget::drawMidPolygons(QPainter *painter,std::vector<QPolygon> mid_area)
{
  painter->setPen(QColor(254,153,51));
  for(int i=0; i < mid_area.size(); i++){
    QPolygon polygon = mid_area.at(i);
    painter->drawPolygon(polygon);
  }
}
void PointWidget::drawOutPolygons(QPainter *painter,std::vector<QPolygon> out_area)
{
  painter->setPen(Qt::blue);
  for(int i=0; i < out_area.size(); i++){
    QPolygon polygon = out_area.at(i);
    painter->drawPolygon(polygon);
  }
}
/**
  * @brief  set area_vector from sceneTable->scenes..
  * @param  scene: a scene from sceneTable Class.
  * @retval None
  */
void PointWidget::drawShowScene(ST_SCENE scene)
{
  inner_area.clear();
  mid_area.clear();
  out_area.clear();
  std::string str = "polygon";
  for(int j = 0; j < scene.areas.area_inner.size(); j++){
    QPolygon polygon_qt = polygonToPolygon_qt(scene.areas.area_inner[str+std::to_string(j+1)]);
    inner_area.push_back(polygon_qt);
  }
  for(int j = 0; j < scene.areas.area_mid.size(); j++){
    QPolygon polygon_qt = polygonToPolygon_qt(scene.areas.area_mid[str+std::to_string(j+1)]);
    mid_area.push_back(polygon_qt);
  }
  for(int j = 0; j < scene.areas.area_out.size(); j++){
    QPolygon polygon_qt = polygonToPolygon_qt(scene.areas.area_out[str+std::to_string(j+1)]);
    out_area.push_back(polygon_qt);
  }
  update();
}

/**
  * @brief
  * @param  point
  * @param  point_qt
  * @retval None
  */
void PointWidget::setPoint(QPoint point,QPoint point_qt)
{
  clickPoint = point;
  clickPoint_qt = point_qt;
}
/**
  * @brief
  * @param  point_qt
  * @retval None
  */
void PointWidget::setMarkPoint(QPoint point_qt)
{
  markPoint_qt = point_qt;
}

void PointWidget::setMarkPolygon(QPolygon polygon_qt)
{
  markPolygon_qt = polygon_qt;
}

void PointWidget::pushbackPointCloud(QPoint point)
{
  this->pointCloud.push_back(point);
}
/**
  * @brief  According to this->inner_area drawing
  * @param  tableVectorPolygon_qt: polygon table data.
  * @param  tablePolygon_qt: point table data.
  * @retval None
  */
void PointWidget::updateVectorInnerArea(std::vector<QPolygon> tableVectorPolygon_qt,QPolygon tablePolygon_qt)
{
  inner_area.clear();
  for(int i=0; i<tableVectorPolygon_qt.size(); i++){
    inner_area.push_back(tableVectorPolygon_qt.at(i));
  }
  inner_area.push_back(tablePolygon_qt);
  update();
}
//mid
void PointWidget::updateVectorMidArea(std::vector<QPolygon> tableVectorPolygon_qt,QPolygon tablePolygon_qt)
{
  mid_area.clear();
  for(int i=0; i<tableVectorPolygon_qt.size(); i++){
    mid_area.push_back(tableVectorPolygon_qt.at(i));
  }
  mid_area.push_back(tablePolygon_qt);
  update();
}
// out
void PointWidget::updateVectorOutArea(std::vector<QPolygon> tableVectorPolygon_qt,QPolygon tablePolygon_qt)
{
  out_area.clear();
  for(int i=0; i<tableVectorPolygon_qt.size(); i++){
    out_area.push_back(tableVectorPolygon_qt.at(i));
  }
  out_area.push_back(tablePolygon_qt);
  update();
}
