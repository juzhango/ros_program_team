#ifndef POINTWIDGET_H
#define POINTWIDGET_H

#include <QWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPolygon>
#include <vector>
#include <iostream>
#include "scenetable.h"
class PointWidget : public QWidget
{
  Q_OBJECT
public:
  explicit PointWidget(QWidget *parent = nullptr);
  QPoint pointToPoint_qt(QPoint);
  QPoint point_qtToPoint(QPoint);
  QPolygon polygonToPolygon_qt(QPolygon);
  QPolygon polygon_qtToPolygon(QPolygon);
  void paintEvent(QPaintEvent *);
  void mousePressEvent(QMouseEvent *ev);


  void drawBox(QPainter *painter);
  void drawClickPoint(QPainter *painter);
  void drawMarkPoint(QPainter *painter);
  void drawMarkPolygon(QPainter *painter);
  void drawLaserDev(QPainter *painter);
  void drawLaserPolygon(QPainter *painter, QPolygon polygon);
  void drawInnerPolygons(QPainter *painter,std::vector<QPolygon> inner_area);
  void drawMidPolygons(QPainter *painter,std::vector<QPolygon> mid_area);
  void drawOutPolygons(QPainter *painter,std::vector<QPolygon> out_area);
  void drawShowScene(ST_SCENE scene);

  QPoint getOriginPos_qt(){return originPos;}
  void setPoint(QPoint point,QPoint point_qt);
  void setMarkPoint(QPoint point_qt);
  void setMarkPolygon(QPolygon polygon_qt);
  void resetMarkPoint(){markPoint_qt = originPos;}
  void resetMarkPolygon(){markPolygon_qt.clear();update();}
  QPoint getCurrentClickPoint() {return clickPoint;}
  QPoint getCurrentClickPoint_qt() {return clickPoint_qt;}
  void pushbackPointCloud(QPoint);
  void cleanPointCloud(){ this->pointCloud.clear();}

  void updateVectorInnerArea(std::vector<QPolygon> tableVectorPolygon_qt, QPolygon tablePolygon_qt);
  void updateVectorMidArea(std::vector<QPolygon> tableVectorPolygon_qt, QPolygon tablePolygon_qt);
  void updateVectorOutArea(std::vector<QPolygon> tableVectorPolygon_qt, QPolygon tablePolygon_qt);


Q_SIGNALS:
public Q_SLOTS:


private:
  QPoint originPos;
  QPoint clickPoint;
  QPoint clickPoint_qt;
  QPoint markPoint_qt;
  QPolygon markPolygon_qt;

  QPolygon pointCloud;
  std::vector<QPolygon> inner_area;
  std::vector<QPolygon> mid_area;
  std::vector<QPolygon> out_area;
};

#endif // POINTWIDGET_H
