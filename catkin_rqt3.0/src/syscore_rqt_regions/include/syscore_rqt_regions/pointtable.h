
#ifndef POINTTABLE_H
#define POINTTABLE_H

#include <QTableWidget>
#include <QStandardItemModel>
#include <QWidget>
#include <QHeaderView>

#include "../syscore_rqt_regions/polygontable.h"

class PointTable : public QTableWidget
{
  Q_OBJECT
public:
  explicit PointTable(QTableWidget *parent = nullptr);
  void mouseDoubleClickEvent(QMouseEvent *);  //双击

  void cleanPolygon();
  void delAppointPolygon(int i);
  void insertAppointPolygon(QPoint point, QPoint point_qt, int i);

  void replacePoint(QPoint point, QPoint point_qt, int i);

  void setTablePolygon(QPolygon polygon,QPolygon polygon_qt);
  QPolygon getTablePolygon(){return this->tablePolygon;}
  QPolygon getTablePolygon_qt(){return this->tablePolygon_qt;}
  void pushbackPolygon(QPoint point, QPoint point_qt);
  void popbackPolygon();

  void updateTable();
  void cleanTable();



public Q_SLOTS:
  void slot_menu_del_action();
  void slot_menu_insert_action();

Q_SIGNALS:
  void del_point(int);
  void painter_Mark(QPoint);
  void insert_point(int);
  void request_enable_replaceBtn();
private:
  int setCount;

  // bace_polygon
  QPolygon tablePolygon;
  QPolygon tablePolygon_qt;

  //menu
  QMenu *m_contextMenu;
  QAction *m_insertAction;
  QAction *m_delAction;
};

#endif // POINTTABLE_H
