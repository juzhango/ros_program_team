#ifndef POLYGONTABLE_H
#define POLYGONTABLE_H
#include "QTableWidget"
#include "vector"
#include "iostream"
#include "QPolygon"
#include <QHeaderView>
#include <QStandardItemModel>
#include <QMenu>
#include <QAction>
#include <QContextMenuEvent>
#include <QMouseEvent>
#include "../syscore_rqt_regions/scenetable.h"

class PolygonTable : public QTableWidget
{
  Q_OBJECT
public:
  explicit PolygonTable(QTableWidget *parent = nullptr);

  void setOriginPos(QPoint);
  void mouseDoubleClickEvent(QMouseEvent *);  //双击

  QPoint pointToPoint_qt(QPoint);
  QPoint point_qtToPoint(QPoint);
  QPolygon polygonToPolygon_qt(QPolygon);
  QPolygon polygon_qtToPolygon(QPolygon);

  void pushbackVectorPolygon(QPolygon polygon,QPolygon polygon_qt);
  void popbackVectorPolygon();
  void cleanTable();
  void updateTable();
  void cleanVectorPolygon();



  void setTableVectorPolygon(std::vector<QPolygon>);
  std::vector<QPolygon> getTableVectorPolygon(){return this->tableVectorPolygon;}
  std::vector<QPolygon> getTableVectorPolygon_qt(){return this->tableVectorPolygon_qt;}
  int getPolygonCount(){return this->polygonCount;}
  void setVectorPolygonFromSceneMap(std::map<std::string, QPolygon> area);
  void delAppointPolygon(int i);

  void setReadOnly(bool b){polygonReadOnly = b;}
  bool getReadOnly(){return polygonReadOnly;}

public Q_SLOTS:
  void slot_menu_del_action();
  void slot_menu_edit_action();
Q_SIGNALS:
  void painter_MarkPolygon(QPolygon,QPolygon);
  void request_Update();
  void request_EditTable();

private:
  int polygonCount;   //polygonCount is  number of tableVectorPolygon
  std::vector<QPolygon> tableVectorPolygon;
  std::vector<QPolygon> tableVectorPolygon_qt;

  QPoint originPos;
  bool polygonReadOnly;

  QMenu *m_contextMenu;
  QAction *m_editAction;
  QAction *m_delAction;
};

#endif // POLYGONTABLE_H
