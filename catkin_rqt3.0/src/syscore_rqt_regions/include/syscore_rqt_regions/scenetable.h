#ifndef SCENETABLE_H
#define SCENETABLE_H

#include <QTableWidget>
#include <QStandardItemModel>
#include <QWidget>
#include <QHeaderView>
#include <map>
#include <iostream>
#include <string>
#include <vector>
#include <QPolygon>
#include <string>

#include "../syscore_rqt_regions/polygontable.h"

struct ST_AREAS{
  std::map<std::string, QPolygon> area_inner;
  std::map<std::string, QPolygon> area_mid;
  std::map<std::string, QPolygon> area_out;
};
struct ST_SCENE{
  std::string name;
  bool default_scene;
  ST_AREAS areas;
};

struct st_avoid_area_kind_{
  std::string area_out = "area_out";
  std::string area_mid = "area_mid";
  std::string area_inner = "area_inner";
};


class SceneTable : public QTableWidget
{
  Q_OBJECT
public:
  explicit SceneTable(QTableWidget *parent = nullptr);

  void mouseDoubleClickEvent(QMouseEvent *ev);

  void cleanTable();
  void updateTable();

  void setParamName(QString str){paramName = str;}
  QString getParamName(){return paramName;}
  std::map<std::string, QPolygon> generateAreaMap(std::vector<QPolygon> vectorPolygon);
  int getSceneTableCurrentRow(){return currentRow();}
  void pushbackScene(ST_SCENE scene);
  void popbackScene();
  void cleanScene();
  void generateScene(std::vector<QPolygon> innerVectorPolygon, std::vector<QPolygon> midVectorPolygon, std::vector<QPolygon> outVectorPolygon);
  void replaceScene(std::vector<QPolygon> innerVectorPolygon, std::vector<QPolygon> midVectorPolygon, std::vector<QPolygon> outVectorPolygon);
  void generateAllScene(std::vector<ST_SCENE> scenes_input);
  std::vector<ST_SCENE> getScenes(){return scenes;}
  void delAppointScene(int i);
public Q_SLOTS:
  void slot_menu_edit_action();
  void slot_menu_del_action();
Q_SIGNALS:
  void launchUpdateTable(std::vector<QPolygon>,std::vector<QPolygon>,std::vector<QPolygon>);
  void updatePainter();
  void menu_edit_scene();



private:
  std::vector<ST_SCENE> scenes;
  int sceneCount; //number of scene;
  int tableRowCount; //number of table
  QString paramName;

  QMenu *m_contextMenu;
  QAction *m_delAction;
  QAction *m_editAction;
};


#endif // SCENETABLE_H
