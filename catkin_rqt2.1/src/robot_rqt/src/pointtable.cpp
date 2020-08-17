#include "../include/robot_rqt/pointtable.h"
#include "QDebug"
#include "../include/robot_rqt/common.h"

PointTable::PointTable(QTableWidget *parent) : QTableWidget(parent)
{
  setCount = 0;

  this->setColumnCount(2);
  this->setMinimumSize(190,350);
  this->setMaximumSize(190,350);
  this->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
  this->setHorizontalHeaderLabels(QStringList()<<"X"<<"Y");

  this->setEditTriggers(QAbstractItemView::NoEditTriggers);
  this->setSelectionBehavior(QAbstractItemView::SelectRows);
  this->setSelectionMode(QAbstractItemView::SingleSelection); //设置选择模式，选择单行

  //创建一个菜单
  m_contextMenu = new QMenu;
  m_insertAction = new QAction("insert Item",this);
  m_delAction = new QAction("del Item",this);
  m_contextMenu->addAction(m_insertAction);
  m_contextMenu->addAction(m_delAction);

  QObject::connect(m_insertAction,SIGNAL(triggered()),this,SLOT(slot_menu_insert_action()));
  QObject::connect(m_delAction,SIGNAL(triggered()),this,SLOT(slot_menu_del_action()));

}
/**
  * @brief  left click display,right click menu.
  * @param  *ev:
  * @retval None
  */
void PointTable::mouseDoubleClickEvent(QMouseEvent *ev)
{

  if (ev->button() == Qt::LeftButton && currentRow() >= 0){
    QPoint point_qt = this->tablePolygon_qt.at(currentRow());
    Q_EMIT painter_Mark(point_qt);
    Q_EMIT request_enable_replaceBtn();
  }
  if (ev->button() == Qt::RightButton){
    m_contextMenu->exec(ev->globalPos());
  }
}

void PointTable::pushbackPolygon(QPoint point, QPoint point_qt)
{
  this->tablePolygon.push_back(point);
  this->tablePolygon_qt.push_back(point_qt);
}
void PointTable::popbackPolygon()
{
  if(tablePolygon.size()>0){
    tablePolygon.pop_back();
    tablePolygon_qt.pop_back();
  }
}



/**
  * @brief  clean bace_polygon
  * @param  None
  * @retval None
  */
void PointTable::cleanPolygon()
{
  tablePolygon.clear();
  tablePolygon_qt.clear();
  updateTable();
}

void PointTable::cleanTable()
{
  setCount = 0;
  for(int i = rowCount(); i >= 0; i--){
    removeRow(i);
  }
}

/**
  * @brief  according to bace_polygon
  * @param  NOne
  * @retval None
  */
void PointTable::updateTable()
{
  cleanTable();
  for(int i = 0; i < this->tablePolygon.size();i++){
    insertRow(this->rowCount());
    setItem(setCount,0,new QTableWidgetItem(QString::number(tablePolygon.at(i).x())));
    setItem(setCount,1,new QTableWidgetItem(QString::number(tablePolygon.at(i).y())));
    setCount++;
  }
}
/**
  * @brief  delAppoint base polygon / polygon_qt
  * @param  i:
  * @retval None
  */
void PointTable::delAppointPolygon(int i)
{
  if(tablePolygon.size()>0 && i<tablePolygon.size() && i<tablePolygon_qt.size()){
    QPolygon::iterator it = this->tablePolygon.begin()+i;
    this->tablePolygon.erase(it);
    QPolygon::iterator it_qt = this->tablePolygon_qt.begin()+i;
    this->tablePolygon_qt.erase(it_qt);
  }
  updateTable();
}
/**
  * @brief  insertAppoint base polygon / polygon_qt   before i
  * @param  i:
  * @retval None
  */
void PointTable::insertAppointPolygon(QPoint point,QPoint point_qt, int i)
{
  tablePolygon.insert(tablePolygon.begin()+i,point);
  tablePolygon_qt.insert(tablePolygon_qt.begin()+i,point_qt);
  updateTable();
}

/**
  * @brief  replace base polygon's point at(i).
  * @param  point: current click point
  * @param  point_qt: current click point_qt
  * @param  i: polygon.at(i)
  * @retval None
  */
void PointTable::replacePoint(QPoint point, QPoint point_qt, int i)
{
  qDebug()<<tablePolygon;
  QPolygon polygon,polygon_qt;
  for(int j = 0; j < this->tablePolygon.size(); j++){
    if(j != i){
      polygon.push_back(tablePolygon.at(j));
      polygon_qt.push_back(tablePolygon_qt.at(j));
    }
    else{
      polygon.push_back(point);
      polygon_qt.push_back(point_qt);
    }
  }
  cleanPolygon();
  tablePolygon = polygon;
  tablePolygon_qt = polygon_qt;
  updateTable();
}

void PointTable::setTablePolygon(QPolygon polygon,QPolygon polygon_qt)
{
  /*
   * 1.clean base polygon
   * 2.get base polygon
   * 3.update table
   */
  cleanPolygon();
  tablePolygon = polygon;
  tablePolygon_qt = polygon_qt;
  updateTable();
}


void PointTable::slot_menu_insert_action()
{
  if(this->currentRow()>=0){
    int i = this->currentRow();
    Q_EMIT insert_point(i);
  }

}
void PointTable::slot_menu_del_action()
{
  if(this->currentRow()>=0){
    int i = this->currentRow();
    Q_EMIT del_point(i);
  }
}
