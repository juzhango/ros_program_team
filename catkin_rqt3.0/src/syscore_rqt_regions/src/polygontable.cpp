#include "../include/syscore_rqt_regions/polygontable.h"

#include <QDebug>

#include "../include/syscore_rqt_regions/common.h"


PolygonTable::PolygonTable(QTableWidget *parent) : QTableWidget(parent)
{
  polygonCount = 0;
  this->setColumnCount(1);
  this->setMinimumSize(190,300);
  this->setMaximumSize(190,300);
  this->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
  this->setHorizontalHeaderLabels(QStringList()<<"Polygon_list");

  this->setEditTriggers(QAbstractItemView::NoEditTriggers);
  this->setSelectionBehavior(QAbstractItemView::SelectRows);
  this->setSelectionMode(QAbstractItemView::SingleSelection); //设置选择模式，选择单行

  //创建一个菜单
  m_contextMenu = new QMenu;
  m_editAction = new QAction("edit Item",this);
  m_delAction = new QAction("del Item",this);
  m_contextMenu->addAction(m_editAction);
  m_contextMenu->addAction(m_delAction);

  QObject::connect(m_editAction,SIGNAL(triggered()),this,SLOT(slot_menu_edit_action()));
  QObject::connect(m_delAction,SIGNAL(triggered()),this,SLOT(slot_menu_del_action()));
}
void PolygonTable::setOriginPos(QPoint point)
{
  this->originPos = point;
}

void PolygonTable::mouseDoubleClickEvent(QMouseEvent *ev)
{
  if (ev->button() == Qt::RightButton){
    m_contextMenu->exec(ev->globalPos());
  }
  if (ev->button() == Qt::LeftButton && currentRow() >= 0){
    QPolygon polygon = this->tableVectorPolygon.at(this->currentRow());
    QPolygon polygon_qt = this->tableVectorPolygon_qt.at(this->currentRow());
    setReadOnly(true);
    Q_EMIT painter_MarkPolygon(polygon,polygon_qt);
  }

}
/**
  * @brief  set vPolygon & vPolygon_qt base.
  * @param  vPolygon: area_vectorPolygon
  * @retval None
  */
void PolygonTable::setTableVectorPolygon(std::vector<QPolygon> vPolygon)
{
  tableVectorPolygon.clear();
  tableVectorPolygon_qt.clear();
  for(int i = 0; i < vPolygon.size(); i++)
  {
    tableVectorPolygon.push_back(vPolygon.at(i));
    tableVectorPolygon_qt.push_back(polygonToPolygon_qt(vPolygon.at(i)));
  }
  updateTable();
}
QPoint PolygonTable::pointToPoint_qt(QPoint point)
{
  QPoint point_qt;
  point_qt = QPoint(originPos.x()-point.y(),originPos.y()-point.x());
  return point_qt;
}
QPoint PolygonTable::point_qtToPoint(QPoint point_qt)
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
QPolygon PolygonTable::polygonToPolygon_qt(QPolygon polygon)
{
  QPolygon polygon_qt;
  for(int i = 0; i < polygon.size(); i++){
    QPoint point_qt = pointToPoint_qt(polygon.at(i));
    polygon_qt.push_back(point_qt);
  }
  return polygon_qt;
}
QPolygon PolygonTable::polygon_qtToPolygon(QPolygon polygon_qt)
{
  QPolygon polygon;
  for(int i = 0; i < polygon_qt.size(); i++){
    QPoint point = point_qtToPoint(polygon_qt.at(i));
    polygon.push_back(point);
  }
  return polygon;
}

/**
  * @brief  pushback polygon to this->tableVectorPolygon
  * @param  polygon:
  * @param  polygon_qt:
  * @retval None
  */
void PolygonTable::pushbackVectorPolygon(QPolygon polygon, QPolygon polygon_qt)
{
  if(polygon.size()>0){
    tableVectorPolygon.push_back(polygon);
    tableVectorPolygon_qt.push_back(polygon_qt);
  }
  else{
    qDebug()<<"polygon empty!";
  }
}
/**
  * @brief
  * @param  area: from sceneTable area_map
  * @retval None
  */
void PolygonTable::setVectorPolygonFromSceneMap(std::map<std::string, QPolygon> area)
{
  std::string str = "polygon";
  for(int j = 0; j < area.size(); j++){
    QPolygon polygon = area[str+std::to_string(j+1)];
    QPolygon polygon_qt = polygonToPolygon_qt(polygon);
    pushbackVectorPolygon(polygon, polygon_qt);
  }
}

/**
  * @brief  delAppoint base tableVectorPolygon / tableVectorPolygon_qt
  * @param  i:
  * @retval None
  */
void PolygonTable::delAppointPolygon(int i)
{
  if(i >=0 && i<tableVectorPolygon.size() && i<tableVectorPolygon_qt.size()){
    std::vector<QPolygon>::iterator it = this->tableVectorPolygon.begin()+i;
    this->tableVectorPolygon.erase(it);
    std::vector<QPolygon>::iterator it_qt = this->tableVectorPolygon_qt.begin()+i;
    this->tableVectorPolygon_qt.erase(it_qt);
  }
  updateTable();
}

void PolygonTable::cleanTable()
{
  polygonCount = 0;
  for(int i = rowCount(); i >= 0; i--){
    removeRow(i);
  }
}
void PolygonTable::popbackVectorPolygon()
{
  if(tableVectorPolygon.size()>0){
    tableVectorPolygon.pop_back();
    tableVectorPolygon_qt.pop_back();
  }
}
void PolygonTable::cleanVectorPolygon()
{
  tableVectorPolygon.clear();
  tableVectorPolygon_qt.clear();
  updateTable();
}

/**
  * @brief  According to this->tableVectorPolygon
  * @param  None
  * @retval None
  */
void PolygonTable::updateTable()
{
  cleanTable();
  for(int i = 0; i < this->tableVectorPolygon.size();i++){
    QString str = "polygon";
    str = str + QString::number(polygonCount+1);
    insertRow(this->rowCount());
    setItem(polygonCount,0,new QTableWidgetItem(str));
    polygonCount++;
  }
}
void PolygonTable::slot_menu_edit_action()
{
  setReadOnly(false);
  Q_EMIT request_EditTable();
}
void PolygonTable::slot_menu_del_action()
{
  delAppointPolygon(this->currentRow());
  Q_EMIT request_Update();
}

