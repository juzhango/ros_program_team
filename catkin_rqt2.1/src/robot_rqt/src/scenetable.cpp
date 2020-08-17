#include "../include/robot_rqt/scenetable.h"
#include "QDebug"
SceneTable::SceneTable(QTableWidget *parent) : QTableWidget(parent)
{
  sceneCount = 0;

  this->setColumnCount(1);
  this->setMinimumSize(200,120);
  this->setMaximumSize(200,120);
  this->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
  this->setHorizontalHeaderLabels(QStringList()<<"Scene_list");

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
/**
  * @brief  left click display,right click menu.
  * @param  *ev:
  * @retval None
  */
void SceneTable::mouseDoubleClickEvent(QMouseEvent *ev)
{
  //left click
  if (ev->button() == Qt::LeftButton && currentRow() >= 0){
    std::vector<QPolygon> innerVpolygon,midVpolygon,outVpolygon;
    int i = currentRow();
    std::string str="polygon";
    for(int j = 0; j < scenes.at(i).areas.area_inner[str+std::to_string(j+1)].size(); j++){
      innerVpolygon.push_back(scenes.at(i).areas.area_inner[str+std::to_string(j+1)]);
    }
    for(int j = 0; j < scenes.at(i).areas.area_mid[str+std::to_string(j+1)].size(); j++){
      midVpolygon.push_back(scenes.at(i).areas.area_mid[str+std::to_string(j+1)]);
    }
    for(int j = 0; j < scenes.at(i).areas.area_out[str+std::to_string(j+1)].size(); j++){
      outVpolygon.push_back(scenes.at(i).areas.area_out[str+std::to_string(j+1)]);
    }
    Q_EMIT launchUpdateTable(innerVpolygon,midVpolygon,outVpolygon);
  }
  //right click
  if (ev->button() == Qt::RightButton && currentRow() >= 0){
    m_contextMenu->exec(ev->globalPos());
  }
}
void SceneTable::cleanTable()
{
  tableRowCount = 0;
  for(int i = rowCount(); i >= 0; i--){
    removeRow(i);
  }
}
/**
  * @brief  According to this->scenes.size() update table
  * @param  None
  * @retval None
  */
void SceneTable::updateTable()
{
  cleanTable();
  for(int i = 0; i < this->scenes.size();i++){
    QString str = "scene_";
    str = str + QString::number(tableRowCount+1);
    insertRow(this->rowCount());
    setItem(tableRowCount,0,new QTableWidgetItem(str));
    tableRowCount++;
  }
}

/**
  * @brief  trun polygon_array into area map
  * @param  vectorPolygon: An array of polygons in an area
  * @retval area_map:
  */
std::map<std::string, QPolygon> SceneTable::generateAreaMap(std::vector<QPolygon> vectorPolygon)
{
  std::map<std::string, QPolygon> area_map;
  std::string str = "polygon";
  for(int i=0; i<vectorPolygon.size(); i++){
    area_map[str+std::to_string(i+1)] = vectorPolygon.at(i);
  }
  return area_map;
}

void SceneTable::pushbackScene(ST_SCENE scene)
{
  this->scenes.push_back(scene);

  qDebug()<<"map area_inner size:"<< scene.areas.area_inner.size();
  qDebug()<<"map area_mid size:"<< scene.areas.area_mid.size();
  qDebug()<<"map area_out size:"<< scene.areas.area_out.size();

}
void SceneTable::popbackScene()
{
  if(this->scenes.size()>0){
    this->scenes.pop_back();
  }
  else{
    qDebug()<<"scene empty!";
  }
  updateTable();
}
/**
  * @brief  clear this->scenes.
  * @param  None
  * @retval None
  */
void SceneTable::cleanScene()
{
  /*
   * 1.Clear scenes vector
   * 2.Reset sceneCount
   * 3.update table
   */
  this->scenes.clear();
  sceneCount=0;
  updateTable();
}


/**
  * @brief  generate a scence
  * @param  area_innerVectorPolygon
  * @param  area_midVectorPolygon
  * @param  area_outVectorPolygon
  * @retval None
  */
void SceneTable::generateScene(std::vector<QPolygon> innerVectorPolygon, std::vector<QPolygon> midVectorPolygon, std::vector<QPolygon> outVectorPolygon)
{
  sceneCount = scenes.size()+1;
  std::string str = "scene_";
  ST_SCENE st_scene;

  st_scene.name = str + std::to_string(sceneCount);
  if(sceneCount == 1){
    st_scene.default_scene = true;
  }
  else{
    st_scene.default_scene = false;
  }
  st_scene.areas.area_inner = generateAreaMap(innerVectorPolygon);
  st_scene.areas.area_mid = generateAreaMap(midVectorPolygon);
  st_scene.areas.area_out = generateAreaMap(outVectorPolygon);
  pushbackScene(st_scene);
  updateTable();
}

void SceneTable::replaceScene(std::vector<QPolygon> innerVectorPolygon, std::vector<QPolygon> midVectorPolygon, std::vector<QPolygon> outVectorPolygon)
{
  ST_SCENE st_scene;
  std::string str = "scene_";
  st_scene.name = str + std::to_string(currentRow()+1);
  if(currentRow() == 0){
    st_scene.default_scene = true;
  }
  else{
    st_scene.default_scene = false;
  }
  st_scene.areas.area_inner = generateAreaMap(innerVectorPolygon);
  st_scene.areas.area_mid = generateAreaMap(midVectorPolygon);
  st_scene.areas.area_out = generateAreaMap(outVectorPolygon);
  std::vector<ST_SCENE> scenes_new;
  for(int i = 0; i < scenes.size(); i++){
    if(i != currentRow()){
      scenes_new.push_back(scenes.at(i));
    }
    else{
      scenes_new.push_back(st_scene);
    }
  }
  cleanScene();
  scenes = scenes_new;
  updateTable();
}

/**
  * @brief  Put qnode::scenes_input into SceneTable::scenes
  * @param  scenes_input: qnode::scenes_input
  * @retval None
  */
void SceneTable::generateAllScene(std::vector<ST_SCENE> scenes_input)
{
  cleanScene();
  qDebug()<<"enter generate all scene";
  for(int i = 0; i < scenes_input.size(); i++){
    std::vector<QPolygon> innerVectorPolygon;
    std::vector<QPolygon> midVectorPolygon;
    std::vector<QPolygon> outVectorPolygon;
    std::string str = "polygon";
    for(int j = 0; j < scenes_input.at(i).areas.area_inner[str+std::to_string(j+1)].size(); j++){
      innerVectorPolygon.push_back(scenes_input.at(i).areas.area_inner[str+std::to_string(j+1)]);
    }
    for(int j = 0; j < scenes_input.at(i).areas.area_mid[str+std::to_string(j+1)].size(); j++){
      midVectorPolygon.push_back(scenes_input.at(i).areas.area_mid[str+std::to_string(j+1)]);
    }
    for(int j = 0; j < scenes_input.at(i).areas.area_out[str+std::to_string(j+1)].size(); j++){
      outVectorPolygon.push_back(scenes_input.at(i).areas.area_out[str+std::to_string(j+1)]);
    }
    generateScene(innerVectorPolygon,midVectorPolygon,outVectorPolygon);
  }
  updateTable();
}

/**
  * @brief  delAppoint base sences
  * @param  i:
  * @retval None
  */
void SceneTable::delAppointScene(int i)
{
  if(i >=0 && i<scenes.size() ){
    std::vector<ST_SCENE>::iterator it = this->scenes.begin()+i;
    this->scenes.erase(it);
  }
  updateTable();
}

void SceneTable::slot_menu_edit_action()
{
  Q_EMIT menu_edit_scene();
}

void SceneTable::slot_menu_del_action()
{
  delAppointScene(currentRow());
  Q_EMIT updatePainter();
}
