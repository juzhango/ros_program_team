/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_rqt/main_window.hpp"
#include "../include/robot_rqt/common.h"
#include "../include/robot_rqt/output_format_yaml.h"
#include "../include/robot_rqt/common.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_rqt {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/

    if ( ui.checkbox_remember_settings->isChecked() ) {
        qnode.setScanTopic(ui.lineEdit_topic->text().toStdString().data());
        on_button_connect_clicked(true);
    }else
    {
      //user
      disableROSmaster();
    }


    // init
    QPoint originPos;
    pointWidget = new PointWidget(ui.widget_pointCloud);
    originPos = pointWidget->getOriginPos_qt();

    innerPointTable = new PointTable(ui.table_point_inner);
    midPointTable = new PointTable(ui.table_point_mid);
    outPointTable = new PointTable(ui.table_point_out);

    innerPolygonTable = new PolygonTable(ui.table_polygon_inner);
    midPolygonTable = new PolygonTable(ui.table_polygon_mid);
    outPolygonTable = new PolygonTable(ui.table_polygon_out);
    innerPolygonTable->setOriginPos(originPos);
    midPolygonTable->setOriginPos(originPos);
    outPolygonTable->setOriginPos(originPos);

    sceneTable = new SceneTable(ui.table_scene);

    connect(&qnode,SIGNAL(get_scan(QPolygon)),this,SLOT(slot_update_laserScan(QPolygon)));  //point cloud callBack
    connect(sceneTable,SIGNAL(updatePainter()),this,SLOT(slot_clean_point_polygon_painter()));//===scene del update
    connect(sceneTable,SIGNAL(menu_edit_scene()),this,SLOT(slot_menu_edit_scene()));//===scene edit update
    connect(innerPointTable,SIGNAL(del_point(int)),this,SLOT(slot_inner_del_painter(int)));//=== del appiont point
    connect(midPointTable,SIGNAL(del_point(int)),this,SLOT(slot_mid_del_painter(int)));
    connect(outPointTable,SIGNAL(del_point(int)),this,SLOT(slot_out_del_painter(int)));
    connect(innerPointTable,SIGNAL(painter_Mark(QPoint)),this,SLOT(slot_painter_Mark(QPoint)));//=== mark point
    connect(midPointTable,SIGNAL(painter_Mark(QPoint)),this,SLOT(slot_painter_Mark(QPoint)));
    connect(outPointTable,SIGNAL(painter_Mark(QPoint)),this,SLOT(slot_painter_Mark(QPoint)));
    connect(innerPointTable,SIGNAL(insert_point(int)),this,SLOT(slot_inner_insert_point(int)));//=== insert
    connect(midPointTable,SIGNAL(insert_point(int)),this,SLOT(slot_mid_insert_point(int)));
    connect(outPointTable,SIGNAL(insert_point(int)),this,SLOT(slot_out_insert_point(int)));
    connect(innerPointTable,SIGNAL(request_enable_replaceBtn()),this,SLOT(slot_request_enable_replaceBtn()));//=== enable replace btn
    connect(midPointTable,SIGNAL(request_enable_replaceBtn()),this,SLOT(slot_request_enable_replaceBtn()));
    connect(outPointTable,SIGNAL(request_enable_replaceBtn()),this,SLOT(slot_request_enable_replaceBtn()));
    connect(innerPolygonTable,SIGNAL(request_Update()),this,SLOT(slot_update_painter()));//=== polygon request
    connect(midPolygonTable,SIGNAL(request_Update()),this,SLOT(slot_update_painter()));
    connect(outPolygonTable,SIGNAL(request_Update()),this,SLOT(slot_update_painter()));
    connect(innerPolygonTable,SIGNAL(request_EditTable()),this,SLOT(slot_edit_innerPolygon_table()));//=== polygonTable edit request
    connect(midPolygonTable,SIGNAL(request_EditTable()),this,SLOT(slot_edit_midPolygon_table()));
    connect(outPolygonTable,SIGNAL(request_EditTable()),this,SLOT(slot_edit_outPolygon_table()));
    connect(sceneTable,SIGNAL(launchUpdateTable(std::vector<QPolygon>,std::vector<QPolygon>,std::vector<QPolygon>)),this,SLOT(slot_launchUpdateTable(std::vector<QPolygon>,std::vector<QPolygon>,std::vector<QPolygon>)));
    connect(innerPolygonTable,SIGNAL(painter_MarkPolygon(QPolygon,QPolygon)),this,SLOT(slot_painter_inner_MarkPolygon(QPolygon,QPolygon)));//=== mark Polygon
    connect(midPolygonTable,SIGNAL(painter_MarkPolygon(QPolygon,QPolygon)),this,SLOT(slot_painter_mid_MarkPolygon(QPolygon,QPolygon)));
    connect(outPolygonTable,SIGNAL(painter_MarkPolygon(QPolygon,QPolygon)),this,SLOT(slot_painter_out_MarkPolygon(QPolygon,QPolygon)));
    connect(ui.btn_set_point_inner,SIGNAL(pressed()),this,SLOT(slot_btn_set_point_inner_pressed()));//=== inner point
    connect(ui.btn_replace_point_inner,SIGNAL(pressed()),this,SLOT(slot_btn_replace_point_inner_pressed()));
    connect(ui.btn_clean_point_inner,SIGNAL(pressed()),this,SLOT(slot_btn_clean_point_inner_pressed()));
    connect(ui.btn_set_point_mid,SIGNAL(pressed()),this,SLOT(slot_btn_set_point_mid_pressed()));//=== mid point
    connect(ui.btn_replace_point_mid,SIGNAL(pressed()),this,SLOT(slot_btn_replace_point_mid_pressed()));
    connect(ui.btn_clean_point_mid,SIGNAL(pressed()),this,SLOT(slot_btn_clean_point_mid_pressed()));
    connect(ui.btn_set_point_out,SIGNAL(pressed()),this,SLOT(slot_btn_set_point_out_pressed()));//=== out point
    connect(ui.btn_replace_point_out,SIGNAL(pressed()),this,SLOT(slot_btn_replace_point_out_pressed()));
    connect(ui.btn_clean_point_out,SIGNAL(pressed()),this,SLOT(slot_btn_clean_point_out_pressed()));
    connect(ui.btn_set_polygon_inner,SIGNAL(pressed()),this,SLOT(slot_btn_set_polygon_inner_pressed()));//===inner polygon
    connect(ui.btn_close_polygon_inner,SIGNAL(pressed()),this,SLOT(slot_btn_close_polygon_pressed()));
    connect(ui.btn_clean_polygon_inner,SIGNAL(pressed()),this,SLOT(slot_btn_clean_polygon_inner_pressed()));
    connect(ui.btn_set_polygon_mid,SIGNAL(pressed()),this,SLOT(slot_btn_set_polygon_mid_pressed()));//=== mid polygon
    connect(ui.btn_close_polygon_mid,SIGNAL(pressed()),this,SLOT(slot_btn_close_polygon_pressed()));
    connect(ui.btn_clean_polygon_mid,SIGNAL(pressed()),this,SLOT(slot_btn_clean_polygon_mid_pressed()));
    connect(ui.btn_set_polygon_out,SIGNAL(pressed()),this,SLOT(slot_btn_set_polygon_out_pressed()));//=== out polygon
    connect(ui.btn_close_polygon_out,SIGNAL(pressed()),this,SLOT(slot_btn_close_polygon_pressed()));
    connect(ui.btn_clean_polygon_out,SIGNAL(pressed()),this,SLOT(slot_btn_clean_polygon_out_pressed()));
    connect(ui.btn_set_scene,SIGNAL(pressed()),this,SLOT(slot_btn_set_scene_pressed()));//=== scene btn
    connect(ui.btn_replace_scene,SIGNAL(pressed()),this,SLOT(slot_btn_replace_scene_pressed()));
    connect(ui.btn_clean_scene,SIGNAL(pressed()),this,SLOT(slot_btn_clean_scene_pressed()));
    connect(ui.btn_scene_output,SIGNAL(pressed()),this,SLOT(slot_btn_scene_output_pressed()));//=== output
    connect(ui.btn_scene_input,SIGNAL(pressed()),this,SLOT(slot_btn_scene_input_pressed()));//=== input
    connect(ui.btn_set_topic,SIGNAL(pressed()),this,SLOT(slot_btn_set_topic_pressed()));//=== set topic btn
    ui.btn_reset_topic->setEnabled(false);
    connect(ui.btn_reset_topic,SIGNAL(pressed()),this,SLOT(slot_btn_reset_topic_pressed()));
    connect(ui.btn_set_paramName,SIGNAL(pressed()),this,SLOT(slot_btn_set_paramName_pressed()));//=== set param name btn
    ui.btn_reset_paramName->setEnabled(false);
    connect(ui.btn_reset_paramName,SIGNAL(pressed()),this,SLOT(slot_btn_reset_paramName_pressed()));
}


MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robot_rqt");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robot_rqt");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}



void MainWindow::disableROSmaster()
{
  ui.button_connect->setEnabled(false);
  ui.line_edit_master->setReadOnly(true);
  ui.line_edit_host->setReadOnly(true);
  ui.line_edit_topic->setReadOnly(true);
}
void MainWindow::enableROSmaster()
{
  ui.button_connect->setEnabled(true);
  ui.line_edit_master->setReadOnly(false);
  ui.line_edit_host->setReadOnly(false);
  ui.line_edit_topic->setReadOnly(false);
}


}  // namespace robot_rqt


/**
  * @brief  Remember to cleanPointCloud() first !!
  * @param  polygon: get callBack msg=>polygon
  * @retval None
  */
void robot_rqt::MainWindow::slot_update_laserScan(QPolygon polygon)
{
  pointWidget->cleanPointCloud();
  for(int i = 0; i < polygon.size(); i++){
    QPoint point_qt = pointWidget->pointToPoint_qt(polygon.at(i));
    pointWidget->pushbackPointCloud(point_qt);
  }
  pointWidget->update();
}

void robot_rqt::MainWindow::disableRepalceBtn()
{
  ui.btn_replace_point_inner->setEnabled(false);
  ui.btn_replace_point_mid->setEnabled(false);
  ui.btn_replace_point_out->setEnabled(false);
}
void robot_rqt::MainWindow::enableRepalceBtn()
{
  ui.btn_replace_point_inner->setEnabled(true);
  ui.btn_replace_point_mid->setEnabled(true);
  ui.btn_replace_point_out->setEnabled(true);
}
// inner
void robot_rqt::MainWindow::slot_btn_set_point_inner_pressed(){
  enableSetPolytonBtn();
  ui.btn_close_polygon_inner->setEnabled(false);
  disableRepalceBtn();
  innerPointTable->pushbackPolygon(pointWidget->getCurrentClickPoint(),pointWidget->getCurrentClickPoint_qt());
  innerPointTable->updateTable();

  // update painter
  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_btn_replace_point_inner_pressed()
{
  disableRepalceBtn();
  QPoint point = pointWidget->getCurrentClickPoint();
  QPoint point_qt = pointWidget->getCurrentClickPoint_qt();
  int i = innerPointTable->currentRow();
  innerPointTable->replacePoint(point, point_qt, i);

  pointWidget->resetMarkPoint();
  // update painter
  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_btn_clean_point_inner_pressed()
{
  innerPointTable->cleanPolygon();
  innerPointTable->updateTable();
  // update painter
  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());

}
// mid
void robot_rqt::MainWindow::slot_btn_set_point_mid_pressed()
{
  enableSetPolytonBtn();
  ui.btn_close_polygon_mid->setEnabled(false);
  disableRepalceBtn();
  midPointTable->pushbackPolygon(pointWidget->getCurrentClickPoint(),pointWidget->getCurrentClickPoint_qt());
  midPointTable->updateTable();
  // update painter
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_btn_replace_point_mid_pressed(){
  disableRepalceBtn();
  QPoint point = pointWidget->getCurrentClickPoint();
  QPoint point_qt = pointWidget->getCurrentClickPoint_qt();
  int i = midPointTable->currentRow();
  midPointTable->replacePoint(point, point_qt, i);

  pointWidget->setMarkPoint(pointWidget->getOriginPos_qt());
  // update painter
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_btn_clean_point_mid_pressed()
{
  midPointTable->cleanPolygon();
  midPointTable->updateTable();

  pointWidget->resetMarkPoint();
  // update painter
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
}
// out
void robot_rqt::MainWindow::slot_btn_set_point_out_pressed(){
  enableSetPolytonBtn();
  ui.btn_close_polygon_out->setEnabled(false);
  disableRepalceBtn();
  outPointTable->pushbackPolygon(pointWidget->getCurrentClickPoint(),pointWidget->getCurrentClickPoint_qt());
  outPointTable->updateTable();
  // update painter
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_btn_replace_point_out_pressed(){
  disableRepalceBtn();
  QPoint point = pointWidget->getCurrentClickPoint();
  QPoint point_qt = pointWidget->getCurrentClickPoint_qt();
  int i = outPointTable->currentRow();
  outPointTable->replacePoint(point, point_qt, i);


  pointWidget->setMarkPoint(pointWidget->getOriginPos_qt());
  // update painter
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_btn_clean_point_out_pressed(){
  outPointTable->cleanPolygon();
  outPointTable->updateTable();

  pointWidget->resetMarkPoint();
  // update painter
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}


void robot_rqt::MainWindow::slot_btn_close_polygon_pressed()
{
  enablePointBtn();
  innerPointTable->cleanPolygon();
  midPointTable->cleanPolygon();
  outPointTable->cleanPolygon();
  pointWidget->resetMarkPolygon();
}
// inner
void robot_rqt::MainWindow::slot_btn_set_polygon_inner_pressed(){
  //qnode.log(qnode.Info,std::string("set a inner polygonn"));

  innerPolygonTable->pushbackVectorPolygon(innerPointTable->getTablePolygon(),innerPointTable->getTablePolygon_qt());
  innerPolygonTable->updateTable();

  innerPointTable->cleanPolygon();
  innerPointTable->updateTable();
  // update painter
  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_btn_close_polygon_inner_pressed(){
  enablePointBtn();
  ui.btn_set_polygon_inner->setEnabled(true);

}
void robot_rqt::MainWindow::slot_btn_clean_polygon_inner_pressed(){
  pointWidget->resetMarkPoint();
  pointWidget->resetMarkPolygon();

  innerPolygonTable->cleanVectorPolygon();
  innerPolygonTable->updateTable();

  innerPointTable->cleanPolygon();
  innerPointTable->updateTable();
  // update painter
  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
}
// mid
void robot_rqt::MainWindow::slot_btn_set_polygon_mid_pressed(){
  //qnode.log(qnode.Info,std::string("set a mid polygonn"));
  midPolygonTable->pushbackVectorPolygon(midPointTable->getTablePolygon(),midPointTable->getTablePolygon_qt());
  midPolygonTable->updateTable();

  midPointTable->cleanPolygon();
  midPointTable->updateTable();

  // update painter
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_btn_close_polygon_mid_pressed(){
  enablePointBtn();
  ui.btn_set_polygon_mid->setEnabled(true);
  midPointTable->cleanPolygon();
  pointWidget->resetMarkPolygon();
}
void robot_rqt::MainWindow::slot_btn_clean_polygon_mid_pressed(){
  pointWidget->resetMarkPoint();
  pointWidget->resetMarkPolygon();

  midPolygonTable->cleanVectorPolygon();
  midPolygonTable->updateTable();

  // update painter
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
}
// out
void robot_rqt::MainWindow::slot_btn_set_polygon_out_pressed(){
  //qnode.log(qnode.Info,std::string("set a out polygonn"));

  outPolygonTable->pushbackVectorPolygon(outPointTable->getTablePolygon(),outPointTable->getTablePolygon_qt());
  outPolygonTable->updateTable();

  outPointTable->cleanPolygon();
  outPointTable->updateTable();

  // update painter
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_btn_close_polygon_out_pressed(){
  enablePointBtn();
  ui.btn_set_polygon_out->setEnabled(true);
  outPointTable->cleanPolygon();
  pointWidget->resetMarkPolygon();
}
void robot_rqt::MainWindow::slot_btn_clean_polygon_out_pressed(){
  pointWidget->resetMarkPoint();
  pointWidget->resetMarkPolygon();

  outPolygonTable->cleanVectorPolygon();
  outPolygonTable->updateTable();

  // update painter
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}
// scenes
void robot_rqt::MainWindow::slot_btn_set_scene_pressed()
{
  /* 0.judge empty
   * 1.generate scene
   * 2.update scene table
   * 3.clean vector_polygon & polygon
   * 4.painer update
   */

  if(innerPolygonTable->getTableVectorPolygon().size()>0 &&
     midPolygonTable->getTableVectorPolygon().size()>0 &&
     outPolygonTable->getTableVectorPolygon().size()>0)
  {
    sceneTable->generateScene(innerPolygonTable->getTableVectorPolygon(), midPolygonTable->getTableVectorPolygon(), outPolygonTable->getTableVectorPolygon());

    innerPointTable->cleanPolygon();
    midPointTable->cleanPolygon();
    outPointTable->cleanPolygon();

    innerPolygonTable->cleanVectorPolygon();
    midPolygonTable->cleanVectorPolygon();
    outPolygonTable->cleanVectorPolygon();

    pointWidget->resetMarkPoint();
    pointWidget->resetMarkPolygon();
    pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
    pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
    pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
  }
}
void robot_rqt::MainWindow::slot_btn_replace_scene_pressed()
{
  /* 0.judge empty
   * 1.replace scene
   * 2.update scene table
   * 3.clean vector_polygon & polygon
   * 4.painer update
   */

  if(innerPolygonTable->getTableVectorPolygon().size()>0 &&
     midPolygonTable->getTableVectorPolygon().size()>0 &&
     outPolygonTable->getTableVectorPolygon().size()>0)
  {
    sceneTable->replaceScene(innerPolygonTable->getTableVectorPolygon(), midPolygonTable->getTableVectorPolygon(), outPolygonTable->getTableVectorPolygon());

    innerPointTable->cleanPolygon();
    midPointTable->cleanPolygon();
    outPointTable->cleanPolygon();

    innerPolygonTable->cleanVectorPolygon();
    midPolygonTable->cleanVectorPolygon();
    outPolygonTable->cleanVectorPolygon();

    pointWidget->resetMarkPoint();
    pointWidget->resetMarkPolygon();
    pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
    pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
    pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
  }
}
void robot_rqt::MainWindow::slot_btn_clean_scene_pressed()
{
  pointWidget->resetMarkPoint();
  pointWidget->resetMarkPolygon();

  innerPointTable->cleanPolygon();
  midPointTable->cleanPolygon();
  outPointTable->cleanPolygon();
  innerPolygonTable->cleanVectorPolygon();
  midPolygonTable->cleanVectorPolygon();
  outPolygonTable->cleanVectorPolygon();
  sceneTable->cleanScene();

  //update painter
  pointWidget->resetMarkPoint();
  pointWidget->resetMarkPolygon();
  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}

/**
  * @brief  clean base point polygon && update painnter
  * @param  None
  * @retval None
  */
void robot_rqt::MainWindow::clean_point_polygon_painter()
{
  innerPointTable->cleanPolygon();
  midPointTable->cleanPolygon();
  outPointTable->cleanPolygon();

  innerPolygonTable->cleanVectorPolygon();
  midPolygonTable->cleanVectorPolygon();
  outPolygonTable->cleanVectorPolygon();

  pointWidget->resetMarkPolygon();
  pointWidget->resetMarkPoint();

  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}
/**
  * @brief  generate config.yaml
  * @param  None
  * @retval None
  */
void robot_rqt::MainWindow::slot_btn_scene_output_pressed()
{
  if(sceneTable->rowCount() > 0){
    std::vector<ST_SCENE_OUT> scenes_out_vector;
    QString path = QFileDialog::getExistingDirectory(this,"打开文件","/home/ubuntu/test_yaml");
    path = path + '/' + ui.lineEdit_fileName->text() + ".yaml";
    QString strName = ui.lineEdit_fileName->text();
    qDebug()<<"output path:"<< path;
    scenes_out_vector = scenesToScenesOut(sceneTable->getScenes());
    outputYamlFile(path, scenes_out_vector, strName);
  }
}
void robot_rqt::MainWindow::slot_btn_scene_input_pressed()
{
  /*
   * 1.clean scene
   * 2.get a scene from qnode::scence_input
   * 3.update sceneTable
   * 4.clean all
   * 5.update comboBox
   * 6.clean ...
   */

  sceneTable->generateAllScene(qnode.getScenesInput());

  clean_point_polygon_painter();
}


void robot_rqt::MainWindow::slot_btn_set_topic_pressed()
{
  QString topic = ui.lineEdit_topic->text();
  qnode.setScanTopic(topic.toStdString().data());
  ui.btn_set_topic->setEnabled(false);
  ui.btn_reset_topic->setEnabled(true);
  enableROSmaster();
  qDebug()<< "Please reconnect";
}
void robot_rqt::MainWindow::slot_btn_reset_topic_pressed()
{
  /*
   * 1.disable btn_reset
   * 2.enable btn_set
   * 3.disable rosmaster set
   */
  ui.btn_reset_topic->setEnabled(false);
  ui.btn_set_topic->setEnabled(true);
  disableROSmaster();
}
void robot_rqt::MainWindow::slot_btn_set_paramName_pressed()
{
  std::string str = ui.lineEdit_paramName->text().toStdString();
  ui.btn_set_paramName->setEnabled(false);
  ui.btn_reset_paramName->setEnabled(true);
  qnode.set_inputParamName(str);
  enableROSmaster();
  qDebug()<< "Please reconnect";
}
void robot_rqt::MainWindow::slot_btn_reset_paramName_pressed()
{
  /*
   * 1.disable btn_reset
   * 2.enable btn_set
   * 3.disable rosmaster set
   */
  ui.btn_reset_paramName->setEnabled(false);
  ui.btn_set_paramName->setEnabled(true);
  disableROSmaster();
}

void robot_rqt::MainWindow::enableSetPolytonBtn()
{
  ui.btn_set_polygon_inner->setEnabled(true);
  ui.btn_set_polygon_mid->setEnabled(true);
  ui.btn_set_polygon_out->setEnabled(true);
}

void robot_rqt::MainWindow::disablePolygonBtn()
{
  ui.btn_set_polygon_inner->setEnabled(false);
  ui.btn_set_polygon_mid->setEnabled(false);
  ui.btn_set_polygon_out->setEnabled(false);
  ui.btn_close_polygon_inner->setEnabled(false);
  ui.btn_close_polygon_mid->setEnabled(false);
  ui.btn_close_polygon_out->setEnabled(false);
  ui.btn_clean_polygon_inner->setEnabled(false);
  ui.btn_clean_polygon_mid->setEnabled(false);
  ui.btn_clean_polygon_out->setEnabled(false);
}

void robot_rqt::MainWindow::disableInnerPointBtn()
{
  ui.btn_set_point_inner->setEnabled(false);
  ui.btn_replace_point_inner->setEnabled(false);
  ui.btn_clean_point_inner->setEnabled(false);
}
void robot_rqt::MainWindow::disableMidPointBtn()
{
  ui.btn_set_point_mid->setEnabled(false);
  ui.btn_replace_point_mid->setEnabled(false);
  ui.btn_clean_point_mid->setEnabled(false);

}
void robot_rqt::MainWindow::disableOutPointBtn()
{
  ui.btn_set_point_out->setEnabled(false);
  ui.btn_replace_point_out->setEnabled(false);
  ui.btn_clean_point_out->setEnabled(false);
}
void robot_rqt::MainWindow::disablePointBtn()
{
  disableInnerPointBtn();
  disableMidPointBtn();
  disableOutPointBtn();
}
void robot_rqt::MainWindow::enableInnerPointBtn()
{
  ui.btn_set_point_inner->setEnabled(true);
  ui.btn_replace_point_inner->setEnabled(true);
  ui.btn_clean_point_inner->setEnabled(true);
}
void robot_rqt::MainWindow::enableMidPointBtn()
{
  ui.btn_set_point_mid->setEnabled(true);
  ui.btn_replace_point_mid->setEnabled(true);
  ui.btn_clean_point_mid->setEnabled(true);

}
void robot_rqt::MainWindow::enableOutPointBtn()
{
  ui.btn_set_point_out->setEnabled(true);
  ui.btn_replace_point_out->setEnabled(true);
  ui.btn_clean_point_out->setEnabled(true);
}
void robot_rqt::MainWindow::enablePointBtn()
{
  enableInnerPointBtn();
  enableMidPointBtn();
  enableOutPointBtn();
}

void robot_rqt::MainWindow::slot_inner_del_painter(int i)
{
  if(innerPolygonTable->getReadOnly() == false){
    innerPointTable->delAppointPolygon(i);
  }
  // update painter from base polygon_qt
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_mid_del_painter(int i)
{
  if(midPolygonTable->getReadOnly() == false){
    midPointTable->delAppointPolygon(i);
  }
  // update painter from base polygon_qt
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_out_del_painter(int i)
{
  if(outPolygonTable->getReadOnly() == false){
    outPointTable->delAppointPolygon(i);
  }
  // update painter from base polygon_qt
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_clean_point_polygon_painter()
{
  clean_point_polygon_painter();
}
/**
  * @brief
  * @param  row: scene table current row
  * @retval None
  */
void robot_rqt::MainWindow::slot_menu_edit_scene()
{
  /*
   * 1.enable scene btn
   *
   *
   */
  ui.btn_set_scene->setEnabled(true);
  ui.btn_replace_scene->setEnabled(true);
}

void robot_rqt::MainWindow::slot_painter_Mark(QPoint point_qt)
{
  pointWidget->setMarkPoint(point_qt);
  pointWidget->update();
}
// right click menu insert
void robot_rqt::MainWindow::slot_inner_insert_point(int i)
{
  if(innerPolygonTable->getReadOnly() == false){
    innerPointTable->insertAppointPolygon(pointWidget->getCurrentClickPoint(),pointWidget->getCurrentClickPoint_qt(),i);
    // update painter from base polygon_qt
    pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
  }
}
void robot_rqt::MainWindow::slot_mid_insert_point(int i)
{
  if(midPolygonTable->getReadOnly() == false){
    midPointTable->insertAppointPolygon(pointWidget->getCurrentClickPoint(),pointWidget->getCurrentClickPoint_qt(),i);
    // update painter from base polygon_qt
    pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
  }
}
void robot_rqt::MainWindow::slot_out_insert_point(int i)
{
  if(outPolygonTable->getReadOnly() == false){
    outPointTable->insertAppointPolygon(pointWidget->getCurrentClickPoint(),pointWidget->getCurrentClickPoint_qt(),i);
    // update painter from base polygon_qt
    pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
  }
}

/**
  * @brief  doubleClick table_polygon
  * @param
  * @param
  * @retval None
  */
void robot_rqt::MainWindow::slot_painter_inner_MarkPolygon(QPolygon polygon, QPolygon polygon_qt)
{
  /*
   * 1.enable btn edit
   * 2.setTablePoint base && update table
   * 3.painter markPolygon
   * 4.update painter
   */
  disablePointBtn();
  ui.btn_set_polygon_inner->setEnabled(false);
  ui.btn_close_polygon_inner->setEnabled(true);
  innerPointTable->setTablePolygon(polygon,polygon_qt);
  pointWidget->setMarkPolygon(polygon_qt);
  pointWidget->update();
}
void robot_rqt::MainWindow::slot_painter_mid_MarkPolygon(QPolygon polygon, QPolygon polygon_qt)
{
  /*
   * 1.enable btn edit
   * 2.setTablePoint base && update table
   * 3.painter markPolygon
   * 4.update painter
   */
  disablePointBtn();
  ui.btn_set_polygon_mid->setEnabled(false);
  ui.btn_close_polygon_mid->setEnabled(true);
  midPointTable->setTablePolygon(polygon,polygon_qt);
  pointWidget->setMarkPolygon(polygon_qt);
  pointWidget->update();
}
void robot_rqt::MainWindow::slot_painter_out_MarkPolygon(QPolygon polygon, QPolygon polygon_qt)
{
  /*
   * 1.enable btn edit
   * 2.setTablePoint base && update table
   * 3.painter markPolygon
   * 4.update painter
   */
  disablePointBtn();
  ui.btn_set_polygon_out->setEnabled(false);
  ui.btn_close_polygon_out->setEnabled(true);
  outPointTable->setTablePolygon(polygon,polygon_qt);
  pointWidget->setMarkPolygon(polygon_qt);
  pointWidget->update();
}
void robot_rqt::MainWindow::slot_edit_innerPolygon_table()
{
  /*
   * 1.btn enable
   * 2.clean appoint polygon table
   * 3.cancer mark_polygon
   * 4.update polygon table
   */
  enablePointBtn();
  ui.btn_set_polygon_inner->setEnabled(true);
  ui.btn_close_polygon_inner->setEnabled(false);

  innerPolygonTable->delAppointPolygon(innerPolygonTable->currentRow());


  pointWidget->resetMarkPolygon();
  pointWidget->resetMarkPoint();
  // update painter
  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_edit_midPolygon_table()
{
  /*
   * 1.btn enable
   * 2.clean appoint polygon table
   * 3.cancer mark_polygon
   * 4.update polygon table
   */
  enablePointBtn();
  ui.btn_set_polygon_mid->setEnabled(true);
  ui.btn_close_polygon_mid->setEnabled(false);

  midPolygonTable->delAppointPolygon(midPolygonTable->currentRow());


  pointWidget->resetMarkPolygon();
  pointWidget->resetMarkPoint();
  // update painter
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
}
void robot_rqt::MainWindow::slot_edit_outPolygon_table()
{
  /*
   * 1.btn enable
   * 2.clean appoint polygon table
   * 3.cancer mark_polygon
   * 4.update polygon table
   */
  enablePointBtn();
  ui.btn_set_polygon_out->setEnabled(true);
  ui.btn_close_polygon_out->setEnabled(false);

  outPolygonTable->delAppointPolygon(outPolygonTable->currentRow());


  pointWidget->resetMarkPolygon();
  pointWidget->resetMarkPoint();
  // update painter
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}

void robot_rqt::MainWindow::slot_request_enable_replaceBtn()
{
  if(!innerPolygonTable->getReadOnly()){
    ui.btn_replace_point_inner->setEnabled(true);
  }
  if(!midPolygonTable->getReadOnly()){
    ui.btn_replace_point_mid->setEnabled(true);
  }
  if(!outPolygonTable->getReadOnly()){
    ui.btn_replace_point_out->setEnabled(true);
  }

}

void robot_rqt::MainWindow::slot_launchUpdateTable(std::vector<QPolygon> innerVpolygon,std::vector<QPolygon> midVpolygon,std::vector<QPolygon> outVpolygon)
{
  /*
   * 0.disable scene set replace btn
   * 1.resetMarkPoint
   * 2.clean point table
   * 3.set Polygon table
   * 4.update painter
   */

  ui.btn_set_scene->setEnabled(false);//=== read only
  ui.btn_replace_scene->setEnabled(false);

  pointWidget->resetMarkPoint();
  innerPointTable->cleanPolygon();
  midPointTable->cleanPolygon();
  outPointTable->cleanPolygon();

  pointWidget->resetMarkPolygon();

  innerPolygonTable->setTableVectorPolygon(innerVpolygon);
  midPolygonTable->setTableVectorPolygon(midVpolygon);
  outPolygonTable->setTableVectorPolygon(outVpolygon);
  // update painter
  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}

void robot_rqt::MainWindow::slot_update_painter()
{
  // update painter
  pointWidget->updateVectorInnerArea(innerPolygonTable->getTableVectorPolygon_qt(),innerPointTable->getTablePolygon_qt());
  pointWidget->updateVectorMidArea(midPolygonTable->getTableVectorPolygon_qt(),midPointTable->getTablePolygon_qt());
  pointWidget->updateVectorOutArea(outPolygonTable->getTableVectorPolygon_qt(),outPointTable->getTablePolygon_qt());
}
