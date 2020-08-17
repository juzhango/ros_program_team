/**
 * @file /include/syscore_rqt_regions/main_window.hpp
 *
 * @brief Qt based gui for syscore_rqt_regions.
 *
 * @date November 2010
 **/
#ifndef syscore_rqt_regions_MAIN_WINDOW_H
#define syscore_rqt_regions_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "QDebug"

#include "../syscore_rqt_regions/output_format_yaml.h"
#include "../syscore_rqt_regions/pointtable.h"
#include "../syscore_rqt_regions/pointwidget.h"
#include "../syscore_rqt_regions/polygontable.h"
#include "../syscore_rqt_regions/qnode.hpp"
#include "../syscore_rqt_regions/scenetable.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace syscore_rqt_regions {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();


  //user
  void clean_point_polygon_painter();
  void updateComboBox();
  void disableROSmaster();
  void enableROSmaster();

  void disablePointBtn();
  void disableInnerPointBtn();
  void disableMidPointBtn();
  void disableOutPointBtn();
  void disableRepalceBtn();

  void enablePointBtn();
  void enableInnerPointBtn();
  void enableMidPointBtn();
  void enableOutPointBtn();
  void enableRepalceBtn();

  void enableSetPolytonBtn();
  void disablePolygonBtn();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void slot_update_painter();//update painter
    void slot_inner_del_painter(int i);
    void slot_mid_del_painter(int i);
    void slot_out_del_painter(int i);
    void slot_clean_point_polygon_painter();//=== clean base point polygon update painter
    void slot_edit_innerPolygon_table();//===polyton edit
    void slot_menu_edit_scene();//=== scnen edit
    void slot_edit_midPolygon_table();
    void slot_edit_outPolygon_table();
    void slot_request_enable_replaceBtn();
    void slot_launchUpdateTable(std::vector<QPolygon>,std::vector<QPolygon>,std::vector<QPolygon>);

    void slot_painter_Mark(QPoint);
    void slot_painter_inner_MarkPolygon(QPolygon polygon,QPolygon polygon_qt); //=== markPolygon
    void slot_painter_mid_MarkPolygon(QPolygon polygon,QPolygon polygon_qt);
    void slot_painter_out_MarkPolygon(QPolygon polygon,QPolygon polygon_qt);
    void slot_inner_insert_point(int);
    void slot_mid_insert_point(int);
    void slot_out_insert_point(int);

    void slot_update_laserScan(QPolygon);

    void slot_btn_set_point_inner_pressed();//=== inner point btn
    void slot_btn_replace_point_inner_pressed();
    void slot_btn_clean_point_inner_pressed();
    void slot_btn_set_point_mid_pressed();//=== mid point btn
    void slot_btn_replace_point_mid_pressed();
    void slot_btn_clean_point_mid_pressed();
    void slot_btn_set_point_out_pressed();//=== out point btn
    void slot_btn_replace_point_out_pressed();
    void slot_btn_clean_point_out_pressed();

    void slot_btn_close_polygon_pressed();
    void slot_btn_set_polygon_inner_pressed();//=== inner polygon
    void slot_btn_close_polygon_inner_pressed();
    void slot_btn_clean_polygon_inner_pressed();
    void slot_btn_set_polygon_mid_pressed();//=== mid polygon
    void slot_btn_close_polygon_mid_pressed();
    void slot_btn_clean_polygon_mid_pressed();
    void slot_btn_set_polygon_out_pressed();//=== out polygon
    void slot_btn_close_polygon_out_pressed();
    void slot_btn_clean_polygon_out_pressed();

    void slot_btn_set_scene_pressed();
    void slot_btn_replace_scene_pressed();
    void slot_btn_clean_scene_pressed();

    void slot_btn_scene_output_pressed();
    void slot_btn_scene_input_pressed();


    void slot_btn_set_topic_pressed();
    void slot_btn_reset_topic_pressed();

    void slot_btn_set_paramName_pressed();
    void slot_btn_reset_paramName_pressed();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  PointWidget * pointWidget;
  PointTable * innerPointTable;
  PointTable * midPointTable;
  PointTable * outPointTable;
  PolygonTable * innerPolygonTable;
  PolygonTable * midPolygonTable;
  PolygonTable * outPolygonTable;

  SceneTable * sceneTable;
};

}  // namespace syscore_rqt_regions

#endif // syscore_rqt_regions_MAIN_WINDOW_H
