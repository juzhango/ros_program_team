/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTableWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_10;
    QFrame *frame;
    QVBoxLayout *verticalLayout_25;
    QTabWidget *tabWidget_3;
    QWidget *tab;
    QVBoxLayout *verticalLayout_24;
    QGroupBox *groupBox_10;
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *verticalLayout_3;
    QLineEdit *lineEdit_topic;
    QHBoxLayout *horizontalLayout_13;
    QPushButton *btn_set_topic;
    QPushButton *btn_reset_topic;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_26;
    QGroupBox *groupBox_11;
    QVBoxLayout *verticalLayout_23;
    QVBoxLayout *verticalLayout_7;
    QLineEdit *lineEdit_paramName;
    QHBoxLayout *horizontalLayout_15;
    QPushButton *btn_set_paramName;
    QPushButton *btn_reset_paramName;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_6;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QLineEdit *line_edit_master;
    QLabel *label_2;
    QLineEdit *line_edit_host;
    QLabel *label_3;
    QLineEdit *line_edit_topic;
    QCheckBox *checkbox_use_environment;
    QCheckBox *checkbox_remember_settings;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *button_connect;
    QPushButton *quit_button;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout;
    QPushButton *btn_set_scene;
    QPushButton *btn_replace_scene;
    QPushButton *btn_clean_scene;
    QTableWidget *table_scene;
    QLabel *label_5;
    QHBoxLayout *horizontalLayout_14;
    QLineEdit *lineEdit_fileName;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_11;
    QPushButton *btn_scene_output;
    QPushButton *btn_scene_input;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_9;
    QTabWidget *tabWidget;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_8;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_12;
    QGridLayout *gridLayout_3;
    QListView *view_logging;
    QWidget *tab_4;
    QVBoxLayout *verticalLayout_10;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_9;
    QWidget *widget_pointCloud;
    QTabWidget *tabWidget_2;
    QWidget *tab_5;
    QVBoxLayout *verticalLayout_15;
    QVBoxLayout *verticalLayout_11;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_12;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *btn_set_point_inner;
    QPushButton *btn_replace_point_inner;
    QPushButton *btn_clean_point_inner;
    QTableWidget *table_point_inner;
    QGroupBox *groupBox_5;
    QVBoxLayout *verticalLayout_13;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *btn_set_polygon_inner;
    QPushButton *btn_close_polygon_inner;
    QPushButton *btn_clean_polygon_inner;
    QTableWidget *table_polygon_inner;
    QSpacerItem *verticalSpacer_4;
    QWidget *tab_6;
    QVBoxLayout *verticalLayout_16;
    QVBoxLayout *verticalLayout_14;
    QGroupBox *groupBox_6;
    QVBoxLayout *verticalLayout_17;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *btn_set_point_mid;
    QPushButton *btn_replace_point_mid;
    QPushButton *btn_clean_point_mid;
    QTableWidget *table_point_mid;
    QGroupBox *groupBox_7;
    QVBoxLayout *verticalLayout_18;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *btn_set_polygon_mid;
    QPushButton *btn_close_polygon_mid;
    QPushButton *btn_clean_polygon_mid;
    QTableWidget *table_polygon_mid;
    QSpacerItem *verticalSpacer_3;
    QWidget *tab_7;
    QVBoxLayout *verticalLayout_19;
    QVBoxLayout *verticalLayout_20;
    QGroupBox *groupBox_8;
    QVBoxLayout *verticalLayout_21;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *btn_set_point_out;
    QPushButton *btn_replace_point_out;
    QPushButton *btn_clean_point_out;
    QTableWidget *table_point_out;
    QGroupBox *groupBox_9;
    QVBoxLayout *verticalLayout_22;
    QHBoxLayout *horizontalLayout_8;
    QPushButton *btn_set_polygon_out;
    QPushButton *btn_close_polygon_out;
    QPushButton *btn_clean_polygon_out;
    QTableWidget *table_polygon_out;
    QSpacerItem *verticalSpacer_2;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(1568, 973);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout_10 = new QHBoxLayout(centralwidget);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setMinimumSize(QSize(250, 0));
        frame->setMaximumSize(QSize(250, 16777215));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_25 = new QVBoxLayout(frame);
        verticalLayout_25->setObjectName(QString::fromUtf8("verticalLayout_25"));
        tabWidget_3 = new QTabWidget(frame);
        tabWidget_3->setObjectName(QString::fromUtf8("tabWidget_3"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_24 = new QVBoxLayout(tab);
        verticalLayout_24->setObjectName(QString::fromUtf8("verticalLayout_24"));
        groupBox_10 = new QGroupBox(tab);
        groupBox_10->setObjectName(QString::fromUtf8("groupBox_10"));
        verticalLayout_5 = new QVBoxLayout(groupBox_10);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        lineEdit_topic = new QLineEdit(groupBox_10);
        lineEdit_topic->setObjectName(QString::fromUtf8("lineEdit_topic"));

        verticalLayout_3->addWidget(lineEdit_topic);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        btn_set_topic = new QPushButton(groupBox_10);
        btn_set_topic->setObjectName(QString::fromUtf8("btn_set_topic"));

        horizontalLayout_13->addWidget(btn_set_topic);

        btn_reset_topic = new QPushButton(groupBox_10);
        btn_reset_topic->setObjectName(QString::fromUtf8("btn_reset_topic"));

        horizontalLayout_13->addWidget(btn_reset_topic);


        verticalLayout_3->addLayout(horizontalLayout_13);


        verticalLayout_5->addLayout(verticalLayout_3);


        verticalLayout_24->addWidget(groupBox_10);

        tabWidget_3->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_26 = new QVBoxLayout(tab_2);
        verticalLayout_26->setObjectName(QString::fromUtf8("verticalLayout_26"));
        groupBox_11 = new QGroupBox(tab_2);
        groupBox_11->setObjectName(QString::fromUtf8("groupBox_11"));
        verticalLayout_23 = new QVBoxLayout(groupBox_11);
        verticalLayout_23->setObjectName(QString::fromUtf8("verticalLayout_23"));
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        lineEdit_paramName = new QLineEdit(groupBox_11);
        lineEdit_paramName->setObjectName(QString::fromUtf8("lineEdit_paramName"));

        verticalLayout_7->addWidget(lineEdit_paramName);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        btn_set_paramName = new QPushButton(groupBox_11);
        btn_set_paramName->setObjectName(QString::fromUtf8("btn_set_paramName"));

        horizontalLayout_15->addWidget(btn_set_paramName);

        btn_reset_paramName = new QPushButton(groupBox_11);
        btn_reset_paramName->setObjectName(QString::fromUtf8("btn_reset_paramName"));

        horizontalLayout_15->addWidget(btn_reset_paramName);


        verticalLayout_7->addLayout(horizontalLayout_15);


        verticalLayout_23->addLayout(verticalLayout_7);


        verticalLayout_26->addWidget(groupBox_11);

        tabWidget_3->addTab(tab_2, QString());

        verticalLayout_25->addWidget(tabWidget_3);

        groupBox = new QGroupBox(frame);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMinimumSize(QSize(0, 300));
        groupBox->setMaximumSize(QSize(16777215, 300));
        verticalLayout_6 = new QVBoxLayout(groupBox);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFrameShape(QFrame::StyledPanel);
        label->setFrameShadow(QFrame::Raised);

        verticalLayout->addWidget(label);

        line_edit_master = new QLineEdit(groupBox);
        line_edit_master->setObjectName(QString::fromUtf8("line_edit_master"));

        verticalLayout->addWidget(line_edit_master);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setFrameShape(QFrame::StyledPanel);
        label_2->setFrameShadow(QFrame::Raised);

        verticalLayout->addWidget(label_2);

        line_edit_host = new QLineEdit(groupBox);
        line_edit_host->setObjectName(QString::fromUtf8("line_edit_host"));

        verticalLayout->addWidget(line_edit_host);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setFrameShape(QFrame::StyledPanel);
        label_3->setFrameShadow(QFrame::Raised);

        verticalLayout->addWidget(label_3);

        line_edit_topic = new QLineEdit(groupBox);
        line_edit_topic->setObjectName(QString::fromUtf8("line_edit_topic"));
        line_edit_topic->setEnabled(false);

        verticalLayout->addWidget(line_edit_topic);

        checkbox_use_environment = new QCheckBox(groupBox);
        checkbox_use_environment->setObjectName(QString::fromUtf8("checkbox_use_environment"));
        checkbox_use_environment->setLayoutDirection(Qt::RightToLeft);

        verticalLayout->addWidget(checkbox_use_environment);

        checkbox_remember_settings = new QCheckBox(groupBox);
        checkbox_remember_settings->setObjectName(QString::fromUtf8("checkbox_remember_settings"));
        checkbox_remember_settings->setLayoutDirection(Qt::RightToLeft);

        verticalLayout->addWidget(checkbox_remember_settings);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        button_connect = new QPushButton(groupBox);
        button_connect->setObjectName(QString::fromUtf8("button_connect"));
        button_connect->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(button_connect->sizePolicy().hasHeightForWidth());
        button_connect->setSizePolicy(sizePolicy1);

        horizontalLayout_2->addWidget(button_connect);

        quit_button = new QPushButton(groupBox);
        quit_button->setObjectName(QString::fromUtf8("quit_button"));
        sizePolicy1.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy1);

        horizontalLayout_2->addWidget(quit_button);


        verticalLayout->addLayout(horizontalLayout_2);


        verticalLayout_6->addLayout(verticalLayout);


        verticalLayout_25->addWidget(groupBox);

        groupBox_2 = new QGroupBox(frame);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setMinimumSize(QSize(0, 285));
        groupBox_2->setMaximumSize(QSize(16777215, 285));
        verticalLayout_4 = new QVBoxLayout(groupBox_2);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        btn_set_scene = new QPushButton(groupBox_2);
        btn_set_scene->setObjectName(QString::fromUtf8("btn_set_scene"));

        horizontalLayout->addWidget(btn_set_scene);

        btn_replace_scene = new QPushButton(groupBox_2);
        btn_replace_scene->setObjectName(QString::fromUtf8("btn_replace_scene"));

        horizontalLayout->addWidget(btn_replace_scene);

        btn_clean_scene = new QPushButton(groupBox_2);
        btn_clean_scene->setObjectName(QString::fromUtf8("btn_clean_scene"));

        horizontalLayout->addWidget(btn_clean_scene);


        verticalLayout_4->addLayout(horizontalLayout);

        table_scene = new QTableWidget(groupBox_2);
        table_scene->setObjectName(QString::fromUtf8("table_scene"));
        table_scene->setMinimumSize(QSize(200, 120));
        table_scene->setMaximumSize(QSize(200, 120));

        verticalLayout_4->addWidget(table_scene);

        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setMinimumSize(QSize(0, 20));
        label_5->setMaximumSize(QSize(16777215, 20));
        label_5->setFrameShape(QFrame::StyledPanel);

        verticalLayout_4->addWidget(label_5);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        lineEdit_fileName = new QLineEdit(groupBox_2);
        lineEdit_fileName->setObjectName(QString::fromUtf8("lineEdit_fileName"));

        horizontalLayout_14->addWidget(lineEdit_fileName);

        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_14->addWidget(label_4);


        verticalLayout_4->addLayout(horizontalLayout_14);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        btn_scene_output = new QPushButton(groupBox_2);
        btn_scene_output->setObjectName(QString::fromUtf8("btn_scene_output"));

        horizontalLayout_11->addWidget(btn_scene_output);

        btn_scene_input = new QPushButton(groupBox_2);
        btn_scene_input->setObjectName(QString::fromUtf8("btn_scene_input"));

        horizontalLayout_11->addWidget(btn_scene_input);


        verticalLayout_4->addLayout(horizontalLayout_11);


        verticalLayout_25->addWidget(groupBox_2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_25->addItem(verticalSpacer);


        horizontalLayout_10->addWidget(frame);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout_8 = new QVBoxLayout(tab_3);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        tab_manager = new QTabWidget(tab_3);
        tab_manager->setObjectName(QString::fromUtf8("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_status = new QWidget();
        tab_status->setObjectName(QString::fromUtf8("tab_status"));
        verticalLayout_2 = new QVBoxLayout(tab_status);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        groupBox_12 = new QGroupBox(tab_status);
        groupBox_12->setObjectName(QString::fromUtf8("groupBox_12"));
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(groupBox_12->sizePolicy().hasHeightForWidth());
        groupBox_12->setSizePolicy(sizePolicy2);
        gridLayout_3 = new QGridLayout(groupBox_12);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        view_logging = new QListView(groupBox_12);
        view_logging->setObjectName(QString::fromUtf8("view_logging"));

        gridLayout_3->addWidget(view_logging, 0, 0, 1, 1);


        verticalLayout_2->addWidget(groupBox_12);

        tab_manager->addTab(tab_status, QString());

        verticalLayout_8->addWidget(tab_manager);

        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        verticalLayout_10 = new QVBoxLayout(tab_4);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        groupBox_3 = new QGroupBox(tab_4);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        verticalLayout_9 = new QVBoxLayout(groupBox_3);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        widget_pointCloud = new QWidget(groupBox_3);
        widget_pointCloud->setObjectName(QString::fromUtf8("widget_pointCloud"));
        widget_pointCloud->setMinimumSize(QSize(1000, 720));
        widget_pointCloud->setMaximumSize(QSize(1000, 720));

        verticalLayout_9->addWidget(widget_pointCloud);


        verticalLayout_10->addWidget(groupBox_3);

        tabWidget->addTab(tab_4, QString());

        horizontalLayout_9->addWidget(tabWidget);

        tabWidget_2 = new QTabWidget(centralwidget);
        tabWidget_2->setObjectName(QString::fromUtf8("tabWidget_2"));
        tabWidget_2->setMinimumSize(QSize(240, 0));
        tabWidget_2->setMaximumSize(QSize(240, 16777215));
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        verticalLayout_15 = new QVBoxLayout(tab_5);
        verticalLayout_15->setObjectName(QString::fromUtf8("verticalLayout_15"));
        verticalLayout_11 = new QVBoxLayout();
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        groupBox_4 = new QGroupBox(tab_5);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        verticalLayout_12 = new QVBoxLayout(groupBox_4);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        btn_set_point_inner = new QPushButton(groupBox_4);
        btn_set_point_inner->setObjectName(QString::fromUtf8("btn_set_point_inner"));

        horizontalLayout_3->addWidget(btn_set_point_inner);

        btn_replace_point_inner = new QPushButton(groupBox_4);
        btn_replace_point_inner->setObjectName(QString::fromUtf8("btn_replace_point_inner"));

        horizontalLayout_3->addWidget(btn_replace_point_inner);

        btn_clean_point_inner = new QPushButton(groupBox_4);
        btn_clean_point_inner->setObjectName(QString::fromUtf8("btn_clean_point_inner"));

        horizontalLayout_3->addWidget(btn_clean_point_inner);


        verticalLayout_12->addLayout(horizontalLayout_3);

        table_point_inner = new QTableWidget(groupBox_4);
        table_point_inner->setObjectName(QString::fromUtf8("table_point_inner"));
        table_point_inner->setMinimumSize(QSize(190, 350));
        table_point_inner->setMaximumSize(QSize(190, 350));

        verticalLayout_12->addWidget(table_point_inner);


        verticalLayout_11->addWidget(groupBox_4);

        groupBox_5 = new QGroupBox(tab_5);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        verticalLayout_13 = new QVBoxLayout(groupBox_5);
        verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        btn_set_polygon_inner = new QPushButton(groupBox_5);
        btn_set_polygon_inner->setObjectName(QString::fromUtf8("btn_set_polygon_inner"));

        horizontalLayout_4->addWidget(btn_set_polygon_inner);

        btn_close_polygon_inner = new QPushButton(groupBox_5);
        btn_close_polygon_inner->setObjectName(QString::fromUtf8("btn_close_polygon_inner"));

        horizontalLayout_4->addWidget(btn_close_polygon_inner);

        btn_clean_polygon_inner = new QPushButton(groupBox_5);
        btn_clean_polygon_inner->setObjectName(QString::fromUtf8("btn_clean_polygon_inner"));

        horizontalLayout_4->addWidget(btn_clean_polygon_inner);


        verticalLayout_13->addLayout(horizontalLayout_4);

        table_polygon_inner = new QTableWidget(groupBox_5);
        table_polygon_inner->setObjectName(QString::fromUtf8("table_polygon_inner"));
        table_polygon_inner->setMinimumSize(QSize(190, 300));
        table_polygon_inner->setMaximumSize(QSize(190, 300));

        verticalLayout_13->addWidget(table_polygon_inner);


        verticalLayout_11->addWidget(groupBox_5);


        verticalLayout_15->addLayout(verticalLayout_11);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_15->addItem(verticalSpacer_4);

        tabWidget_2->addTab(tab_5, QString());
        tab_6 = new QWidget();
        tab_6->setObjectName(QString::fromUtf8("tab_6"));
        verticalLayout_16 = new QVBoxLayout(tab_6);
        verticalLayout_16->setObjectName(QString::fromUtf8("verticalLayout_16"));
        verticalLayout_14 = new QVBoxLayout();
        verticalLayout_14->setObjectName(QString::fromUtf8("verticalLayout_14"));
        groupBox_6 = new QGroupBox(tab_6);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        verticalLayout_17 = new QVBoxLayout(groupBox_6);
        verticalLayout_17->setObjectName(QString::fromUtf8("verticalLayout_17"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        btn_set_point_mid = new QPushButton(groupBox_6);
        btn_set_point_mid->setObjectName(QString::fromUtf8("btn_set_point_mid"));

        horizontalLayout_5->addWidget(btn_set_point_mid);

        btn_replace_point_mid = new QPushButton(groupBox_6);
        btn_replace_point_mid->setObjectName(QString::fromUtf8("btn_replace_point_mid"));

        horizontalLayout_5->addWidget(btn_replace_point_mid);

        btn_clean_point_mid = new QPushButton(groupBox_6);
        btn_clean_point_mid->setObjectName(QString::fromUtf8("btn_clean_point_mid"));

        horizontalLayout_5->addWidget(btn_clean_point_mid);


        verticalLayout_17->addLayout(horizontalLayout_5);

        table_point_mid = new QTableWidget(groupBox_6);
        table_point_mid->setObjectName(QString::fromUtf8("table_point_mid"));
        table_point_mid->setMinimumSize(QSize(190, 350));
        table_point_mid->setMaximumSize(QSize(190, 350));

        verticalLayout_17->addWidget(table_point_mid);


        verticalLayout_14->addWidget(groupBox_6);

        groupBox_7 = new QGroupBox(tab_6);
        groupBox_7->setObjectName(QString::fromUtf8("groupBox_7"));
        verticalLayout_18 = new QVBoxLayout(groupBox_7);
        verticalLayout_18->setObjectName(QString::fromUtf8("verticalLayout_18"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        btn_set_polygon_mid = new QPushButton(groupBox_7);
        btn_set_polygon_mid->setObjectName(QString::fromUtf8("btn_set_polygon_mid"));

        horizontalLayout_6->addWidget(btn_set_polygon_mid);

        btn_close_polygon_mid = new QPushButton(groupBox_7);
        btn_close_polygon_mid->setObjectName(QString::fromUtf8("btn_close_polygon_mid"));

        horizontalLayout_6->addWidget(btn_close_polygon_mid);

        btn_clean_polygon_mid = new QPushButton(groupBox_7);
        btn_clean_polygon_mid->setObjectName(QString::fromUtf8("btn_clean_polygon_mid"));

        horizontalLayout_6->addWidget(btn_clean_polygon_mid);


        verticalLayout_18->addLayout(horizontalLayout_6);

        table_polygon_mid = new QTableWidget(groupBox_7);
        table_polygon_mid->setObjectName(QString::fromUtf8("table_polygon_mid"));
        table_polygon_mid->setMinimumSize(QSize(190, 300));
        table_polygon_mid->setMaximumSize(QSize(190, 300));

        verticalLayout_18->addWidget(table_polygon_mid);


        verticalLayout_14->addWidget(groupBox_7);


        verticalLayout_16->addLayout(verticalLayout_14);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_16->addItem(verticalSpacer_3);

        tabWidget_2->addTab(tab_6, QString());
        tab_7 = new QWidget();
        tab_7->setObjectName(QString::fromUtf8("tab_7"));
        verticalLayout_19 = new QVBoxLayout(tab_7);
        verticalLayout_19->setObjectName(QString::fromUtf8("verticalLayout_19"));
        verticalLayout_20 = new QVBoxLayout();
        verticalLayout_20->setObjectName(QString::fromUtf8("verticalLayout_20"));
        groupBox_8 = new QGroupBox(tab_7);
        groupBox_8->setObjectName(QString::fromUtf8("groupBox_8"));
        verticalLayout_21 = new QVBoxLayout(groupBox_8);
        verticalLayout_21->setObjectName(QString::fromUtf8("verticalLayout_21"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        btn_set_point_out = new QPushButton(groupBox_8);
        btn_set_point_out->setObjectName(QString::fromUtf8("btn_set_point_out"));

        horizontalLayout_7->addWidget(btn_set_point_out);

        btn_replace_point_out = new QPushButton(groupBox_8);
        btn_replace_point_out->setObjectName(QString::fromUtf8("btn_replace_point_out"));

        horizontalLayout_7->addWidget(btn_replace_point_out);

        btn_clean_point_out = new QPushButton(groupBox_8);
        btn_clean_point_out->setObjectName(QString::fromUtf8("btn_clean_point_out"));

        horizontalLayout_7->addWidget(btn_clean_point_out);


        verticalLayout_21->addLayout(horizontalLayout_7);

        table_point_out = new QTableWidget(groupBox_8);
        table_point_out->setObjectName(QString::fromUtf8("table_point_out"));
        table_point_out->setMinimumSize(QSize(190, 350));
        table_point_out->setMaximumSize(QSize(190, 350));

        verticalLayout_21->addWidget(table_point_out);


        verticalLayout_20->addWidget(groupBox_8);

        groupBox_9 = new QGroupBox(tab_7);
        groupBox_9->setObjectName(QString::fromUtf8("groupBox_9"));
        verticalLayout_22 = new QVBoxLayout(groupBox_9);
        verticalLayout_22->setObjectName(QString::fromUtf8("verticalLayout_22"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        btn_set_polygon_out = new QPushButton(groupBox_9);
        btn_set_polygon_out->setObjectName(QString::fromUtf8("btn_set_polygon_out"));

        horizontalLayout_8->addWidget(btn_set_polygon_out);

        btn_close_polygon_out = new QPushButton(groupBox_9);
        btn_close_polygon_out->setObjectName(QString::fromUtf8("btn_close_polygon_out"));

        horizontalLayout_8->addWidget(btn_close_polygon_out);

        btn_clean_polygon_out = new QPushButton(groupBox_9);
        btn_clean_polygon_out->setObjectName(QString::fromUtf8("btn_clean_polygon_out"));

        horizontalLayout_8->addWidget(btn_clean_polygon_out);


        verticalLayout_22->addLayout(horizontalLayout_8);

        table_polygon_out = new QTableWidget(groupBox_9);
        table_polygon_out->setObjectName(QString::fromUtf8("table_polygon_out"));
        table_polygon_out->setMinimumSize(QSize(190, 300));
        table_polygon_out->setMaximumSize(QSize(190, 300));

        verticalLayout_22->addWidget(table_polygon_out);


        verticalLayout_20->addWidget(groupBox_9);


        verticalLayout_19->addLayout(verticalLayout_20);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_19->addItem(verticalSpacer_2);

        tabWidget_2->addTab(tab_7, QString());

        horizontalLayout_9->addWidget(tabWidget_2);


        horizontalLayout_10->addLayout(horizontalLayout_9);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1568, 22));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));

        tabWidget_3->setCurrentIndex(1);
        tabWidget->setCurrentIndex(1);
        tab_manager->setCurrentIndex(0);
        tabWidget_2->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        groupBox_10->setTitle(QApplication::translate("MainWindowDesign", "topic_choose", 0, QApplication::UnicodeUTF8));
        lineEdit_topic->setText(QApplication::translate("MainWindowDesign", "/scan", 0, QApplication::UnicodeUTF8));
        btn_set_topic->setText(QApplication::translate("MainWindowDesign", "topic_set", 0, QApplication::UnicodeUTF8));
        btn_reset_topic->setText(QApplication::translate("MainWindowDesign", "topic_reset", 0, QApplication::UnicodeUTF8));
        tabWidget_3->setTabText(tabWidget_3->indexOf(tab), QApplication::translate("MainWindowDesign", "Tab 1", 0, QApplication::UnicodeUTF8));
        groupBox_11->setTitle(QApplication::translate("MainWindowDesign", "ParamName", 0, QApplication::UnicodeUTF8));
        lineEdit_paramName->setText(QApplication::translate("MainWindowDesign", "/robot_rqt/laser_avoid_area_config", 0, QApplication::UnicodeUTF8));
        btn_set_paramName->setText(QApplication::translate("MainWindowDesign", "set param", 0, QApplication::UnicodeUTF8));
        btn_reset_paramName->setText(QApplication::translate("MainWindowDesign", "reset param", 0, QApplication::UnicodeUTF8));
        tabWidget_3->setTabText(tabWidget_3->indexOf(tab_2), QApplication::translate("MainWindowDesign", "Tab 2", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindowDesign", "Ros Master", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindowDesign", "Ros Master Url", 0, QApplication::UnicodeUTF8));
        line_edit_master->setText(QApplication::translate("MainWindowDesign", "http://192.168.1.2:11311/", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindowDesign", "Ros IP", 0, QApplication::UnicodeUTF8));
        line_edit_host->setText(QApplication::translate("MainWindowDesign", "192.168.1.67", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindowDesign", "Ros Hostname", 0, QApplication::UnicodeUTF8));
        line_edit_topic->setText(QApplication::translate("MainWindowDesign", "unused", 0, QApplication::UnicodeUTF8));
        checkbox_use_environment->setText(QApplication::translate("MainWindowDesign", "Use environment variables", 0, QApplication::UnicodeUTF8));
        checkbox_remember_settings->setText(QApplication::translate("MainWindowDesign", "Remember settings", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        button_connect->setToolTip(QApplication::translate("MainWindowDesign", "Set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        button_connect->setStatusTip(QApplication::translate("MainWindowDesign", "Clear all waypoints and set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        button_connect->setText(QApplication::translate("MainWindowDesign", "Connect", 0, QApplication::UnicodeUTF8));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindowDesign", "scenes", 0, QApplication::UnicodeUTF8));
        btn_set_scene->setText(QApplication::translate("MainWindowDesign", "set", 0, QApplication::UnicodeUTF8));
        btn_replace_scene->setText(QApplication::translate("MainWindowDesign", "replace", 0, QApplication::UnicodeUTF8));
        btn_clean_scene->setText(QApplication::translate("MainWindowDesign", "clean", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindowDesign", "File name :", 0, QApplication::UnicodeUTF8));
        lineEdit_fileName->setText(QApplication::translate("MainWindowDesign", "laser_avoid_area_config", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindowDesign", ".yaml", 0, QApplication::UnicodeUTF8));
        btn_scene_output->setText(QApplication::translate("MainWindowDesign", "output", 0, QApplication::UnicodeUTF8));
        btn_scene_input->setText(QApplication::translate("MainWindowDesign", "input", 0, QApplication::UnicodeUTF8));
        groupBox_12->setTitle(QApplication::translate("MainWindowDesign", "Logging", 0, QApplication::UnicodeUTF8));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Ros Communications", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindowDesign", "Tab 1", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("MainWindowDesign", "Point Cloud", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindowDesign", "Tab 2", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("MainWindowDesign", "Point", 0, QApplication::UnicodeUTF8));
        btn_set_point_inner->setText(QApplication::translate("MainWindowDesign", "set", 0, QApplication::UnicodeUTF8));
        btn_replace_point_inner->setText(QApplication::translate("MainWindowDesign", "replace", 0, QApplication::UnicodeUTF8));
        btn_clean_point_inner->setText(QApplication::translate("MainWindowDesign", "clean", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("MainWindowDesign", "Polygon", 0, QApplication::UnicodeUTF8));
        btn_set_polygon_inner->setText(QApplication::translate("MainWindowDesign", "set", 0, QApplication::UnicodeUTF8));
        btn_close_polygon_inner->setText(QApplication::translate("MainWindowDesign", "close", 0, QApplication::UnicodeUTF8));
        btn_clean_polygon_inner->setText(QApplication::translate("MainWindowDesign", "clean", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_5), QApplication::translate("MainWindowDesign", "area_inner", 0, QApplication::UnicodeUTF8));
        groupBox_6->setTitle(QApplication::translate("MainWindowDesign", "Point", 0, QApplication::UnicodeUTF8));
        btn_set_point_mid->setText(QApplication::translate("MainWindowDesign", "set", 0, QApplication::UnicodeUTF8));
        btn_replace_point_mid->setText(QApplication::translate("MainWindowDesign", "replace", 0, QApplication::UnicodeUTF8));
        btn_clean_point_mid->setText(QApplication::translate("MainWindowDesign", "clean", 0, QApplication::UnicodeUTF8));
        groupBox_7->setTitle(QApplication::translate("MainWindowDesign", "Polygon", 0, QApplication::UnicodeUTF8));
        btn_set_polygon_mid->setText(QApplication::translate("MainWindowDesign", "set", 0, QApplication::UnicodeUTF8));
        btn_close_polygon_mid->setText(QApplication::translate("MainWindowDesign", "close", 0, QApplication::UnicodeUTF8));
        btn_clean_polygon_mid->setText(QApplication::translate("MainWindowDesign", "clean", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_6), QApplication::translate("MainWindowDesign", "area_mid", 0, QApplication::UnicodeUTF8));
        groupBox_8->setTitle(QApplication::translate("MainWindowDesign", "Point", 0, QApplication::UnicodeUTF8));
        btn_set_point_out->setText(QApplication::translate("MainWindowDesign", "set", 0, QApplication::UnicodeUTF8));
        btn_replace_point_out->setText(QApplication::translate("MainWindowDesign", "replace", 0, QApplication::UnicodeUTF8));
        btn_clean_point_out->setText(QApplication::translate("MainWindowDesign", "clean", 0, QApplication::UnicodeUTF8));
        groupBox_9->setTitle(QApplication::translate("MainWindowDesign", "Polygon", 0, QApplication::UnicodeUTF8));
        btn_set_polygon_out->setText(QApplication::translate("MainWindowDesign", "set", 0, QApplication::UnicodeUTF8));
        btn_close_polygon_out->setText(QApplication::translate("MainWindowDesign", "close", 0, QApplication::UnicodeUTF8));
        btn_clean_polygon_out->setText(QApplication::translate("MainWindowDesign", "clean", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_7), QApplication::translate("MainWindowDesign", "area_out", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
