#include "../include/robot_rqt/common.h"






void showMsgBox(QString str)
{
  QMessageBox msgBox;
  msgBox.setText(str);
  msgBox.exec();
}

