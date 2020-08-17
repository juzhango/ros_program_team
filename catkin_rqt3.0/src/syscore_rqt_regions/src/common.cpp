#include "../include/syscore_rqt_regions/common.h"






void showMsgBox(QString str)
{
  QMessageBox msgBox;
  msgBox.setText(str);
  msgBox.exec();
}

