#include "mainwindow.h"

#include <QApplication>
#include "LSM9DS1_Types.h"
#include "LSM9DS1.h"
#include "ECEF.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.InitMainWindow();
    w.showMaximized();
    return a.exec();
}
