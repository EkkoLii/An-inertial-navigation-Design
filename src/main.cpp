#include "mainwindow.h"

#include <QApplication>
#include <unistd.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.InitMainWindow();
    w.showMaximized();
    return a.exec();
}
