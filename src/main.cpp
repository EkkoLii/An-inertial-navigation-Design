#include "mainwindow.h"

#include <QApplication>
#include <unistd.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.InitMainWindow();
    w.showMaximized();
    /*
    for(int i=0;i<15;i++)
    {
        w.set_cur_pos(w.get_cur_pos().x()+i*50,w.get_cur_pos().y()+i*40);
        w.update_window();
    }
    */
    return a.exec();
}
