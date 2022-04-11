#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "navigation_info.h"
#include "update_trigger.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow, public update_trigger
{
    Q_OBJECT
private:
    //mainwindow control, timer driven
    int timer_id;

    //variables relevant to display
    QImage map_image;


    double scale;//meters per pixel
    double zoom_of_map;//display window on the map
    QRect cur_window_on_map;//display region of the map
    bool left_button_down;
    QPoint cur_mouse_point;

    //variables relevant to navigation
    Nav_navigation_info nav_info;
    bool in_navigation;
    QPoint nav_des_pos;

private:
    //functions relevant to display calculation, used only in funcions of this class
    QPoint trans_pos_in_map_to_window(const QPoint& cur_position, const QRect& window_rect_in_map);
    QPoint trans_window_to_pos_on_map(const QPoint& cur_position, const QRect& window_rect_in_map);
    QPoint locate_pos_in_cellcenter(QPoint pos);
    QRect calc_image_center_pos(QPoint& cur_position, QRect& image_rect, double scale_of_zoom);
    void trans_meter_to_pixel();

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    int InitMainWindow();

private:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void changeEvent(QEvent *event);
    void mouseDoubleClickEvent(QMouseEvent* event);
    void timerEvent(QTimerEvent* event);
    void keyPressEvent(QKeyEvent* event);

public:
    Nav_navigation_info* get_nav_info_ref();

    virtual void update_window();
    void start_navigation();


private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
