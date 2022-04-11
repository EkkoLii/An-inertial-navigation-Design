#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QMouseEvent>
#include <iostream>
#include <qtimer.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->grabKeyboard();
}

MainWindow::~MainWindow()
{
    delete ui;
}

int MainWindow::InitMainWindow()
{
    map_image.load(":/rc/maps/map.png");
    //should be revised to the initial location
    scale=3;
    zoom_of_map=1;

    QPoint cur_position(nav_info.get_cur_pos_on_map());
    QRect image_rect=map_image.rect();
    cur_window_on_map=calc_image_center_pos(cur_position,image_rect,zoom_of_map);
    nav_info.set_cell_number_x(map_image.rect().width()/nav_info.get_cell_size()+1);
    nav_info.set_cell_number_y(image_rect.height()/nav_info.get_cell_size()+1);
    nav_info.init_grid_info();
    nav_info.set_cur_pos_on_map(QPoint(300,300));

    this->setFocusPolicy(Qt::StrongFocus);

    timer_id=startTimer(50);
    left_button_down=false;
    return 1;
}

//Move the window on the map image to show the part of image which current position is in the center of.
QRect MainWindow::calc_image_center_pos(QPoint& cur_position, QRect& image_rect, double scale_of_zoom)
{
    QRect window_rect(this->rect());
    int win_height = window_rect.height();
    int win_width = window_rect.width();
    QPoint image_lefttop(cur_position.x()-win_width/2/scale_of_zoom, cur_position.y()-win_height/2/scale_of_zoom);
    QPoint image_rightbottom(image_lefttop.x()+window_rect.width()/scale_of_zoom, image_lefttop.y()+window_rect.height()/scale_of_zoom);

    //If the window is out of the image borders, we have to correct the window to the right position.
    if(image_lefttop.x()<0)
    {
        image_lefttop.setX(0);
        image_rightbottom.setX(window_rect.width()/scale_of_zoom);
    }
    if(image_lefttop.y()<0)
    {
        image_lefttop.setY(0);
        image_rightbottom.setY(window_rect.height()/scale_of_zoom);
    }
    if(image_rightbottom.x()>image_rect.bottomRight().x())
    {
        image_rightbottom.setX(image_rect.bottomRight().x());
        image_lefttop.setX(image_rect.bottomRight().x()-window_rect.width()/scale_of_zoom);
    }
    if(image_rightbottom.y()>image_rect.bottomRight().y())
    {
        image_rightbottom.setY(image_rect.bottomRight().y());
        image_lefttop.setY(image_rect.bottomRight().y()-window_rect.height()/scale_of_zoom);
    }
    QRect image_show_rect(image_lefttop, image_rightbottom);
    return image_show_rect;
}

QPoint MainWindow::trans_pos_in_map_to_window(const QPoint& cur_position, const QRect& window_rect_in_map)
{
    return QPoint(cur_position.x()-window_rect_in_map.topLeft().x(),
                  cur_position.y()-window_rect_in_map.topLeft().y());
}
QPoint MainWindow::trans_window_to_pos_on_map(const QPoint& cur_position, const QRect& window_rect_in_map)
{
    return QPoint(cur_position.x()+window_rect_in_map.topLeft().x(),
                  cur_position.y()+window_rect_in_map.topLeft().y());
}
QPoint MainWindow::locate_pos_in_cellcenter(QPoint pos)
{
    index_2d index=nav_info.trans_pos_to_index(pos);
    return nav_info.trans_index_to_pos(index.x,index.y);
}
void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton)
    {
        left_button_down=true;
        cur_mouse_point=event->pos();
    }
    return QMainWindow::mousePressEvent(event);
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton)
    {
        left_button_down=false;
    }
    return QMainWindow::mouseReleaseEvent(event);
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if(left_button_down)
    {
        //move the mainwindow on the map
        QPoint move_distance(event->pos().x()-cur_mouse_point.x(),event->pos().y()-cur_mouse_point.y());
        cur_window_on_map.setLeft(cur_window_on_map.left()-move_distance.x());
        cur_window_on_map.setRight(cur_window_on_map.right()-move_distance.x());
        cur_window_on_map.setTop(cur_window_on_map.top()-move_distance.y());
        cur_window_on_map.setBottom(cur_window_on_map.bottom()-move_distance.y());

        cur_mouse_point=event->pos();
        this->repaint();
    }
    return QMainWindow::mouseMoveEvent(event);
}

void MainWindow::changeEvent(QEvent *event)
{
    QMainWindow::changeEvent(event);
    std::cout<<this->rect().left()<<this->rect().right();
    std::cout<<this->rect().bottom()<<this->rect().top();
    cur_window_on_map=this->rect();
    return;
}

void MainWindow::mouseDoubleClickEvent(QMouseEvent* event)
{
    if(event->button()==Qt::LeftButton)
    {
        in_navigation=true;
        nav_des_pos=this->trans_window_to_pos_on_map(event->pos(), cur_window_on_map);
        nav_info.navigation_onmap(nav_des_pos);
        this->repaint();
    }
    return QMainWindow::mouseDoubleClickEvent(event);
}

void MainWindow::timerEvent(QTimerEvent *event)
{
    if(event->timerId()==this->timer_id)
    {
        //this->set_cur_pos(this->get_cur_pos().x()+10, this->get_cur_pos().y()+10);//////

        nav_info.set_cur_pos_real(this->get_cur_pos());
        nav_info.update_nav_path();
        this->update_window();
    }
    return QMainWindow::timerEvent(event);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    int step=50;
    switch(event->key())
    {
    case Qt::Key_Up:
    {
        this->set_cur_pos(this->get_cur_pos().x(), this->get_cur_pos().y()-step);
        break;
    }
    case Qt::Key_Down:
    {
        this->set_cur_pos(this->get_cur_pos().x(), this->get_cur_pos().y()+step);
        break;
    }
    case Qt::Key_Right:
    {
        this->set_cur_pos(this->get_cur_pos().x()+step, this->get_cur_pos().y());
        break;
    }
    case Qt::Key_Left:
    {
        this->set_cur_pos(this->get_cur_pos().x()-step,this->get_cur_pos().y());
        break;
    }
    default:
        break;
    }
    this->update_window();

    //std::cout<<this->get_cur_pos().x()<<" "<<this->get_cur_pos().y()<<" : "<<this->nav_info.get_cur_pos_on_map().x()<<" "<<this->nav_info.get_cur_pos_on_map().y()<<std::endl;
    return QMainWindow::keyPressEvent(event);
}
void MainWindow::paintEvent(QPaintEvent *event)
{
    //draw map in the window
    QRect image_rect=map_image.rect();
    QRect rect_window(this->rect());
    QPainter paintermap(this);
    paintermap.drawImage(rect_window,map_image,cur_window_on_map);

    //draw location on the map
    paintermap.setBrush(Qt::white);
    QPen border_pen(Qt::white);
    paintermap.setPen(border_pen);
    paintermap.drawEllipse(trans_pos_in_map_to_window(locate_pos_in_cellcenter(nav_info.get_cur_pos_on_map()),cur_window_on_map),12,12);

    QColor inner_circle(75,70,247);
    paintermap.setBrush(QBrush(inner_circle));
    paintermap.setPen(Qt::white);
    paintermap.drawEllipse(trans_pos_in_map_to_window(locate_pos_in_cellcenter(nav_info.get_cur_pos_on_map()),cur_window_on_map),10,10);

    //draw the path in the map
    auto nav=this->get_nav_info_ref();
    std::vector<index_2d>* path=this->nav_info.get_nav_path_ref();
    bool start_draw_path=false;
    index_2d cur_index(nav_info.trans_pos_to_index(nav_info.get_cur_pos_on_map()));
    for(auto it=path->rbegin();it!=path->rend();it++)
    {
        if(start_draw_path)
        {
            QPoint path_pos=nav->trans_index_to_pos(it->x,it->y);
            paintermap.drawEllipse(trans_pos_in_map_to_window(path_pos,cur_window_on_map),5,5);
        }
        if(it->x==cur_index.x && it->y==cur_index.y)
            start_draw_path=true;
    }

    //draw destination on the map if navigation is open
    if(in_navigation)
    {
        paintermap.setBrush(Qt::white);
        paintermap.setPen(Qt::black);
        paintermap.drawEllipse(locate_pos_in_cellcenter(trans_pos_in_map_to_window(nav_des_pos,cur_window_on_map)),15,15);

        paintermap.drawEllipse(locate_pos_in_cellcenter(trans_pos_in_map_to_window(nav_des_pos,cur_window_on_map)),7,7);
    }
}

void MainWindow::update_window()
{
    nav_info.update_pos_info(this->get_cur_pos().x(),this->get_cur_pos().y(),scale);
    this->repaint();
}

Nav_navigation_info* MainWindow::get_nav_info_ref()
{
    return &nav_info;
}

void MainWindow::start_navigation()
{
    nav_info.navigation_onmap(QPoint(600,600));
    this->repaint();
    return;
}
