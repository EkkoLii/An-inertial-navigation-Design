#ifndef NAVIGATION_INFO_H
#define NAVIGATION_INFO_H
#include <QPoint>
#include <QSize>
#include <vector>

struct cell
{
    int cell_type;
    bool in_navigation_path;
    cell()
    {
        cell_type=0;
        in_navigation_path=false;
    }
};

struct index_2d
{
    int x;
    int y;
    index_2d()
    {
        x=-1;
        y=-1;
    }
    index_2d(int x, int y)
    {
        this->x=x;
        this->y=y;
    }
};

struct cell_status
{
    bool explored;
    index_2d prev_nearest;
    cell_status()
    {
        explored=false;
        prev_nearest.x=-1;
        prev_nearest.y=-1;
    }
};
class Nav_navigation_info
{
private:
    QPoint cur_pos_real;//Position information with real distance.
    QPoint cur_pos_on_map;//The position in grids of map.

    double cell_size;//pixels

    QSize map_size;//pixels

    std::vector<cell> grid_info;
    int cell_number_x;
    int cell_number_y;


public:
    Nav_navigation_info();

    QPoint get_cur_pos_real();
    void set_cur_pos_real(const QPoint& pos);
    QPoint get_cur_pos_on_map();
    void set_cur_pos_on_map(const QPoint& pos);

    void set_cell_number_x(int x);
    int get_cell_number_x();
    void set_cell_number_y(int y);
    int get_cell_number_y();
    double get_cell_size();
    void set_cell_size(double s);

    void init_grid_info();
    void cur_pos_real_to_map(double scale);
    void update_pos_info(double x, double y, double scale);

    //about path planning
private:
    std::vector<index_2d> nav_path;//reserve the index of the cells on the path
    index_2d cur_cell;
    index_2d des_cell;
public:
    std::vector<index_2d>* get_nav_path_ref();
    int tran2d_to_1d(int x, int y);
    bool index_exist(int x, int y);
    QPoint trans_index_to_pos(int x, int y);
    index_2d trans_pos_to_index(QPoint pos);

    void path_planning(int src_x, int src_y, int des_x, int des_y);
    void navigation_onmap(QPoint dest);
    int update_nav_path();


};

#endif // NAVIGATION_INFO_H
