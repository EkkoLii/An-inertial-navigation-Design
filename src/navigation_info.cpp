#include "navigation_info.h"
#include <list>

Nav_navigation_info::Nav_navigation_info()
{
    this->cell_size=20.0;
    this->cur_pos_real.setX(0);
    this->cur_pos_real.setY(0);
    this->cur_pos_on_map.setX(0);
    this->cur_pos_on_map.setY(0);
}
QPoint Nav_navigation_info::get_cur_pos_on_map()
{
    return cur_pos_on_map;
}
void Nav_navigation_info::set_cur_pos_on_map(const QPoint& pos)
{
    cur_pos_on_map.setX(pos.x());
    cur_pos_on_map.setY(pos.y());
    return;
}

QPoint Nav_navigation_info::get_cur_pos_real()
{
    return cur_pos_real;
}
void Nav_navigation_info::set_cur_pos_real(const QPoint &pos)
{
    cur_pos_real.setX(pos.x());
    cur_pos_real.setY(pos.y());
    return;
}

void Nav_navigation_info::cur_pos_real_to_map(double scale)
{
    cur_pos_on_map.setX(cur_pos_real.x()/scale);
    cur_pos_on_map.setY(cur_pos_real.y()/scale);
    return;
}


void Nav_navigation_info::set_cell_number_x(int x)
{
    this->cell_number_x=x;
}
int Nav_navigation_info::get_cell_number_x()
{
    return this->cell_number_x;
}
void Nav_navigation_info::set_cell_number_y(int y)
{
    this->cell_number_y=y;
}
int Nav_navigation_info::get_cell_number_y()
{
    return this->cell_number_y;
}
double Nav_navigation_info::get_cell_size()
{
    return this->cell_size;
}
void Nav_navigation_info::init_grid_info()
{
    this->grid_info=std::vector<cell>(this->cell_number_x*this->cell_number_y);
    //for(auto it=grid_info.begin();it!=grid_info.end();it++)
    return;
}
void Nav_navigation_info::set_cell_size(double s)
{
    this->cell_size=s;
    return;
}

void Nav_navigation_info::update_pos_info(double x, double y, double scale)
{
    set_cur_pos_real(QPoint(x,y));
    cur_pos_real_to_map(scale);
    return;
}

//////tool functions//////////
int Nav_navigation_info::tran2d_to_1d(int x, int y)
{
    return y*this->cell_number_x+x;
}

bool Nav_navigation_info::index_exist(int x, int y)
{
    if(x<0 || x>=this->cell_number_x)
        return false;
    if(y<0 || y>=this->cell_number_y)
        return false;
    return true;
}

//////////////////////////////
void Nav_navigation_info::path_planning(int src_x, int src_y, int des_x, int des_y)
{
    nav_path.clear();
    if(src_x==des_x && src_y==des_y)
        return;
    if(src_x<0 || src_y<0 || des_x<0 || des_y<0)
        return;

    std::list<index_2d> start_list;
    int grid_size=grid_info.size();
    std::vector<cell_status> dis_info=std::vector<cell_status>(grid_size);
    index_2d start;
    start.x=src_x;
    start.y=src_y;

    start_list.emplace_back(start);
    int cur_dis=0;
    cell_status start_cell=dis_info[tran2d_to_1d(start.x, start.y)];
    start_cell.explored=true;
    start_cell.prev_nearest.x=-1;
    start_cell.prev_nearest.y=-1;

    int type0_number=0;
    for(int i=0;i<grid_info.size();i++)
    {
        if(grid_info[i].cell_type==0)
        {
            type0_number+=1;
        }
    }

    int dis=0;//distance from start point
    int list_size=1;
    bool endloop=false;
    while(start_list.size()<type0_number)
    {
        dis+=1;
        auto it=start_list.begin();
        for(int i=0;i<list_size;i++)
        {
            //right
            if(index_exist(it->x+1,it->y))
            {
                int index=tran2d_to_1d(it->x+1,it->y);
                if(!dis_info.at(index).explored)
                {
                    start_list.emplace_back(index_2d(it->x+1,it->y));
                    dis_info.at(index).prev_nearest=index_2d(it->x,it->y);
                    dis_info.at(index).explored=true;
                    if(des_x==it->x+1 && des_y==it->y)
                    {
                        endloop=true;
                        break;
                    }
                }

            }
            //bottom
            if(index_exist(it->x,it->y+1))
            {
                int index=tran2d_to_1d(it->x,it->y+1);
                if(!dis_info.at(index).explored)
                {
                    start_list.emplace_back(index_2d(it->x,it->y+1));
                    dis_info.at(index).prev_nearest=index_2d(it->x,it->y);
                    dis_info.at(index).explored=true;
                    if(des_x==it->x && des_y==it->y+1)
                    {
                        endloop=true;
                        break;
                    }
                }
            }
            //left
            if(index_exist(it->x-1,it->y))
            {
                int index=tran2d_to_1d(it->x-1,it->y);
                if(!dis_info.at(index).explored)
                {
                    start_list.emplace_back(index_2d(it->x-1,it->y));
                    dis_info.at(index).prev_nearest=index_2d(it->x,it->y);
                    dis_info.at(index).explored=true;
                    if(des_x==it->x-1 && des_y==it->y)
                    {
                        endloop=true;
                        break;
                    }
                }
            }
            //top
            if(index_exist(it->x,it->y-1))
            {
                int index=tran2d_to_1d(it->x,it->y-1);
                if(!dis_info.at(index).explored)
                {
                    start_list.emplace_back(index_2d(it->x,it->y-1));
                    dis_info.at(index).prev_nearest=index_2d(it->x,it->y);
                    dis_info.at(index).explored=true;
                    if(des_x==it->x && des_y==it->y-1)
                    {
                        endloop=true;
                        break;
                    }
                }
            }
            it++;
        }

        list_size=start_list.size();
        if(endloop)
            break;
    }

    index_2d path_cell_index(des_x,des_y);

    for(int i=0;i<dis && (path_cell_index.x!=src_x || path_cell_index.y!=src_y);i++)
    {
        nav_path.emplace_back(path_cell_index);
        auto show=dis_info.at(tran2d_to_1d(path_cell_index.x,path_cell_index.y));
        path_cell_index=dis_info.at(tran2d_to_1d(path_cell_index.x,path_cell_index.y)).prev_nearest;
    }
    nav_path.emplace_back(index_2d(src_x,src_y));

    return;
}

std::vector<index_2d>* Nav_navigation_info::get_nav_path_ref()
{
    return &nav_path;
}
QPoint Nav_navigation_info::trans_index_to_pos(int x, int y)
{
    QPoint res;
    res.setX(x*this->cell_size+this->cell_size/2);
    res.setY(y*this->cell_size+this->cell_size/2);
    return res;
}


index_2d Nav_navigation_info::trans_pos_to_index(QPoint pos)
{
    int x=pos.x()/this->cell_size;
    int y=pos.y()/this->cell_size;
    return index_2d(x,y);
}
void Nav_navigation_info::navigation_onmap(QPoint des_pos)
{
    index_2d cur_index=this->cur_cell=trans_pos_to_index(this->cur_pos_on_map);
    this->des_cell=trans_pos_to_index(des_pos);
    path_planning(cur_index.x, cur_index.y, des_cell.x, des_cell.y);
    return;
}

int Nav_navigation_info::update_nav_path()
{
    cur_cell=trans_pos_to_index(this->cur_pos_on_map);
    //if cur_index is not in nav_path, new path_planning is required.
    for(auto it=this->nav_path.rbegin();it!=this->nav_path.rend();it++)
    {
        if(it->x==cur_cell.x && it->y==cur_cell.y)
            return 0;
    }
    path_planning(cur_cell.x,cur_cell.y,des_cell.x,des_cell.y);
    return 1;
}
