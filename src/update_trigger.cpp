#include "update_trigger.h"
void update_trigger::set_cur_pos(double x, double y)
{
    cur_pos.setX(x);
    cur_pos.setY(y);
}

void update_trigger::set_init_pos(double x, double y)
{
    init_pos.setX(x);
    init_pos.setY(y);
}

QPoint update_trigger::get_cur_pos()
{
    return cur_pos;
}

QPoint update_trigger::get_init_pos()
{
    return init_pos;
}
