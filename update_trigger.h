#ifndef UPDATE_TRIGGER_H
#define UPDATE_TRIGGER_H
#include "QPoint"

class update_trigger
{
private:
    QPoint cur_pos;//in meters
    QPoint init_pos;//in meters
public:
    void set_cur_pos(double x, double y);
    void set_init_pos(double x, double y);
    QPoint get_cur_pos();
    QPoint get_init_pos();
    virtual void update_window()=0;
};

#endif // UPDATE_TRIGGER_H
