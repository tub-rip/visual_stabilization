#ifndef D_EVENT_H
#define D_EVENT_H

#include <ros/ros.h>

class dEvent{
  private:

  public:
    dEvent();
    dEvent(double et, double ex, double ey, bool pol);
    ~dEvent(void);

    double ts;
    double x;
    double y;
    bool polarity;
};

#endif // D_EVENT_H
