#include "visual_stabilization/d_event.hpp"

dEvent::dEvent(){}

dEvent::dEvent(double et, double ex, double ey, bool pol){
  ts = et;
  x = ex;
  y = ey;
  polarity = pol;
}

dEvent::~dEvent(){}
