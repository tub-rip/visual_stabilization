#ifndef EVENT_ACCUMULATOR_H
#define EVENT_ACCUMULATOR_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>

#include <std_msgs/Time.h>
#include <dvs_msgs/Event.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cstring>
#include <string>
#include <map>


class eventWindowing{
  private:
    int cameraWidth_, cameraHeight_, max_n_events_, roiThreshold_, n_cols_, n_rows_, event_accumulator_;
    std::string updateType_;
    double tWarp_, tZero_;

    struct roi{
      roi(int xmax, int xmin, int ymax, int ymin, int counter): x_max(xmax),x_min(xmin),y_max(ymax),y_min(ymin),e_counter(counter) {};
      int x_max;
      int x_min;
      int y_max;
      int y_min;
      int e_counter;
    };

    std::vector <roi> roiList_;
    std::vector <int> roiPixelTable_;

    enum accumlationType {BATCH = 0, NUMBEROFEVENTS = 1, TIME = 2, AREA = 3};
    std::map<std::string,accumlationType> updateTypeVec_;

    void loadDefinitions();

    void initRoiList();
    void computeRoiTable();
    void restartRoiCounters();

  public:
    eventWindowing();
    eventWindowing(std::string accumlationType,  int accNevents, int roiTh, double accTime, int n_rows, int n_cols);
    ~eventWindowing(void);

    void initParameters(int width, int height, double time);
    bool update(int x, int y, double currentT);
};
#endif //  EVENT_ACCUMULATOR_H
