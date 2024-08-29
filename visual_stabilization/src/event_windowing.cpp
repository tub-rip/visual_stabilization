#include "visual_stabilization/event_windowing.hpp"

eventWindowing::eventWindowing(void){
  loadDefinitions();
}
eventWindowing::eventWindowing(std::string accumlationType,  int accNevents, int roiTh, double accTime, int n_rows, int n_cols){
  updateType_ = accumlationType;
  max_n_events_ = accNevents;
  roiThreshold_ = roiTh;
  tWarp_ = accTime;
  n_cols_ = n_cols;
  n_rows_ = n_rows;

  loadDefinitions();

  switch(updateTypeVec_[updateType_]){
    case NUMBEROFEVENTS :
      ROS_INFO("Event accumulator activated. Type : %s  n_events : %d" , updateType_.c_str(), max_n_events_);
      break;
    case TIME :
      ROS_INFO("Event accumulator activated. Type : %s  accumulation time : %f" , updateType_.c_str(), tWarp_);
      break;
    case AREA :
      ROS_INFO("Event accumulator activated. Type : %s  events per window : %d , n columns : %d , n rows :  %d " , updateType_.c_str(), roiThreshold_, n_cols_, n_rows_);
      break;
    default:
      ROS_INFO("Error, invalid accumulation type ");
  }
}

eventWindowing::~eventWindowing(void){}

void eventWindowing::loadDefinitions(){
  // Define event windowing approach
  updateTypeVec_["number_of_events"]=NUMBEROFEVENTS;
  updateTypeVec_["time"]=TIME;
  updateTypeVec_["area"]=AREA;
}

void eventWindowing::initParameters(int width, int height, double time){
  cameraWidth_ = width;
  cameraHeight_ = height;
  tZero_ = time;
  event_accumulator_ = 0;

  if(updateTypeVec_[updateType_] == AREA)
    initRoiList();
}

bool eventWindowing::update(int x, int y, double currentT){
  if(x < 0 || x >= cameraWidth_ || y < 0 || y >= cameraHeight_)
    return false;

  bool accumulationReady = false;
  switch(updateTypeVec_[updateType_]){
    case NUMBEROFEVENTS :
      if(event_accumulator_ > max_n_events_){
        accumulationReady = true;
        event_accumulator_ = 1;
      }
      else
        event_accumulator_++;
      break;
    case TIME :
      if(currentT - tZero_ > tWarp_){
        accumulationReady = true;
        tZero_ = currentT;
      }
      break;
    case AREA :
      roiList_[roiPixelTable_[y*cameraWidth_ + x]].e_counter++;
      if(roiList_[roiPixelTable_[y*cameraWidth_ + x]].e_counter > roiThreshold_){
        accumulationReady = true;
        restartRoiCounters();
      }
      break;
  }
  return accumulationReady;
}

void eventWindowing::initRoiList(){

  if(n_cols_ > 0 && n_rows_> 0){
    int y_min, y_max = -1, x_step = (cameraWidth_/n_cols_), y_step = (cameraHeight_/n_rows_);
    for (int ii = 1; ii <= n_rows_; ii++){
      y_min = y_max+1;
      if(ii == n_rows_)
        y_max = cameraHeight_-1;
      else
        y_max = y_step*ii;

      int x_min, x_max = -1;
      for (int jj = 1; jj <= n_cols_; jj++){
        x_min = x_max+1;
        if(jj == n_cols_)
          x_max = cameraWidth_-1;
        else
          x_max = x_step*jj;

        roiList_.emplace_back(roi(x_max, x_min, y_max, y_min, 0));
      }
    }
  }
  else
    ROS_INFO("Missig number of rows and cols");

  ROS_INFO("List of rois initialized .... number of rois : %d", (int)roiList_.size());
  // Init roi table
  computeRoiTable();
}

void eventWindowing::computeRoiTable(){
  for (int y = 0; y<cameraHeight_; y++){
    for (int x = 0; x<cameraWidth_; x++){
      for(int k = 0; k<roiList_.size(); k++){
        if(roiList_[k].x_min<= x && x<= roiList_[k].x_max && roiList_[k].y_min<= y && y <= roiList_[k].y_max)
          roiPixelTable_.push_back(k);
      }
    }
  }
  ROS_INFO("Roi table initalized  ...... pixels in image : %d", (int)roiPixelTable_.size());
}

void eventWindowing::restartRoiCounters(){
  for(int k = 0; k<roiList_.size(); k++)
    roiList_[k].e_counter = 0;
}
