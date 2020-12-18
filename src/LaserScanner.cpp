
#include "LaserScanner.h"
#include <vector>

LaserScanner::LaserScanner(){}

LaserScanner::LaserScanner(std::vector<float> scan, float angleMin, float angleMax, float angleInc, float rangeMin, float rangeMax){
    scanValues = scan;
    angle_min = angleMin;
    angle_max = angleMax;
    angle_increment = angleInc;
    range_min = rangeMin;
    range_max = rangeMax;
    scanSize = scan.size();
}

void LaserScanner::setScan(std::vector<float> scan){
  scanValues = scan;
}

int LaserScanner::getScanSize(){
  return scanSize;
}

float LaserScanner::getScan(int index){
  return scanValues.at(index);
}

float LaserScanner::getAngleMin(){
  return angle_min;
}

float LaserScanner::getAngleMax(){
  return angle_max;
}

float LaserScanner::getAngleInc(){
  return angle_increment;
}

float LaserScanner::getRangeMin(){
  return range_min;
}

float LaserScanner::getRangeMax(){
  return range_max;
}
