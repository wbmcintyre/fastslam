#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <vector>


class LaserScanner {

  private:
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    std::vector<float> scanValues;
    int scanSize;

  public:
    LaserScanner();
    LaserScanner(std::vector<float> scan, float angleMin, float angleMax, float angleInc, float rangeMin, float rangeMax);
    void setScan(std::vector<float> scan);
    float getScan(int index);
    float getAngleMin();
    float getAngleMax();
    float getAngleInc();
    float getRangeMin();
    float getRangeMax();
    int getScanSize();

  };

  #endif
