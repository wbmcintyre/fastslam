#ifndef OCCGRID_H
#define OCCGRID_H

#include "LaserScanner.h"
#include <array>
#include <vector>

class OccGrid {

  private:
    float resolution = 0.1; //meters per cell, set default to 0.01 (cannot be greater than 1)
	  int width = 100;  //default 40 by 40 meter grid based on default resolution
		int height = 100;
    std::array<std::array<int,100>,100> map; //2d grid of values ranging from 0 to 100 with -1 as unknown
    float loccupied = 40.0;
    float lfree = -40.0;
    float mapXOffset = width/2.0; //initial robot position in map
    float mapYOffset = height/2.0;

  public:
    OccGrid();
    float getWeightByMeasurements(std::array<float,3> pose, LaserScanner scanner);
    void updateGrid(std::array<float,3> predictPose, LaserScanner scanner);
    std::vector<signed char> getFlattenedMap();
    float getResolution();
    int getWidth();
    int getHeight();
    float getMapCenterX();
    float getMapCenterY();

};

#endif
