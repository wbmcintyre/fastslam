#ifndef OCCGRID_H
#define OCCGRID_H

#include "LaserScanner.h"
#include <array>
#include <set>
#include <vector>
//#include "Point.h"

/*
struct comp
{
    bool operator()(const Point& lhs, const Point& rhs) const
    {
        if (lhs.x == rhs.x)
            return lhs.y < rhs.y;

        return lhs.x < rhs.x;
    }
};*/

class OccGrid {

  private:
    float resolution = 0.05; //meters per cell, set default to 0.01 (cannot be greater than 1)
	  int width = 200;  //default 40 by 40 meter grid based on default resolution
		int height = 200;
    std::array<std::array<int,200>,200> map; //2d grid of values ranging from 0 to 100 with -1 as unknown
    std::array<std::array<int,200>,200> smoothedMap;
    float loccupied = 2.0;
    float lfree = -2.0;
    float mapXOffset = width/2.0; //initial robot position in map
    float mapYOffset = height/2.0;
    //float setAddThresh = 60;
    //float setRemThresh = 40;
    //std::set<Point, comp> mapSet;
    //std::vector<std::array<int,2>> pointcloud;
    bool firstUpdate = true;
    float loccfirst = 5.0;

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
