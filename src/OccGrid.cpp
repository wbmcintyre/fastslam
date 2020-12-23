
#include <ros/ros.h>
#include <math.h>
#include "LaserScanner.h"
#include "OccGrid.h"

#include <vector>
#include <array>
#include <algorithm>


OccGrid::OccGrid(){
  for(int i =0; i < map.size(); i++){
    map[i].fill(-1); //set values of map to -1 (unknown value)
  }
}


float OccGrid::getWeightByMeasurements(std::array<float,3> pose, LaserScanner scanner){
  float weight;
  float totalVal = 0;
  int maxX = 0;
  int maxY = 0;
  int minX = map.size();
  int minY = map.size();
  int x,y;
  std::vector<float> globalVals;
  std::vector<float> localVals;
  float angleMin = scanner.getAngleMin();
  float angleMax = scanner.getAngleMax();
  float angleInc = scanner.getAngleInc();
  float rangeMax = scanner.getRangeMax();
  float rangeMin = scanner.getRangeMin();
  //get min and max X and Y values so the entire grid doesn't need to be searched

  for(int k = 0; k < scanner.getScanSize(); k++){
    float r = scanner.getScan(k);
    if(r < rangeMax && r > rangeMin){   //filter out bad laser scanner values
      x = int(round((pose[0] + r*cos(pose[2]+angleMin+(k*angleInc)))/resolution));   //global map coordinates of scan
      y = int(round((pose[1] + r*sin(pose[2]+angleMin+(k*angleInc)))/resolution));

      //get the min and max X and Y values
      if(x > maxX) {
        maxX = x;
      }
      if(x < minX){
        minX = x;
      }
      if(y > maxY) {
        maxY = y;
      }
      if(y < minY){
        minY = y;
      }
    }

  }
  //ROS_INFO("minX = %d, maxX = %d, minY = %d, maxY = %d", minX, maxX, minY, maxY);
  //to prevent segmentation faults, we limit the search to the map boundaries
  if(maxX > map.size()){ maxX = map.size(); }
  if(maxY > map.size()){ maxY = map[0].size(); }
  if(minX < 0){ minX = 0; }
  if(minY < 0){ minY = 0; }
  //look within the min and max X and Y coordinates for map matching
  for(int i = minX; i <= maxX; i++){
    for(int j = minY; j <= maxY; j++){
      //calculate center of mass of grid marker
      if(map[i][j] != -1) {
        float xi = resolution*((i+1)-0.5);  //grid cell center of mass
        float yi = resolution*((j+1)-0.5);
        float r = sqrt(pow(xi - pose[0], 2) + pow(yi - pose[1], 2)); //distance from robot to grid cell
        if(r < rangeMax && r > rangeMin){      //if scanner is within range of cell (don't check cells that aren't in scanner range)
          float phi = atan2(yi - pose[1], xi - pose[0]) - pose[2]; //sensorTheta = measurementTheta - robotTheta (robotTheta and measurementTheta are 0 when aligned with X axis)
          if(phi < 0){  //sensor range doesn't check negative values but atan2 returns negative values
            phi += 2*M_PI;
          }
          if(phi > angleMin && phi < angleMax) {  //check if phi is within range of sensor scan
            int senseIndex = int(round((phi - angleMin)/angleInc)); //offsetting sensor angles so they start at 0 for estimating sensor index of angle phi
            float senseAngle = angleMin + senseIndex*angleInc; //get true sensor angle based on index
            if( r > scanner.getScan(senseIndex)+resolution/2 || fabs(phi-senseAngle) > M_PI/180){ //filter out unknown scans
                //sensing range doesn't reach grid cell, scan angle mismatch with beam [1 degree width beam])
                //do nothing for scans outside of grid cell resolution
            }
            else if( scanner.getScan(senseIndex) < rangeMax && fabs(scanner.getScan(senseIndex)-r) < resolution/2) {
              totalVal = map[i][j] + 100;
              if(map[i][j] == -1){
                globalVals.push_back(0);
              } else {
                globalVals.push_back(map[i][j]);
              }
              localVals.push_back(100);
            }
            else if( r < scanner.getScan(senseIndex) ){  //if sensing distance past the cell, this cell must be free
              totalVal = map[i][j] + 0;
              if(map[i][j] == -1){
                globalVals.push_back(0);
              } else {
                globalVals.push_back(map[i][j]);
              }
              localVals.push_back(0);
            }
          }
        }
      }
    }
  }

  if(globalVals.empty()){ //if no matches or bad scan, return very low weight
    return .0001;
  }
  //calculating the map matching probability
  float avgVal = totalVal / (2*globalVals.size());
  float numerator = 0;
  float sumGlobalDiff = 0;
  float sumLocalDiff = 0;
  for(int i = 0; i < globalVals.size(); i++){
    numerator += ((globalVals.at(i)-avgVal)*(localVals.at(i)-avgVal));
    sumGlobalDiff += pow(globalVals.at(i)-avgVal,2);
    sumLocalDiff += pow(localVals.at(i)-avgVal,2);
  }

  return std::max(numerator/sqrt(sumGlobalDiff*sumLocalDiff),(float)0.0);
}


void OccGrid::updateGrid(std::array<float,3> predictPose, LaserScanner scanner){

  float rangeMax = scanner.getRangeMax();
  float rangeMin = scanner.getRangeMin();
  float angleMin = scanner.getAngleMin();
  float angleMax = scanner.getAngleMax();
  float angleInc = scanner.getAngleInc();

  for(int i = 0; i < width; i++){
    for(int j = 0; j < height; j++){
      //calculate center of mass of grid marker
      float xi = resolution*((i+1)-0.5);
      float yi = resolution*((j+1)-0.5);

      float r = sqrt(pow(xi - predictPose[0], 2) + pow(yi - predictPose[1], 2)); //distance from robot to grid cell

      if(r < rangeMax && r > rangeMin){      //if scanner is within range of cell (don't check cells that aren't in scanner range)
        float phi = atan2( yi - predictPose[1] , xi - predictPose[0]) - predictPose[2]; //sensorTheta = measurementTheta - robotTheta (robotTheta is 0 when aligned with Y axis)
        if(phi < 0){  //sensor range doesn't check negative values but atan2 returns negative values
          phi += 2*M_PI;
        }
        //ROS_INFO("X, Y, R, phi: %f %f %f %f", xi, yi, r, phi);
        if(phi > angleMin && phi < angleMax) {  //check if phi is within range of sensor scan
          int senseIndex = int(round((phi - angleMin)/angleInc)); //offsetting sensor angles so they start at 0 for estimating sensor index of angle phi
          float senseAngle = angleMin + senseIndex*angleInc; //get true sensor angle based on index
          if( r > scanner.getScan(senseIndex)+resolution/2 || fabs(phi-senseAngle) > M_PI/180){ //filter out unknown scans
              //sensing range doesn't reach grid cell, scan angle mismatch with beam [1 degree width beam])
              //do nothing for scans outside of grid cell
          }
          else if( scanner.getScan(senseIndex) < rangeMax && fabs(scanner.getScan(senseIndex)-r) < resolution/2) {
            if(map[i][j] == -1) {
              map[i][j] = 0;    //grid cell is initialized
            }
            if(map[i][j] + loccupied > 100){ //constrain map values from 0 to 100 once initialized
              map[i][j] = 100;
            } else {
            map[i][j] = map[i][j] + loccupied;  //if sensing within resolution of the grid cell, adjust to occupied
            }
            /*
            ROS_INFO("Map Update %d, %d to %d", i,j, map[i][j]);
            ROS_INFO("RangeMax = %f, RangeMin = %f, AngleMin = %f, AngleMax = %f, AngleInc = %f", rangeMax, rangeMin, angleMin, angleMax, angleInc);
            ROS_INFO("PredictPose = %f, %f, %f ", predictPose[0], predictPose[1], predictPose[2]);
            ROS_INFO("Cell Position = %f, %f", xi, yi);
            ROS_INFO("Distance = %f", r);
            ROS_INFO("Phi = %f", phi);
            ROS_INFO("SenseIndex = %d", senseIndex);
            ROS_INFO("SenseAngle = %f", senseAngle);
            ROS_INFO("ScannerValue at senseIndex = %f, fabs(phi-senseAngle) = %f", scanner.getScan(senseIndex), fabs(phi-senseAngle));
            */
          }
          else if( r < scanner.getScan(senseIndex)){  //if sensing distance past the cell, this cell must be free
            if(map[i][j] == -1) {
              map[i][j] = 0;    //grid cell is initialized
            }
            if(map[i][j] + lfree < 0){ //constrain map values from 0 to 100 once initialized
              map[i][j] = 0;
            } else {
            map[i][j] = map[i][j] + lfree;
            }
            //ROS_INFO("Map Update %d, %d to %d", i,j, map[i][j]);
          }

        }
      }

    }
  }
}

std::vector<signed char> OccGrid::getFlattenedMap(){
  std::vector<signed char> flatMap;
  for(int i = 0; i < height; i++){      //push back in row major order
    for(int j = 0; j < width; j++){
      flatMap.push_back((signed char)map[i][j]);
    }
  }
  return flatMap;
}

float OccGrid::getResolution(){
  return resolution;
}

int OccGrid::getWidth(){
  return width;
}

int OccGrid::getHeight(){
  return height;
}

float OccGrid::getMapCenterX(){
  return mapXOffset*resolution;
}

float OccGrid::getMapCenterY(){
  return mapYOffset*resolution;
}
