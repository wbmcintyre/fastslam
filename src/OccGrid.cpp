
#include <ros/ros.h>
#include <math.h>
#include "LaserScanner.h"
#include "OccGrid.h"

#include <vector>
#include <array>
#include <set>
#include <algorithm>



OccGrid::OccGrid(){
  for(int i =0; i < static_cast<int>(map.size()); i++){
    map[i].fill(0); //log odds value of map
    smoothedMap[i].fill(-1); //set values of map to -1 (unknown value) (range 0 to 1)
    probMap[i].fill(0.5); //convert log odds to probability of map
  }
}


float OccGrid::getWeightByMeasurements(std::array<float,3> pose, LaserScanner scanner){ // map matching with white space included
  float weight;
  float totalVal = 0;
  int maxX = 0;
  int maxY = 0;
  int minX = static_cast<int>(map.size());
  int minY = static_cast<int>(map.size());
  int x,y;
  std::vector<float> globalVals;
  std::vector<float> localVals;
  float angleMin = scanner.getAngleMin();
  float angleMax = scanner.getAngleMax();
  float angleInc = scanner.getAngleInc();
  float rangeMax = scanner.getRangeMax();
  float rangeMin = scanner.getRangeMin();

  float totalWhiteVal = 0;
  float totalBlackVal = 0;
  int blackCounts = 0;
  int whiteCounts = 0;


  //get min and max X and Y values so the entire grid doesn't need to be searched
  for(int k = 0; k < scanner.getScanSize(); k++){
    float r = scanner.getScan(k);
    if(r < rangeMax && r > rangeMin){   //filter out bad laser scanner values
      x = int(round((pose[0] + r*cos(pose[2]+angleMin+(k*angleInc)))/resolution));   //global map coordinates of scan
      y = int(round((pose[1] + r*sin(pose[2]+angleMin+(k*angleInc)))/resolution));

      //get the min and max X and Y values that the scanner reaches
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

  //to prevent segmentation faults, we limit the search to the map boundaries
  if(maxX > map.size()){ maxX = map.size(); }
  if(maxY > map.size()){ maxY = map[0].size(); }
  if(minX < 0){ minX = 0; }
  if(minY < 0){ minY = 0; }

  //look within the min and max X and Y coordinates for map matching
  for(int i = minX + 1; i <= maxX - 1; i++){
    for(int j = minY + 1; j <= maxY - 1; j++){
      //calculate center of mass of grid marker
      if(smoothedMap[i][j] != -1) {
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

              float mapProb = 0;
              int cellCount = 0;
              for(int k = i-1; k <= i+1; k++){  //compare guassian smoothed map to laser scanner
                for(int l = j-1; l <= j+1; l++){
                  if(smoothedMap[k][l] != -1){  //if cell has been initialized
                    mapProb += probMap[k][l];
                    cellCount++;
                  }
                }
              }
              mapProb = mapProb/cellCount;
              totalVal += mapProb + 1;
              globalVals.push_back(mapProb);
              localVals.push_back(1);

            }
            else if( r < scanner.getScan(senseIndex) ){  //if sensing distance past the cell, this cell must be free
              float mapProb = 0;
              int cellCount = 0;
              for(int k = i-1; k <= i+1; k++){  //compare guassian smoothed map to laser scanner
                for(int l = j-1; l <= j+1; l++){
                  if(smoothedMap[k][l] != -1){
                    mapProb += probMap[k][l];
                    cellCount++;
                  }
                }
              }
              mapProb = mapProb/cellCount;
              totalVal += mapProb + 0;
              globalVals.push_back(mapProb);
              localVals.push_back(0);

            }
          }
        }
      }
    }
  }


  if(globalVals.empty()){ //if no matches or bad scan, return very low weight
    return 0;
  }

  //calculating the map matching probability
  float avgVal = totalVal / (2*static_cast<float>(globalVals.size()));
  float numerator = 0;
  float sumGlobalDiff = 0;
  float sumLocalDiff = 0;
  for(int i = 0; i < static_cast<int>(globalVals.size()); i++){
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
  int locc = loccupied;
  if(firstUpdate){
    locc = loccfirst;
    firstUpdate = false;
  }

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
        if(phi > angleMin && phi < angleMax) {  //check if phi is within range of sensor scan
          int senseIndex = int(round((phi - angleMin)/angleInc)); //offsetting sensor angles so they start at 0 for estimating sensor index of angle phi
          float senseAngle = angleMin + senseIndex*angleInc; //get true sensor angle based on index
          if( r > scanner.getScan(senseIndex)+resolution/2 || fabs(phi-senseAngle) > M_PI/180){ //filter out unknown scans
              //sensing range doesn't reach grid cell, scan angle mismatch with beam [1 degree width beam])
              //do nothing for scans outside of grid cell
          }
          else if( scanner.getScan(senseIndex) < rangeMax && fabs(scanner.getScan(senseIndex)-r) < resolution/2) {
            if(smoothedMap[i][j] == -1) {
              smoothedMap[i][j] = 0;     //grid cell is initialized
            }
            map[i][j] = map[i][j] + locc;  //if sensing within resolution of the grid cell, adjust to occupied
            probMap[i][j] = (1-(1/(1+exp(map[i][j]))));
          }
          else if( r < scanner.getScan(senseIndex)){  //if sensing distance past the cell, this cell must be free
            if(smoothedMap[i][j] == -1) {
              smoothedMap[i][j] = 0;     //grid cell is initialized
            }
            map[i][j] = map[i][j] + lfree;
            probMap[i][j] = (1-(1/(1+exp(map[i][j]))));
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
      if(smoothedMap[j][i] == -1) {
        flatMap.push_back(-1);
      } else {
        flatMap.push_back((signed char)(probMap[j][i]*100));
      }
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
