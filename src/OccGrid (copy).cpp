
#include <ros/ros.h>
#include <math.h>
#include "LaserScanner.h"
#include "OccGrid.h"
//#include "Point.h"

#include <vector>
#include <array>
#include <set>
#include <algorithm>



OccGrid::OccGrid(){
  for(int i =0; i < static_cast<int>(map.size()); i++){
    map[i].fill(0); //log odds value of map
    smoothedMap[i].fill(-1); //set values of map to -1 (unknown value) (range 0 to 1)
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
              totalVal += map[i][j] + 100;
              globalVals.push_back(1-(1/(1+exp(map[i][j]))));
              localVals.push_back(1);
              //totalBlackVal += 100 - map[i][j];
              //blackCounts ++;

            }
            else if( r < scanner.getScan(senseIndex) ){  //if sensing distance past the cell, this cell must be free
              totalVal += map[i][j] + 0;
              globalVals.push_back(1-(1/(1+exp(map[i][j]))));
              localVals.push_back(0);
              //totalWhiteVal += map[i][j];
              //whiteCounts ++;

            }
          }
        }
      }
    }
  }

  /*
  int totalCounts = whiteCounts + blackCounts;
  float percBlkCorrect = 1 - (totalBlackVal/(blackCounts*100));
  float percWhtCorrect = 1 - (totalWhiteVal/(whiteCounts*100));
  ROS_INFO("Blk perc: %f , C: %d", percBlkCorrect, blackCounts);
  ROS_INFO("Wht perc: %f , C: %d", percWhtCorrect, whiteCounts);
  return percBlkCorrect*percWhtCorrect;
  */


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

/*
float OccGrid::getWeightByMeasurements(std::array<float,3> pose, LaserScanner scanner){
  float totalVal = 0;
  int x,y;
  int numMatches = 0;
  std::vector<float> globalVals;
  std::vector<float> localVals;
  float rangeMax = scanner.getRangeMax();
  float rangeMin = scanner.getRangeMin();
  float angleMin = scanner.getAngleMin();
  float angleInc = scanner.getAngleInc();

  for(int k = 0; k < scanner.getScanSize(); k++){
    float r = scanner.getScan(k);
    float angle = pose[2]+angleMin+(k*angleInc);
    if(r < rangeMax && r > rangeMin){   //filter out bad laser scanner values
      x = int(round((pose[0] + r*cos(angle))/resolution));   //global map coordinates of scan
      y = int(round((pose[1] + r*sin(angle))/resolution));
      if(map[x][y] != -1) {
        totalVal += 100 - map[x][y];
        numMatches++;
      }
    }
    //ROS_INFO("R, Angle, X, Y, Xi, Yi %f %f %f %f %d %d", r, angle, pose[0], pose[1], x, y );
  }


  return (1 - (totalVal)/(numMatches*100));

}*/
/*
float OccGrid::getWeightByMeasurements(std::array<float,3> pose, LaserScanner scanner){

  int x,y;
  float sumDistErr = 0;
  float rangeMax = scanner.getRangeMax();
  float rangeMin = scanner.getRangeMin();
  float angleMin = scanner.getAngleMin();
  float angleInc = scanner.getAngleInc();

  for(int k = 0; k < scanner.getScanSize(); k++){
    float r = scanner.getScan(k);
    float angle = pose[2]+angleMin+(k*angleInc);
    if(r < rangeMax && r > rangeMin){   //filter out bad laser scanner values
      x = int(round((pose[0] + r*cos(angle))/resolution));   //global map coordinates of scan
      y = int(round((pose[1] + r*sin(angle))/resolution));

      float sqDist = pow(width,2)*pow(height,2);
      float testDist;

      for(auto p : pointcloud){
        testDist = pow((p[0]-x),2)+pow((p[1]-y),2);
        if(testDist < sqDist){
          sqDist = testDist;
        }
      }
      sumDistErr += sqrt(sqDist);

      /*Point newPoint(x,y);
      float sqDist = pow(width,2)*pow(height,2);
      float testDist;

      //search list of map set to find minimum distance to occupied cell
      for(auto p : mapSet){
        testDist = pow((p.x-x),2)+pow((p.y-y),2);
        if(testDist < sqDist){
          sqDist = testDist;
        }
      }
      sumDistErr += sqrt(sqDist);
    }
  }

  ROS_INFO("DistErr: %f", sumDistErr);
  return (1/(1 + sumDistErr));

}*/

void OccGrid::updateGrid(std::array<float,3> predictPose, LaserScanner scanner){

  float rangeMax = scanner.getRangeMax();
  float rangeMin = scanner.getRangeMin();
  float angleMin = scanner.getAngleMin();
  float angleMax = scanner.getAngleMax();
  float angleInc = scanner.getAngleInc();
  /*int locc = loccupied;
  if(firstUpdate){
    locc = loccfirst;
    firstUpdate = false;
  }*/

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
        //ROS_INFO("X, Y, T, Xi, Yi, R, phi: %f %f %f %f %f %f %f", predictPose[0], predictPose[1], predictPose[2], xi, yi, r, phi);
        if(phi > angleMin && phi < angleMax) {  //check if phi is within range of sensor scan
          int senseIndex = int(round((phi - angleMin)/angleInc)); //offsetting sensor angles so they start at 0 for estimating sensor index of angle phi
          float senseAngle = angleMin + senseIndex*angleInc; //get true sensor angle based on index
          if( r > scanner.getScan(senseIndex)+resolution/2 || fabs(phi-senseAngle) > M_PI/180){ //filter out unknown scans
              //sensing range doesn't reach grid cell, scan angle mismatch with beam [1 degree width beam])
              //do nothing for scans outside of grid cell
          }
          else if( scanner.getScan(senseIndex) < rangeMax && fabs(scanner.getScan(senseIndex)-r) < resolution/2) {
            if(smoothedMap[i][j] == -1) {
              smoothedMap[i][j] = 0; //grid cell is initialized
            }/*
            if(map[i][j] < setAddThresh && map[i][j] + locc >= setAddThresh){
              std::array<int,2> pAdd = {i,j};
              pointcloud.push_back(pAdd);
              //Point p(i,j);
              //mapSet.insert(p);
            }*/
            map[i][j] = map[i][j] + locc;  //if sensing within resolution of the grid cell, adjust to occupied
            /*if(map[i][j] > 100){ //constrain map values from 0 to 100 once initialized
              map[i][j] = 100;
            }*/
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
            if(smoothedMap[i][j] == -1) {
              smoothedMap[i][j] = 0;    //grid cell is initialized
            }
            /*
            if(map[i][j] > setRemThresh && map[i][j] + lfree <= setRemThresh){
              for(int i = 0; i < pointcloud.size(); i++){
                if(pointcloud.at(i)[0] == i && pointcloud.at(i)[1] == j){
                  pointcloud.erase(pointcloud.begin()+i);
                }
              }
              //Point p(i,j);
              //mapSet.erase(p);
            }*/
            map[i][j] = map[i][j] + lfree;
            /*if(map[i][j] < 0){ //constrain map values from 0 to 100 once initialized
              map[i][j] = 0;
            }*/
            //ROS_INFO("Map Update %d, %d to %d", i,j, map[i][j]);
          }

        }
      }

    }
  }
  //ROS_INFO("Pointcloud Size: %d", static_cast<int>(pointcloud.size()));
}

std::vector<signed char> OccGrid::getFlattenedMap(){
  std::vector<signed char> flatMap;
  for(int i = 0; i < height; i++){      //push back in row major order
    for(int j = 0; j < width; j++){
      if(smoothedMap[i][j] == -1) {
        flatMap.push_back(-1);
      } else {
        flatMap.push_back((signed char)(1-(1/(1+exp(map[j][i]))))*100);
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
