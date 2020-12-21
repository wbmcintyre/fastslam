//include ros directories (ros.h, standard msgs, etc..)
//include other headers

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <array>
#include <vector>


#include "OccGrid.h"
#include "Particle.h"
#include "LaserScanner.h"



/*** Thoughts ****

  - check to see if rosSpinOnce only gets subscriber information once prior to running script
  - check single/multi thread

  -robot offset in occupancy grid (measurement and update grid)

  -test order of operations for subscribing/publishing (print statements) ROS_INFO

*/

std::array<float,3> pose = {0.0,0.0,0.0}; //initialize robot pose (x)
std::array<float,2> velocities = {0.0,0.0}; //initialize command velocities (u)
LaserScanner scanner;
bool firstScan = true;
bool waitScanner = true;
bool waitCmdVel = true;


//time variables for predicting poses
ros::Time curTime;
float deltaTime = 0.1;
bool firstDelta = true;

//obtain the most probable particle to display on rviz
Particle bestParticle;
int maxWeightIndex = 0;



//delete these temp variables
ros::Time tempTime;
float dtempTime;


//collect current command velocity values to predict robot pose
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg){

  velocities[0] = msg->linear.x;
  velocities[1] = msg->angular.z;
  ros::Time newTime = ros::Time::now();
  deltaTime = (newTime - curTime).toSec();
  curTime = newTime;
  ROS_INFO("Received Vel");
  waitCmdVel = false;
  if(firstDelta == true){ //first time interval doesn't calculate correctly
    deltaTime = 0.1;
    firstDelta = false;
  }
}



//collect laser scan values for measurement matching
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

  std::vector<float> scan = msg->ranges;
  if(firstScan) {
    float angleMin = msg->angle_min;
    float angleMax = msg->angle_max;
    float angleInc = msg->angle_increment;
    float rangeMin = msg->range_min;
    float rangeMax = msg->range_max;
    scanner = LaserScanner(scan, angleMin, angleMax, angleInc, rangeMin, rangeMax);

  } else {
    scanner.setScan(scan);
  }
  waitScanner = false;
  ROS_INFO("Received Scan");

}



std::vector<Particle> particleFilter(std::vector<Particle> partLst) {
  ROS_INFO("PartFilter Start| waits: %d, %d", waitCmdVel, waitScanner);
  float weight[partLst.size()];  //weights used for resampling particles
  float weightSum = 0;
  std::vector<Particle> partListSample;  //particle list created from predicted Poses and updated Maps
  std::vector<Particle> partListResample; //particle list used for resampling and is returned
  for(int i = 0; i < partLst.size(); i++){
    std::array<float,3> predictPose = partLst[i].predictPose(velocities, deltaTime);
    OccGrid updatedMap = partLst[i].getMap();

    /*
    tempTime = ros::Time::now();
    ros::Time xTime = ros::Time::now();
    dtempTime = (xTime - tempTime).toSec();
    */
    //ROS_INFO("GettingWeight");
    weight[i] = updatedMap.getWeightByMeasurements(predictPose, scanner);
    /*xTime = ros::Time::now();
    dtempTime = (xTime - tempTime).toSec();
    tempTime = xTime;
    ROS_INFO("Measure DT = %f", dtempTime);*/

    //ROS_INFO("UpdateGrid");
    updatedMap.updateGrid(predictPose, scanner);
    /* xTime = ros::Time::now();
    dtempTime = (xTime - tempTime).toSec();
    ROS_INFO("updateGride DT = %f", dtempTime); */
    //ROS_INFO("updateGrid Done");
    partListSample.push_back(Particle(predictPose,updatedMap));  //create new list of particles for resampling
    weightSum = weightSum + weight[i];
    //get index of particle that has the maximum weight
    if(weight[i] > weight[maxWeightIndex]){
      bestParticle = Particle(predictPose,updatedMap); //set bestParticle to be used for publishing map later
    }
  }
  ROS_INFO("Resampling");
  //resample particles based on weights
  int listLength = static_cast<int>(partListSample.size());
  int index = rand() % listLength;
  float beta = 0;
  for(int i = 0; i < listLength; i++){      //resampling wheel algorithm
    beta = beta + ((double) rand() / (RAND_MAX)) * 2 * weight[maxWeightIndex];
    while( beta > weight[index]) {
      beta = beta - weight[index];
      index = (index + 1) % listLength;
    }
    partListResample.push_back(partListSample.at(index)); //set resampled values into new particle list
  }
  ROS_INFO("PF Done");
  return partListResample;
};



int main( int argc, char** argv) {

  ros::init(argc, argv, "fastslam");
  ros::NodeHandle n;
  curTime = ros::Time::now();
  ros::Rate r(10);

  //subscribe to command velocity and laser scanner
  ros::Subscriber velocitySub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, velocityCallback);
  ros::Subscriber scannerSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, scanCallback);
  //publish to my_map topic
  ros::Publisher mapPub = n.advertise<nav_msgs::OccupancyGrid>("my_map", 1);

  OccGrid map;  //initialize empty occupancyGrid
  pose[0] = map.getRobotXOffset()*map.getResolution();  //set robot pose offset to center robot in map
  pose[1] = map.getRobotYOffset()*map.getResolution();
  pose[2] = 1.5708; //easier for visualizing
  ROS_INFO("test2");
  Particle initialParticle(pose, map);
  std::vector<Particle> particleList; //initialize array of particles
  for(int i = 0; i < 50; i++){
    particleList.push_back(initialParticle);
  }
  ROS_INFO("test3");
  //information needed for publishing OccupancyGrid
  nav_msgs::OccupancyGrid bestMap;
    //tf information
  bestMap.header.seq = 1;
  bestMap.header.stamp = ros::Time::now();
  bestMap.header.frame_id = "base_footprint";
  bestMap.info.origin.position.x = -pose[0];
  bestMap.info.origin.position.y = -pose[1];
  bestMap.info.origin.position.z = 0;
  bestMap.info.origin.orientation.x = 0;
  bestMap.info.origin.orientation.y = 0;
  bestMap.info.origin.orientation.z = 0;
  bestMap.info.origin.orientation.w = 0;
  bestMap.info.resolution = map.getResolution();
  bestMap.info.width = map.getWidth();
  bestMap.info.height = map.getHeight();
  ROS_INFO("test4");

  //endlessly run the particle filter
  while(ros::ok){
    ros::spinOnce();  //get subscriber information
    if(!waitScanner && !waitCmdVel){
      bestMap.info.map_load_time = ros::Time::now();
      bestMap.header.stamp = ros::Time::now();
      particleList = particleFilter(particleList);
      bestMap.header.seq++;
      bestMap.data = bestParticle.getMap().getFlattenedMap();
      mapPub.publish(bestMap);  //publish to map topic
      ROS_INFO("Published Map");
      waitScanner = true;
      waitCmdVel = true;
    }
  }


}
