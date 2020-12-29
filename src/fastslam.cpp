
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <array>
#include <vector>


#include "OccGrid.h"
#include "Particle.h"
#include "LaserScanner.h"



std::array<float,3> initPose = {0.0,0.0,0.0}; //initialize robot pose (x)
std::array<float,2> velocities = {0.0,0.0}; //initialize command velocities (u)
int numParticles = 300;
LaserScanner scanner;
std::vector<Particle> particleList;
bool firstScan = true;
bool firstUpdateVel = false;
bool firstUpdateScan = false;
bool waitScanner = true;
bool waitCmdVel = true;
bool partFilterFinished = true;
float robotGazeboOffsetX = -2.0;
float robotGazeboOffsetY = -0.5;
float mapXOffset;
float mapYOffset;


//time variables for predicting poses
ros::Time curTime;
float deltaTime = 0.1;
bool firstDelta = true;
float totalLinearDist = 0;
float totalAngularDist = 0;
float totalDeltaTime = 0;


//obtain the most probable particle to display on rviz
Particle bestParticle;

//Array of poses for rviz display
std::vector<geometry_msgs::Pose> poses;


void initializeParticles(){
  OccGrid map;
  map.updateGrid(initPose, scanner);
  for(int i = 0; i < numParticles; i++){    //initialize array of particles
    Particle initialParticle(initPose, map);
    //initialParticle.addNoise();
    particleList.push_back(initialParticle);
  }
}


void addParticleToPoseArray(std::array<float,3> particlePose){    //adds pose information to pose array for display in rviz
  geometry_msgs::Pose initGeomPose;
  initGeomPose.position.x = particlePose[0]-mapXOffset;
  initGeomPose.position.y = particlePose[1]-mapYOffset;
  initGeomPose.position.z = 0;
  initGeomPose.orientation.w = cos(particlePose[2]/2);
  initGeomPose.orientation.x = 0;
  initGeomPose.orientation.y = 0;
  initGeomPose.orientation.z = sin(particlePose[2]/2);
  poses.push_back(initGeomPose);
}

//collect current command velocity values to predict robot pose
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg){


  ros::Time newTime = ros::Time::now();
  float intervalTime = (newTime - curTime).toSec();
  curTime = newTime;
  //ROS_INFO("Received Vel");
  if(firstDelta == true){ //first time interval doesn't calculate correctly (need to figure out this issue)
    intervalTime = 0.1;
    firstDelta = false;
    firstUpdateVel = true;
  }

  totalLinearDist += msg->linear.x*intervalTime;
  totalAngularDist += msg->angular.z*intervalTime;
  totalDeltaTime += intervalTime;

  if((totalLinearDist < .01 && totalLinearDist > -.01) && (totalAngularDist < .01 && totalAngularDist > -.01)){  //only run the particle filter after the robot has moved a certain distance
    waitCmdVel = true;
  } else if (partFilterFinished && !waitScanner){   //if the robot has moved and the particle filter has finished running, set new velocities
    velocities[0] = totalLinearDist/totalDeltaTime; //set velocity values as average velocity
    velocities[1] = totalAngularDist/totalDeltaTime;
    deltaTime = totalDeltaTime;
    totalLinearDist = 0;
    totalAngularDist = 0;
    totalDeltaTime = 0;
    waitCmdVel = false;
    partFilterFinished = false;
    ROS_INFO("V, W, dt: %f %f %f",velocities[0], velocities[1], deltaTime);
  }

}


//collect laser scan values for measurement matching
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

  std::vector<float> scan = msg->ranges;
  if(firstScan) {
    float angleMin = msg->angle_min;
    float angleMax = msg->angle_max;
    float angleInc = msg->angle_increment;
    float rangeMin = msg->range_min;   //overwrite due to robot seeing itself (value based on urdf file)
    float rangeMax = msg->range_max;
    scanner = LaserScanner(scan, angleMin, angleMax, angleInc, rangeMin, rangeMax);
    ROS_INFO("INITIALIZING PARTICLES");
    initializeParticles();
    firstScan = false;
    firstUpdateScan = true;
  } else if (partFilterFinished){
    scanner.setScan(scan);
    waitScanner = false;
  }

}


void particleFilter() {
  ROS_INFO("PartFilter Start, Size: %d", static_cast<int>(particleList.size()));
  float weight[static_cast<int>(particleList.size())];  //weights used for resampling particles
  float weightSum = 0;
  int maxWeightIndex = 0;
  std::vector<Particle> partListSample;  //particle list created from predicted Poses and updated Maps
  std::vector<Particle> partListResample; //particle list used for resampling and is returned
  partListSample.clear();
  partListResample.clear();
  for(int i = 0; i < static_cast<int>(particleList.size()); i++){
    std::array<float,3> predictPose = particleList[i].predictPose(velocities, deltaTime);
    OccGrid updatedMap = particleList[i].getMap();
    weight[i] = updatedMap.getWeightByMeasurements(predictPose, scanner);
    updatedMap.updateGrid(predictPose, scanner);
    partListSample.push_back(Particle(predictPose,updatedMap));  //create new list of particles for resampling
    weightSum = weightSum + weight[i];


    //get index of particle that has the maximum weight
    if(i == 0){
      bestParticle = Particle(predictPose,updatedMap); //set bestParticle to be used for publishing map later
      maxWeightIndex = 0;
    } else if(weight[i] > weight[maxWeightIndex]){
      bestParticle = Particle(predictPose,updatedMap); //set bestParticle to be used for publishing map later
      maxWeightIndex = i;
    }
  }

  for(int j = 0; j < static_cast<int>(particleList.size()); j++){
    weight[j] = weight[j]/weightSum;
  }
  ROS_INFO("BestParticlePose x,y,t : %f %f %f", bestParticle.getPose()[0], bestParticle.getPose()[1], bestParticle.getPose()[2]);
  ROS_INFO("MaxWeight : %f", weight[maxWeightIndex]);


  //resample particles based on weights
  ROS_INFO("Resampling");
  int listLength = static_cast<int>(partListSample.size());
  int index = rand() % listLength;
  float beta = 0;
  poses.clear();  //clear pose array to add new resampled particles
  for(int i = 0; i < listLength; i++){      //resampling wheel algorithm
    beta = beta + ((double) rand() / (RAND_MAX)) * 2 * weight[maxWeightIndex];
    while( beta > weight[index]) {
      beta = beta - weight[index];
      index = (index + 1) % listLength;
    }
    partListResample.push_back(partListSample.at(index)); //set resampled values into new particle list
    addParticleToPoseArray(partListSample.at(index).getPose()); //add particle to pose array
  }
  ROS_INFO("PF Done");
  particleList = partListResample;
};



int main( int argc, char** argv) {

  ros::init(argc, argv, "fastslam");
  ros::NodeHandle n;
  curTime = ros::Time::now();
  ros::Rate r(10);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  OccGrid map;  //initialize empty occupancyGrid
  mapXOffset = map.getMapCenterX();
  mapYOffset = map.getMapCenterY();
  initPose[0] = mapXOffset+robotGazeboOffsetX;  //set robot pose offset to center map in rviz (according to gazebo file)
  initPose[1] = mapYOffset+robotGazeboOffsetY;
  initPose[2] = 0; //easier for rviz visualization (optional to rotate map instead)

  //subscribe to command velocity and laser scanner
  ros::Subscriber velocitySub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, velocityCallback);
  ros::Subscriber scannerSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, scanCallback);

  //publish to map topic and particles topic
  ros::Publisher mapPub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
  ros::Publisher posePub = n.advertise<geometry_msgs::PoseArray>("particles",1);
  ros::Publisher bestPosePub = n.advertise<geometry_msgs::PoseStamped>("bestPose",1);

  //information needed for publishing OccupancyGrid
  nav_msgs::OccupancyGrid bestMap;
  bestMap.header.seq = 1;
  bestMap.header.stamp = ros::Time::now();
  bestMap.header.frame_id = "odom";
  bestMap.info.origin.position.x = -map.getMapCenterX();
  bestMap.info.origin.position.y = -map.getMapCenterY();
  bestMap.info.origin.position.z = 0;
  bestMap.info.origin.orientation.w = 0;
  bestMap.info.origin.orientation.x = 0;
  bestMap.info.origin.orientation.y = 0;
  bestMap.info.origin.orientation.z = 0;
  bestMap.info.resolution = map.getResolution();
  bestMap.info.width = map.getWidth();
  bestMap.info.height = map.getHeight();

  //information needed for publishing PoseArray of particles
  geometry_msgs::PoseArray poseArr;
  poseArr.header.seq = 1;
  poseArr.header.stamp = ros::Time::now();
  poseArr.header.frame_id = "odom";

  //information needed for publishing Pose of best particle
  geometry_msgs::PoseStamped bestPose;
  bestPose.header.seq = 1;
  bestPose.header.stamp = ros::Time::now();
  bestPose.header.frame_id = "odom";


  //endlessly run the particle filter
  while(ros::ok){
    if((!waitScanner && !waitCmdVel) || (firstUpdateVel && firstUpdateScan)){ //run when initialized, otherwise wait until callbacks are called

      particleFilter();  //run particle filter
      firstUpdateVel = false;   //used to initialize map prior to robot moving
      firstUpdateScan = false;


      //publish to map topic
      bestMap.info.map_load_time = ros::Time::now();
      bestMap.header.stamp = ros::Time::now();
      bestMap.header.seq++;
      bestMap.data = bestParticle.getMap().getFlattenedMap();
      mapPub.publish(bestMap);

      //publish pose array
      poseArr.header.seq++;
      poseArr.header.stamp = ros::Time::now();
      poseArr.poses = poses;
      posePub.publish(poseArr);

      //publish best estimated pose
      std::array<float,3> bp = bestParticle.getPose();
      bestPose.pose.position.x = bp[0]-mapXOffset;
      bestPose.pose.position.y = bp[1]-mapYOffset;
      bestPose.pose.position.z = 0;
      bestPose.pose.orientation.w = cos(bp[2]/2);
      bestPose.pose.orientation.x = 0;
      bestPose.pose.orientation.y = 0;
      bestPose.pose.orientation.z = sin(bp[2]/2);
      bestPosePub.publish(bestPose);


      ROS_INFO("Published Info");
      waitScanner = true;   //wait for scan and command velocity
      waitCmdVel = true;
      partFilterFinished = true;
    }
  }

  ros::waitForShutdown();


}
