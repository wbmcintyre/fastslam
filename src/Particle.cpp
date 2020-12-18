
#include "Particle.h"
#include "OccGrid.h"
#include <math.h>
#include <array>
#include <ros/ros.h>

Particle::Particle(){}

Particle::Particle(std::array<float,3> p, OccGrid m){
  pose = p;
  map = m;
}


//returns the robot pose based on the kinematics of the robot
std::array<float,3> Particle::predictPose(std::array<float,2> vel, float deltaTime){
  float noisyV = vel[0] + noise_sample(alpha1*vel[0] + alpha2*vel[1]); //linear velocity with added noise
  float noisyW = vel[1] + noise_sample(alpha3*vel[0] + alpha4*vel[1]); //angular velocity with added noise
  float gamma = noise_sample(alpha5*vel[0] + alpha6*vel[1]); //noise sample for theta
  std::array<float,3> predictPose;
  if(noisyW != 0){    //if robot is moving, update robot motion
    float r = noisyV/noisyW;
    predictPose[0] = pose[0] - r*(float)sin(pose[2]) + r*(float)sin(pose[2] + noisyW*deltaTime);   //kinematics for predicted pose
    predictPose[1] = pose[1] + r*(float)cos(pose[2]) - r *(float)cos(pose[2] + noisyW*deltaTime);
    predictPose[2] = pose[2] + noisyW*deltaTime + gamma*deltaTime;
    //ROS_INFO("%f, %f, %f, %f", predictPose[0], predictPose[1], predictPose[2],deltaTime);
    return predictPose;
  }
  return pose;
}

float Particle::noise_sample(float b) { //create noise from -b to b
  float noise = 0;
  float rootB = sqrt(b);
  for(int i = 0; i < 12; i++){
    noise = noise + ((float)rand()/(float)RAND_MAX) * (2*rootB) - rootB;
  }
  return 0.5 * noise;
}


OccGrid Particle::getMap(){
  return map;
}
