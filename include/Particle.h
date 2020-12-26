#ifndef PARTICLE_H
#define PARTICLE_H

#include "OccGrid.h"
#include <array>

class Particle{

  private:
    std::array<float,3> pose;
    OccGrid map;
    float alpha1 = 0.1;
    float alpha2 = 0.1;
    float alpha3 = 0.1;
    float alpha4 = 0.1;
    float alpha5 = 0.01;
    float alpha6 = 0.01;

  public:
    Particle();
    Particle(std::array<float,3> pose, OccGrid m);
    std::array<float,3> predictPose(std::array<float,2> vel, float deltaTime);
    OccGrid getMap();
    float noise_sample(float b);
    std::array<float,3> getPose();
    void addNoise();

};

#endif
