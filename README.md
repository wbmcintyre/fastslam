# Grid based Fastslam from scratch
This project was created to implement the grid based fastslam algorithm similar to the built in gmapping package for ros. This project uses the turtlebot3 packages so as not to recreate the robot model (urdf file), environments(world files), and teleoperation control.

## Setup

To run this project, begin by cloning this repository into the src folder of a catkin workspace. The following link has instruction on setting up the remaining turtlebot3 packages that will also need to be cloned into the same catkin workspace(https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/).

After cloning all of the packages, open up 4 different terminals to run 3 launch files and the fastslam cpp file. Each of the below scripts should be ran in their own terminal.

The following will launch your gazebo world.
`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

The following will launch the rviz environment.
`roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch`

The following will launch the teleop control of the robot.
`roslaunch turtlebot3_teleop turtlebot3_telop.launch`

The following will run the fastslam algorithm.
`rosrun fastslam fastslam`


After all of these processes are running, the rostopics for fastslam can be added into rviz. Go to rviz and click Add in the bottom left to add the /map, /particles, and /bestPose topics for display. The rviz display will now show the robot, the laser scanner (in red), the particles pose array (in red), the most probable robot pose (in blue), and the map in the background.

While rviz is on the screen, open up the terminal containing the teloperation control. This can be used to move the robot around and to observe the SLAM algorithm. The particle filter and resampling only occur after the robot has moved .01 meters in any direction. This is to prevent the robot from resampling out poses while not moving. Below are the following parameters that can be adjusted to better increase performance of this current algorithm.

Particle.h file
  - alpha1 to alpha6 - noise variables used for sampling predicted robot poses
OccGrid.h file
  - resolution - change the map resolution (.05 meters by default)
  - width - change size of map
  - height
fastslam.cpp file
  - numParticles - change how many particles 


## Analysis
To be done...



## Example SLAM images
Below are example images to demonstrate how the robot localizes and maps the environment when running this application.

![Image1](https://github.com/wbmcintyre/fastslam/blob/main/images/image1.PNG)
![Image2](https://github.com/wbmcintyre/fastslam/blob/main/images/image2.PNG)
![Image3](https://github.com/wbmcintyre/fastslam/blob/main/images/image3.PNG)
![Image4](https://github.com/wbmcintyre/fastslam/blob/main/images/image4.PNG)
![Image5](https://github.com/wbmcintyre/fastslam/blob/main/images/image5.PNG)
![Image6](https://github.com/wbmcintyre/fastslam/blob/main/images/image6.PNG)
![Image7](https://github.com/wbmcintyre/fastslam/blob/main/images/image7.PNG)
![Image8](https://github.com/wbmcintyre/fastslam/blob/main/images/image8.PNG)


## Future possibilities
- Navigation (Markov Decision Process)
- ICP for better localization
- Dynamic number of particles
- Reduce resampling based on measurement weights
- Neural Network to optimize noise alpha values
- Combine all packages and use xterm for launching


## License

The content of this repository is licensed under a [Creative Commons Attribution License](https://creativecommons.org/licenses/by/3.0/us/).
