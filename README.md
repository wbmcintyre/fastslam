# Grid based Fastslam from scratch
This project was created to implement the grid based fastslam algorithm similar to the built in gmapping package for ros. This project uses the turtlebot3 packages so as not to recreate the robot model (urdf file), environments(world files), and teleoperation control.


## Example SLAM images
(https://github.com/wbmcintyre/fastslam/blob/main/images/image1.png?raw=true)
(https://github.com/wbmcintyre/fastslam/blob/main/images/image2.png?raw=true)


## Future possibilities
- ICP for better localization
- Dynamic number of particles
- Reduce resampling based on measurement weights
- Neural Network to optimize noise alpha values
