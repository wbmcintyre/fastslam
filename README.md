# Grid based Fastslam from scratch
This project was created to implement the grid based fastslam algorithm similar to the built in gmapping package for ros. This project uses the turtlebot3 packages so as not to recreate the robot model (urdf file), environments(world files), and teleoperation control.

## Setup
To be done...

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
