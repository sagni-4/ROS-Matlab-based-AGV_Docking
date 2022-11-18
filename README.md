# ROS-Matlab-based-AGV_Docking
This Repository contains ROS and Matlab based Autonomously Guided Vehicle docking method projects on Gazebo simulation environment.The project consists of a LiDAR sensor based AGV docking station detection method and proximity ruler based docking algorithim.
The AGV positioning and Localization method used for this project is based on UltraWide Band(UWB) positioning method by using a public repository from Advoard Localization pulic repository and you can find all steps to launch this file [here](https://github.com/advoard/advoard_localization). 

## Launching The whole projects
The below two launch command will launch the AGV on the Gazebo and rviz environment ,making it ready for simualtion.
1. roslaunch robothesis_2dnav robothesis_configuration.launch
2. roslaunch robothesis_2dnav robothesis_2dnav.launch
Then 
3.roslaunch advoard_localization pozyx_sim_kalman.launch , to start the UWB based AGV Localization.
After launching the project using the above commands, all the data from the sensor are ready and starts getting published on their corresponding ROS topic.
For examples:
LiDAR data on :/scan
UWB data on :/uwb_data_topic
proximity sensor data on :/proximty_frontLeft/range/lasers, and etc.

![AGV_Docking](https://user-images.githubusercontent.com/67055717/202656236-c1ce706a-40fa-4ee5-a2dc-69a3e4a87cc5.gif)
