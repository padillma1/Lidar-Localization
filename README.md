# Mario-Padilla-Rodriguez-University of Detroit Mercy ELEE4031-4032
Senior Capstone Design MATLAB code and Simulink model used for Lidar localization

The ouster_test_1.m file is the MATLAB file containing the code written and used. Will need to have the Lidar Toolbox installed. If using Ouster lidar like in this code, also need to create an ouster lidar object at the beginning of the program:

obj = ousterlidar("ouster-model","/path-to-metadata-file"), 
where "path-to-metadata-file" is the location on your computer where the .json file for the lidar is, with name format:
os-99xxxxxxxxxx-metadata.json

If attempting to use ouster lidar on a new pc for the first time, will first need SDK (software dev. kit), which can be accessed from the following link:
https://static.ouster.dev/sdk-docs/
Follow the steps to get necessary files installed for ROS implementation
When roslaunching the lidar in the terminal, use this modified command instead, which specifies the lidar port (crucial) among other things:

roslaunch ouster_ros sensor.launch sensor_hostname:=os-992210001133.local udp_dest:=10.42.1.121 lidar_port:=7502 lidar_mode:=1024x20 timestamp_mode:=TIME_FROM_ROS_TIME



The Simulink model consists of a few blocks. In the final design (not current), will want to fully implement the ouster_test_1.m as the MATLAB function block inside, but still need to iron out a few errors. The current MATLAB function block contains only a small portion of this code intended to test if the model was capable of publishing data to a ROS topic on the network (it does). 
There are a few settings, however, that need to be changed from inside Simulink. 
1. Under Simulation: Model Settings --> Simulation Target --> Advanced Parameters --> Check Box for "Dynamic Memory Allocation in MATLAB Functions"
2. Again under Simulation: Variable Size Messages --> For sensor_msgs/PointCloud2, uncheck "Use default limits for this message type" --> Set the Maximum length (items) to an arbitrary large number (I used 1000000) to avoid index size errors

Doing the steps above should allow the Simulink model to work
