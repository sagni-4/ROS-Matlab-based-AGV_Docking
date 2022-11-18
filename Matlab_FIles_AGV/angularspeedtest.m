
rosshutdown;
clear all;
clc;
pause(1)
rosinit;
robot_odom= rossubscriber("/odom","DataFormat","struct");
robot_cmdVelPub=rospublisher("/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
odom=receive(robot_odom);
angularspeed=odom.Twist.Twist.Angular.Z;
cmdvel_msg=rosmessage(robot_cmdVelPub);
rot=-0.5;
cmdvel_msg.Angular.Z=rot;

send(robot_cmdVelPub,cmdvel_msg);