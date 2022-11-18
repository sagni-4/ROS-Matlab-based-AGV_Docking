rosshutdown;
clear;
clc;
rosinit;
DockSta1Cord=[-0.0380 -8 0];
DockSta2Cord=[8.15 -1.5 0];
DockSta3Cord=[3.66 7.63 0];
DockSta4Cord=[-3.88 2.03 0];
DockStaCord=DockSta4Cord;
robot_odom= rossubscriber("/odom","DataFormat","struct");
odom_data=receive(robot_odom);
[Poses,Orients,~,~]=robot2PoseCallback(odom_data);
[distD,~,~,DockOreint]=DockwithoutLidar(DockStaCord,Poses,Orients);
[~,~,y]=Quaternions2EulerAngles(Orients);
robot_heading=atand(y);
if (robot_heading>=DockOreint)
TargetAngle=(robot_heading-DockOreint);
elseif(robot_heading<DockOreint)
TargetAngle=(DockOreint-robot_heading);
end