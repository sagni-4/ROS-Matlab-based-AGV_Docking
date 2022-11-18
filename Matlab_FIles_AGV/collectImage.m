
rosshutdown;
clear;
clc;
rosinit;
DockSta1Cord=[-0.0380 -8 0];
DockSta2Cord=[8.15 -1.5 0];
DockSta3Cord=[3.66 7.63 0];
DockSta4Cord=[-3.88 2.03 0];
while(1)
x=rossubscriber("/scan","DataFormat","struct"); 
uwb_data=rossubscriber("/localization_data_topic","DataFormat","struct");
data1=receive(uwb_data);
x1=data1.Point.X;
y1=data1.Point.Y;
scanData=receive(x);
scans = rosReadLidarScan(scanData);
angles = rosReadScanAngles(scanData);
cart = rosReadCartesian(scanData);
maxRange = 15; % meters
resolution = 10; % cells per meter
slamObj = lidarSLAM(resolution,maxRange);
slamObj.LoopClosureThreshold = 360;
slamObj.LoopClosureSearchRadius = 8;

for i = 1:numel(scans)

    addScan(slamObj,scans(i));
    
    if rem(i,10) == 0
        show(slamObj);
    end
end
[scansSLAM,poses] = scansAndPoses(slamObj);
occMap = buildMap(scansSLAM,poses,resolution,maxRange);
figure(1)
show(occMap)
title('')
set(gca,'Visible','off')
set(gca,'XColor','none','YColor','none','TickDir','out')
% Requires R2020a or later
%/////////naming the map///////
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
distD=round(distD);
robot_heading=round(robot_heading);
%//////end of data for naming the map//////////////////
 t = sprintf('stat4P18_dist_%d_heading_%d.png',distD,robot_heading); 
exportgraphics(gca, t ,'Resolution',400) 
 pause(1)
end
