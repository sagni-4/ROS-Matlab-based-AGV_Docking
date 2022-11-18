rosshutdown;
clear;
clc;
rosinit;
%while(1)
robot_PointCloud= rossubscriber("/filtered_cloud","DataFormat","struct");
% PCData=robot2_PointCloud.LatestMessage;
PointCloudData=receive(robot_PointCloud);
scans=rossubscriber("/scan","DataFormat","struct");
scanData=receive(scans);
lidarpoints = rosReadLidarScan(scanData);
angles = rosReadScanAngles(scanData);
cart = rosReadCartesian(scanData);
cartxyz=zeros(length(cart),3);
cartxyz(:,1:2)=cart;
cartxyz(:,3)=0.2;
% rosPlot(PointCloudData)
% change the lidar points into point clouds
ptCloud = pointCloud(cartxyz);
 pcshow(ptCloud)
 pcwrite(ptCloud,'object3d.pcd','Encoding','ascii');
 frame = helperPrepareData(ptCloud);
%end