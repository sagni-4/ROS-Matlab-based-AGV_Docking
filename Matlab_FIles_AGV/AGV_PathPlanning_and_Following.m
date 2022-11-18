
clear;
rosshutdown;
rosinit;


DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];

pozyxData=rossubscriber("/localization_data_topic","DataFormat","struct");
ground_truth=rossubscriber("/ground_truth/state","DataFormat","struct");
laserScan=rossubscriber('/scan');
[velPub,velMsg] = rospublisher('/cmd_vel');

laser_data=receive(laserScan);
True_pose_data=receive(ground_truth);
pozyx_data=receive(pozyxData);



% [obstacle_range,agv_truePose,pozyx_agvPose]=rosTopicData(laser_data,True_pose_data,pozyx_data);
ranges = double(laser_data.Ranges);
angles = double(laser_data.readScanAngles);
obstacle_range=[ranges angles];

Pose=True_pose_data.Pose.Pose.Position;
Orient=True_pose_data.Pose.Pose.Orientation;
Poses=[Pose.X, Pose.Y, Pose.Z];
Orients=[Orient.X ,Orient.Y, Orient.Z ,Orient.W];

orient_euler=Quaternions2EulerAngles(Orients);
agv_truePose=[Pose.X, Pose.Y, orient_euler];
pozyx_agvPose=[pozyx_data.Point.X, pozyx_data.Point.Y ,orient_euler];

vfh = controllerVFH;
vfh.UseLidarScan = true;
vfh.DistanceLimits = [0.1 1];
vfh.RobotRadius = 0.25;%chassis radius+ both wheel radious
vfh.MinTurningRadius = 0.1;
vfh.SafetyDistance = 0.2;

dockTo=randi([1 4]);
if dockTo==1
path=[pozyx_data.Point.X, pozyx_data.Point.Y ; 
    0.2 -9;
      DockSta1Cord];
elseif dockTo==2
   path=[pozyx_data.Point.X, pozyx_data.Point.Y ;
       4 -2.1;
         DockSta2Cord];
elseif dockTo==3
    path=[pozyx_data.Point.X, pozyx_data.Point.Y ;
        3.2 3;
          DockSta3Cord];
elseif dockTo==4
    path=[pozyx_data.Point.X, pozyx_data.Point.Y ;
        -4.8 1.2;
          DockSta4Cord];
end


robotInitialLocation = path(1,:);
robotGoal = path(end,:);
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;
goalRadius = 0.05;
robotCurrentPose = [pozyx_data.Point.X, pozyx_data.Point.Y ,orient_euler];

initialOrientation = agv_truePose(3);
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
distanceToGoal = norm(robotInitialLocation - robotGoal);
while( distanceToGoal > goalRadius )    

% Initialize the simulation loop
sampleTime = 0.1;

vizRate = rateControl(1/sampleTime);


    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
%     vel = derivative(robot, robotCurrentPose, [v w]);
    targetDir = omega;
   % Update the current pose
  % robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    laserscan = receive(laserScan);
	ranges = double(laserscan.Ranges);
	angles = double(laserscan.readScanAngles);
 
	% Create a lidarScan object from the ranges and angles
        scan = lidarScan(ranges,angles);
        
	% Call VFH object to computer steering direction
	steerDir = vfh(scan, targetDir);  
   % steerDir=vfh(ranges,angles,targetDir);
	% Calculate velocities
if ~isnan(steerDir) % If steering direction is valid
		desiredV = v;
		w = steerDir;
	else % Stop and search for valid direction
		desiredV = 0.0;
		w = omega;
 end

    velMsg.Linear.X =  desiredV;
%     vel(1);
%     velMsg.Linear.Y =  
%     vel(2);
	velMsg.Angular.Z = w; 
%     vel(3);
	velPub.send(velMsg);
    % Re-compute the distance to the goal
    True_pose_data=receive(ground_truth);
    pozyx_data=receive(pozyxData);
    Orient=True_pose_data.Pose.Pose.Orientation;
    Orients=[Orient.X ,Orient.Y, Orient.Z ,Orient.W];
    orient_euler=Quaternions2EulerAngles(Orients);
    robotCurrentPose = [pozyx_data.Point.X, pozyx_data.Point.Y ,orient_euler];
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:))
    % Update the current pose
    
end



% image=imread('Matlab_SLAM_Map.png');
% grayimage = rgb2gray(image);
% bwimage = grayimage < 0.5;
% grid = binaryOccupancyMap(bwimage);
% show(grid)
% figure
% show(map)
% prm = mobileRobotPRM(map, maxNodes);
% load('malbaloccupancymap.mat');
% figure(1)
% show(myOccMap)
% simpleMap= occupancyMatrix(myOccMap);
% map = binaryOccupancyMap(simpleMap);
% xSize = 10;
% ySize = 10;
% resolution =10;
% map = occupancyMap(xSize,ySize, resolution);
% map.GridOriginInLocal= [-mapSize(1)/2 -mapSize(2)/2];
% setOccupancy(map, [-xSize/2, -ySize/2], simpleMap);
% inflate(map,0.2);
% figure(2)
% show(map)