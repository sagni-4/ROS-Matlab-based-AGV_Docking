rosshutdown;
rosinit;
DockSta1Cord=[-0.0380 -8 0];
DockSta2Cord=[8.15 -1.5 0];
DockSta3Cord=[3.66 7.63 0];
DockSta4Cord=[-3.88 2.03 0];
Dock_sta_cord=DockSta1Cord;
while(1)
laserSub = rossubscriber('/scan');
uwb_data=rossubscriber("/localization_data_topic","DataFormat","struct");
[velPub,velMsg] = rospublisher('/cmd_vel');

data1=receive(uwb_data);
  x=data1.Point.X;
  y=data1.Point.Y;
  curpose=[x,y,0];

vfh = controllerVFH;
vfh.UseLidarScan = true;
vfh.DistanceLimits = [0.1 3];
vfh.RobotRadius = 0.35;%chassis radius+ both wheel radious
vfh.MinTurningRadius = 0.1;
vfh.SafetyDistance = 0.2;
% vfh.TargetDirectionWeight=5;
% vfh.CurrentDirectionWeight=1.5;
% vfh.PreviousDirectionWeight=1.5;
% vfh.HistogramThresholds=[3,10];
cpp = controllerPurePursuit;
cpp.DesiredLinearVelocity=0.5;
cpp.LookaheadDistance=2;
cpp.MaxAngularVelocity=1;
cpp.Waypoints=[0 0;Dock_sta_cord(1) Dock_sta_cord(2)];
[vel,angvel] = cpp(curpose);
targetDir = 45;
rate = rateControl(10);
% while rate.TotalElapsedTime < 30

	% Get laser scan data
	laserScan = receive(laserSub);
	ranges = double(laserScan.Ranges);
	angles = double(laserScan.readScanAngles);
 
	% Create a lidarScan object from the ranges and angles
        scan = lidarScan(ranges,angles);
        
	% Call VFH object to computer steering direction
	steerDir = vfh(scan, targetDir);  
   % steerDir=vfh(ranges,angles,targetDir);
	% Calculate velocities
	if ~isnan(steerDir) % If steering direction is valid
		desiredV = vel;
		w = steerDir;
	else % Stop and search for valid direction
		desiredV = 0.0;
		w = angvel;
	end

	% Assign and send velocity commands
	velMsg.Linear.X = desiredV;
	velMsg.Angular.Z = w;
	velPub.send(velMsg);
end
