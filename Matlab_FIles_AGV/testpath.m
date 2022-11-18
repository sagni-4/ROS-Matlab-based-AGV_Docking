clear;
rosshutdown;
rosinit;


DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];
pozyxData=rossubscriber("/localization_data_topic","DataFormat","struct");
ground_truth=rossubscriber("/ground_truth/state","DataFormat","struct");
[velPub,velMsg] = rospublisher('/cmd_vel');
pozyx_data=receive(pozyxData);
True_pose_data=receive(ground_truth);
Orient=True_pose_data.Pose.Pose.Orientation;
Orients=[Orient.X ,Orient.Y, Orient.Z ,Orient.W];
orient_euler=Quaternions2EulerAngles(Orients);
pozyx_initPose=[pozyx_data.Point.X, pozyx_data.Point.Y ,orient_euler];
path=[0 0;
    DockSta1Cord;
    DockSta2Cord;
    DockSta3Cord;
    DockSta4Cord
                 ];
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = pozyx_initPose(3);
robotCurrentPose = [robotInitialLocation initialOrientation]';
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);
while( distanceToGoal > goalRadius )
   release(controller);
   path(1,:)=[robotCurrentPose(1) robotCurrentPose(1)];
   controller.Waypoints = path;
    % Compute the controller outputs, i.e., the inputs to the robot
%     release(controller);
    [v, omega] = controller(robotCurrentPose);
    velMsg.Linear.X = v;
	velMsg.Angular.Z = omega;
	velPub.send(velMsg);

    newpozyx_data=receive(pozyxData);
    newTrue_pose_data=receive(ground_truth);
    Orient=newTrue_pose_data.Pose.Pose.Orientation;
    Orients=[Orient.X ,Orient.Y, Orient.Z ,Orient.W];
    orient_euler=Quaternions2EulerAngles(Orients);
    robotCurrentPose =[pozyx_data.Point.X, pozyx_data.Point.Y ,orient_euler];
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:))
end