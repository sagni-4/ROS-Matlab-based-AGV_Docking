

rosshutdown;
clear all;
clc;
% pause(1)
rosinit;
robot_odom= rossubscriber("/odom","DataFormat","struct");
robot_cmdVelPub=rospublisher("/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
LaserScan=rossubscriber("/scan","DataFormat","struct"); 
pozyx_data=rossubscriber("/localization_data_topic","DataFormat","struct");
proxim1_data=rossubscriber("/proximty_frontLeft/range/lasers","DataFormat","struct");
proxim2_data=rossubscriber("/proximty_frontRight/range/lasers","DataFormat","struct");
proxim3_data=rossubscriber("/proximty_LsideFront/range/lasers","DataFormat","struct");
proxim4_data=rossubscriber("/proximty_LsideRear/range/lasers","DataFormat","struct");

% range= rossubscriber("/range/lasers","DataFormat","struct");
% pause(2)
% roll=0;
% pitch=0;
% yaw=0;
% Heading=0;

%    OdomData=robot_odom.LatestMessage;
%    [~,Orients,~,~]=robot2PoseCallback(OdomData);
%    Heading=Quaternions2EulerAngles(Orients);
%    Heading=atand(Heading);
%    theth=0;
% Coordinates of docking station and odcking gates.
windowsize=5;
window=zeros(windowsize,1);
k=1;
DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];
Gate1=[2.17 -7.777];
Gate2=[8.046 0.221];
Gate3=[2.18 7.28];
Gate4=[-3.63 0.93];
%selecting the docking station randomly
dockTo=randi([1 4]);
if dockTo==1
path=[Gate1;
      DockSta1Cord];
elseif dockTo==2
   path=[Gate2;
         DockSta2Cord];
elseif dockTo==3
    path=[Gate3;
          DockSta3Cord];
elseif dockTo==4
    path=[Gate4;
          DockSta4Cord];
end
% shifting between approaching phase and docking phase.
for i=1:2
 if i==1
     DockStaCord=path(1,:);
 elseif i==2
     DockStaCord=path(2,:);
end
%move to the destination.
if DockStaCord==Gate1
    theth=180;
elseif DockStaCord==Gate2
    theth=-90;
elseif DockStaCord==Gate3
    theth=0;  
elseif DockStaCord==Gate4
    theth=90;
elseif DockStaCord==DockSta1Cord
    theth=-180;
elseif DockStaCord==DockSta2Cord
    theth=-90;
elseif DockStaCord==DockSta3Cord
    theth=0;
elseif DockStaCord==DockSta4Cord
    theth=90;
end
% Starting the approaching phase
if i==1
  while(1)
  %Set heading toward the docking gate.
  recievedData=robot_odom.LatestMessage;
  scanmsg=LaserScan.LatestMessage;
  Pozyx_pose=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(recievedData,Pozyx_pose);
  cmdvel_msg=rosmessage(robot_cmdVelPub);
  [distD,Lin_vel,target,dockOrient]=DockwithoutLidar(DockStaCord,pozyxposes,Orients);
  [rot_Speed]=SetRobotHeading(dockOrient,Orients);
  cmdvel_msg.Angular.Z=rot_Speed;
  send(robot_cmdVelPub,cmdvel_msg);
  target_rad=target*pi/180;
  rot_Speed
  if (abs(rot_Speed)<=0.000525)
    cmdvel_msg.Angular.Z=0.00;
    %Move the AGV toward the docking gate
    while(1)
 recievedData=robot_odom.LatestMessage;
 scanmsg=LaserScan.LatestMessage;       
 Pozyx_pose=pozyx_data.LatestMessage;
 [Poses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(recievedData,Pozyx_pose);
 [distD,Lin_vel,~,~]=MoveToGoalAndDock(robot_odom,pozyx_data,theth,path(2,:));
 dist2goal=distD;
 dist2gate=norm(path(1,:)-pozyxposes);
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
 dist2goal
 dist2gate
 if dist2goal<5 
     while(1)
 recievedData=robot_odom.LatestMessage;
 scanmsg=LaserScan.LatestMessage;       
 Pozyx_pose=pozyx_data.LatestMessage;
 [Poses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(recievedData,Pozyx_pose);
 [distD,Lin_vel,~,~]=MoveToGoalAndDock(robot_odom,pozyx_data,theth,path(2,:));
 dist2goal=distD;
 dist2gate=norm(path(1,:)-pozyxposes);
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
 dist2goal
 dist2gate

 [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(scanmsg,window,windowsize,k); 
 k=k+1;
 if k==windowsize
     k=1;
 end
% check if distance to docking gate is less than 5cm and start docking phase
 if(dist2gate<=0.5)

     %Oreint the heading of the agv toward the docking point.
     while(1)
    recievedData=robot_odom.LatestMessage;
    scanmsg=LaserScan.LatestMessage;
    pozyx_pose=pozyx_data.LatestMessage;
    [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,pozyx_pose);
    [distD,Lin_vel,target,dockOrient]=DockwithoutLidar(path(2,:),pozyxposes,Orients);
    [Orient]=SetRobotHeading(dockOrient,Orients);
    cmdvel_msg.Linear.X=0.00;
    cmdvel_msg.Angular.Z=Orient;
    send(robot_cmdVelPub,cmdvel_msg); 

%     if k==windowsize
%      k=1;
%     end
%     [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(scanmsg,window,windowsize,k); 
%     k=k+1;
    Orient 
  if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg);  
       break;
  end
     end
     if(dist2gate<=0.5)
     cmdvel_msg.Linear.X=0;
     send(robot_cmdVelPub,cmdvel_msg);
       break;
     end
 end
     end
     cmdvel_msg.Linear.X=0;
     send(robot_cmdVelPub,cmdvel_msg);
     break;
 end
    end 
 break;
  end
  end
 Lin_vel
 dist2goal
 dist2gate

%Docking using proximity sensor

elseif(i==2)
    %Start the docking phase
 while(1)
 recievedData=robot_odom.LatestMessage;
 scanmsg=LaserScan.LatestMessage;       
 Pozyx_pose=pozyx_data.LatestMessage;
 [FSL,FSR,LSF,LSR]=distance_sensor_ranges(proxim1_data,proxim2_data,proxim3_data,proxim4_data);
 [Poses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(recievedData,Pozyx_pose);
 distToDockStat=sqrt((pozyxposes(2)-DockStaCord(2))^2+(pozyxposes(1)-DockStaCord(1))^2)
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=0.1;
 send(robot_cmdVelPub,cmdvel_msg);
%check if the agvis close to the docking point.
   
 if distToDockStat<=0.4
    while(1)
  Pozyx_pose=pozyx_data.LatestMessage;
 [FSL,FSR,LSF,LSR]=distance_sensor_ranges(proxim1_data,proxim2_data,proxim3_data,proxim4_data);
 [Poses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(recievedData,Pozyx_pose);
 distToDockStat=sqrt((pozyxposes(2)-DockStaCord(2))^2+(pozyxposes(1)-DockStaCord(1))^2)
 %Check the distance to the docking station using the left side distance sensors
   if LSR>0.25||LSF>0.25 && distToDockStat> 0.3
  cmdvel_msg.Linear.X=0.1;
 send(robot_cmdVelPub,cmdvel_msg);
LSR
LSF
distToDockStat
%  if (LSR<=0.25||LSF<=0.25) && distToDockStat<0.15
%    cmdvel_msg.Linear.X=0.0;
%    send(robot_cmdVelPub,cmdvel_msg);
%    break;
%   end
%    end
   if (LSR<=0.25||LSF<=0.25) && distToDockStat<=0.3
   cmdvel_msg.Linear.X=0.0;
   send(robot_cmdVelPub,cmdvel_msg);
   while(1)
       %Orient the AGV in parallel to the docking station.
    recievedData=robot_odom.LatestMessage;
    scanmsg=LaserScan.LatestMessage;
    pozyx_pose=pozyx_data.LatestMessage;
    [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,pozyx_pose);
    [distD,Lin_vel,target,dockOrient]=DockwithoutLidar(path(2,:),pozyxposes,Orients);
    [Orient]=SetRobotHeading(theth,Orients);
    cmdvel_msg.Linear.X=0.00;
    cmdvel_msg.Angular.Z=Orient;
    send(robot_cmdVelPub,cmdvel_msg); 
%     [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(scanmsg,window,windowsize,k); 
    Orient 
    %Dock the AGV and stop until the next Docking station is selected.
  if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg);  
       break;
  end
   end
       break;
   end
   end
 end
if (LSR<=0.25||LSF<=0.25) && distToDockStat<=0.3
    cmdvel_msg.Linear.X=0.00;
%     cmdvel_msg.Angular.Z=Orient;
    send(robot_cmdVelPub,cmdvel_msg); 

break;
end
end
end
end
end
