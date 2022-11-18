%% Some useful scripts are in here!.
% rosshutdown;
% clear;
% clc;
% rosinit;
% 
% roll=0;
% pitch=0;
% yaw=0;
% 
% theth=0;
% prevD=0;
% %subscribe to ros topics
% robot_odom= rossubscriber("/odom","DataFormat","struct");
% robot_cmdVelPub=rospublisher("/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
% pozyxData=rossubscriber("/localization_data_topic","DataFormat","struct");
% ground_truth=rossubscriber("/ground_truth/state","DataFormat","struct");
% pause(2)
% 
% DockSta1Cord=[-0.0380 -8];
% DockSta2Cord=[8.15 -1.5];
% DockSta3Cord=[3.66 7.63];
% DockSta4Cord=[-3.88 2.03];
% Dock_sta_cord=DockSta1Cord;
% 
% %Select the docking station and path to dock the agv
% 
% dockTo=randi([1 4]);
% if dockTo==1
% path=[0.2 -9;
%       DockSta1Cord];
% elseif dockTo==2
%    path=[4 -2.1;
%          DockSta2Cord];
% elseif dockTo==3
%     path=[3.2 3;
%           DockSta3Cord];
% elseif dockTo==4
%     path=[-4.8 1.2;
%           DockSta4Cord];
% end
% 
% kp=0.5;
% while(1)
%    OdomData=robot_odom.LatestMessage;
%    posyx_data=receive(pozyxData);
%    [~,Orients,~,~]=robot2PoseCallback(OdomData,posyx_data);
%    Agv_Heading=Quaternions2EulerAngles(Orients);
%    Agv_Heading=atand(Agv_Heading);
%    ChangHeading_angVel=SetRobotHeading(Agv_Heading,Orients);
%    
%    SetHeading_msg=rosmessage(robot_cmdVelPub);
%    SetHeading_msg.Angular.Z= ChangHeading_angVel;
%    send(robot_cmdVelPub,SetHeading_msg);
%    ChangHeading_angVel
%    if( ChangHeading_angVel<=0.001)
%        SetHeading_msg.Angular.Z=0;
%        send(robot_cmdVelPub,SetHeading_msg);
%         break
%     end
% end
% 
% %cmdvel_msg.Angular.Z=targt*pi/180;%change robot heading frist.
% 
% while(1)
% odomData=robot_odom.LatestMessage;
% pozyx_data=receive(pozyxData);
% [Poses,Orients,~,~,pozyxpose]=robot2PoseCallback(odomData,pozyx_data);
% %[roll,pitch,yaw]=Quaternions2EulerAngles(Orients);
% cmdvel_msg=rosmessage(robot_cmdVelPub);
% [~,~,target,dockOrient]=agvMoveParamCalc(path(1,:),pozyxpose,Orients);
% [rot_Speed]=SetRobotHeading(dockOrient,Orients);
% %%rot_Speed=kp*(target_rad);
% cmdvel_msg.Angular.Z=rot_Speed;
% send(robot_cmdVelPub,cmdvel_msg);
% %pause(2)
% target_rad=target*pi/180
% %yaw
% rot_Speed
% %% moving to the docking station
% if (abs(rot_Speed)<=0.0005)
%     cmdvel_msg.Angular.Z=0.00;
% %     cmdvel_msg.Linear.X=Lin_vel;
% % send(robot2_cmdVelPub,cmdvel_msg);
% % Lin_vel
% % distD
% break
% end
% 
% end
% while(1)
%  odomData=robot_odom.LatestMessage;
%  pozyx_data=receive(pozyxData);
% [Poses,Orients,~,~,pozyxpose]=robot2PoseCallback(odomData,pozyx_data);
% [distD,Lin_vel,~]=agvMoveParamCalc(path(1,:),pozyxpose,Orients);
% 
%  cmdvel_msg=rosmessage(robot_cmdVelPub);
%  cmdvel_msg.Linear.X=Lin_vel;
%  send(robot_cmdVelPub,cmdvel_msg);
% % DockSta3Cord
%  if(distD<=0.6) 
% while(1)
%  [~,~,y]=Quaternions2EulerAngles(Orients);
%  targAngle=atand(y);
% %if (targAngle>=thetha)
% if dockTo==1
%     thetha=0;
% elseif dockTo==2
%     thetha=90;
% elseif dockTo==3
%     thetha=180;
% elseif dockTo==4
%     thetha=-90;
% end
%  parallelOrient=kp*pi/180*(thetha-targAngle);
%  cmdvel_msg.Linear.X=0.00;
%  cmdvel_msg.Angular.Z=parallelOrient;
%  send(robot_cmdVelPub,cmdvel_msg); 
%  parallelOrient
%  if(abs(parallelOrient)<=0.0085)
%      cmdvel_msg.Angular.Z=0.00;
%      send(robot_cmdVelPub,cmdvel_msg); 
%      break
%  end
% end
% break
%  end
%  Lin_vel
%  distD
%  end

%% Working script for movement between docking stations
rosshutdown;
clear;
clc;
rosinit;
robot_odom= rossubscriber("/odom","DataFormat","struct");
robot_cmdVelPub=rospublisher("/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
pause(2)
roll=0;
pitch=0;
yaw=0;
 Heading=0;
%    OdomData=robot_odom.LatestMessage;
%    [~,Orients,~,~]=robot2PoseCallback(OdomData);
%    Heading=Quaternions2EulerAngles(Orients);
%    Heading=atand(Heading);
% % theth=0;
prevD=0;
DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];
DockStaCord=DockSta1Cord;

kp=0.5;
%Set robot heading if necessary.
while(1)
   OdomData=robot_odom.LatestMessage;
   [~,Orients,~,~]=robot2PoseCallback(OdomData);
   ChangHeading_angVel=SetRobotHeading(Heading,Orients);
   SetHeading_msg=rosmessage(robot_cmdVelPub);
   SetHeading_msg.Angular.Z= ChangHeading_angVel;
   send(robot_cmdVelPub,SetHeading_msg);
   ChangHeading_angVel
   if( ChangHeading_angVel<=0.00036)
       SetHeading_msg.Angular.Z=0;
       send(robot_cmdVelPub,SetHeading_msg);
        break
    end
end

%cmdvel_msg.Angular.Z=targt*pi/180;%change robot heading frist.
%Adjust robot heading toward next destination.
while(1)
recievedData=robot_odom.LatestMessage;
[Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData);
%[roll,pitch,yaw]=Quaternions2EulerAngles(Orients);
cmdvel_msg=rosmessage(robot_cmdVelPub);
[distD,Lin_vel,target,dockOrient]=DockwithoutLidar(DockStaCord,Poses,Orients);
[rot_Speed]=SetRobotHeading(dockOrient,Orients);
%%rot_Speed=kp*(target_rad);
cmdvel_msg.Angular.Z=rot_Speed;
send(robot_cmdVelPub,cmdvel_msg);
%pause(2)
target_rad=target*pi/180
%yaw
rot_Speed
%moving to the docking station
if (abs(rot_Speed)<=0.000625)
    cmdvel_msg.Angular.Z=0.00;
%     cmdvel_msg.Linear.X=Lin_vel;
% send(robot2_cmdVelPub,cmdvel_msg);
% Lin_vel
% distD
break
end

end
%move to the destination.
if (DockStaCord==DockSta1Cord)
    theth=-180;
elseif DockStaCord==DockSta2Cord
    theth=-90;
elseif DockStaCord==DockSta3Cord
    theth=0;
elseif DockStaCord==DockSta4Cord
    theth=90;
end
while(1)
 [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,theth,DockStaCord);
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
coo
DockStaCord
% if certain distance reached, stop and orient the agv in parallel to the station
 if(distD<=0.6) 
 while(1)
   recievedData=robot_odom.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData);
  [parallelOrient]=SetRobotHeading(theth,Orients);
  cmdvel_msg.Linear.X=0.00;
  cmdvel_msg.Angular.Z=parallelOrient;
  send(robot_cmdVelPub,cmdvel_msg); 
  parallelOrient
   if(abs(parallelOrient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg); 
     break
  end
end
break
 end
 Lin_vel
 distD
end
%% Working scripts for movement toward the virtual gate and docking station is added to the above scripts.
rosshutdown;
clear all;
clc;
pause(1)
rosinit;
robot_odom= rossubscriber("/odom","DataFormat","struct");
robot_cmdVelPub=rospublisher("/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
LaserScan=rossubscriber("/scan","DataFormat","struct"); 
pozyx_data=rossubscriber("/localization_data_topic","DataFormat","struct");
pause(2)
roll=0;
pitch=0;
yaw=0;
 Heading=0;

%    OdomData=robot_odom.LatestMessage;
%    [~,Orients,~,~]=robot2PoseCallback(OdomData);
%    Heading=Quaternions2EulerAngles(Orients);
%    Heading=atand(Heading);
% % theth=0;
prevD=0;
DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];
Gate1=[2 -6];
Gate2=[7 -4];
Gate3=[6 6];
Gate4=[-3.05 5];
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
for i=1:2
 if i==1
       DockStaCord=path(1,:);
        kp=0.5;
    %Set robot heading if necessary.
    while(1)
     OdomData=robot_odom.LatestMessage;
     [~,Orients,~,~]=robot2PoseCallback(OdomData);
     ChangHeading_angVel=SetRobotHeading(Heading,Orients);
     SetHeading_msg=rosmessage(robot_cmdVelPub);
     SetHeading_msg.Angular.Z= ChangHeading_angVel;
     send(robot_cmdVelPub,SetHeading_msg);
     ChangHeading_angVel
     if( ChangHeading_angVel<=0.00036)
       SetHeading_msg.Angular.Z=0;
       send(robot_cmdVelPub,SetHeading_msg);
        break
     end
   end


   

%cmdvel_msg.Angular.Z=targt*pi/180;%change robot heading frist.
%Adjust robot heading toward next destination.
  while(1)
  recievedData=robot_odom.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData);
  %[roll,pitch,yaw]=Quaternions2EulerAngles(Orients);
  cmdvel_msg=rosmessage(robot_cmdVelPub);
  [distD,Lin_vel,target,dockOrient]=DockwithoutLidar(DockStaCord,Poses,Orients);
  [rot_Speed]=SetRobotHeading(dockOrient,Orients);
  %%rot_Speed=kp*(target_rad);
  cmdvel_msg.Angular.Z=rot_Speed;
  send(robot_cmdVelPub,cmdvel_msg);
  %pause(2)
  target_rad=target*pi/180;
  %yaw
  rot_Speed
  %% moving to the docking station
  if (abs(rot_Speed)<=0.000625)
    cmdvel_msg.Angular.Z=0.00;
  %cmdvel_msg.Linear.X=Lin_vel;
  % send(robot2_cmdVelPub,cmdvel_msg);
  % Lin_vel
  % distD
  break
 end

  end
  elseif i==2
        DockStaCord=path(2,:);
end
%move to the destination.
% [~,~,~,DockOreint]=DockwithoutLidar(DockStation_pose,robot_pose,robot_orient)
if DockStaCord==Gate1
    theth=-120;
elseif DockStaCord==Gate2
    theth=60;
elseif DockStaCord==Gate3
    theth=120;  
elseif DockStaCord==Gate4
    theth=-120;
elseif DockStaCord==DockSta1Cord
    theth=-180;
elseif DockStaCord==DockSta2Cord
    theth=-90;
elseif DockStaCord==DockSta3Cord
    theth=0;
elseif DockStaCord==DockSta4Cord
    theth=90;
end
if (i==1)
while(1)
 [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,theth,DockStaCord);
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
coo
DockStaCord
% if certain distance reached, stop and orient the agv in parallel to the station
 if(distD<=0.4) 
 while(1)
   recievedData=robot_odom.LatestMessage;
   Scanmsg=LaserScan.LatestMessage;
   pozyxmsg=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData);
  [Orient]=SetRobotHeading(theth,Orients);
  cmdvel_msg.Linear.X=0.00;
  cmdvel_msg.Angular.Z=Orient;
  send(robot_cmdVelPub,cmdvel_msg); 
  Orient
   if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg); 
     if(DockStaCord==Gate1)
%           if (i==1)
         DockStaCord=path(2,:);
%           else
%               break;
%           end
     [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     elseif(DockStaCord==Gate2)
%          if (i==1)
         DockStaCord=path(2,:);
%           else
%               break;
%          end
         [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     elseif (DockStaCord==Gate3)
%          if (i==1)
          DockStaCord=path(2,:);
%           else
%               break;
%          end
         [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     elseif (DockStaCord==Gate4)
%         if (i==1)
         DockStaCord=path(2,:);
%           else
%               break;
%          end
         [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     end
     break
  end
end
break
 end %for if(distD<=0.4) && (i==1)
%  elseif(distD<=0.4) && (i==2)
 Lin_vel
 distD
end
elseif(i==2)
    %docking maneuver starts here!.
    %Check id docking station is detectedat this point!
 if (dockTo==1 && stat1DetectAccuracy>0.5)||(dockTo==2 && stat2DetectAccuracy>0.5)||(dockTo==3 && stat3DetectAccuracy>0.5)||(dockTo==4 && stat4DetectAccuracy>0.5)
 while(1)
 [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,theth,DockStaCord);
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
coo
DockStaCord
% if certain distance reached, stop and orient the agv in parallel to the station
 if(distD<=0.4) %% USE DISTANCE SENSORS RESULT TO EVALUATE THIS PORTION.
 while(1)
   recievedData=robot_odom.LatestMessage;
   Scanmsg=LaserScan.LatestMessage;
   pozyxmsg=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData);
  [Orient]=SetRobotHeading(theth,Orients);
  cmdvel_msg.Linear.X=0.00;
  cmdvel_msg.Angular.Z=Orient;
  send(robot_cmdVelPub,cmdvel_msg); 
  Orient
   if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg);  
   end
 end
 end
 end
 %The above portion of scripts is for docking maneuver
 elseif (dockTo==1 && stat1DetectAccuracy<0.5)||(dockTo==2 && stat2DetectAccuracy<0.5)||(dockTo==3 && stat3DetectAccuracy<0.5)||(dockTo==4 && stat4DetectAccuracy<0.5)
      oldHeading= dockOrient;
     while(1) % search until docking station is detected
        %calculate current agv heading
     if dockTo==1
        newHeading=oldHeading-30;
    elseif dockTo==2
        newHeading=oldHeading+30;
    elseif dockTo==3
        newHeading=oldHeading+30;
    elseif dockTo==4
        newHeading=oldHeading-30;
     end
     while(1)
      recievedData=robot_odom.LatestMessage;
      [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData);
      [Orient]=SetRobotHeading(newHeading,Orients);
      cmdvel_msg.Linear.X=0.00;
      cmdvel_msg.Angular.Z=Orient;
      send(robot_cmdVelPub,cmdvel_msg); 
      Orient
   if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg); 
     break;
   end
    end
     [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     if (dockTo==1 && stat1DetectAccuracy>0.5)||(dockTo==2 && stat2DetectAccuracy>0.5)||(dockTo==3 && stat3DetectAccuracy>0.5)||(dockTo==4 && stat4DetectAccuracy>0.5)
     
         [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,theth,DockStaCord);
         cmdvel_msg=rosmessage(robot_cmdVelPub);
         cmdvel_msg.Linear.X=Lin_vel;
         send(robot_cmdVelPub,cmdvel_msg);
         coo
         DockStaCord
         % if certain distance reached, stop and orient the agv in parallel to the station
         if(distD<=0.4) %% USE DISTANCE SENSORS RESULT TO EVALUATE THIS PORTION.
         while(1)
         recievedData=robot_odom.LatestMessage;
         Scanmsg=LaserScan.LatestMessage;
         pozyxmsg=pozyx_data.LatestMessage;
         [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData);
         [Orient]=SetRobotHeading(theth,Orients);
         cmdvel_msg.Linear.X=0.00;
         cmdvel_msg.Angular.Z=Orient;
         send(robot_cmdVelPub,cmdvel_msg); 
         Orient
        if(abs(parallelOrient)<=0.0008)
        cmdvel_msg.Angular.Z=0.00;
        send(robot_cmdVelPub,cmdvel_msg);  
        end
        end
         end
         break;
     end
      oldHeading=newHeading;
     end
    
 end
% this is the end of for loop
end
end
%% This part is functional with new virtual gates declared
rosshutdown;
clear all;
clc;
pause(1)
rosinit;
robot_odom= rossubscriber("/odom","DataFormat","struct");
robot_cmdVelPub=rospublisher("/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
LaserScan=rossubscriber("/scan","DataFormat","struct"); 
pozyx_data=rossubscriber("/localization_data_topic","DataFormat","struct");
pause(2)
roll=0;
pitch=0;
yaw=0;
Heading=0;

%    OdomData=robot_odom.LatestMessage;
%    [~,Orients,~,~]=robot2PoseCallback(OdomData);
%    Heading=Quaternions2EulerAngles(Orients);
%    Heading=atand(Heading);
%    theth=0;
DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];
Gate1=[2.17 -7.777];
Gate2=[8.046 0.221];
Gate3=[2.18 7.28];
Gate4=[-3.63 0.93];
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
for i=1:2
 if i==1
       DockStaCord=path(1,:);
        kp=0.5;
%     %Set robot heading if necessary.
%     while(1)
%      OdomData=robot_odom.LatestMessage;
%      [~,Orients,~,~]=robot2PoseCallback(OdomData);
%      ChangHeading_angVel=SetRobotHeading(Heading,Orients);
%      SetHeading_msg=rosmessage(robot_cmdVelPub);
%      SetHeading_msg.Angular.Z= ChangHeading_angVel;
%      send(robot_cmdVelPub,SetHeading_msg);
%      ChangHeading_angVel
%      if( ChangHeading_angVel<=0.00036)
%        SetHeading_msg.Angular.Z=0;
%        send(robot_cmdVelPub,SetHeading_msg);
%         break
%      end
%    end


   

%cmdvel_msg.Angular.Z=targt*pi/180;%change robot heading frist.
%Adjust robot heading toward next destination.
  while(1)
  recievedData=robot_odom.LatestMessage;
  scanmsg=LaserScan.LatestMessage;
  Pozyx_pose=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,Pozyx_pose);
  %[roll,pitch,yaw]=Quaternions2EulerAngles(Orients);
  cmdvel_msg=rosmessage(robot_cmdVelPub);
  [distD,Lin_vel,target,dockOrient]=DockwithoutLidar(DockStaCord,Poses,Orients);
  [rot_Speed]=SetRobotHeading(dockOrient,Orients);
  %%rot_Speed=kp*(target_rad);
  cmdvel_msg.Angular.Z=rot_Speed;
  send(robot_cmdVelPub,cmdvel_msg);
  %pause(2)
  target_rad=target*pi/180;
  %yaw
  rot_Speed
  %% moving to the docking station
  if (abs(rot_Speed)<=0.000625)
    cmdvel_msg.Angular.Z=0.00;
  %cmdvel_msg.Linear.X=Lin_vel;
  % send(robot2_cmdVelPub,cmdvel_msg);
  % Lin_vel
  % distD
  break
 end

  end
  elseif i==2
        DockStaCord=path(2,:);
end
%move to the destination.
% [~,~,~,DockOreint]=DockwithoutLidar(DockStation_pose,robot_pose,robot_orient)
if DockStaCord==Gate1
    theth=180;
elseif DockStaCord==Gate2
    theth=-90;
elseif DockStaCord==Gate3
    theth=0;  
elseif DockStaCord==Gate4
    theth=90;
    % cancel this and adjust the angles using distance sensors data.
elseif DockStaCord==DockSta1Cord
    theth=-180;
elseif DockStaCord==DockSta2Cord
    theth=-90;
elseif DockStaCord==DockSta3Cord
    theth=0;
elseif DockStaCord==DockSta4Cord
    theth=90;
end
if (i==1)
while(1)
 [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,pozyx_data,theth,DockStaCord);
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
coo
DockStaCord
% if certain distance reached, stop and orient the agv in parallel to the station

 if(distD<=0.4) 
 while(1)
   recievedData=robot_odom.LatestMessage;
   Scanmsg=LaserScan.LatestMessage;
   pozyx_pose=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,pozyx_pose);
  [Orient]=SetRobotHeading(theth,Orients);
  cmdvel_msg.Linear.X=0.00;
  cmdvel_msg.Angular.Z=Orient;
  send(robot_cmdVelPub,cmdvel_msg); 
  Orient
   if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg); 
     if(DockStaCord==Gate1)
%           if (i==1)
         DockStaCord=path(2,:);
%           else
%               break;
%           end
     [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     elseif(DockStaCord==Gate2)
%          if (i==1)
         DockStaCord=path(2,:);
%           else
%               break;
%          end
         [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     elseif (DockStaCord==Gate3)
%          if (i==1)
          DockStaCord=path(2,:);
%           else
%               break;
%          end
         [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     elseif (DockStaCord==Gate4)
%         if (i==1)
         DockStaCord=path(2,:);
%           else
%               break;
%          end
         [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     end
     break
  end
end
break
 end %for if(distD<=0.4) && (i==1)
%  elseif(distD<=0.4) && (i==2)
 Lin_vel
 distD
end
elseif(i==2)
    %docking maneuver starts here!.
    %Check id docking station is detectedat this point!
 if (dockTo==1 && stat1DetectAccuracy>0.5)||(dockTo==2 && stat2DetectAccuracy>0.5)||(dockTo==3 && stat3DetectAccuracy>0.5)||(dockTo==4 && stat4DetectAccuracy>0.5)
 while(1)
 [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,pozyx_data,theth,DockStaCord);
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
coo
DockStaCord
% if certain distance reached, stop and orient the agv in parallel to the station
 if(distD<=0.4) %% USE DISTANCE SENSORS RESULT TO EVALUATE THIS PORTION.
 while(1)
   recievedData=robot_odom.LatestMessage;
   Scanmsg=LaserScan.LatestMessage;
   pozyx_pose=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,pozyx_pose);
  [Orient]=SetRobotHeading(theth,Orients);
  cmdvel_msg.Linear.X=0.00;
  cmdvel_msg.Angular.Z=Orient;
  send(robot_cmdVelPub,cmdvel_msg); 
  Orient
   if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg);  
   end
 end
 end
 end
 %The above portion of scripts is for docking maneuver
 elseif (dockTo==1 && stat1DetectAccuracy<0.5)||(dockTo==2 && stat2DetectAccuracy<0.5)||(dockTo==3 && stat3DetectAccuracy<0.5)||(dockTo==4 && stat4DetectAccuracy<0.5)
      oldHeading= dockOrient;
     while(1) % search until docking station is detected
        %calculate current agv heading
     if dockTo==1
        newHeading=oldHeading-30;
    elseif dockTo==2
        newHeading=oldHeading+30;
    elseif dockTo==3
        newHeading=oldHeading+30;
    elseif dockTo==4
        newHeading=oldHeading-30;
     end
     while(1)
      recievedData=robot_odom.LatestMessage;
      pozyx_pose=pozyx_data.LatestMessage;
      [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,pozyx_pose);
      [Orient]=SetRobotHeading(newHeading,Orients);
      cmdvel_msg.Linear.X=0.00;
      cmdvel_msg.Angular.Z=Orient;
      send(robot_cmdVelPub,cmdvel_msg); 
      Orient
   if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg); 
     break;
   end
    end
     [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     if (dockTo==1 && stat1DetectAccuracy>0.5)||(dockTo==2 && stat2DetectAccuracy>0.5)||(dockTo==3 && stat3DetectAccuracy>0.5)||(dockTo==4 && stat4DetectAccuracy>0.5)
     
         [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,pozyx_data,theth,DockStaCord);
         cmdvel_msg=rosmessage(robot_cmdVelPub);
         cmdvel_msg.Linear.X=Lin_vel;
         send(robot_cmdVelPub,cmdvel_msg);
         coo
         DockStaCord
         % if certain distance reached, stop and orient the agv in parallel to the station
         if(distD<=0.4) %% USE DISTANCE SENSORS RESULT TO EVALUATE THIS PORTION.
         while(1)
         recievedData=robot_odom.LatestMessage;
         Scanmsg=LaserScan.LatestMessage;
         pozyx_pose=pozyx_data.LatestMessage;
         [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,pozyx_pose);
         [Orient]=SetRobotHeading(theth,Orients);
         cmdvel_msg.Linear.X=0.00;
         cmdvel_msg.Angular.Z=Orient;
         send(robot_cmdVelPub,cmdvel_msg); 
         Orient
        if(abs(parallelOrient)<=0.0008)
        cmdvel_msg.Angular.Z=0.00;
        send(robot_cmdVelPub,cmdvel_msg);  
        end
        end
         end
         break;
     end
      oldHeading=newHeading;
     end
    
 end
% this is the end of for loop
end
end

%% This part can move between the docking station ,but no detection
rosshutdown;
clear all;
clc;
pause(1)
rosinit;
robot_odom= rossubscriber("/odom","DataFormat","struct");
robot_cmdVelPub=rospublisher("/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
LaserScan=rossubscriber("/scan","DataFormat","struct"); 
pozyx_data=rossubscriber("/localization_data_topic","DataFormat","struct");
pause(2)
roll=0;
pitch=0;
yaw=0;
Heading=0;

%    OdomData=robot_odom.LatestMessage;
%    [~,Orients,~,~]=robot2PoseCallback(OdomData);
%    Heading=Quaternions2EulerAngles(Orients);
%    Heading=atand(Heading);
%    theth=0;
DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];
Gate1=[2.17 -7.777];
Gate2=[8.046 0.221];
Gate3=[2.18 7.28];
Gate4=[-3.63 0.93];
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
for i=1:2
 if i==1
       DockStaCord=path(1,:);
%         kp=0.5;
%     %Set robot heading if necessary.
%     while(1)
%      OdomData=robot_odom.LatestMessage;
%      [~,Orients,~,~]=robot2PoseCallback(OdomData);
%      ChangHeading_angVel=SetRobotHeading(Heading,Orients);
%      SetHeading_msg=rosmessage(robot_cmdVelPub);
%      SetHeading_msg.Angular.Z= ChangHeading_angVel;
%      send(robot_cmdVelPub,SetHeading_msg);
%      ChangHeading_angVel
%      if( ChangHeading_angVel<=0.00036)
%        SetHeading_msg.Angular.Z=0;
%        send(robot_cmdVelPub,SetHeading_msg);
%         break
%      end
%    end


   

%cmdvel_msg.Angular.Z=targt*pi/180;%change robot heading frist.
%Adjust robot heading toward next destination.
  while(1)
  recievedData=robot_odom.LatestMessage;
  scanmsg=LaserScan.LatestMessage;
  Pozyx_pose=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(recievedData,Pozyx_pose);
  %[roll,pitch,yaw]=Quaternions2EulerAngles(Orients);
  cmdvel_msg=rosmessage(robot_cmdVelPub);
  [distD,Lin_vel,target,dockOrient]=DockwithoutLidar(DockStaCord,pozyxposes,Orients);
  [rot_Speed]=SetRobotHeading(dockOrient,Orients);
  %%rot_Speed=kp*(target_rad);
  cmdvel_msg.Angular.Z=rot_Speed;
  send(robot_cmdVelPub,cmdvel_msg);
  %pause(2)
  target_rad=target*pi/180;
  %yaw
  rot_Speed
  %% moving to the docking station
  if (abs(rot_Speed)<=0.000625)
    cmdvel_msg.Angular.Z=0.00;
  %cmdvel_msg.Linear.X=Lin_vel;
  % send(robot2_cmdVelPub,cmdvel_msg);
  % Lin_vel
  % distD
  break
 end

  end
  elseif i==2
        DockStaCord=path(2,:);
end
%move to the destination.
% [~,~,~,DockOreint]=DockwithoutLidar(DockStation_pose,robot_pose,robot_orient)
if DockStaCord==Gate1
    theth=180;
elseif DockStaCord==Gate2
    theth=-90;
elseif DockStaCord==Gate3
    theth=0;  
elseif DockStaCord==Gate4
    theth=90;
    % cancel this and adjust the angles using distance sensors data.
elseif DockStaCord==DockSta1Cord
    theth=-180;
elseif DockStaCord==DockSta2Cord
    theth=-90;
elseif DockStaCord==DockSta3Cord
    theth=0;
elseif DockStaCord==DockSta4Cord
    theth=90;
end
if (i==1)
while(1)
 Pozyx_pose=pozyx_data.LatestMessage;
 [Poses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(recievedData,Pozyx_pose);
 [distD,Lin_vel,~,~]=MoveToGoalAndDock(robot_odom,pozyx_data,theth,path(2,:));
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
% if certain distance reached, stop and orient the agv in parallel to the station
dist2goal=distD;
dist2gate=norm(path(1,:)-pozyxposes);
% if dist2goal<=5
%   scanmsg=LaserScan.LatestMessage;
%   [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(scanmsg);   
% end
dist2goal
dist2gate

 if(dist2gate<=0.4) 
 while(1)
   recievedData=robot_odom.LatestMessage;
   Scanmsg=LaserScan.LatestMessage;
   pozyx_pose=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(recievedData,pozyx_pose);
  [Orient]=SetRobotHeading(theth,Orients);
  cmdvel_msg.Linear.X=0.00;
  cmdvel_msg.Angular.Z=Orient;
  send(robot_cmdVelPub,cmdvel_msg); 
  Orient
   if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg); 
   end
end
% break
 end %for if(distD<=0.4) && (i==1)
%  elseif(distD<=0.4) && (i==2)
 Lin_vel
 distD
 dist2gate

end
elseif(i==2)
    %docking maneuver starts here!.
    %Check id docking station is detectedat this point!
 if (dockTo==1 && stat1DetectAccuracy>0.5)||(dockTo==2 && stat2DetectAccuracy>0.5)||(dockTo==3 && stat3DetectAccuracy>0.5)||(dockTo==4 && stat4DetectAccuracy>0.5)
 while(1)
 [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,pozyx_data,theth,DockStaCord);
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
coo
DockStaCord
% if certain distance reached, stop and orient the agv in parallel to the station
 if(distD<=0.4) %% USE DISTANCE SENSORS RESULT TO EVALUATE THIS PORTION.
 while(1)
   recievedData=robot_odom.LatestMessage;
   Scanmsg=LaserScan.LatestMessage;
   pozyx_pose=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,pozyx_pose);
  [Orient]=SetRobotHeading(theth,Orients);
  cmdvel_msg.Linear.X=0.00;
  cmdvel_msg.Angular.Z=Orient;
  send(robot_cmdVelPub,cmdvel_msg); 
  Orient
   if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg);  
   end
 end
 end
 end
    
 end
% this is the end of for loop
end
end

%% Movement upto docking gate working properly

rosshutdown;
clear all;
clc;
pause(1)
rosinit;
robot_odom= rossubscriber("/odom","DataFormat","struct");
robot_cmdVelPub=rospublisher("/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
LaserScan=rossubscriber("/scan","DataFormat","struct"); 
pozyx_data=rossubscriber("/localization_data_topic","DataFormat","struct");
pause(2)
roll=0;
pitch=0;
yaw=0;
Heading=0;

%    OdomData=robot_odom.LatestMessage;
%    [~,Orients,~,~]=robot2PoseCallback(OdomData);
%    Heading=Quaternions2EulerAngles(Orients);
%    Heading=atand(Heading);
%    theth=0;
DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];
Gate1=[2.17 -7.777];
Gate2=[8.046 0.221];
Gate3=[2.18 7.28];
Gate4=[-3.63 0.93];
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
for i=1:2
 if i==1
     DockStaCord=path(1,:);
 elseif i==2
     DockStaCord=path(2,:);
%        docking with proximity here
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
    % cancel this and adjust the angles using distance sensors data.
elseif DockStaCord==DockSta1Cord
    theth=-180;
elseif DockStaCord==DockSta2Cord
    theth=-90;
elseif DockStaCord==DockSta3Cord
    theth=0;
elseif DockStaCord==DockSta4Cord
    theth=90;
end
if i==1
  while(1)
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
  if (abs(rot_Speed)<=0.000625)
    cmdvel_msg.Angular.Z=0.00;
    while(1)
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
 if(dist2gate<=0.4)
     while(1)
    recievedData=robot_odom.LatestMessage;
    Scanmsg=LaserScan.LatestMessage;
    pozyx_pose=pozyx_data.LatestMessage;
    [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,pozyx_pose);
    [Orient]=SetRobotHeading(theth,Orients);
    cmdvel_msg.Linear.X=0.00;
    cmdvel_msg.Angular.Z=Orient;
    send(robot_cmdVelPub,cmdvel_msg); 
%   [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(scanmsg); 
    Orient 
  if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg);  
       break;
  end
     end
     cmdvel_msg.Linear.X=0;
     send(robot_cmdVelPub,cmdvel_msg);
       break;
 end

    end 
 break;
  end%if rot_speed<0.000625
  end
 Lin_vel
 dist2goal
 dist2gate
elseif(i==2)
    %docking maneuver starts here!.
    %Check if docking station is detected at this point!
 if (dockTo==1 && stat1DetectAccuracy>0.5)||(dockTo==2 && stat2DetectAccuracy>0.5)||(dockTo==3 && stat3DetectAccuracy>0.5)||(dockTo==4 && stat4DetectAccuracy>0.5)
 while(1)
 [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,pozyx_data,theth,DockStaCord);
 cmdvel_msg=rosmessage(robot_cmdVelPub);
 cmdvel_msg.Linear.X=Lin_vel;
 send(robot_cmdVelPub,cmdvel_msg);
coo
DockStaCord
% if certain distance reached, stop and orient the agv in parallel to the station
 if(distD<=0.4) %% USE DISTANCE SENSORS RESULT TO EVALUATE THIS PORTION.
 while(1)
   recievedData=robot_odom.LatestMessage;
   Scanmsg=LaserScan.LatestMessage;
   pozyx_pose=pozyx_data.LatestMessage;
  [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData,pozyx_pose);
  [Orient]=SetRobotHeading(theth,Orients);
  cmdvel_msg.Linear.X=0.00;
  cmdvel_msg.Angular.Z=Orient;
  send(robot_cmdVelPub,cmdvel_msg); 
  Orient
   if(abs(Orient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg);  
   end
 end
 end
 end
    
 end
% this is the end of for loop
end
end


