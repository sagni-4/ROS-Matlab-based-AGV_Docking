

clc;
rosshutdown;
clear all;
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
 i=0;
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
Gate1=[2.16 -6.10];
Gate2=[7 -4];
Gate3=[6 6];
Gate4=[-2.85 5.06];
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
elseif i==2
        DockStaCord=path(2,:);
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
  [parallelOrient]=SetRobotHeading(theth,Orients);
  cmdvel_msg.Linear.X=0.00;
  cmdvel_msg.Angular.Z=parallelOrient;
  send(robot_cmdVelPub,cmdvel_msg); 
  parallelOrient
   if(abs(parallelOrient)<=0.0008)
     cmdvel_msg.Angular.Z=0.00;
     send(robot_cmdVelPub,cmdvel_msg); 
     if(DockStaCord==Gate1)
          if (i==1)
         DockStaCord=path(2,:);
          else
              break;
          end
     [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     elseif(DockStaCord==Gate2)
         if (i==1)
         DockStaCord=path(2,:);
          else
              break;
         end
         [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     elseif (DockStaCord==Gate3)
         if (i==1)
          DockStaCord=path(2,:);
          else
              break;
         end
         [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     elseif (DockStaCord==Gate4)
        if (i==1)
         DockStaCord=path(2,:);
          else
              break;
         end
         [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(Scanmsg,DockStaCord); 
     end
     break
  end
end
break
 end
 Lin_vel
 distD
end
% this is the end of for loop
end