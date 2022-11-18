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
Heading=-120;
theth=0;
prevD=0;
DockSta1Cord=[-0.0380 -8 0];
DockSta2Cord=[8.15 -1.5 0];
DockSta3Cord=[3.66 7.63 0];
DockSta4Cord=[-3.88 2.03 0];
%rosbag record -O p18dist_2m_head_neg120_orient_neg65_rot_neg125 /scan /ground_truth/states /localization_data_topic /odom


% kp=0.5;
while(1)
   OdomData=robot_odom.LatestMessage;
   [~,Orients,~,~]=robot2PoseCallback(OdomData);
   ChangHeading_angVel=SetRobotHeading(Heading,Orients);
   SetHeading_msg=rosmessage(robot_cmdVelPub);
   SetHeading_msg.Angular.Z= ChangHeading_angVel;
   send(robot_cmdVelPub,SetHeading_msg);
   ChangHeading_angVel
   if( abs(ChangHeading_angVel)<=0.00036)
       SetHeading_msg.Angular.Z=0;
       send(robot_cmdVelPub,SetHeading_msg);
        break
    end
end

%cmdvel_msg.Angular.Z=targt*pi/180;%change robot heading frist.

% while(1)
% recievedData=robot_odom.LatestMessage;
% [Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData);
% %[roll,pitch,yaw]=Quaternions2EulerAngles(Orients);
% cmdvel_msg=rosmessage(robot_cmdVelPub);
% [distD,Lin_vel,target,dockOrient]=DockwithoutLidar(DockSta3Cord,Poses,Orients);
% [rot_Speed]=SetRobotHeading(dockOrient,Orients);
% %%rot_Speed=kp*(target_rad);
% cmdvel_msg.Angular.Z=rot_Speed;
% send(robot_cmdVelPub,cmdvel_msg);
% %pause(2)
% target_rad=target*pi/180
% %yaw
% rot_Speed
% %% moving to the docking station
% if (abs(rot_Speed)<=0.000625)
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
%  [distD,Lin_vel,~,coo]=MoveToGoalAndDock(robot_odom,theth);
%  cmdvel_msg=rosmessage(robot_cmdVelPub);
%  cmdvel_msg.Linear.X=Lin_vel;
%  send(robot_cmdVelPub,cmdvel_msg);
% coo
% DockSta3Cord
%  if(distD<=0.6) 
% while(1)
%  [~,~,parallelOrient]=MoveToGoalAndDock(robot_odom,theth);
%  cmdvel_msg.Linear.X=0.00;
%  cmdvel_msg.Angular.Z=parallelOrient;
%  send(robot_cmdVelPub,cmdvel_msg); 
%  parallelOrient
%  if(abs(parallelOrient)<=0.00012)
%      cmdvel_msg.Angular.Z=0.00;
%      send(robot_cmdVelPub,cmdvel_msg); 
%      break
%  end
% end
% break
%  end
%  Lin_vel
%  distD
% end