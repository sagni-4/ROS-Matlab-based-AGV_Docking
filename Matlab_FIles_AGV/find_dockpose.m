
rosshutdown;
clear all;
clc;
rosinit;
DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];

Gate1=[2.17 -7.777];
Gate2=[8.046 0.221];
Gate3=[2.18 7.28];
Gate4=[-3.63 0.93];

robot_odom= rossubscriber("/odom","DataFormat","struct");
robot_cmdVelPub=rospublisher("/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
scanmsg=rossubscriber("/scan","DataFormat","struct"); 
pozyx_data=rossubscriber("/localization_data_topic","DataFormat","struct");
range= rossubscriber("/range/lasers","DataFormat","struct");
while(1)
   odom_data=robot_odom.LatestMessage;
   scan_data=scanmsg.LatestMessage;
%     Pozyx_pose=receive(pozyx_data);
   Pozyx_pose=pozyx_data.LatestMessage;
    cmdvel_msg=rosmessage(robot_cmdVelPub);
% for count=0:25
msg=receive(range);
header=msg.Header.FrameId;
switch header
    case 'link_1'
      front_Left_dist=msg.Ranges(1);
    case 'link_2'
      front_right_dist=msg.Ranges(1);
    case 'link_3'
      Left_side_front_dist=msg.Ranges(1);
    case 'link_4'
      Left_side_rear_dist=msg.Ranges(1);
otherwise
        disp('no sensor data')
end
% end
while(1)
odom_data=robot_odom.LatestMessage;
scan_data=scanmsg.LatestMessage;
 Pozyx_pose=pozyx_data.LatestMessage;
 cmdvel_msg=rosmessage(robot_cmdVelPub);
[odomPoses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(odom_data,Pozyx_pose);
pozyxposes_nav=[pozyxposes(1), pozyxposes(2), odomPoses(3)]; %adding heading to pozyx data
[distD,Lin_vel,target,dockOrient]=DockwithoutLidar(Gate1,pozyxposes,Orients);
[rot_Speed]=SetRobotHeading(dockOrient,Orients);
cmdvel_msg.Angular.Z=rot_Speed;  
cmdvel_msg.Linear.X=0.5;  
send(robot_cmdVelPub,cmdvel_msg);
% if (abs(rot_Speed)<=0.001)
%     cmdvel_msg.Angular.Z=0.00;
%   %cmdvel_msg.Linear.X=Lin_vel;
% %send(robot2_cmdVelPub,cmdvel_msg);
%   % Lin_vel
%   % distD
%   break
% end
% dist2goal=norm(DockSta1Cord-pozyxposes) 
% dist2gate=norm(Gate1-pozyxposes);
% if(dist2gate<0.3)
%     break;
% end
rot_Speed

dist2goal=norm(DockSta1Cord-pozyxposes) 
dist2gate=norm(Gate1-pozyxposes)
if dist2goal<5 && dist2gate>0.3
%     [stat1DetectAccuracy,stat2DetectAccuracy,stat3DetectAccuracy,stat4DetectAccuracy]=CNN_DockstationDetector(scan_data,DockSta1Cord);
    dist2goal=norm(DockSta1Cord-pozyxposes) 
    dist2gate=norm(Gate1-pozyxposes)
elseif dist2goal<5 && dist2gate<0.3
    cmdvel_msg.Linear.X=0.0; 
    send(robot_cmdVelPub,cmdvel_msg);
    break;
end
end
if dist2gate<0.3
        while(1)
            odom_data=robot_odom.LatestMessage;
            scan_data=scanmsg.LatestMessage;
            Pozyx_pose=pozyx_data.LatestMessage;
            cmdvel_msg=rosmessage(robot_cmdVelPub);
            [odomPoses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(odom_data,Pozyx_pose);
            pozyxposes_nav=[pozyxposes(1), pozyxposes(2), odomPoses(3)]; %adding heading to pozyx data
            [distD,Lin_vel,target,dockOrient]=DockwithoutLidar(DockSta1Cord,pozyxposes,Orients);
            [rot_Speed]=SetRobotHeading(dockOrient,Orients);
            cmdvel_msg.Angular.Z=rot_Speed;
            cmdvel_msg.Linear.X=0.1;  
            send(robot_cmdVelPub,cmdvel_msg);
%             if (abs(rot_Speed)<=0.000625)
%            cmdvel_msg.Angular.Z=0.00;
%             %cmdvel_msg.Linear.X=Lin_vel;
%             % send(robot2_cmdVelPub,cmdvel_msg);
%             % Lin_vel
%             % distD
%            break
%             end
dist2goal=norm(DockSta1Cord-pozyxposes) 
% if dist2goal<0.3
%     break;
% end

            rot_Speed
         end
 end
end
if (front_Left_dist<=0.25) || ( front_right_dist<=0.25)
    cmdvel_msg=rosmessage(robot_cmdVelPub);
    cmdvel_msg.Linear.X=0;
    send(robot_cmdVelPub,cmdvel_msg);
%     break;
end

