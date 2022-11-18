
clear all;
rosshutdown;
rosinit;
pozyx_data=rossubscriber("/localization_data_topic","DataFormat","struct");
proxim1_data=rossubscriber("/proximty_frontLeft/range/lasers","DataFormat","struct");
proxim2_data=rossubscriber("/proximty_frontRight/range/lasers","DataFormat","struct");
proxim3_data=rossubscriber("/proximty_LsideFront/range/lasers","DataFormat","struct");
proxim4_data=rossubscriber("/proximty_LsideRear/range/lasers","DataFormat","struct");
robot_odom= rossubscriber("/odom","DataFormat","struct");
DockSta1Cord=[-0.0380 -8];
DockSta2Cord=[8.15 -1.5];
DockSta3Cord=[3.66 7.63];
DockSta4Cord=[-3.88 2.03];
D=DockSta2Cord;
recievedData=receive(robot_odom);
p=receive(pozyx_data);
[Poses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(recievedData,p);
[~,~,y]=Quaternions2EulerAngles(Orients);
if D==DockSta1Cord
    theth=-180;
elseif D==DockSta2Cord
    theth=-90;
elseif D==DockSta3Cord
    theth=0;
elseif D==DockSta4Cord
    theth=90;
end
head=y*180/pi;
head_error=head-theth;
dist_docking_error=sqrt((D(2)-pozyxposes(2))^2+(D(1)-pozyxposes(1))^2);
[FSL,FSR,LSF,LSR]=distance_sensor_ranges(proxim1_data,proxim2_data,proxim3_data,proxim4_data);
