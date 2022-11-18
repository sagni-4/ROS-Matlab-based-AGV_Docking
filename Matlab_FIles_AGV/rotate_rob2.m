rosshutdown;
rosinit;
robot2_odom= rossubscriber("/robot2/odom","DataFormat","struct");
robot2_cmdVelPub=rospublisher("/robot2/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
pause(2)
roll=0;
pitch=0;
yaw=0;
targt=45;
SetRobotHeading(targt);
%cmdvel_msg.Angular.Z=targt*pi/180;%change robot heading frist.
kp=0.1;
while(1)
recievedData=robot2_odom.LatestMessage;
[Poses,Orients,Twist_ang,Twist_lin]=robot2PoseCallback(recievedData);
[roll,pitch,yaw]=Quaternions2EulerAngles(Orients);
cmdvel_msg=rosmessage(robot2_cmdVelPub);
DockStaCord=[-0.036778,5.18348,0];
[distD,Lin_vel,target]=DockwithoutLidar(DockStaCord,Poses,Orients);
target_rad=target*pi/180;
rot_Speed=kp*(target_rad-yaw);
cmdvel_msg.Angular.Z=rot_Speed;
send(robot2_cmdVelPub,cmdvel_msg);
pause(2)
target_rad
yaw
rot_Speed
%% moving to the docking station
if (abs(rot_Speed)<=0.000363)
    cmdvel_msg.Angular.Z=0.00;
    cmdvel_msg.Linear.X=Lin_vel;
send(robot2_cmdVelPub,cmdvel_msg);
Lin_vel
distD
end

end