function [rotate_robot]=SetfinalHeading(thetha1,thetha2)
kp=0.7;
   target_rad=thetha1*pi/180;
   y=thetha2*pi/180;
%     [~,~,y]=Quaternions2EulerAngles(robot_orient);
    if (thetha1)<=180
    rotate_robot=kp*(target_rad-y);
    elseif(thetha1>180)
        
    rotate_robot=kp*(-target_rad+y);
    end