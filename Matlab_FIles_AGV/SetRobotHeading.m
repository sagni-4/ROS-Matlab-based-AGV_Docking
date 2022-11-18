function [rotate_robot]=SetRobotHeading(thetha,robot_orient)
kp=0.5;
   target_rad=thetha*pi/180;
    [~,~,y]=Quaternions2EulerAngles(robot_orient);
    if (thetha)<=180
    rotate_robot=kp*(target_rad-y);
    elseif(thetha>180)
        
    rotate_robot=kp*(-target_rad+y);
    end
%     head=atand(y);
% if (head>=thetha)
% RotAngle=-(head-thetha);
% rotate_robot=kp*RotAngle*pi/180;
% elseif(head<thetha)
% RotAngle=(thetha-y);
% rotate_robot=kp*RotAngle*pi/180;
% end

%     function [rotate_robot]=SetRobotHeading(thetha,robot_orient)
%     kp=0.5;
%     target_rad=thetha*pi/180;
%     [~,~,y]=Quaternions2EulerAngles(robot_orient);
%     if (thetha)<=180
%     rotate_robot=kp*(target_rad-y);
%     elseif(thetha>180)
%         
%     rotate_robot=kp*(-target_rad+y);
%     end