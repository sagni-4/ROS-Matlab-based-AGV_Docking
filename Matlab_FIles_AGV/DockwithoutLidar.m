function[DistanceTodockStation,Vel,TargetAngle,DockOreint]=DockwithoutLidar(DockStation_pose,robot_pose,robot_orient)
kp=0.2;
time=3;
DistanceTodockStation=sqrt((DockStation_pose(1)-robot_pose(1))^2+(DockStation_pose(2)-robot_pose(2))^2);
Vel=kp*DistanceTodockStation/time;
if(DistanceTodockStation<=0.003)
Vel=0.00;
end
angle=atand((DockStation_pose(2)-robot_pose(2))/(DockStation_pose(1)-robot_pose(1)));
if (DockStation_pose(1) >robot_pose(1)) && (DockStation_pose(2)>robot_pose(1)) && angle>=0
    DockOreint=angle;
elseif (DockStation_pose(1) >robot_pose(1)) && (DockStation_pose(2)>robot_pose(2)) && angle<0
    DockOreint=angle+180;
elseif  (DockStation_pose(1) <robot_pose(1)) && (DockStation_pose(2)>robot_pose(2))&& angle<0
    DockOreint=angle+180;   
elseif  (DockStation_pose(1) <robot_pose(1)) && (DockStation_pose(2)>robot_pose(2))&& angle>=0
    DockOreint=angle; 
elseif  (DockStation_pose(1) <robot_pose(1)) && (DockStation_pose(2)<robot_pose(2))&& angle<=0
    DockOreint=angle;
elseif (DockStation_pose(1) <robot_pose(1)) && (DockStation_pose(2)<robot_pose(2))&& angle>=0
    DockOreint=angle-180;  
elseif  (DockStation_pose(1) >robot_pose(1)) && (DockStation_pose(2)<robot_pose(2))&& angle>0
    DockOreint=angle-180;  
 elseif  (DockStation_pose(1) >robot_pose(1)) && (DockStation_pose(2)<robot_pose(2))&& angle<=0
    DockOreint=angle;    
end
[~,~,y]=Quaternions2EulerAngles(robot_orient);
targAngle=atand(y);
if (targAngle>=DockOreint)
TargetAngle=kp*(targAngle-DockOreint);
elseif(targAngle<DockOreint)
TargetAngle=kp*(DockOreint-targAngle);
end
% if (DockStation_pose(1) >robot_pose(1)) && (DockStation_pose(2)>robot_pose(1)) && angle>=0
%     DockOreint=angle;
% elseif (DockStation_pose(1) >robot_pose(1)) && (DockStation_pose(2)>robot_pose(2)) && angle<0
%     DockOreint=angle+180;
% elseif  (DockStation_pose(1) <robot_pose(1)) && (DockStation_pose(2)>robot_pose(2))&& angle<0
%     DockOreint=angle+180;   
% elseif  (DockStation_pose(1) <robot_pose(1)) && (DockStation_pose(2)>robot_pose(2))&& angle>=0
%     DockOreint=angle; 
% elseif  (DockStation_pose(1) <robot_pose(1)) && (DockStation_pose(2)<robot_pose(2))&& angle<=0
%     DockOreint=angle;
% elseif (DockStation_pose(1) <robot_pose(1)) && (DockStation_pose(2)<robot_pose(2))&& angle>=0
%     DockOreint=angle-180;  
% elseif  (DockStation_pose(1) >robot_pose(1)) && (DockStation_pose(2)<robot_pose(2))&& angle>0
%     DockOreint=angle-180;  
%  elseif  (DockStation_pose(1) >robot_pose(1)) && (DockStation_pose(2)<robot_pose(2))&& angle<=0
%     DockOreint=angle;    
% end