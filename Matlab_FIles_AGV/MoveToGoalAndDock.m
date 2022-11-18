function [distD,Lin_vel,parallelOrient,Poses]=MoveToGoalAndDock(msg,pozyxmsg, thetha,DockStaCord)
kp=0.5;
dockPoseData=msg.LatestMessage;
pozyxpose=pozyxmsg.LatestMessage;
[Poses,Orients,~,~,pozyxposes]=robot2PoseCallback(dockPoseData,pozyxpose);
[distD,Lin_vel,~]=DockwithoutLidar(DockStaCord,pozyxposes,Orients);
[~,~,y]=Quaternions2EulerAngles(Orients);
targAngle=atand(y);
%if (targAngle>=thetha)
parallelOrient=kp*pi/180*(thetha-targAngle);
%elseif(targAngle<thetha)
%%end