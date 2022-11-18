%% subscribing to the /localization_data _topic of pozyx system and /ground_truth/state of the AGV's real Location
rosshutdown;
rosinit;
a = animatedline;
b = animatedline;
e=animatedline;
c = a.Color;
d=b.Color;
f=e.Color;
a.Color = 'green';
a.LineWidth=1;
a.Marker='*';
b.Color='blue';
b.LineWidth=1;
b.Marker='*';
e.Color='red';
e.LineWidth=0.5;
e.Marker='*';
robot_truepose= rossubscriber("/ground_truth/state","DataFormat","struct");
pozyx_pose=rossubscriber("/localization_data_topic","DataFormat","struct");
error_data=[];
groundtruth_data=[];
pozyxdata=[];
count=1;
while(1)
    
    True_pose=receive(robot_truepose);
    pozyxpose=receive(pozyx_pose);
    err_x=True_pose.Pose.Pose.Position.X-pozyxpose.Point.X;
    err_y=True_pose.Pose.Pose.Position.Y-pozyxpose.Point.Y;
    error=[err_x,err_y];
    error_data(count,:)=error;
    groundtruth_data(count,:)=[True_pose.Pose.Pose.Position.X,True_pose.Pose.Pose.Position.Y];
    pozyxdata(count,:)=[pozyxpose.Point.X,pozyxpose.Point.Y];
    addpoints(a,True_pose.Pose.Pose.Position.X,True_pose.Pose.Pose.Position.Y);
    hold on;
    addpoints(b,pozyxpose.Point.X, pozyxpose.Point.Y);
    addpoints(e,err_x,err_y);
    drawnow limitrate;
    legend('True pose','Pozyx pose','Positioning error')
    xlabel('X-axis')
    ylabel('y-axis')
    title('Path covered by AGV moving at 0.5m/s')
    count=count+1;
end
drawnow;
%% Calculating displacement error between ground state truth and pozyx positioning system
displacement_error=[];
for i=1:length(error_data)
    displacement_error(i,:)=sqrt(error_data(i,1)^2+error_data(i,2)^2);
end
maximum_displacement_error=max(displacement_error);
minimum_displacement_error=min(displacement_error);
average_displacement_error=mean(displacement_error);
%% plot
plot( groundtruth_data(:,1), groundtruth_data(:,2),'go','MarkerSize',10,'LineWidth',2)
hold on
plot(pozyxdata(:,1),pozyxdata(:,2),'ko','MarkerSize',10,'LineWidth',2)
    
plot(error_data(:,1),error_data(:,2),'ro','MarkerSize',10,'LineWidth',2)
 xlabel('X-coordinates','FontSize',16)
 ylabel('y-coordinates','FontSize',16)
legend({'True pose','Pozyx pose','Positioning error'},'FontSize',14)
title('Path covered by the AGV moving at 0.5m/s','FontSize',18)
