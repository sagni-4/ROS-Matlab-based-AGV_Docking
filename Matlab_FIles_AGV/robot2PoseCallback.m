function [odomPoses,Orients,Twist_ang,Twist_lin,pozyxposes]=robot2PoseCallback(msg1,pozyxmsg)
Pose=msg1.Pose.Pose.Position;
Orient=msg1.Pose.Pose.Orientation;
TwistAng=msg1.Twist.Twist.Angular;
TwistLin=msg1.Twist.Twist.Linear;
odomPoses=[Pose.X,Pose.Y,Pose.Z];   

Orients=[Orient.X,Orient.Y,Orient.Z,Orient.W];
Twist_ang=[TwistAng.X,TwistAng.Y,TwistAng.Z];
Twist_lin=[TwistLin.X,TwistLin.Y,TwistLin.Z];
pozyxposes=[pozyxmsg.Point.X, pozyxmsg.Point.Y];
end