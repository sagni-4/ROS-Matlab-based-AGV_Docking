function [Poses,Orients,Twist_ang,Twist_lin,pozxyPose]=robotPoseCallback(msg1, msg2)
Pose=msg1.Pose.Pose.Position;
Orient=msg1.Pose.Pose.Orientation;
TwistAng=msg1.Twist.Twist.Angular;
TwistLin=msg1.Twist.Twist.Linear;

Poses=[Pose.X,Pose.Y,Pose.Z];
Orients=[Orient.X,Orient.Y,Orient.Z,Orient.W];
Twist_ang=[TwistAng.X,TwistAng.Y,TwistAng.Z];
Twist_lin=[TwistLin.X,TwistLin.Y,TwistLin.Z];
pozxyPose=[msg2.Point.X, msg2.Point.Y];
end