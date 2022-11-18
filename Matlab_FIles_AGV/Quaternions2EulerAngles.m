function [r,p,y] = Quaternions2EulerAngles(q0123)
 eul = quat2eul(q0123);
 r=eul(1);
 p=eul(2);
 y=eul(3);