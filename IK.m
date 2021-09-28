function [ position ] = IK( arm, pose )
RAD2DEG = 180/pi;
x=pose(1);y=pose(2);theta=pose(3)/RAD2DEG;
head = [x-arm.l3*cos(theta);
 y-arm.l3*sin(theta)];
s2 = acos((head(1)^2+head(2)^2-arm.l1^2-arm.l2^2)/(2*arm.l1*arm.l2));
A = [arm.l1+arm.l2*cos(s2), arm.l2*sin(s2);
      -arm.l2*sin(s2), arm.l1+arm.l2*cos(s2)];
theta1 = A*head/(head(1)^2+head(2)^2);
s1 = atan2(theta1(2),theta1(1));
position(1) = RAD2DEG*s1;
position(2) = RAD2DEG*s2;
position(3) = pose(3)-position(1)-position(2);
end