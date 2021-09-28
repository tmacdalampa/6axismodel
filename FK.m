function [ pose ] = FK( arm, position )
RAD2DEG = 180/pi;
s1 = position(1)/RAD2DEG;
s2 = position(2)/RAD2DEG;
s3 = position(3)/RAD2DEG;
pose(1) = arm.l1*cos(s1)+arm.l2*cos(s1+s2)+arm.l3*cos(s1+s2+s3);
pose(2) = arm.l1*sin(s1)+arm.l2*sin(s1+s2)+arm.l3*sin(s1+s2+s3);
pose(3) = (s1+s2+s3)*RAD2DEG;
end

