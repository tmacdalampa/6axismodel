clc;
clear all;
arm.l1=0.425;
arm.l2=0.425;
arm.l3=0.0775;
%pp1 = [45,45,90];
%pp2 = [25,75,78];
%p1 = FK(arm, pp1);
%p2 = FK(arm, pp2);
p1 = [0.225, 0.7255, 180];
p2 = [0.225, 0.6, 180];

r0 = 0.5*(p1+p2);
%position = IK(arm, p2);
%pose = FK(arm, pp1);
%moveP2P(p1, p2);
%moveline(p1,p2);
movecircular(p1,p2,r0);