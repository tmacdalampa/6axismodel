clc;
clear all;
arm.d1 = 0.426; arm.a3 = 0.425; arm.d4 = 0.425; arm.d6 = 0.0755;
current_position = [0,90,0,0,90,0];
position1 = [-20,35,40,55,70,120];
%pose1 = IK(arm, position1, current_position);
p2 = FK(arm, position1);

%p1 = FK(arm, current_position);
%p3 =[p2(1), p2(2), p2(3), 180, 0, 0];
%moveline(p1,p3);
