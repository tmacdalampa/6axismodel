clc;
clear all;
%syms t1 t2 t3 t4 t5 t6;
%syms d1 a3 d4 d6;
d1 = 0.426; a3 = 0.425; d4 = 0.425; d6 = 0.0755;
%alpha a theta d 
%t1 = 0;t2=0.5*pi;t3=0.5*pi;t4=0;t5=0.5*pi;t6=0;
deg2rad=pi/180;
t1 = 10*deg2rad;t2=50*deg2rad;t3=40*deg2rad;t4=50*deg2rad;t5=70*deg2rad;t6=20*deg2rad;
% alpha a theta d
DH = [0,      0,   t1, d1;
     0.5*pi,  0,   t2, 0;
     0,       a3,  t3, 0;
     0.5*pi,  0,   t4, d4;
     0.5*pi,  0,   t5, 0;
     -0.5*pi, 0,   t6, d6];
 for i = 1:1:6
  T = [cos(DH(i,3)),             -sin(DH(i,3)),             0,             DH(i,2);
      sin(DH(i,3))*cos(DH(i,1)), cos(DH(i,3))*cos(DH(i,1)), -sin(DH(i,1)), -sin(DH(i,1))*DH(i, 4);
      sin(DH(i,3))*sin(DH(i,1)), cos(DH(i,3))*sin(DH(i,1)), cos(DH(i,1)),  cos(DH(i,1))*DH(i, 4);
      0,                         0,                         0,             1];   
     eval(['T',num2str(i-1),num2str(i),'=','T',';']);
 end
%T06 = T01*T12*T23*T34*T45*T56;
P34 = T34(:,4);
syms f1 f2; %T23*P34 = F
F =[f1;f2;0;1];
%f1 = a3 + d4*sin(t3);
%f2 = -d4*cos(t3);
%P04=G=T12*T23*P34 = T12*F=rotation center 
%G = T12*[f1;f2;0;1];
%syms wx wy wz
%inv(T01)*[wx;wy;wz;1] = G
