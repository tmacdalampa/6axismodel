function [ pose ] = FK( arm, position )
RAD2DEG = 180/pi;
DH = [0,      0,   position(1)/RAD2DEG, arm.d1;
     0.5*pi,  0,   position(2)/RAD2DEG, 0;
     0,       arm.a3,  position(3)/RAD2DEG, 0;
     0.5*pi,  0,   position(4)/RAD2DEG, arm.d4;
     0.5*pi,  0,   position(5)/RAD2DEG, 0;
     -0.5*pi, 0,   position(6)/RAD2DEG, arm.d6];
 for i = 1:1:6
  T = [cos(DH(i,3)),             -sin(DH(i,3)),             0,             DH(i,2);
      sin(DH(i,3))*cos(DH(i,1)), cos(DH(i,3))*cos(DH(i,1)), -sin(DH(i,1)), -sin(DH(i,1))*DH(i, 4);
      sin(DH(i,3))*sin(DH(i,1)), cos(DH(i,3))*sin(DH(i,1)), cos(DH(i,1)),  cos(DH(i,1))*DH(i, 4);
      0,                         0,                         0,             1];   
     eval(['T',num2str(i-1),num2str(i),'=','T',';']);
 end
 T06 = T01*T12*T23*T34*T45*T56;
 ABC = RT2ABC(T06(1:3,1:3));
 pose(1:3) = T06(1:3,4);
 pose(4:6) = ABC;
end

