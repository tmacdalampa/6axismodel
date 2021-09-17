function [ position ] = IK( arm, pose, current_position )
RAD2DEG = 180/pi;
r = pose(4)/RAD2DEG; p = pose(5)/RAD2DEG; yaw = pose(6)/RAD2DEG; %in rad
wrist_x = pose(1)-arm.d6*(cos(r)*cos(yaw)*sin(p)+sin(r)*sin(yaw));
wrist_y = pose(2)-arm.d6*(-cos(yaw)*sin(r)+cos(r)*sin(p)*sin(yaw));
wrist_z = pose(3)-arm.d6*cos(p)*cos(r);
t1 = atan2(wrist_y, wrist_x);
t1_tmp = atan2(-wrist_y, -wrist_x);
position(1) = ChooseNearst(t1,t1_tmp, current_position(1)/RAD2DEG);
x_dot = wrist_x * cos(position(1)) + wrist_y * sin(position(1));
y_dot = wrist_y * cos(position(1)) - wrist_x * sin(position(1));
z_dot = (wrist_z - arm.d1);
t3 = asin((x_dot*x_dot+z_dot*z_dot-arm.a3*arm.a3-arm.d4*arm.d4)/(2*arm.a3*arm.d4));
t3_tmp = pi-t3;
position(3) = ChooseNearst(t3,t3_tmp, current_position(3)/RAD2DEG);
f1 = arm.a3 + arm.d4*sin(position(3));
f2 = -arm.d4*cos(position(3));
if (f2 * f2 + f1 * f1 >= x_dot * x_dot)
    u = (-f2 * x_dot + f1 * sqrt(f1 * f1 + f2 * f2 - x_dot * x_dot)) / (f1 * f1 + f2 * f2);
    u_tmp = (-f2 * x_dot - f1 * sqrt(f1 * f1 + f2 * f2 - x_dot * x_dot)) / (f1 * f1 + f2 * f2);
        
    if (f2 == 0)
        position(2)=0;
    else
        t2 = atan2(u, (x_dot + f2 * u) / f1);%??
        t2_tmp = atan2(u_tmp, (x_dot + f2 * u_tmp) / f1);%??
        position(2) = ChooseNearst(t2,t2_tmp, current_position(2)/RAD2DEG);
    end
end
T06 = [cos(yaw)*cos(p), cos(yaw)*sin(p)*sin(r)-sin(yaw)*cos(r), cos(yaw)*sin(p)*cos(r)+sin(yaw)*sin(r), pose(1);
       sin(yaw)*cos(p), sin(yaw)*sin(p)*sin(r)+cos(yaw)*cos(r), sin(yaw)*sin(p)*cos(r)-cos(yaw)*sin(r), pose(2);
        -sin(p), cos(p)*sin(r), cos(p)*cos(r), pose(3);
       0,0,0,1];
inv_T03 = [(cos(position(1))*cos(position(2) + position(3))), (cos(position(2) + position(3))*sin(position(1))), (sin(position(2) + position(3))), (-arm.a3*cos(position(3)) -arm.d1*sin(position(2) + position(3)));
          (-cos(position(1))*sin(position(2) + position(3))), (-sin(position(1))*sin(position(2) + position(3))), (cos(position(2) + position(3))), (arm.a3*sin(position(3)) - arm.d1*cos(position(2) + position(3)));
          (sin(position(1))), (-cos(position(1))), 0, 0;
                0, 0, 0, 1];
T36 = inv_T03*T06;

if (T36(2, 3) ~= -1 && T36(2,3) ~= 1)
    t5 = atan2(sqrt(T36(2,1)^2+T36(2,2)^2), -T36(2,3));
    t5_tmp = atan2(-sqrt(T36(2,1)^2+T36(2,2)^2), -T36(2,3));
    position(5) = ChooseNearst(t5,t5_tmp, current_position(5)/RAD2DEG);
%     t4 = atan2(-T36(3,3)/sin(t5), -T36(1,3)/sin(t5));
%     t4_tmp = atan2(T36(3,3)/sin(t5), T36(1,3)/sin(t5));
%     position(4) = ChooseNearst(t4,t4_tmp, current_position(4)/RAD2DEG);
%     t6 = atan2(T36(2,2)/sin(t5), -T36(2,1)/sin(t5));
%     t6_tmp = atan2(-T36(2,2)/sin(t5), T36(2,1)/sin(t5));
%     position(6) = ChooseNearst(t6,t6_tmp, current_position(6)/RAD2DEG);
    position(4) = atan2(-T36(3,3)/sin(t5), -T36(1,3)/sin(t5));
    position(6) = atan2(T36(2,2)/sin(t5), -T36(2,1)/sin(t5));
else
    %singular point
    position(4) = 0.5 * atan2(T36(3, 1), T36(1, 1));
    position(5) = 0;
    position(6) = position(4);
end
for i=1:1:6
    position(i) = position(i)*RAD2DEG;
end
end