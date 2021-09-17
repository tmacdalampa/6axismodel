function [] = moveline( p1,p2 )
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
arm.d1 = 0.426; arm.a3 = 0.425; arm.d4 = 0.425; arm.d6 = 0.0755;
position = [0, 90, 0, 0, 90, 0];
%position_init = IK(arm, p1, [0, 90, 0, 0, 90]);
v = 0.05;%m/sec
a = 0.1;%m/sec^2
S = sqrt((p2(1)-p1(1))^2+((p2(2)-p1(2))^2+((p2(3)-p1(3))^2)));
Theta = sqrt((p2(4)-p1(4))^2+((p2(5)-p1(5))^2+((p2(6)-p1(6))^2)));
u_t = [(p2(1)-p1(1))/S, (p2(2)-p1(2))/S, (p2(3)-p1(3))/S];
u_r = [(p2(4)-p1(4))/Theta, (p2(5)-p1(5))/Theta, (p2(6)-p1(6))/Theta];
Ta = v/a;
Tt = (S/v)+Ta;
w=Theta/(Tt-Ta);
alpha = w/Ta;
i=1;dt=0.001; vel = 0; pos = p1(1:3);ang_vel = 0;ang_pos = p1(4:6);
for t=0:dt:Tt
    if t>=0 && t< Ta %acc
        vel = vel + a*dt;
        ang_vel = ang_vel + alpha*dt;
    elseif t>Tt-Ta %dcc
        vel = vel - a*dt;
        ang_vel = ang_vel - alpha*dt;
    else %end
        vel = vel;
        ang_vel = ang_vel;
    end
    pos = pos + u_t*vel*dt;
    ang_pos = ang_pos + u_r*ang_vel*dt;
    vel_m(i) = vel;
    ang_vel_m(i) = ang_vel;
    pos_m(i,:) = pos;
    ang_pos_m(i,:) = ang_pos;
    pose(1:3) = pos;
    pose(4:6) = ang_pos;
    position_new = IK(arm, pose, position);
    position_m(i,:) = position_new;
    position = position_new;
    t_m(i) = t;
    i=i+1;
end
length = size(position_m);

for j=2:1:length(1)
    vel = (position_m(j,:) - position_m(j-1,:))/dt;
    vel_j_m(j,:) = vel;
end
vel_j_m(1,:) = [0,0,0,0,0,0];
figure(2) 

subplot(2,2,1); plot(t_m, 1000*vel_m,t_m, ang_vel_m,'LineWidth',5);xlabel('time(sec)');ylabel('velocity');grid on;title('cart space velocity');xlim([0 Tt]);legend('trans vel(mm/sec)', 'ang vel(deg/sec)');
subplot(2,2,2); plot3(pos_m(:,1), pos_m(:,2), pos_m(:,3),'LineWidth',5);xlabel('x-position');ylabel('y-position');zlabel('z-position');grid on;title('xy trajectory');
%subplot(2,2,3); plot(t_m, ang_pos_m,'LineWidth',5);xlabel('time');ylabel('ang pos(deg)');grid on;title('ang pos-time');legend('A','B','C');xlim([0 Tt]);
subplot(2,2,3); plot(t_m, position_m,'LineWidth',5);xlabel('time(sec)');ylabel('axis position');legend('axis1','axis2','axis3', 'axis4', 'axis5', 'axis6');grid on;title('axis position');xlim([0 Tt]);
subplot(2,2,4); plot(t_m, vel_j_m,'LineWidth',5);xlabel('time(sec)');ylabel('axis velocity');legend('axis1','axis2','axis3', 'axis4', 'axis5', 'axis6');grid on;title('axis velocity');xlim([0 Tt]);
end

