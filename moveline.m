function [] = moveline( p1,p2 )
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
arm.l1=0.425;
arm.l2=0.425;
arm.l3=0.0775;

v = 0.05;%m/sec
a = 0.1;%m/sec^2
S = sqrt((p2(1)-p1(1))^2+((p2(2)-p1(2))^2+((p2(3)-p1(3))^2)));
u = [(p2(1)-p1(1))/S, (p2(2)-p1(2))/S, (p2(3)-p1(3))/S];
Ta = v/a;
Tt = (S/v)+Ta;
i=1;dt=0.001; vel = 0; pos = p1;
for t=0:dt:Tt
    if t>=0 && t< Ta %acc
        vel = vel + a*dt;
    elseif t>Tt-Ta %dcc
        vel = vel - a*dt;
    else %end
        vel = vel;
    end
    pos = pos + u*vel*dt;
    vel_m(i) = vel;
    pos_m(i,:) = pos;
    position = IK(arm, pos);
    position_m(i,:) = position;
    t_m(i) = t;
    i=i+1;
end
length = size(position_m);
%vel_j_m(1,:) = [0,0,0];
for j=2:1:length(1)
    vel = (position_m(j,:) - position_m(j-1,:))/dt;
    vel_j_m(j,:) = vel;
end
vel_j_m(1,:) = [0,0,0];
figure(2) 
subplot(2,2,1); plot(t_m, vel_m,'LineWidth',5);xlabel('time(sec)');ylabel('velocity(m/sec)');grid on;title('cart space velocity');xlim([0 Tt]);
subplot(2,2,2); plot(pos_m(:,1), pos_m(:,2),'LineWidth',5);xlim([0.1 0.4]);ylim([0.5 0.8]);xlabel('x-position');ylabel('y-position');grid on;title('xy trajectory');
subplot(2,2,3); plot(t_m, vel_j_m,'LineWidth',5);xlabel('time(sec)');ylabel('axis velocity');legend('axis1','axis2','axis3');grid on;title('axis velocity');xlim([0 Tt]);
subplot(2,2,4); plot(t_m, position_m,'LineWidth',5);xlabel('time(sec)');ylabel('axis position');legend('axis1','axis2','axis3');grid on;title('axis position');xlim([0 Tt]);
end

