function [] = moveP2P( p1, p2 )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
arm.l1=0.425;
arm.l2=0.425;
arm.l3=0.0775;
p1_j = IK(arm, p1);
p2_j = IK(arm, p2);
v_j = 10;% deg/sec
a_j = 20;%deg/sec^2
S = sqrt((p2_j(1)-p1_j(1))^2+((p2_j(2)-p1_j(2))^2+((p2_j(3)-p1_j(3))^2)));
v_joint = [v_j*(p2_j(1)-p1_j(1))/S,v_j*(p2_j(2)-p1_j(2))/S,v_j*(p2_j(3)-p1_j(3))/S];
a_joint = [a_j*(p2_j(1)-p1_j(1))/S,a_j*(p2_j(2)-p1_j(2))/S,a_j*(p2_j(3)-p1_j(3))/S];
Ta = v_j/a_j;
Tt = (S/v_j)+Ta;
i=1;dt=0.001; vel = [0,0,0]; theta = p1_j;
for t=0:dt:Tt
    if t>=0 && t< Ta %acc
        vel = vel + a_joint*dt;
    elseif t>Tt-Ta %dcc
        vel = vel - a_joint*dt;
    else %end
        vel = vel;
    end
    theta = theta + vel*dt;
    vel_m(i,:) = vel;
    theta_m(i,:) = theta;
    pose = FK(arm, theta);
    pose_m(i,:) = pose;
    t_m(i) = t;
    i=i+1;
end

for j=2:1:length(1)
    vel_c = (pose_m(j,:) - pose_m(j-1,:))/dt;
    vel_c_m(j,:) = vel_c;
end
vel_c_m(1,:) = [0,0,0];
figure(1)
subplot(2,2,2); plot(pose_m(:,1), pose_m(:,2),'LineWidth',5);xlim([0.1 0.4]);ylim([0.5 0.8]);xlabel('x-position');ylabel('y-position');grid on;title('xy trajectory');
subplot(2,2,3); plot(t_m, vel_m,'LineWidth',5);xlabel('time(sec)');ylabel('axis velocity');legend('axis1','axis2','axis3');grid on;title('axis velocity');xlim([0 Tt]);
subplot(2,2,4); plot(t_m, theta_m,'LineWidth',5);xlabel('time(sec)');ylabel('axis position');legend('axis1','axis2','axis3');grid on;title('axis position');xlim([0 Tt]);
end

