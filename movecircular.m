function [  ] = movecircular( p1,p2,r0 )
arm.l1=0.425;
arm.l2=0.425;
arm.l3=0.0775;
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
vector0 = [1,0,0];
vector1=p1-r0;
vector2=p2-r0;
v = 0.05;%m/sec
a = 0.1;%m/sec^2
R=sqrt((p1(1)-r0(1))^2+(p2(2)-r0(2))^2);
CosTheta1 = max(min(dot(vector0,vector1)/(norm(vector0)*norm(vector1)),1),-1);
s1 = real(acos(CosTheta1));
CosTheta2 = max(min(dot(vector0,vector2)/(norm(vector0)*norm(vector2)),1),-1);
s2 = real(acos(CosTheta2));
Ta=v/a;
Tt = (R*(s1+s2)/v)+Ta;
x=p1(1); y=p1(2); s = s1; dt = 0.001; w = 0; i=1;
for t=0:dt:Tt
    if (t>=0 && t< Ta) %acc
        a_n = w*w*R;
        a_t = sqrt(a*a-a_n*a_n);
        alpha = a_t/R;
    elseif (t>=Tt-Ta && t<=Tt) %dcc
        alpha = 0;
        a_n = w*w*R;
        a_t = sqrt(a*a-a_n*a_n);
        alpha = -a_t/R;
    else
        alpha = 0;
    end
    w = w-alpha*dt;
    s = s+w*dt;
    Vx = -w*R*sin(s);
    Vy = w*R*cos(s);
    V = sqrt(Vx*Vx+Vy*Vy);
    V_m(i) = V;
    x=x + Vx*dt;
    y=y + Vy*dt;
    x_m(i) = x;
    y_m(i) = y;
    pos = [x,y,180];
    position = IK(arm, pos);
    position_m(i,:) = position;
    t=t+dt;
    t_m(i) = t;
    s_m(i) = s;
    w_m(i) = w;
    alpha_m(i) = alpha;
    a=sqrt(a_t*a_t+a_n*a_n);
    a_m(i) = a;
    i=i+1;
end
length = size(position_m);
for j=2:1:length(1)
    vel = (position_m(j,:) - position_m(j-1,:))/dt;
    vel_j_m(j,:) = vel;
end
vel_j_m(1,:) = [0,0,0];
figure(3)
subplot(2,2,1);plot(t_m, V_m,'LineWidth',5);xlim([0 Tt]);grid on;ylabel('time(sec)');title('cart space velocity');
subplot(2,2,2);plot(x_m, y_m,'LineWidth',5);xlim([0.1 0.4]);ylim([0.5 0.8]);grid on;title('trajectory');xlabel('x-position');ylabel('y-position');
subplot(2,2,3);plot(t_m, vel_j_m,'LineWidth',5);xlabel('time(sec)');ylabel('axis velocity');legend('axis1','axis2','axis3');grid on;title('axis velocity');xlim([0 Tt]);
subplot(2,2,4);plot(t_m, position_m,'LineWidth',5);xlabel('time(sec)');ylabel('axis position');legend('axis1','axis2','axis3');grid on;title('axis position');xlim([0 Tt]);
end

