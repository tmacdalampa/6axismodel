function [ R ] = ABC2RT( ABC )
RAD2DEG = 180/pi;
roll = ABC(1)/RAD2DEG;
pitch = ABC(2)/RAD2DEG;
yaw = ABC(3)/RAD2DEG;
R = [cos(yaw) * cos(pitch), cos(yaw)* sin(pitch)* sin(roll) - sin(yaw) * cos(roll), cos(yaw)* sin(pitch)* cos(roll) + sin(yaw) * sin(roll);
        sin(yaw)* cos(pitch), sin(yaw)* sin(pitch)* sin(roll) + cos(yaw) * cos(roll), sin(yaw)* sin(pitch)* cos(roll) - cos(yaw) * sin(roll);
        -sin(pitch), cos(pitch)* sin(roll), cos(pitch)* cos(roll)];

end

