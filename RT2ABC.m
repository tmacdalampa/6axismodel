function [ ABC ] = RT2ABC( R )
RAD2DEG = 180/pi;
pitch = atan2(-R(3,1), sqrt(R(1,1)*R(1,1) + R(2,1)*R(2,1)));

if (cos(pitch) ~= 0)
    yaw = atan2(R(2,1)/cos(pitch), R(1,1)/cos(pitch));
    roll = atan2(R(3,2)/cos(pitch), R(3,3)/cos(pitch));
else
    yaw = 0;
    roll = 0;
end
    ABC(1) = roll*RAD2DEG;
    ABC(2) = pitch*RAD2DEG;
    ABC(3) = yaw*RAD2DEG;



end

