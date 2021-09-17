function [ res ] = ChooseNearst( a,b,c )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    if (abs(a - c) <= abs(b - c))
        res = a;
    else
        res = b;
    end

end

