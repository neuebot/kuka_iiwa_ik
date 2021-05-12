function [ T ] = dh_calc(a, alpha, d, theta )
%DH Summary of this function goes here
%   Detailed explanation goes here

T = [cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a*cos(theta);
    sin(theta),   cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a*sin(theta);
    0.0,          sin(alpha),               cos(alpha),              d;
    0.0,          0.0,                      0.0,                     1];

end

