function [ m ] = skew( v )
%SKEW Summary of this function goes here
%   Detailed explanation goes here

m=[0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0 ];

end

