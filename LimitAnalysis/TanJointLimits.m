function [ lim ] = TanJointLimits( an, ad, bn, bd, cn, cd, jl)
%ARMANGLETANJOINT
%   Computes the stationary points of the arm angle for a tan joint

sing_lim = [];
% joint limits mapped to psi values
ptlim = [];
% allowed interval
lim = [];

%Threshold to detect singularities
thr = 1e-6;

% The differentiation checks out
at = bd*cn - bn*cd;
bt = an*cd - ad*cn;
ct = an*bd - ad*bn;

% CTan = at^2 + bt^2 - ct^2;
% KL1 = 2*atan(at - sqrt(at^2 + bt^2 - ct^2)/(bt - ct));
% KL2 = 2*atan(at + sqrt(at^2 + bt^2 - ct^2)/(bt - ct));

%     WHAT ABOUT SINGULARITIES???
if(abs(at^2 + bt^2 - ct^2) < 1e-6)
    % Condition (31): one stationary point exists
    %
    % Both the numerator as the denominator are zero, meaning the arm angle
    % is at a singular point because, the angle theta_i is indeterminate at
    % this arm angle
    sing = anglemod(2*atan2(at,(bt - ct)));
    safe_distance = torad(7);
    sing_lim = [-pi sing-safe_distance sing+safe_distance pi];
end

% Stationary points not always map the theta max or min. Specially when we
% have configurations with negative shoulder, elbow or wrist values, we
% tend to have discontinuities in the function theta(psi) eq.23.
% These discontinuities are not contemplated in Shimizu paper, due to the
% values reported and the joint limits of his setup.
% These discontinuities are not singularities, but are related to the 
% domain of tan. Thus, when a monotonic function reaches -pi or pi a
% discontinuity happens, and the theta value shifts 2*pi or -2*pi.
% Since it crosses the joint limit values, they create a new interval where
% psi values lead to joint limit violation.
% 1) check whether the joint limits map to a psi value:
pt1 = ArmAngleFunction([an ad], [bn bd], [cn cd], -jl, 1);
pt2 = ArmAngleFunction([an ad], [bn bd], [cn cd], jl, 1);
ptlim = [pt1 pt2]; %every psi that crosses limits
%There is at least a psi value that matches a joint limit
if(~isempty(ptlim))
    % 2) Order array of psi at theta limits
    ptlim = sort(ptlim);
    %What if the limit case where it only touches the line ?
%   COMMENTED PERFORMANCE    
%     assert(rem(length(ptlim),2)==0, 'PTLIM should always have pair length'); 
    
    % 3) Classifies the limit points as enter_avoid = 1 or enter_allow = 0
    lim_class = zeros(size(ptlim)); 
    for i=1:length(ptlim) 
        %Theta value (either -jl or jl)
        tlim = atan2((an*sin(ptlim(i)) + bn*cos(ptlim(i)) + cn) , (ad*sin(ptlim(i)) + bd*cos(ptlim(i)) + cd));
        dlim = at*sin(ptlim(i)) + bt*cos(ptlim(i)) + ct;
        lim_class(i) = sign(tlim) == sign(dlim);
    end
    % 4) lim_class is either [0 1 0 1 ...] or [1 0 1 0 ...]
    % If it starts in an enter avoid, means that it starts in an allowed
    % interval, so we concatenate [-pi ptlim pi] so it always starts with
    % an enter allowed
    if(lim_class(1)==1) 
        ptlim = [-pi ptlim pi];
    end
    lim = ptlim;
else
    % If no psi values match the joint limits border: either no solutions
    % are allowed or all solutions are allowed.
    % So we just need to check the value of any point contained in the
    % function to know if the codomain is within or outside joint limits
    psi = 0;
    tlim = atan2((an*sin(psi) + bn*cos(psi) + cn) , (ad*sin(psi) + bd*cos(psi) + cd));
    if(tlim > -jl && tlim < jl)
        %All interval is possible
        lim = [-pi, pi];
    else
        %No interval possible
        lim = nan; 
    end
end

if(~isempty(sing_lim))
    lim = IntersectIntervals(lim,sing_lim);
end

end
