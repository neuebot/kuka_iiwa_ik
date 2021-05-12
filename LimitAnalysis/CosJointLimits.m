function [ lim ] = CosJointLimits( a, b, c, conf, jl )
%COSJOINTLIMITS 
%   Computes the stationary points of the arm angle for a cos joint

%Tolerance
tol = 1e-6;

% SINGULARITIES eq (39) and (40)
if(abs(a^2 + b^2 - (c-1)^2) < tol)
%     disp('Sing39');
    val = 2*atan2(a, (b-(c-1)));
end    
if(abs(a^2+b^2-(c+1)^2) < tol)
%     disp('Sing40');
    val = 2*atan2(a, (b-(c+1)));
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
pt1 = ArmAngleFunction(a, b, c, -jl, 2);
pt2 = ArmAngleFunction(a, b, c,  jl, 2);
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
        tlim = conf * acos(a*sin(ptlim(i)) + b*cos(ptlim(i)) + c);
        ct = a*sin(ptlim(i))+b*cos(ptlim(i))+c; %cos(theta_i) eq. (24) or cos(-theta_i)
        st = sqrt(1-ct^2);
        dlim = (-1/st)*(a*cos(ptlim(i))-b*sin(ptlim(i))); %cos(theta_i)' eq. (36) 
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
    tlim = conf * acos(a*sin(psi) + b*cos(psi) + c);
    if(tlim > -jl && tlim < jl)
        %All interval is possible
        lim = [-pi, pi];
    else
        %No interval possible
        lim = nan; 
    end
end
