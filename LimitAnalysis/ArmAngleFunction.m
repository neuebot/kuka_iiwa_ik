function [ out ] = ArmAngleFunction( a, b, c, theta, opt )
%ARMANGLEJOINTANGLE
% Solving equation 23 or 24 in order of psi
%
% If theta cannot be reached with any value of psi [-pi, pi]
% Returns nothing 
% Else
% Returns 1 or 2 solutions 

% If there is no solution for the psi(theta), means that no value of psi
% reaches the theta angle relative to the joint limits 
out = [];

% a = round(a,6);
% b = round(b,6);
% c = round(c,6);

% TAN function
% We need to work with atan2 because most of the time the theta we are
% working with are outside the 1st or 4th quadrant.
if(opt==1)
    v = tan(theta);
    as = v*(c(2)-b(2)) + (b(1)-c(1));
    bs = v*(2*a(2)) - (2*a(1));
    cs = v*(b(2)+c(2)) - (b(1)+c(1));

    if(bs^2 - 4*as*cs >= 0)
        sol(1) = 2*atan2(-(bs - (sqrt(bs^2 - 4*as*cs))),(2*as));
        sol(2) = 2*atan2(-(bs + (sqrt(bs^2 - 4*as*cs))),(2*as));

        %solution in the [-pi, pi range]
        sol = anglemod(sol);

        if(CheckSolution(a, b, c, theta, opt, sol(1)))
            out = [out anglemod(sol(1))];
        end
        if(CheckSolution(a, b, c, theta, opt, sol(2)))
            out = [out anglemod(sol(2))];
        end
    end
else
    v = cos(theta);
    as = v + b - c;
    bs = -2*a;
    cs = v - b - c;

    if(bs^2 - 4*as*cs >= 0)
        sol(1) = 2*atan2(-(bs - (sqrt(bs^2 - 4*as*cs))),(2*as));
        sol(2) = 2*atan2(-(bs + (sqrt(bs^2 - 4*as*cs))),(2*as));

        %solution in the [-pi, pi range]
        sol = anglemod(sol);

        if(CheckSolution(a, b, c, theta, opt, sol(1)))
            out = [out anglemod(sol(1))];
        end
        if(CheckSolution(a, b, c, theta, opt, sol(2)))
            out = [out anglemod(sol(2))];
        end
    end
end

end

function [correct] = CheckSolution(a, b, c, jl, opt, psi)
    tol = 1e-6;
    if(opt==1) % TAN
        theta = atan2((a(1)*sin(psi) + b(1)*cos(psi) + c(1)),(a(2)*sin(psi) + b(2)*cos(psi) + c(2)));
        correct = abs(jl-theta)<tol;
    else % COS
        theta = acos(a*sin(psi) + b*cos(psi) + c);
        correct = abs(jl-theta)<tol;
    end
end