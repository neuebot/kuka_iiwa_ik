function [ allow_interval ] = PsiLimits( rconf, s_mat, w_mat, jl)
%TANGENTPSILIMIT
%   Detailed explanation goes here

[ s, e, w ] = Configuration(rconf);

As = s_mat(:,:,1);
Bs = s_mat(:,:,2);
Cs = s_mat(:,:,3);

Aw = w_mat(:,:,1);
Bw = w_mat(:,:,2);
Cw = w_mat(:,:,3);

%Joint 1
[lim1] = TanJointLimits(s*As(2,2), s*As(1,2), s*Bs(2,2), s*Bs(1,2), s*Cs(2,2), s*Cs(1,2), jl(1));
%Joint 2
[lim2] = CosJointLimits(As(3,2), Bs(3,2), Cs(3,2), s, jl(2));
%Joint 3
[lim3] = TanJointLimits(s*-As(3,3), s*-As(3,1), s*-Bs(3,3), s*-Bs(3,1), s*-Cs(3,3), s*-Cs(3,1), jl(3));

%Joint 5
[lim5] = TanJointLimits(w*Aw(2,3), w*Aw(1,3), w*Bw(2,3), w*Bw(1,3), w*Cw(2,3), w*Cw(1,3), jl(5));
%Joint 6
[lim6] = CosJointLimits(Aw(3,3), Bw(3,3), Cw(3,3), w, jl(6));
%Joint 7
[lim7] = TanJointLimits(w*Aw(3,2), w*-Aw(3,1), w*Bw(3,2), w*-Bw(3,1), w*Cw(3,2), w*-Cw(3,1), jl(7));

% Here we receive the limits for each of the shoulder and wrist joints.
lim12 = IntersectIntervals(lim1, lim2);
lim35 = IntersectIntervals(lim3, lim5);
lim67 = IntersectIntervals(lim6, lim7);
lim1235 = IntersectIntervals(lim12,lim35);
allow_interval = IntersectIntervals(lim1235,lim67);

end