function [ times ] = TimeIntervals( filename, trial )
%TIMEFULLIK Summary of this function goes here
%   Detailed explanation goes here

%Robot parameters
%Link length
l = [0.34 0.4 0.4 0.126];
%Denavit-Hartenberg parameters 7 DoF
%DH: [a, alpha,    d, theta] 
dh = [0	 -pi/2	 l(1)    0;
      0	  pi/2      0    0;
      0	  pi/2   l(2)	 0;
      0  -pi/2      0    0;
      0  -pi/2   l(3)    0;
      0   pi/2      0    0;
      0      0   l(4)    0];

jl = torad([170 120 170 120 170 120 170]);
  
list = csvread(filename);

sz = length(list);
times = zeros(sz,1);
psi_ev = zeros(sz,2);
psi_ev(:,1) = (list(:,14));

str = sprintf('Wait Intervals being calculated for %d samples...', sz);
h = waitbar(0,str);

k = 0.5;
a2 = 25;


for i=1:length(list)
    pose = reshape(list(i,1:12),3,4); 
    pose(4,:) = [0 0 0 1]; 

    GC = (list(i,13));

    psi = (list(i,14));

    tic;

    %code 
    [j, s, w] = TimeIK_Int(l, dh, pose, GC, psi);
    allow = PsiLimits(GC, s, w, jl);
    %find which interval i am in
    for j=1:length(allow)/2
        inf = allow(j*2-1);
        sup = allow(j*2);
        if(psi>=inf && psi<=sup)
            next_psi = (k*((sup-inf)/2)) * (exp(-a2*((psi-inf)/(sup-inf))) - (exp(-a2*((sup-psi)/(sup-inf)))));
            break;
        end
    end

    t = toc;

    times(i) = t;
    psi_ev(i,2) = next_psi;

    dlmwrite(sprintf('time_all_%d.csv',trial), psi_ev, '-append' , 'delimiter', ',');
    waitbar(i / sz)
end


close(h);

end

