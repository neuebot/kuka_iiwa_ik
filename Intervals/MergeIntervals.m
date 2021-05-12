function [ merged_interval ] = MergeIntervals( interval1,interval2 )
    %intervals are assumed to be 1xN    
    %A is the interval array: column 1 lower bounds, column 2 upper bounds
    aux = [interval1 interval2];
    n = size(aux,2)/2; %number of intervals
    A = zeros(n,2);
    A(:,1) = aux(1:2:end);
    A(:,2) = aux(2:2:end);
    
    [t,p] = sort(A(:));
    z = cumsum(accumarray((1:2*n)',2*(p<=n)-1));
	z1 = [0;z(1:end-1)];
	A2 = [t(z1==0 & z>0),t(z1>0 & z==0)];
    merged_interval = sort(A2(:))';
end