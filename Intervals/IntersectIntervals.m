function [ intersect_interval ] = IntersectIntervals( interval1, interval2 )
    %intervals are assumed to be 1xN    
    %A is the interval array: column 1 lower bounds, column 2 upper bounds
    intersect_interval = [];
    %remove NaNs
    interval1(isnan(interval1)) = [];
    interval2(isnan(interval2)) = [];
    
    aux = [interval1 interval2];
    n = int16(size(aux,2)/2); %number of intervals
    if(n>0)        
        A = zeros(n,2);
        
        A(:,1) = aux(1:2:end);
        A(:,2) = aux(2:2:end);

        [t,p] = sort(A(:));
        z = cumsum(accumarray((1:2*n)',2*(p<=n)-1));
        z1 = [0;z(1:end-1)];
        A2 = [t(z1==1 & z>1),t(z1>1 & z==1)];
        intersect_interval = sort(A2(:))';
    end
end
