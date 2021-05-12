function [ output_intervals ] = NegativeInterval( input_intervals )
%Returns the negative of the intervals passed.
%If passed avoid intervals, returns allowed intervals and vice-versa

N = length(input_intervals)/2; %number of input intervals
tol = 1e-4;

%number of output intervals (minimum = input - 1)
output_intervals = zeros(1,(N-1)*2);

%Start with inner intervals
for i=1:N-1
    output_intervals(i*2-1:i*2) = [input_intervals(i*2) input_intervals((i+1)*2-1)];
end
%Check whether the intervals contain [-pi and pi]
if(abs(input_intervals(1) - (-pi)) > tol) %does not contain -pi
    output_intervals = [-pi input_intervals(1) output_intervals];
end
if(abs(input_intervals(end) - pi) > tol) %contains pi
    output_intervals = [output_intervals input_intervals(end) pi];
end

end

