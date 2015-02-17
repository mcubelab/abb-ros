function [ output_args ] = basic( l )

t = linspace(0,10);

p = [a*t + b ; c*t + d];
dp = [a ; c];

s = @(t) [sqrt(l^2 - p(2,t).^2) + p(1,t) ; 0];
ds = @(t) [(-p(2,t).*dp(2,t))./sqrt(l^2 - p(2,t).^2) + dp(1,t); 0];


end

