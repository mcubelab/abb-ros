global l;
global curr_s;

l = 10;
curr_s = [l;0];

% figure(1);
% hold on
% scatter(curr_s(1),curr_s(2));

start = [-0.1;0.1];
nmax = 10;
maxattempts = 30;
next = [0;0];
curr = start;
attempts = 1;
path = [0;0];

while (abs(next(2))<l) && (attempts<maxattempts)
%for (i=[1:2])
    n=1;
while (n<=nmax)
    
    [next,v] = interpolate(curr);
    
    if (sum(abs(next-curr))<0.001)
        break;
    else
        curr = next;
    end
    n = n+1;

end

attempts = attempts+1;
path = [path next];
curr_s = model(next);
% figure(1);
% scatter(curr_s(1),curr_s(2))
end

path;
figure;
scatter(path(1,:),path(2,:));
hold on
plot(path(1,:),path(2,:));