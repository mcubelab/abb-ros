grav = 9.8;
M = 10;
m = 1;
l = 0.1;
I = 1;
b = 1;


% t:{0,1}
x = @(t) l*cos(t);
dx = @(t) -l*sin(t);
ddx = @(t) -l*cos(t);
y = @(t) l*sin(t);
dy = @(t) l*cos(t);
ddy = @(t) -l*sin(t);
theta = @(t) (2*pi).*t;
dtheta = @(t) (2*pi);
ddtheta = @(t) 0;


f = @(t) (M+m)*ddx(t) - m*l*cos(theta(t))*ddtheta(t) + m*l*sin(theta(t))*(dtheta(t)^2); 
g = @(t) (M+m)*ddy(t) + m*l*sin(theta(t))*ddtheta(t) - m*l*cos(theta(t))*(dtheta(t)^2) - m*grav*cos(theta(t));
z = @(t) (m*l^2 + I)*ddtheta(t) + b*dtheta(t) - m*l*ddx(t)*cos(theta(t)) + m*l*ddy(t)*sin(theta(t)) + m*grav*sin(theta(t));
 



