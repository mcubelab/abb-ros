function [ x,y ] = getWayPts( theta )
    
    t = linspace(0.01,2.5);  
    
% 	f = @(x) cos(x.*sin(x).*csc(x) - (1/2)*log(sin(x))).*(-cos(x)+sqrt(cos(x).^2 - (1-5*sin(x).^2)))./(2*sin(x));
% 	h = @(x) sin(x.*sin(x).*csc(x) - (1/2).*log(sin(x))).*(-cos(x)+sqrt(cos(x).^2 - (1-5.*sin(x).^2)))./(2.*sin(x));
%     x = f(t);
%     y = h(t);

    f = @(x) cos(x).^2.*(-sin(x).*cos(x)-sin(x));
    h = @(x) sin(x).^2.*(-sin(x).*cos(x)-sin(x));
    x = -f(t);
    y = h(t);
    size(x)
    size(y)
    plot(x,y,'b')
    
    xstr = '{';
    ystr = '{';
    for i=1:size(t,2)
        xstr = strcat(xstr,num2str(x(i)),',');
        ystr = strcat(ystr,num2str(y(i)),',');
    end
    xstr = strcat(xstr,'}');
    ystr = strcat(ystr,'}');
    
    
    xstr
    ystr
    
end

