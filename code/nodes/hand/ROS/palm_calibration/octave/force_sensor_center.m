%% -*- mode: octave; -*-
%% Octave script to fit a circle to set of points sampled at fixed radius.

%% This is useful when manually calibrating the location of the
%% reference force sensor by touching the edge of the palm to the sensor
%% at several points.

%% The palm radius is nominally 50.8mm and the ruby sphere radius is
%% precisely 2.50mm so the computed radius should be about 50.8+2.5 =
%% 53.3.

%% pts = [105.9 747.5;  149.6 830.9;     63.6 832.4 ];

%% recorded at Z = 140mm
pts = [ 113.9 852.6 ; 151.4 828.4 ; 148.7 768.3 ; 115.4 748.3 ; 58.9 777.9 ; 60.3 825.1 ];

%% find (cx, cy, R) which minimizes (x - cx)^2 + (y - cy)^2 - R^2

function err = circlefit(params, pts)
  cx = params(1);
  cy = params(2);
   R = params(3);
   x = pts(:,1);
   y = pts(:,2);
  diff = (x - cx).^2 + (y - cy).^2 - R^2;
  err = sum(diff.^2);
  %% printf("error for (%f, %f), radius %f is %f\n", cx, cy, R, err);
endfunction

%% minimize an anonymous function used to pass additional parameters to the error function:
p = fminunc( @(params) circlefit(params, pts), [106 801 53.3] );

printf("circle center at (%f, %f), radius %f, with residual %f\n", p(1), p(2), p(3), circlefit(p, pts));

center = [p(1) p(2)];
radius = p(3);
save "force_sensor_location.dat" center radius
