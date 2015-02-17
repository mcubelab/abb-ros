%% -*- mode: octave; -*-
%% Octave script to infer the virtual spring properties.

%% Turn off the annoying default pager.
page_screen_output(0);

%% More user-friendly numeric display format, minimize the exponents.
format short g;

%% load the fitted parameters
load "calibration.dat";

%% Load the calibration data and break it out into separate variables.
load_calibration_data;

%% The columns of C reflect the contribution of each sensor to the contact wrench.
%% The ratios of rows 2 and 3 to 1 reflect the virtual spring locations.
%%  Tx = dy .* Fz;
%%  Ty = -dx .* Fz;
%% so
%%  vx = -Ty / Fz
%%  vy =  Tx / Fz

vspos = [ (-C(2,1:3) ./ C(1,1:3))' (C(3,1:3) ./ C(1,1:3))' ]
vrad = sqrt( vspos(:,1).^2 + vspos(:,2).^2 )
vtheta = atan2( vspos(:,2), vspos(:,1) )


%% check the null input:
%% [0 0 0]' = C * [s1 s2 s3 1]'

N = null(C);
sensorNull = N ./ N(4)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% try an alternate calibration using just the deeper contacts

%% Correct the force calibration probe data by subtracting the offset error.
Fz = Fz - Fzerr;

%% Computed applied calibration torques Tx and Ty:
Tx =  dy .* Fz;
Ty = -dx .* Fz;

%% Assemble a set of force-torque reference wrenches [Fz Tx Ty]:
wrench = [ Fz Tx Ty ];

%% Adjust the commanded Z probe position using the calibration plane
%% fitted from the trace log data.  Zerr * [x y 1]' is the fit of the
%% location of the palm surface relative to the ideal model at each
%% point:
dzOffset = (Zerr * [ dx dy ones(samples,1) ]')';
dz -= dzOffset;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now delete the entries for Fz too small

small = Fz > -8;
wrench(small,:) = [];
raw(small,:) = [];
printf("After removing light touches, using %d samples.\n", size(raw,1) );

% and refit
Calt = wrench' * pinv( raw' );

vsposalt = [ (-Calt(2,1:3) ./ Calt(1,1:3))' (Calt(3,1:3) ./ Calt(1,1:3))' ]
vradalt = sqrt( vsposalt(:,1).^2 + vsposalt(:,2).^2 )
vthetaalt = atan2( vsposalt(:,2), vsposalt(:,1) )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot (vspos(:,1), vspos(:,2),"*", vsposalt(:,1), vsposalt(:,2),"+")
axis([-60 60 -60 60]);
title("Computed Virtual Spring Locations");
