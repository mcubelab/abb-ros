%% -*- mode: octave; -*-
%% Octave script to analyze the gravity response map.

%% Turn off the annoying default pager.
page_screen_output(0);

%% More user-friendly numeric display format, minimize the exponents.
format short g;

%% Load the fitted parameters.
load "calibration.dat";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load the gravity data and break it out into separate variables.
calibrationFileName = "gravity.data";

%% Load gravity response data.  Samples were collected at a grid of orientations.
printf("Opening calibration file '%s'\n", calibrationFileName );
data = load ( calibrationFileName );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Extract specific columns to use in fitting.
timestamp = data(:,1);    % timestamp of sample on data recording host
raw   = data(:, 14:16 );  % raw force readings from hand
% raw   = data(:, 17:19 );  % raw force readings from hand with low-pass filtering
theta = data(:, 33);      % latitude (elevation) of the orientation of gravity vector
phi   = data(:, 34);      % longitude (azimuth) of the orientation of gravity vector

%% Augment the raw data with a row of ones so that offsets can be computed as well as scale.
samples = size(data,1);
raw = [raw ones(samples,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute the wrench on the palm due to gravity.
%% wrench = [Fz Tx Ty ; ...]
wrench = (C * raw')';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Remember, the hand was calibrated with the palm pointed straight
%% down. So the zero response is expected at theta = pi/2, with lower
%% values resulting in a negative Fz, i.e. gravity pulling the
%% palm into compression.
%% latitude (theta) ==  0    was defined as gravity pulling parallel to the palm
%% latitude (theta) ==  pi/2 was defined as gravity pulling the palm out (Fz > 0)
%% latitude (theta) == -pi/2 was defined as gravity pushing the palm in (Fz < 0)

%% longitude (phi)  == 0    was defined as gravity pulling along -Y.
%% longitude (phi)  == pi/2 was defined as gravity pulling along -X.

%% The gravity vector expressed in the palm frame with g > 0 is 
%%  g *  [ -cos(theta)*sin(phi)  -cos(theta)*cos(phi)     sin(theta)]

%% The torque on the palm resulting from gravity is cross( r, f ).

%% m*g has units of Newtons, mass location has units of millimeters
mass  = 0.167; % kg, measured using a scale
grav  = 9.81;  % m/sec^2, assumed

%% compute the gravity acceleration vector for each sample
st = sin(theta);
ct = cos(theta);
sp = sin(phi);
cp = cos(phi);
gravity = grav * [ -ct.*sp    -ct.*cp     st ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fit the mass using just the Fz data, since it has no dependence on
%% the center of mass location. The offset results from calibrating the
%% palm in the palm-down position, which adds an offset to the Fz
%% measurement.

function err = merr (params, theta, phi, wrench, grav)
  mass = params(1);
  model = mass * grav * (sin(theta) - 1.0);
  diff = wrench(:,1) - model;
  err = sum(diff .* diff);
endfunction

%% minimize an anonymous function used to pass additional parameters to the error function:
p = fminunc( @(params) merr(params, theta, phi, wrench, grav), [ mass ] );
mass = p(1)

%% The optimizer isn't necessary, again as a linear solution:
%% Fz = wrench(:,1) = mass * grav * (sin(theta) - 1.0);
mass2 = pinv(grav * (sin(theta)-1))*wrench(:,1)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fit the center of mass location using just the Tx and Tx data.  These
%% were also calibrated in the palm-down orientation, so the [Tx Ty] has
%% an offset of cross(com, m * g * [0 0 1]).

function err = comerr (com, theta, phi, wrench, gravity, grav, mass)
  samples = size(phi,1);
  offset = cross(com, mass * grav * [0 0 1]);

  %% model is the predicted torque
  model = cross( repmat(com,samples,1), mass * gravity) - repmat(offset,samples,1);

  %% compare the Tx and Tx components of the model vector with the calibrated sensor reading
  diff = wrench(:,2:3) - model(:,1:2);

  %% sum of squared error
  err = sum(sum(diff .* diff));
endfunction

%% minimize an anonymous function used to pass additional parameters to the error function:
printf("Optimizer solution for center of mass:\n");
com = fminunc( @(com) comerr(com, theta, phi, wrench, gravity, grav, mass), [ 0 0  5.5 ] )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Try it again as a linear problem.

%% Solve for the center of mass using all samples:
%% [tx ty] = (cross( com, mass * gravity ) - cross( com, mass * grav * [ 0 0 1] ))(:, 1:2)
%%   moment = mass * [ (comy*gravz-comz*gravy) (comz*gravx-comx*gravz) ];
%%   offset = mass * grav * [ comy  -comx];

%% [tx ty] = mom - off;
%%         = mass  * [ (comy*gravz-comz*gravy-g*comy) (comz*gravx-comx*gravz+g*comx) ];
%%         = mass  * [ (comy*(gravz-g)-comz*gravy) (comz*gravx-comx*(gravz-g)) ];

%% tx      = mass  * [ 0           (gravz-g)  -gravy ] * [comx comy comz ]';
%% ty      = mass  * [ -(gravz-g)  0           gravx ] * [comx comy comz ]';

%% stack tx and ty on the left hand side:
lhs = [ wrench(:, 2) ; wrench(:, 3) ];

%% set up the cross product matrix:
A = mass * [ zeros(samples,1)  (gravity(:,3)-grav)  -gravity(:,2) ;  -(gravity(:,3)-grav)  zeros(samples,1)  gravity(:,1) ];

%% now lhs = A * com';
%% solve for com:
printf("Linear solution for center of mass:\n");
com2 = (pinv(A) * lhs)'

%% getting the same result

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% write out a plot file for gnuplot

plotdata = [theta phi wrench]; save("-ascii", "plots/gravity_response","plotdata");

%% write out an idealized reference model
force = mass * grav * (sin(theta)-1);
offset = cross(com, mass * grav * [0 0 1]);
tau = cross( repmat(com,samples,1),  mass * gravity ) - repmat(offset,samples,1);
plotdata = [theta phi force tau(:,1:2)]; save("-ascii", "plots/gravity_model","plotdata");

%% save the computed gravity vectors (points on a sphere)
save ("-ascii","plots/gravity_vectors", "gravity")

%% select a cross-section of points along longitude zero
slice = (phi == 0);

plotdata = [theta(slice) wrench(slice,:)];
save("-ascii", "plots/gravity_response_y","plotdata");

plotdata = [theta(slice) force(slice) tau(slice,1:2)];
save("-ascii", "plots/gravity_model_y","plotdata");

%% select a cross-section of points along longitude -pi/2
slice = (abs(phi + pi/2) < 0.01);

plotdata = [theta(slice) wrench(slice,:)];
save("-ascii", "plots/gravity_response_x","plotdata");

plotdata = [theta(slice) force(slice) tau(slice,1:2)];
save("-ascii", "plots/gravity_model_x","plotdata");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% save gravity response calibration
save "gravity_response.dat" mass grav com

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% visualize
plot3( theta, phi, wrench(:,1), "-", theta, phi, force, "*");
title("Fz as function of orientation");
pause

plot3( theta, phi, wrench(:,2), "-", theta, phi, tau(:,1), "*");
title("Tx as function of orientation");
pause

plot3( theta, phi, wrench(:,3), "-", theta, phi, tau(:,2), "*");
title("Ty as function of orientation");
pause


