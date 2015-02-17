%% -*- mode: octave; -*-
%% Octave script to process palm calibration collected by the palm_calibration node.

%% This script analyzes a palm calibration log file to compute the
%% actual location of the palm surface and check the relationship
%% between the programmed position and actual robot position. This might
%% help compensate for small errors in robot motion and force sensor
%% position calibration.

%% Turn off the annoying default pager.
page_screen_output(0);

if (nargin < 2) 
  traceFileName = "log";  
else
  args = argv();
  traceFileName = args(2);
endif

%% Load raw trace of experiment, potentially a big file.
printf("Opening trace file '%s'\n", traceFileName);
logdata = load( traceFileName ); 
logsamples   = size(logdata, 1);  % number of raw samples in the trace file
logtimestamp = logdata(:,1);      % time stamps, used to correlate trace file to calibration file
logFz        = logdata(:,5);      % measured normal force
logProbe     = logdata(:,10:12);  % programmed dx,dy,dz motions
logTCP       = logdata(:,21:23);  % reported TCP locations

%% Load the calibration data and break it out into separate variables.
load_calibration_data;

%% Load the most recent set of fitted parameters.
printf("Opening previous parameter file 'calibration.dat'\n");
load ("calibration.dat");

printf("Calibration records loaded.  Found %d probe points and %d raw trace records.\n", samples, logsamples);

%% In the first phase of calibration, the arm is moving slowly from a
%% reference pose to the force sensor, so the force sensor is not in
%% contact. Use this data to check the force sensor zero level.
newFzerr = mean( logFz(1:1000) );

printf("Computed Fz zero level of %f (previously %f)\n", newFzerr, Fzerr);
Fzerr = newFzerr;

%% For each calibration probe, find the index in the trace data, then find the actual touch point.
surface = zeros(samples, 3);  % points on surface in TCP coordinates
dxdydz  = zeros(samples, 3);  % probe points in ideal local coordinates
actual  = zeros(samples, 3);  % probe points in TCP coordinates

%% assume any normal force < -0.1 N is contact
Fzthreshold = Fzerr - 0.1; 

%% check this against the assumed-no-contact data
if (min(logFz(1:1000)) < Fzthreshold)
  printf("Error: Fzthreshold not low enough.\n");
endif

%% start looking after the initial data
logidx = 1000;

for sample = 1:samples
  %% printf("Locating event for probe %d at time %f.\n", sample, timestamp(sample));
  
  while (logtimestamp(logidx) < timestamp(sample))
    logidx += 1;
  endwhile
  %% printf("Found event at log index %d\n", logidx );
  
  %% Save the actual and programmed locations.
  dxdydz( sample, :) = logProbe( logidx, :);
  actual( sample, :) = logTCP  ( logidx, :);
  
  %% find the initial contact immediately before this probe
  offset = -1;
  while ( logFz( logidx + offset ) < Fzthreshold )
    offset -= 1;
  endwhile

  %% Save the recorded TCP locations for fitting a plane.
  surface(sample,:) = logTCP( logidx + offset, : );
endfor

%% Compute the relationship between ideal local coordinates and actual
%% robot position (TCP coordinates).
%% dxdydz = D x = D [actual 1]'    (ideal is 3 x samples, D is 3x4, x is 4 x samples)
TCPtoSensor = dxdydz' * pinv( [ actual ones(samples,1)]' );

printf("Homogeneous transform from TCP to force sensor coordinates:\n");
format short g;
disp(TCPtoSensor);

%% Transform the surface points from TCP coordinates into palm coordinates.
%% The XY values reflect the accuracy of the robot control, the Z value the 
%% actual height of the contact point.
palmsurf = (TCPtoSensor * [ surface ones(samples,1) ]' )';

%% Fit a plane to the surface points using the measured XYZ contacts:
%%  z' = A x = A [ x y 1 ]'  (A is 1x3, x is 3 x samples, z' is 1 x samples)
%%  A = z' * pinv(x)
%% plane = A
plane = palmsurf(:,3)' * pinv( [ palmsurf(:,1:2) ones(samples,1)]' );

%% Alternate fitting which uses the programmed XY and the measured Z:
Zerr = palmsurf(:,3)' * pinv( [ dxdydz(:,1:2) ones(samples,1)]' );

printf("Homogenous coefficients for Z error:\n");
disp(Zerr);

%% visualize the result
%% zg = plane * [ palmsurf(:,1:2) ones(samples,1)]';
%% plot3(palmsurf(:,1), palmsurf(:,2), palmsurf(:,3), "*", palmsurf(:,1), palmsurf(:,2), zg,"+");

save "palm_surface.dat" Fzerr TCPtoSensor Zerr;
