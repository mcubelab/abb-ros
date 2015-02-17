%% -*- mode: octave; -*-
%% Octave script to process palm calibration collected by the palm_calibration node.
%% This common script reads in a calibration data file into a set of
%% variables and sets up some useful transformations.

%% current data format:
%% 1:t 2:force_t 3:fx 4:fy 5:fz 6:tx 7:ty 8:tz 9:motion_t 10:dx 11:dy 12:dz 13:palm_t 14:f1 15:f2 16:f3 17:f1s 18:f2s 19:f3s 20:arm_t 21:tcp.x 22:tcp.y 23:tcp.z 24:quat.0 25:quat.x 26:quat.y 27:quat.z 28:hand_t 29:q1 30:q2 31:q3 32:motor

%% check arguments for file name
if (nargin < 1) 
  calibrationFileName = "data";  
else
  args = argv();
  calibrationFileName = args(2);
endif

%% Load contact data.  Samples were collected at a grid of contact states.
printf("Opening calibration file '%s'\n", calibrationFileName );
data = load ( calibrationFileName );
samples = size(data,1);

%% Extract specific columns to use in fitting.
timestamp = data(:,1);    % timestamp of sample on data recording host
Fz    = data(:,5);        % force sensor vertical force (Fz)
probe = data(:, 10:12 );  % probe position relative to hand (dz, dy, dz)
raw   = data(:, 14:16 );  % raw force readings from hand

%% probe X and Y provide ground truth for contact location
%% probe Z can't really be trusted to the resolution we require
%% however, Fz provides the ground truth for normal force

%% Effective spring radius.
R = 36;

%% Calibration touch locations.
dx = probe(:,1);  
dy = probe(:,2);
dz = probe(:,3);

%% Raw sensor values.
S1 = raw(:,1);
S2 = raw(:,2);
S3 = raw(:,3);

%% Augment the raw data with a row of ones so that offsets can be computed as well as scale.
raw = [raw ones(samples,1)];
