%% -*- mode: octave; -*-
%% Octave script to process palm calibration collected by the palm_calibration node.

%% Initialize calibration values.
Zerr = 0.0;

%% Initial guess at individual spring stiffness.
K = 7.78;

%% Load results from the trace file analysis.
load("palm_surface.dat");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load the calibration data and break it out into separate variables.
load_calibration_data;

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
%% This section fits a spring function to the measured (Fz, dx, dy, dz) data. 

%% We expect to see:
%%   Fz = k(x,y) * dz, where  k(x,y) = (3 * K * R^2) / (2 * x^2 + 2 * y^2 + R^2)
%%
%%   K is the stiffness of each individual spring element
%%   R is the radial location of the spring elements
%%   (x, y) is the contact point in the hand frame

%% Can we solve for K and R?
%% Rewriting:
%%
%%  Fz * (2 * x^2 + 2 * y^2 + R^2) == dz * (3 * K * R^2)
%%  2*Fz * (x^2 + y^2) + (Fz * R^2) - dz * (3 * K * R^2) == 0

%% Define an error function to minimize.  (This is quite inefficient, it
%% continually recomputes the element products of static vectors.)
function err = f (params, Fz, dx, dy, dz )
  K = params(1);
  R = params(2);
  %% zerr = params(2);
  %% R =  params(3);
  %% dz = dz + zerr;
  %% printf("K is %f, R is %f\n", K, R);
  diff = ((2*Fz) .* (dx.*dx + dy.*dy)) + (Fz * R^2) - (dz * 3 * K * R^2);
  err = sum(diff .* diff);
endfunction

%% minimize an anonymous function used to pass additional parameters to the error function:
p = fminunc( @(params) f(params, Fz, dx, dy, dz ), [K R] );

% extract the fit parameters (and print them)
K    = p(1)
R    = p(2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% This section computes the linear calibration matrix for processing
%% raw sensor signals into a contact wrench (Fz Tx Ty).

%% The simplest model is that the measured spring displacements are a
%% linear function of the applied force and torque.  If the springs and
%% sensors are linear this is exact.

%% The torque about the palm center applied by a single contact is
%% related to the contact location vector R and the normal force:

%% T = R x Fz

%% R and Fz are always perpendicular, so this is simply: 

%% Tx =  dy Fz 
%% Ty = -dx Fz

%% Just calibrate the one to the other, e.g., find C where:
%%    wrench' = C * raw'
%% (wrench' is 3xN, C is 3x4, raw' is 4xN )

%% This is a highly redundant problem, so use a pseudoinverse, which uses
%% a singular value decomposition with zero suppression to find a
%% least-squares solution:

C = wrench' * pinv( raw' )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% and save the full calibration result
save "calibration.dat" Fzerr Zerr K R C

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
