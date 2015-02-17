%% -*- mode: octave; -*-
%% Octave script to plot palm calibration data collected by the palm_calibration node.

%% Octave is fine for plotting to the screen, but it isn't flexible
%% enough for creating PDF output, so this also writes out ASCII data
%% files as input for gnuplot.

%% Turn off the annoying default pager.
page_screen_output(0);

%% load the fitted parameters
load "calibration.dat";

%% load the recorded calibration data
load_calibration_data;

%% Normalize the force calibration probe data by subtracting the offset error.
Fz = Fz - Fzerr;

%% Computed applied calibration torques Tx and Ty:
Tx = dy .* Fz;
Ty = -dx .* Fz;

%% Assemble a set of force-torque reference wrenches [Fz Tx Ty]:
wrench = [ Fz Tx Ty ];

%% Adjust the commanded Z probe position using the calibration plane
%% fitted from the trace log data:
dzOffset = (Zerr * [ dx dy ones(samples,1) ]')';
dz -= dzOffset;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% To check overall fit, try reconstructing the reference data (Fz Tx Ty):.
fit = (C * raw')';

%% And computing the difference:
err = wrench - fit;

%% Reconstruct the contact locations from the fit.  
%%  Tx =  dy Fz, so dy = Tx / F
%%  Ty = -dx Fz, so dx = -Ty / F
j = 1;
clear fitxy;
for i = 1:samples
  if (fit(i,1) < -2.0)
    fitxy(j, 1) = -fit( i, 3) / fit(i, 1);  % dx = -Ty / Fz
    fitxy(j, 2) =  fit( i, 2) / fit(i, 1);  % dy =  Tx / Fz
    fitxy(j, 3:4)= probe(i,1:2);  % actual dx, dy
    j += 1;
  else
    printf("Warning: sample %d has predicted Fz of %f, not including in XY fit\n", i, fit(i,1));
  endif
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% predict the normal force data using the spring calibration
mdz = dz;
mFz = mdz .* ((3 * K * R^2) ./ ( 2 * dx.^2 + 2* dy.^2 + R**2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% first plot the contact locations
plot( dx, dy, "*;touch point;" )
xlabel("X location (mm)");
ylabel("Y location (mm)");
title("Calibration Probe Touch Locations");
axis([-50 50 -50 50], "equal");
%%axis("equal");  %% equal square normal
%%print"plots/calibration_locations.pdf");
plotdata = [dx dy]; save("-ascii", "plots/calibration_locations","plotdata");
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot measured Fz vs reconstructed Fz
plot( Fz, fit(:,1), "*");
xlabel("measured Fz (N)");
ylabel("estimated Fz (N)");
title("Estimated versus Measured Normal Force");
axis([-25 0 -25 0]);
axis("equal");  %% force x distance to equal y distance
%%print"plots/Fz_estimation_error.pdf");
plotdata = [Fz fit(:,1)]; save("-ascii", "plots/Fz_estimation_error","plotdata");
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot measured Tx vs reconstructed Tx
plot( Tx, fit(:,2), "*");
xlabel("measured Tx (N-mm)");
ylabel("estimated Tx (N-mm)");
title("Estimated versus Measured Torque Tx");
axis([-350 350 -350 350]);
axis("equal");
%%print"plots/Tx_estimation_error.pdf");
plotdata = [Tx fit(:,2)]; save("-ascii", "plots/Tx_estimation_error","plotdata");
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot measured Tx vs reconstructed Ty
plot( Ty, fit(:,3), "*");
xlabel("measured Ty (N-mm)");
ylabel("estimated Ty (N-mm)");
title("Estimated versus Measured Torque Ty");
axis([-350 350 -350 350]);
axis("equal");
%%print"plots/Ty_estimation_error.pdf");
plotdata = [Ty fit(:,3)]; save("-ascii", "plots/Ty_estimation_error","plotdata");
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the reconstructed contact coordinates versus actual
plot( fitxy(:,3), fitxy(:,1), "*");
xlabel("actual X location (mm)");
ylabel("estimated X location (mm)");
title("Reconstructed Probe X Location versus Actual (for Fz < threshold)");
axis([-50 50 -50 50]);
axis("equal");  %% force x distance to equal y distance
# %%print"plots/locations.pdf");
pause

% plot the reconstructed contact coordinates versus actual
plot( fitxy(:,4), fitxy(:,2), "+;Y;");
xlabel("actual Y location (mm)");
ylabel("estimated Y location (mm)");
title("Reconstructed Probe Y Location versus Actual (for Fz < threshold)");
axis([-50 50 -50 50]);
axis("equal");  %% force x distance to equal y distance
# %%print"plots/locations.pdf");
pause
save("-ascii", "plots/XY_touch_estimate","fitxy");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot the reconstructed contact locations
%% the quiver plot below is more useful
# plot( fitxy(:,1), fitxy(:,2), "-;touch point;" )
# xlabel("X location (mm)");
# ylabel("Y location (mm)");
# title("Reconstructed Probe Locations");
# axis([-50 50 -50 50]);
# axis("equal");  %% force x distance to equal y distance
# %%print"plots/locations.pdf");
# pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot the torque estimation error as a function of location
# quiver( dx, dy, err(:, 2), err(:, 3));
# xlabel("X location (mm)");
# ylabel("Y location (mm)");
# title("Reconstructed Torque error");
# axis([-50 50 -50 50]);
# axis("equal");  %% force x distance to equal y distance
# %%print"plots/locations.pdf");
# pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot the touch location error as a function of location
quiver( fitxy(:,3), fitxy(:,4), fitxy(:,1) - fitxy(:,3), fitxy(:,2) - fitxy(:,4), 0 ); 
xlabel("X location (mm)");
ylabel("Y location (mm)");
title("Reconstructed Touch Location Error");
axis([-50 50 -50 50]);
axis("equal");  %% force x distance to equal y distance
%%print"plots/XY_estimation_error.pdf");
plotdata = [fitxy(:,3) fitxy(:,4) (fitxy(:,1) - fitxy(:,3)) (fitxy(:,2) - fitxy(:,4))];
save("-ascii", "plots/XY_estimation_error","plotdata");
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot the estimated versus measured stiffness function in 2D
r = linspace (0, 50, 41)';
kg = (3 * K * R^2) ./ (2 * r.^2 + R^2);
radius = sqrt(dx.^2 + dy.^2);
plot( r, kg, "-;fitted;", radius, Fz ./ mdz, "+;measured;" );
%% this wasn't so useful: "radius, mFz ./ mdz, "*;reconstructed;","
xlabel("radial location (mm)");
ylabel("stiffness (N/mm)");
title("Single Point Stiffness");
axis([0 50 0 30]);
plotdata = [ r kg ];               save("-ascii", "plots/fitted_stiffness","plotdata");
plotdata = [ radius (Fz ./ mdz) ]; save("-ascii", "plots/measured_stiffness","plotdata");
%%print("plots/stiffness_estimation.pdf");
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot calibration signal
plot3( dx, dy, Fz, "*;Fz;" )
xlabel("X location (mm)");
ylabel("Y location (mm)");
ylabel("Fz (N)");
title("Normal Force on Calibration Samples");
axis([-50 50 -50 50]);
pause
%% axis("equal");  %% force x distance to equal y distance
%%print"plots/locations.pdf");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot calibration error versus location
# plot3( dx, dy, err(:,1), "*;Fz error;" )
# xlabel("X location (mm)");
# ylabel("Y location (mm)");
# ylabel("Fz error (N)");
# title("Normal Force Error on Calibration Samples");
# axis([-50 50 -50 50 -3 3]);
# %% axis("equal");  %% force x distance to equal y distance
# %%print("plots/locations.pdf");
# pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% computed F1 versus calibrated F1 to check linearity
%% for "virtual" spring locations at 0, 2 Pi/3, and 4 Pi/3

% forces computed from measurement
%% F1 = (-Fz .* (R + 2*dx)) / (3*R);
%% F2 = (Fz .* (-R + dx - sqrt(3)*dy)) / (3*R);
%% F3 = (Fz .* (-R + dx + sqrt(3)*dy)) / (3*R);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot the raw optical sensor data versus touch position
# plot3( dx, dy, raw(:,1), "*;F1;", dx, dy, raw(:,2), "*;F2;", dx, dy, raw(:,3), "*;F3;" );
# xlabel("X location (mm)");
# ylabel("Y location (mm)");
# title("Raw Force Displacement Signals");
# axis([-50 50 -50 50]);
# %axis("equal");  %% force x distance to equal y distance
# %%print("plots/locations.pdf");
# pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot the stiffness function in 3D

xg = yg = linspace (-50, 50, 41)';
[xx, yy] = meshgrid (xg, yg);
zg = (3 * K * R^2) ./ (2 * xx.^2 + yy.^2 + R^2);
mesh (xg, yg, zg);
xlabel("X location (mm)");
ylabel("Y location (mm)");
zlabel("stiffness (N/mm)");
title("Modeled Stiffness for Single Contacts");
axis([-50 50 -50 50]);
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot the measured stiffness function in 3D
plot3( dx, dy, Fz ./ dz, "*" );
xlabel("X location (mm)");
ylabel("Y location (mm)");
zlabel("stiffness (N/mm)");
title("Measured Stiffness for Calibration Contacts");
axis([-50 50 -50 50]);
pause
plotdata = [ dx dy Fz ./ dz]; save("-ascii", "plots/stiffness_all_xy","plotdata");

################################################################
# emit the stiffness data as contours, just using the harder presses
plotfile = fopen( "plots/stiffness_xy", "w");
stiffness = Fz ./ dz;
count = 0;
for i = 1:samples

  %% detect the stiff pushes
  if ( probe(i,3) < -0.75 )

    %% save the first point of each contour to repeat it later
    if (count == 0)
      first = [ dx(i), dy(i), stiffness(i)];
    else
      %% detect each full orbit, and break them into separate contours, but repeat the first point of each isoline
      if ( abs(atan2( dy(i), dx(i))) < 0.1) 
	fprintf( plotfile, "%f %f %f\n", first );
	fprintf(plotfile,"\n");
	count = 0;
	first = [ dx(i), dy(i), stiffness(i)];
	endif
    endif

    %% emit the x,y,stiffness point on this contour line
    count += 1;
    fprintf( plotfile, "%f %f %f\n", dx(i), dy(i), stiffness(i) );
  endif
endfor
fprintf( plotfile, "%f %f %f\n", first );
fclose(plotfile);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% actual normal force versus spring function prediction
plot3( dx, dy, Fz, "*;Fz;", dx, dy, mFz,"+;model;"  );
xlabel("X location (mm)");
ylabel("Y location (mm)");
ylabel("Fz (N)");
title("Actual versus Measured Normal Force on Calibration Samples");
axis([-50 50 -50 50]);
%% axis("equal");  %% force x distance to equal y distance
%%print"plots/locations.pdf");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%