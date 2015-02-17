% This function creates a figure that lets the user check if they have
% created their object description and point cloud file correctly. It takes
% in a .objd file as input, and does the rest on its own
%
% Last Modified 2/12/2014

function check_objd(filename)

cfg_data = read_config_file(filename);

% Now let's open the specified pcd file and plot the points
[scriptpath, ~,~] = fileparts(mfilename('fullpath'));
pcdpath = fullfile(scriptpath, '..', 'pcd_files', cfg_data.pcdFile);

% Every PCD file is different, but we will assume that the data starts
% after the "DATA ascii" line. Let's find out which line that is at, so we
% can start reading data after that line
fid = fopen(pcdpath); 
header_lines = 0;
data_found = 0;
while 1
    tline = fgetl(fid);
    if (~ischar(tline))
        break
    end
    if (isempty(strfind(tline, 'DATA')))
        header_lines = header_lines + 1;
    else
        data_found = 1;
        header_lines = header_lines + 1;
        break;
    end
end
fclose(fid);

if (data_found ~= 1)
    fprintf('DATA keyword not found in pcdfile: %s\n', pcdfile);
    points = [];
else
    % Now let's read in our data
    fid = fopen(pcdpath); 
    C = textscan(fid, '%f %f %f' ,'HeaderLines',header_lines);
    fclose(fid);
    points = cell2mat(C);
end

% Now let's start plotting stuff. 
% First, plot the points
figure(1);clf; hold on;
plot3(points(:,1), points(:,2), points(:,3),'o', 'MarkerSize', 2);

% Next, plot the axes
AXIS_LENGTH = 0.05;
plot3([0 AXIS_LENGTH], [0 0], [0 0], 'r', 'LineWidth', 10);
plot3([0 0], [0 AXIS_LENGTH], [0 0], 'g', 'LineWidth', 10);
plot3([0 0], [0 0], [0 AXIS_LENGTH], 'b', 'LineWidth', 10);

% Next, if there are symmetries, plot those
if (cfg_data.symType > 0)
    for i=1:size(cfg_data.symPoints,1)
        cur_point = cfg_data.symPoints(i,:);
        switch(cfg_data.symOrders(i))
            case -1
                % For cylindrical symmetry, plot the axis emanating from
                % the point
                plot3([cur_point(1) cur_point(1) + AXIS_LENGTH * cfg_data.symAxes(i,1)], ...
                    [cur_point(2) cur_point(2) + AXIS_LENGTH * cfg_data.symAxes(i,2)], ...
                    [cur_point(3) cur_point(3) + AXIS_LENGTH * cfg_data.symAxes(i,3)], ...
                    'LineWidth', 10, 'Color', [0 0 0]);
                plot3(cur_point(1), cur_point(2), cur_point(3), '+k', 'MarkerSize', 20);
            case -2
                % For spherical symmetry, just plot the point
                plot3(cur_point(1), cur_point(2), cur_point(3), '*y', 'MarkerSize', 20);
            otherwise
                % For ordered symmetry, plot each of the axes emanating
                % from each of the points, and change the color of the axis
                % based on the order
                str = dec2bin(cfg_data.symOrders(i),3);
                cc = eval(sprintf('[%c %c %c]', str));
                plot3([cur_point(1) cur_point(1) + AXIS_LENGTH * cfg_data.symAxes(i,1)], ...
                    [cur_point(2) cur_point(2) + AXIS_LENGTH * cfg_data.symAxes(i,2)], ...
                    [cur_point(3) cur_point(3) + AXIS_LENGTH * cfg_data.symAxes(i,3)], ...
                    'LineWidth', 10, 'Color', cc);
                plot3(cur_point(1), cur_point(2), cur_point(3), 'x', 'MarkerSize', 20, 'Color', cc);
        end
    end
end



% Finally, if we are guessing with planes, display that
if (cfg_data.guessType == 2)
    % We will display the planes as polygonal patches
    patch_points = cell(size(cfg_data.guessPlanes,1),1);
    
    % First, using the intersection list, let's find all of our
    % intersection points
    for i=1:size(cfg_data.guessInts,1)
        % Extract the planes that intersect
        ints = cfg_data.guessInts(i,:);
        intplanes = cfg_data.guessPlanes(ints,:);
        if (det(intplanes(:,1:3)) == 0)
            fprintf('ERROR: Planes %d, %d, and %d do not intersect', gints(i,:));
            continue;
        end
        
        % Find the intersection point
        pt = -1*pinv(intplanes(:,1:3)) * intplanes(:,4);
        for j=1:length(ints)
            patch_points{ints(j)} = [patch_points{ints(j)} pt];
        end
        % Plot the intersection
        plot3(pt(1), pt(2), pt(3), 'ok', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    end
    
    % Now, let's plot our patches
    for i=1:length(patch_points)
        pts = patch_points{i};
        
        % If we don't have enough points to make a patch for this plane, we
        % will project all of the points onto the plane, and plot the
        % convex hull.
        if (size(pts,2) < 3)
            
            % Get the plane, and normalize it
            plane = cfg_data.guessPlanes(i,:);
            plane = plane / norm(plane(1:3));
            
            % Now project all of the points of our object onto the plane
            t = (- plane(1,1:3) * points' - plane(4));
            proj_pts = bsxfun(@plus, points, bsxfun(@times, t', plane(1,1:3)));
            
            % Get a 2-dimensional coordinate system to express all of the
            % points in.
            cp = mean(proj_pts);
            meanpts = bsxfun(@minus, proj_pts, cp);
            [~,~,v] = svd(meanpts);
            planepts = proj_pts * v(:,1:2);
            
            % Find the indices of the points representing the convex hull
            k =convhull(planepts);
            
            % These will be the points we use to draw our patch
            pts = proj_pts(k,:)';
        elseif (size(pts,2) == 3)
            % If we have exactly 3 points, the ordering of the points
            % doesn't matter. We'll proceed directly to plotting the
            % points.
            % Make sure we compute the center of the triangle, which will
            % be used to draw the normal vector for this plane
            cp = mean(pts,2)';
        else
            % If instead we did find enough intersections, now we need to
            % just make sure the points are ordered correctly. We will do
            % this by finding the center of the intersection points,
            % drawing a vector to each of the points, and then using the
            % cross product to order the vectors
            
            % Compute the mean, and vectors from the mean to each
            % intersection
            cp = mean(pts,2);
            vecs = bsxfun(@minus, pts, cp);

            % Our base vector will be the first one, and all of the other
            % vectors will be the ones we try to order
            basevec = vecs(:,1);
            restvecs = vecs(:,2:end);

            % Compute the cross product of the first vector with the rest
            % of the vectors
            crosses = cross(repmat(basevec, 1, size(restvecs,2)), restvecs, 1);
            
            % Compute the product of the magnitude of the first vector with
            % each of the other vectors
            magprod = sqrt(basevec' * basevec) * sqrt(sum(restvecs.^2));

            % Compute the magnitude of the vectors formed by each of our
            % cross products as well
            cross_mag = sqrt(sum(crosses.^2));

            % Since a x b = |a||b|sin(th)*n, where n is a unit vector, we
            % can compute the angle between a and b by: 
            % sin(th) = |a x b| / (|a||b|)
            angs = asin(cross_mag ./ magprod);
            
            % Now, make sure we compute all of the angles in the same
            % direction, by making sure that all of the cross product
            % vectors point in the same direction
            for j=2:size(restvecs,2)
                if (crosses(:,1)'*crosses(:,j) < 0)
                    angs(j) = angs(j) - pi;
                end
            end
            
            % Finally, if a vector happens to be exactly 180 degrees away,
            % we will get weird behavior. Let's set this to be the correct
            % value, which is 180 degrees away. Note that this assumes that
            % no 2 intersection points are within 0.0001 radians from each
            % other.
            for j=1:size(restvecs,2)
                if (abs(angs(j)) < 0.0001)
                    angs(j) = angs(j) + pi;
                end
            end
            % Make sure all of our angles are between 0 and 2pi
            angs = mod(angs, 2*pi);

            % Now we're ready to sort the indices properly.
            [~,k] = sort(angs);

            % Once sorted, we'll put the first vector at the front, and
            % order all of the rest of them appropriately, and now we have
            % our patch points
            k = [1 (k+1)];
            pts = pts(:,k);
        end
        
        % Plot the planes, and change colors just so the user can
        % differentiate
        str = dec2bin(i,3);
        cc = eval(sprintf('[%c %c %c]', str));
        fill3(pts(1,:), pts(2,:), pts(3,:),cc);
        alpha(0.2); % Also make them transparent
        
        % Finally, plot the normals, so the user can confirm they are
        % outward facing
        nv = cfg_data.guessPlanes(i,1:3);
        nv = nv / norm(nv);
        plot3([cp(1) cp(1) + AXIS_LENGTH * nv(1)*0.5], ...
            [cp(2) cp(2) + AXIS_LENGTH * nv(2)*0.5], ...
            [cp(3) cp(3) + AXIS_LENGTH * nv(3)*0.5], ...
            'LineWidth', 5, 'Color', cc);
    end
elseif (cfg_data.guessType == 1)
    % If we're fitting an ellipse, draw it just to see what it would look
    % like
    cp = mean(points);
    % Compute the covariance matrix
    meanpts = bsxfun(@minus, points, cp);
    cov = meanpts' * meanpts;
    [v,L] = eig(cov);
    % Project all the points along these dimensions
    proj_pts = meanpts * v;
    % Look at the max value along each dimension to get a sense of scale
    maxvals = max(abs(proj_pts));
    % Use this to compute the radii of our ellipse, for visualization
    scale = mean(maxvals./sqrt(diag(L))');
    radii = scale*sqrt(diag(L));
    
    % Now create an ellipsoid centered at the origin with this radii
    N = 20;
    [xc,yc,zc] = ellipsoid(0,0,0,radii(1), radii(2), radii(3),N);
    
    % Transform the points along the eigenvector directions and wherever
    % the mean of the data is
    points = [xc(:) yc(:) zc(:)];
    transformed_points = bsxfun(@plus,(v * points'),cp');
    x = reshape(transformed_points(1,:), N+1, N+1);
    y = reshape(transformed_points(2,:), N+1, N+1);
    z = reshape(transformed_points(3,:), N+1, N+1);
    
    % Finally, draw our surface, and make it transparent
    surfl(x,y,z);
    alpha(0.2);
end

% Finally, be sure to set our axes to be equal
axis equal

end
