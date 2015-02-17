function [obj_points, world_points] = find_points(robot_poses, point_clouds)

HOLE_LOCATIONS = ...
    [-0.046, -0.046, 0.085;
    -0.046, 0.046, 0.085;
    0.0, 0.0, 0.085];

% Offsets of the holes with respect to the center of mass, rotated based on
% what SVD will return as the basis directions of the plane (mm)
HOLE_OFFSETS = ...
    [-0.046, 0.00855;
    0.046, 0.00855;
    0.0, -0.03741];

% Number of holes in our calibration pattern
NUM_HOLES = size(HOLE_LOCATIONS,1);

% Our points are in units of meters. We will create a grid that is
% 0.1mm resolution. 
GRID_RES = round(1000/0.1);

M = length(point_clouds);

h1 = figure(1);

% At this point, we have a list of poses and a list of points. Now, let's
% analyze each point cloud, and locate the pattern
obj_points = cell(M,1);
world_points = cell(M,1);
for j=1:M
    j
    tic
    A = point_clouds{j};
    N = size(A,1);
    
    % The first thing to do is to use RANSAC to fit a plane to the pattern
    
    %THRESH = 0.006;     % How far away a point can be away from the plane to 
                        %  be considered an inlier
    THRESH = 0.006;
    MIN_POINTS = N*0.98; % The minimum number of inlier points for a plane to 
                        %  be considered a candidate.
    NUM_POINTS = 3;     % Number of points to draw each time
    NUM_ITERS = 600;    % Number of times to draw random samples


%     best_error = realmax;
%     best_v = zeros(3,1);
%     best_points = [];
% 
%     for i=1:NUM_ITERS
%         % Pick a random set of points
%         idxs = randperm(N);
%         points = A(idxs(1:NUM_POINTS),:);
% 
%         % Fit the plane Ax+By+Cz = 1 to the data
%         v = pinv(points)*ones(NUM_POINTS,1);
% 
%         % Find the distances to all of the points in our data
%         dists = abs(A*v-1)/sqrt(sum(v.^2));
% 
%         % Count the number of points in our data set that are near the fitted
%         % plane
%         num_pts = sum(dists < THRESH);
% 
%         % If this plane has enough inliers, remember it
%         if (num_pts >= MIN_POINTS)
%             % Now let's fit a better plane to all of our inliers
%             points = A(dists < THRESH, :);
%             v = pinv(points)*ones(size(points,1),1);
% 
%             % Let's compute the average squared error
%             dists = abs(points*v-1)/sqrt(sum(v.^2));
%             error = sqrt(dists'*dists) / size(points,1);
% 
%             % If this is the best error we've found so far, then save this
%             % model
%             if (error < best_error)
%                 best_error = error;
%                 best_v = v;
%                 best_points = points;
%             end
%         end
%     end

    most_inliers = 0;
    best_inliers = [];

    for i=1:NUM_ITERS
        % Pick a random set of points
        idxs = randperm(N);
        points = A(idxs(1:NUM_POINTS),:);

        % Fit the plane Ax+By+Cz = 1 to the data
        v = pinv(points)*ones(NUM_POINTS,1);

        % Find the distances to all of the points in our data
        dists = abs(A*v-1)/sqrt(sum(v.^2));

        % Count the number of points in our data set that are near the fitted
        % plane
        inliers = find(dists < THRESH);
        
        % If this is the most number of inliers found thus far, remember
        % it.
        if (length(inliers) > most_inliers)
            most_inliers = length(inliers);
            best_inliers = inliers;
        end
    end
    
    % After all of our iterations, use the most number of inliers found to
    % fit the best plane we can, and this is our best estimate
    best_points = A(best_inliers,:);
    best_v = pinv(best_points) * ones(most_inliers,1);
    
    fprintf('Best fit plane is: %1.5f x + %1.5f y + %1.5f z = 1\n', best_v);
    fprintf('points kept: %d / %d = %1.5f\n', size(best_points,1), N, size(best_points,1)/N);
    
    % Now we will project all of our points onto this plane
    proj_pts = best_points - bsxfun(@times, (best_points*best_v-1)/(sum(best_v.^2)), best_v');
    % Choose the origin as the center of these points
    origin = mean(proj_pts);
    
    vec_to_origin = -origin;
    
    % Get all of our points in terms of plane coordinates
    proj_pts = bsxfun(@minus, proj_pts, origin);
    [~,~,v] = svd(proj_pts);
    
    % Make sure that our third vector, which will be the normal vector to
    % the plane, is pointing towards the camera (0,0,0)
    
    if (vec_to_origin * v(:,3) < 0)
        v(:,3) = - v(:,3);
    end
    
    if (det(v) < 0)
        v(:,1) = - v(:,1);
    end


    temp = proj_pts * v;
    xy = temp(:,1:2);
    
    set(0,'CurrentFigure', h1); clf;
    plot(xy(:,1), xy(:,2), 'o');
    
    % HACK: Need to come up with a better way to figure out if the
    % orientation has flipped
    % If we are not looking at this the right way, rotate it by 180 degrees
    if (min(xy(:,2)) > -0.08)
        v(:,2) = -v(:,2);
        v(:,1) = -v(:,1);
        disp('flipped');
        
        temp = proj_pts * v;
        xy = temp(:,1:2);
    end

    
    % Convert our list of points into an image. We will do this by
    % multiplying by a large grid size, rounding, and converting our points
    % into indices in an image
    pix = round(xy*GRID_RES);
    
    % Find the minimum value and subtract it, so that our minimal index
    % will be 1, both for x and y
    mp = min(pix);
    indices = bsxfun(@minus, pix, mp)+1;
    
    % Create and image that is exactly as large as the largest index, and
    % fill all of the positions where a point exists with a 1
    image = zeros(max(indices));
    image(sub2ind(size(image),indices(:,1), indices(:,2))) = 1;
    
    % Since our grid is small, there will be a lot of empty space. Dilate
    % our image to get mostly a solid surface
    image = imdilate(image,strel('square', round(GRID_RES/300)));
    %figure(2);imshow(image)

    disp('erosion + convhull');
    chimage = imerode(image, strel('square', round(GRID_RES / 50)));
    [chr, chc] = find(chimage);
    q = convhull(chr, chc);
    %figure(1);clf;hold on;plot(chr, chc, 'b.');plot(chr(q), chc(q), 'r.');
    
    keep = poly2mask(chc(q), chr(q), size(image,1), size(image,2));
    %figure(2);clf;imshow(keep);
    %figure(3);clf;imshow(image);
    
    %sum(q)
    disp('done');
    
    % Now, everywhere that is still empty will be the holes in our pattern
    [rows, cols] = find(~image & keep);
    
%     disp('inpolygon');
%     valid = inpolygon(rows, cols, chr(q), chc(q));
%     disp('done');
    
    % Create points by converting back from our image to 2d pose on the
    % plane
    pts = [rows cols];
    pts = bsxfun(@plus, pts, mp) / GRID_RES;
    
    %figure(4);clf;plot(pts(:,1), pts(:,2), 'k.');
    
    % Compute the center of mass, which we will base our cluster guesses
    % off of
    com = mean(xy);
    
    % Compute our cluster guesses
    guess = bsxfun(@plus, com, HOLE_OFFSETS);
%     
%     % Some points will remain on the outside of the calibration pattern.
%     % Filter these out 
%     valid_points = zeros(size(pts,1),1);
%     for k=1:NUM_HOLES
%         dists = sqrt(sum(bsxfun(@minus, guess(k,:), pts).^2,2));
%         valid_points = valid_points | (dists < 0.03);
%     end
%     
    

    [~, C] = kmeans(pts, NUM_HOLES, 'start', guess, 'emptyaction', 'singleton');
    
    h = figure(3);set(h, 'Visible', 'off');clf;hold on;
    plot(xy(:,1), xy(:,2), 'b.');
    plot(pts(:, 1), pts(:,2), 'k.');
    plot(C(:,1), C(:,2), 'g*');
    plot(guess(:,1), guess(:,2), 'r*');
    saveas(h, sprintf('figures_new/%d.png',j));
    
    rpts = bsxfun(@plus, C * v(1:3,1:2)', origin);
    
    % Save points
    obj_points{j} = rpts;

    
    % Given that we know how the calibration object is attached to the
    % robot, and given that we know the pose of the robot, we can also
    % compute the world positions of the object points as well.
    H = toHomo(robot_poses(j,:));
    temp = H * [HOLE_LOCATIONS'; ones(1,size(HOLE_LOCATIONS,1))];
    world_points{j} = temp(1:3,:)';
    toc
end

end

