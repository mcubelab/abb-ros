function [t_c, q_c, num_inliers] = get_transform(obj_points, world_points, robot_poses)

% At this point, we have a list of the positions of the object holes as
% seen by the camera (obj_points), and a list of the positions of the
% object holes in the world.

op = cell2mat(obj_points);
wp = cell2mat(world_points);

N = size(op,1);


% Now, let's use RANSAC to pick a large percentage of points to use as our 

THRESH = 0.003;     % How far away transformed points can be from each 
                    %  other to be considered an inlier (m)
NUM_POINTS = 4;     % Number of points to draw each time
NUM_ITERS = 15000;   % Number of times to draw random samples

most_inliers = 0;
best_point_idxs = [];

for i=1:NUM_ITERS
    % Pick a random set of points
    idxs = randperm(N);
    obj_pts = op(idxs(1:NUM_POINTS),:);
    world_pts = wp(idxs(1:NUM_POINTS),:);

    % Find the best homogeneous transform from these points
    [R, t_c] = findTransform(obj_pts', world_pts');
    H = [R t_c; 0 0 0 1];

    % Transform all of our points using this transform
    new_op = H * [op'; ones(1,N)];
    new_op = new_op(1:3,:)';

    % Find the sum of squares of all of these points
    dists = sum((new_op - wp).^2,2);

    idxs = find(dists < THRESH^2);
    num_pts = length(idxs);

    if (num_pts > most_inliers)
        most_inliers = num_pts;
        best_point_idxs = idxs;
    end
end

N
num_inliers = length(best_point_idxs)

% Now, fit the best plane with the found inliers:
[R, t_c] = findTransform(op(best_point_idxs, :)', wp(best_point_idxs,:)');
q_c = rot2quat(R);

H = [R t_c; 0 0 0 1];

% If robot_poses are provided, let's visualize the solution
if (nargin == 3)
    
    roc = cell(length(obj_points),1);
    rc = cell(length(obj_points),1);
    
    for i=1:length(obj_points)
        % Let's compute where all of the points are in the frame of the robot

        % First compute where all of our points are in the world frame
        oc = H * [obj_points{i}'; ones(1, size(obj_points{i},1))];

        % Now, using the robot pose, compute where all of the points are with
        % respect to the robot
        roc{i} = (inv(toHomo(robot_poses(i,:))) * oc)';
        rc{i} = (inv(toHomo(robot_poses(i,:))) * [world_points{i}'; ones(1, size(world_points{i},1))])';
    end
    
    rocm = cell2mat(roc)';
    rcm = cell2mat(rc)';
    
    outs = ones(N,1);
    outs(best_point_idxs) = 0;
    out_idx = find(outs);
    
    % Separate our points into inliers and outliers
    inlier_points_1 = rocm(1:3, best_point_idxs);
    outlier_points_1 = rocm(1:3, out_idx);
    inlier_points_2 = rcm(1:3, best_point_idxs);
    outlier_points_2 = rcm(1:3, out_idx);
    
    % Draw lines between inliers and lines between outliers
    figure(1); clf; hold on;
    plot3([inlier_points_1(1,:); inlier_points_2(1,:)], ...
        [inlier_points_1(2,:); inlier_points_2(2,:)], ...
        [inlier_points_1(3,:); inlier_points_2(3,:)], 'g');
    plot3([outlier_points_1(1,:); outlier_points_2(1,:)], ...
        [outlier_points_1(2,:); outlier_points_2(2,:)], ...
        [outlier_points_1(3,:); outlier_points_2(3,:)], 'b');
    axis equal;
    
    figure(2);clf;hold on;
    diffs = sqrt(sum((inlier_points_1 - inlier_points_2).^2,1))*1000;
    mean(diffs)
    hist(diffs,100);
end

end










