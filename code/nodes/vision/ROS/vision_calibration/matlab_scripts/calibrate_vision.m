% This function attempts to find the extrinsic calibration of a camera that
% returns 3D point cloud information. The camera will be calibrated with
% respect to the world frame used by the robot. This is accomplished by
% moving the robot holding a target to different positions, and saving the
% robot pose and 3D point cloud captured at that position. We will input
% this data into our function using a calibration file, which is formatted
% in the following way:
%
% x1, y1, z1, q01, qx1, qy1, qz1, /some/directories/filename1.pcd
% .  .  .  .   .   .   .   .
% .  .  .  .   .   .   .   .
% xM, yM, zM, q0M, qxM, qyM, qzM, /some/directories/filenameM.pcd
%
% The first 7 columns represent the robot pose when the 3D point cloud was
% captured, and the final column represents the captured point cloud file.
%
% As these pcd files may be captured on a different system, our function
% also takes in the path to a folder containing all of the ascii pcd files.
%
% The robot pose is the pose of the flange with respect to the world frame.
% Everything is in meters
%

function [t_c, q_c, robot_poses, point_clouds, obj_points, world_points] = calibrate_vision(calfile, folder)

% First, load in our data 
[robot_poses, point_clouds] = load_data(calfile, folder);

% Next, find the relevant centers of our calibration pattern, both in world
% coordinates and camera coordinates
[obj_points, world_points] = find_points(robot_poses, point_clouds);

% Now, using the centers in both coordinate frames, compute the optimal
% transform
[t_c, q_c] = get_transform(obj_points, world_points, robot_poses);

% Finally, let's plot our results
plot_results(point_clouds, robot_poses, [t_c' q_c'], obj_points);

end
