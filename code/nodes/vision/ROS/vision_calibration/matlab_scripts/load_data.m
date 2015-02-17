function [robot_poses, point_clouds, loaded_data] = load_data(calfile, folder)

% First, read in our calibration file and store things into appropriate
% arrays
fileID = fopen(calfile);
C = textscan(fileID,'%f, %f, %f, %f, %f, %f, %f, %s');
fclose(fileID);

% The robot flange poses are the first 7 columns
robot_poses = cell2mat(C(1:7));
%robot_poses(:,1:3) = robot_poses(:,1:3)/1000.0; % If robot pose was in mm

% The point cloud files are the 8th column
files = C{8};

M = length(files);

% This will store all of our points
point_clouds = cell(M, 1);

% Extract the points from each of the files
for i=1:M
    i
    [~,name,ext] = fileparts(files{i});
    fpath = fullfile(folder, strcat(name,ext));
    D = importdata(fpath, ' ', 11);
    point_clouds{i} = D.data;
end

loaded_data.robot_poses = robot_poses;
loaded_data.point_clouds = point_clouds;

disp('read in data');

end

