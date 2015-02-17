% This function reads in all object descriptions in a folder and gets
% information necessary to calculate the distance between 2 poses of a
% certain object. It uses the description files, stl files, and python
% files to collect all of the neccesary info, and then saves everything to
% a global structure that can be loaded using 'init_posedist.m'

function setup_posedist(num_points, objFolder)

  % Here lies our list of object names, and their corresponding number.
  % Note that this should match with RecObj::Type which is located in
  % objRec_comm.h

  OBJ_LIST = {'Arch', 'Big Sphere', 'Big Triangle', 'Old Triangle', 'Cylinder', 'Long Block', 'Rectangle', 'Small Block', 'Small Cube', 'Small Sphere', 'Small Triangle', 'Square Block'};

  NUM_OBJ = length(OBJ_LIST);



  if (nargin < 2)
    objFolder = '/home/simplehands/Documents/hands/code/nodes/vision/ROS/objRec_node/objectFolder';
  end

  if (nargin < 1)
    num_points = 5000;
  end

  scriptsFolder = fullfile(objFolder, 'scripts');


  % First, let's read in all of the config files
  objDescFolder = fullfile(objFolder, 'object_descriptions');
  files = dir(objDescFolder);

  obj_data = cell(NUM_OBJ,1);
  for i=1:length(files)
    if (files(i).name(1) ~= '.' && ~strcmpi(files(i).name, 'README.txt'))
      cfg_data = read_config_file(fullfile(objDescFolder, files(i).name));
      objNameFound = 0;
      for j=1:NUM_OBJ
        if (strcmp(OBJ_LIST{j}, cfg_data.objName))
          obj_data{j} = cfg_data;
          objNameFound = 1;
          break;
        end
      end
      if (objNameFound ~= 1)
        fprintf('WARNING: Could not find object name: %s in our object list. Ignoring.\n', cfg_data.objName);
      end
    end
  end

  % Make sure all of the objects in our object list are accounted for
  for i=1:NUM_OBJ
    if (isempty(obj_data{i}))
        error(sprintf('Could not find object list item: %s. Quitting...', OBJ_LIST{i}));
    end
  end

  stlFolder = fullfile(objFolder, 'stl_files');

  for i=1:NUM_OBJ
    % Let's precompute any necessary things for taking into account object
    % symmetries
    switch(obj_data{i}.symType)

      % No Symmetry
      case 0
        % Nothing to do

      % Order Symmetry
      case 1
        % We'll precompute all of the possible different ways we could
        % transform the object and still have it look the same
        obj_data{i}.numSymTransf = prod(obj_data{i}.symOrders);
        obj_data{i}.symTransf = cell(obj_data{i}.numSymTransf, 1);

        transforms = cell(obj_data{i}.numSyms, 1);

        % For each order symmetry, compute the transform to get us into a 
        % frame where we can rotate a discrete amount and still preserve
        % symmetry.
        for j=1:obj_data{i}.numSyms
          transforms{j} = cell(1,obj_data{i}.symOrders(j));
          w = obj_data{i}.symAxes(j,:)';
          p = obj_data{i}.symPoints(j,:)';
          Z = w;
          Y = cross(w, [1; 0; 0]);
          if (norm(Y) == 0)
            Y = [0; 1; 0];
          end
          X = cross(Y,Z);
          Z = Z / norm(Z);
          Y = Y / norm(Y);
          X = X / norm(X);
          T = [[X Y Z p]; 0 0 0 1];

          % Now that we have the frame, rotate a certain number of degrees
          % according to the order of this particular symmetry
          for k=1:obj_data{i}.symOrders(j)
            R = [[quat2rot([cos(pi / obj_data{i}.symOrders(j) * k); 0; 0; sin(pi / obj_data{i}.symOrders(j) * k)]) [0;0;0]]; 0 0 0 1];
            transforms{j}{k} = T*R*inv(T);
          end
        end

        % Now that we have all of our transforms from each symmetry,
        % combine them together to get all possible transforms
        switch(obj_data{i}.numSyms)
          case 1
            for j=1:obj_data{i}.symOrders(1)
              obj_data{i}.symTransf{j} = transforms{1}{j};
            end
          case 2
            curTransf = 1;
            for j=1:obj_data{i}.symOrders(1)
              tempTransf = transforms{1}{j};
              for k=1:obj_data{i}.symOrders(2)
                obj_data{i}.symTransf{curTransf} = tempTransf * transforms{2}{k};
                curTransf = curTransf + 1;
              end
            end
          case 3
            curTransf = 1;
            for j=1:obj_data{i}.symOrders(1)
              tempTransf = transforms{1}{j};
              for k=1:obj_data{i}.symOrders(2)
                tempTransf = tempTransf * transforms{2}{k};
                for m=1:obj_data{i}.symOrders(3)
                  obj_data{i}.symTransf{curTransf} = tempTransf * transforms{3}{m};
                  curTransf = curTransf + 1;
                end
              end
            end
        end

      % Cylindical Symmetry
      case 2
        % Compute the transform to the cylindrical axis, and we're done!
        w = obj_data{i}.symAxes';
        p = obj_data{i}.symPoints';

        Z = w;
        Y = cross(w,[1;0;0]);
        X = cross(Y,Z);

        Z = Z / norm(Z);
        Y = Y / norm(Y);
        X = X / norm(X);

        obj_data{i}.cylT = [[X Y Z p]; 0 0 0 1];

      % Spherical Symmetry
      case 3
        obj_data{i}.sphereCenter = obj_data{i}.symPoints';

      otherwise
        warn('Unknown Symmetry. Continuing anyways...');

    end


    % Now, for each object in our list, let's compute the surface area of the
    % block using it's STL file, which we will assume has the same name as
    % its PCD file

    [~, fname, ~] = fileparts(obj_data{i}.pcdFile);
    stlFileName = fullfile(stlFolder, strcat(fname, '.STL'));
    obj_data{i}.surfaceArea = getSurfaceAreaFromSTL(stlFileName);

    % Finally, for each object in our list, let's generate points on the
    % surface of the object which will be used when calculating the distance
    % 2 poses are away from one another.
    system(sprintf('python %s %s temp.pcd %d', fullfile(scriptsFolder, 'stl2pcd.py'), stlFileName, num_points));

    % Let's open the point cloud file that our script created and load the
    % points into memory.

    % Every PCD file is different, but we will assume that the data starts
    % after the "DATA ascii" line. Let's find out which line that is at, so we
    % can start reading data after that line
    fid = fopen('temp.pcd'); 
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
      obj_data{i}.pts = [];
    else
      % Now let's read in our data
      fid = fopen('temp.pcd'); 
      C = textscan(fid, '%f %f %f', 'HeaderLines', header_lines);
      fclose(fid);
      obj_data{i}.pts = cell2mat(C);
      obj_data{i}.numPts = size(obj_data{i}.pts,1);
    end  
  end

  system('rm temp.pcd');

  % Save our object data which can be loaded in later by init_posedist
  save('obj_data.mat', 'obj_data');

end

% Return surface area in units corresponding to units in stl file
function surfaceArea = getSurfaceAreaFromSTL(stlFileName)

  surfaceArea = 100;

end

