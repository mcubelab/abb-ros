function cfg_data = read_config_file(filename)

fid = fopen(filename);

have_name = 0;
have_pcdfile = 0;

cfg_data.symType = -1;
cfg_data.guessType = -1;

while 1
    tline = fgetl(fid);
    if (~ischar(tline))
        break
    end
    
    % If this is an empty line, a comment, or a space, ignore it
    if (isempty(tline) || tline(1) == '#' || isspace(tline(1)))
        continue;
    % The name of the object is the first real line in the file
    elseif (have_name == 0)
        cfg_data.objName = tline;
        fprintf('Object Name: %s\n', cfg_data.objName);
        have_name = 1;
        continue;
    % The pcd file name is the second line in the file
    elseif (have_pcdfile == 0)
        cfg_data.pcdFile = tline;
        fprintf('PCD File: %s\n', cfg_data.pcdFile);
        have_pcdfile = 1;
        continue;
    % If we ever see an exclamation mark, we will stop reading the file
    elseif (tline(1) == '!')
        fprintf('End of file reached.\n');
        break;
    % If a '$' is found, this means a symmetry about the object is about to
    % be declared
    elseif (tline(1) == '$')
        % The number following this character is the type of symmetry
        cfg_data.symType = str2double(tline(2:end));
        switch(cfg_data.symType)
            case 0
                % If it's 0, there is no symmetry
                fprintf('No Symmetry\n');
            case 1
                % If it's 1, then it's an ordered symmetry, which means we
                % can have as many symmetry axes as we want, and we specify
                % the order N of the symmetry, which means the object is
                % symmetric every 360/N degrees around the axis
                fprintf('Order Symmetry\n');
                
                % The next line is the number of ordered symmetries
                tline = fgetl(fid);
                cfg_data.numSyms = str2double(tline);
                fprintf('Number of Symmetries: %d\n', cfg_data.numSyms);
                
                % Each line after that actually specifies the symmetry,
                % first the axis, then the point going through the axis,
                % and finally the order of this particular symmetry.
                cfg_data.symAxes = zeros(cfg_data.numSyms, 3);
                cfg_data.symPoints = zeros(cfg_data.numSyms, 3);
                cfg_data.symOrders = zeros(cfg_data.numSyms, 1);
                for i=1:cfg_data.numSyms
                    tline = fgetl(fid);
                    C = textscan(tline, '%f %f %f %f %f %f %d');
                    cfg_data.symAxes(i,:) = cell2mat(C(1:3));
                    cfg_data.symAxes(i,:) = cfg_data.symAxes(i,:) / norm(cfg_data.symAxes(i,:));
                    cfg_data.symPoints(i,:) = cell2mat(C(4:6));
                    cfg_data.symOrders(i) = C{7};
                end
                
                cfg_data.symAxes
                cfg_data.symPoints
                cfg_data.symOrders
                
            case 2
                % If it's 2, then it's a cylindrical symmetry, meaning the
                % object is symmetric regardless of how many degrees you 
                % rotate it around an axis
                fprintf('Cylindrical Symmetry\n');
                
                % The next line contains the axis, followed by the point
                % that the axis goes through
                cfg_data.numSyms = 1;
                cfg_data.symAxes = zeros(cfg_data.numSyms, 3);
                cfg_data.symPoints = zeros(cfg_data.numSyms, 3);
                cfg_data.symOrders = -1;
                tline = fgetl(fid);
                C = textscan(tline, '%f %f %f %f %f %f %f');
                cfg_data.symAxes(1,:) = cell2mat(C(1:3));
                cfg_data.symAxes(1,:) = cfg_data.symAxes(1,:) / norm(cfg_data.symAxes(1,:));
                cfg_data.symPoints(1,:) = cell2mat(C(4:6));
                
                cfg_data.cylOrderSym = C{7};
                if (cfg_data.cylOrderSym == 1)
                    fprintf('This object also has an order symmetry\n');
                end
                
                cfg_data.symAxes
                cfg_data.symPoints
                cfg_data.symOrders
            case 3
                % If it's 3, then it's spherical symmetry, which can only
                % be accomplished with a sphere.
                fprintf('Spherical Symmetry\n');
                
                % The next line contains the center of the sphere relative
                % to the frame of the object
                cfg_data.numSyms = 1;
                cfg_data.symAxes = zeros(cfg_data.numSyms, 3);
                cfg_data.symPoints = zeros(cfg_data.numSyms, 3);
                cfg_data.symOrders = -2;
                tline = fgetl(fid);
                C = textscan(tline, '%f %f %f');
                cfg_data.symPoints(1,:) = cell2mat(C(1:3));
                
                cfg_data.symAxes
                cfg_data.symPoints
                cfg_data.symOrders
            otherwise
                fprintf('Unknown Symmetry: %d\n', cfg_data.symType);
        end
        
    % Finally, if a '&' is found, this means that a guess function for this
    % object is being declared
    elseif (tline(1) == '&')
        % The number after the character specifies the type of guess
        cfg_data.guessType = str2double(tline(2:end));
        switch(cfg_data.guessType)
            case 0
                % If it's a 0, then we will not guess
                fprintf('No Guess\n');
            case 1
                % If it's a 1, we will fit an ellipsoid to the data and
                % object, and try and match it that way
                fprintf('Guess using an ellipsoid\n');
            case 2
                % If it's a 2, we will find planes in the data, and match
                % them with the planes on the object
                fprintf('Guess using planes\n');
                % The next line of the data contains a matlab definition of
                % all of the planes (Each row is a plane, [A B C D] which
                % represents the plane Ax+By+Cz+D=0
                strplanes = fgetl(fid);
                % The line after that contains a matlab definition of all
                % of the intersection points of the planes by specifying
                % the indices that intersect. Each row contains 3 indices,
                % representing the 3 planes that intersect to form a point
                strints = fgetl(fid);
                disp(strplanes);
                disp(strints);
                % Save the planes and intersections
                cfg_data.guessPlanes = eval(strplanes);
                cfg_data.guessInts = eval(strints);
            case 3
                % If it is a 3, then we will be using sample consensus
                % initial alignment with fast point feature histograms. 
                fprintf('Guess using features\n');
                % The next line contains the 5 features we care about
                tline = fgetl(fid);
                C = textscan(tline, '%f %f %d %f %d');
                fprintf('normal_rad: %f\nfeature_rad: %f\nnum_samples: %d\nmin_sample_dist: %f\nk_corr: %d\n', C{:});
            otherwise
                fprintf('Unknown Guess Type: %d\n', cfg_data.guessType);
        end
    end 
end
fclose(fid);

end