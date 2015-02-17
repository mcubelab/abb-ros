function normals = getNormalsFromPointCloud(num_faces, points)
    num_iter = 5000;
    n = size(points,1);
    epsilon = 0.0035;
    min_fit = floor(n/(num_faces+5));
    min_points = 100;
    
    all_points = points;
    
    normals = [];
    
    % iterate over each face of the object
    for i=1:num_faces,
        n = size(points,1);
        inliers = [];
        outliers = [];       
        best_error = Inf;
        best_plane = [];
        
        if size(points,1) < min_points
            return
        end
        
        for j=1:num_iter,
            % choose 3 random points to define a plane
            ind = ceil(rand(1,3)*n);
            a = [points(ind(1),:); points(ind(2),:); points(ind(3),:)];
            f = ones(3,1);
            plane = pinv(a)*f;
            
            % find inliers/outliers for this plane

            d = abs(plane(1)*points(:,1) + plane(2)*points(:,2) + ...
                    plane(3)*points(:,3) - 1)/norm(plane);
            maybe_inliers = points((d<epsilon),:);
            maybe_outliers = points((d>=epsilon),:);
            error = mean(d(d<epsilon));
            
%             if (size(maybe_inliers,1) > min_fit)
%                 plane = pinv(maybe_inliers)*ones(size(maybe_inliers,1),1);
%                 d = abs(plane(1)*points(:,1) + plane(2)*points(:,2) + ...
%                     plane(3)*points(:,3) - 1)/norm(plane);
%                 maybe_inliers = points((d<epsilon),:);
%                 maybe_outliers = points((d>=epsilon),:);
%                 error = mean(d(d<epsilon));
%                 
%                 if (error < best_error)
%                     inliers = maybe_inliers;
%                     outliers = maybe_outliers;
%                     best_error = error;
%                     best_plane = plane;
%                 end
%             end
                
            % test if best plane so far (using # of inliers)
            %if (size(maybe_inliers,1) > min_fit) && ...
            %        (size(maybe_inliers,1) > size(inliers,1))
            if (error < best_error) && (size(maybe_inliers,1) > min_fit),
                inliers = maybe_inliers;
                outliers = maybe_outliers;
                best_error = error;
                best_plane = plane;
                max(d(d<epsilon));
%                 figure(2+i);
%                 hist(d(d<epsilon),100);
            end

        end

        i
        size(inliers)
        
        % refit plane to data
        best_plane = pinv(inliers)*ones(size(inliers,1),1);
        
        % plot the plane with its points
        
        plot_plane(inliers, best_plane, i+1, 1, all_points);
        plot_plane(inliers, best_plane, 1, i,all_points);
        
        
%         figure(i); clf;
%         scatter3(inliers(:,1),inliers(:,2),inliers(:,3))
%         maxs = max(inliers);
%         mins = mins(inliers);
%         [xx,yy] = meshgrid([mins(1):1:maxs(1)],[mins(2):1:maxs(2)]);
%         z = 
        
        normals = [normals best_plane];
        points = outliers;
    end

end