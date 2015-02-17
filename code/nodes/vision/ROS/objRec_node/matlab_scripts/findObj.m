function H = findObj(normals, intersections, fitted_normals)
% intersections = [1 3 5; 1 3 4; 1 4 5; 2 3 5; 2 3 4; 2 4 5]
% normals = [0 0 1 -1;0 0 -1 0;-1 0 0 0;0 -1 0 0;1 1 0 -1];
% %fitted_normals = [0 0 1 -1;0 0 -1 0;0 1 0 0;-1 0 0 0;1 -1 0 -1];
% %fitted_normals = [0 0 -1 0;0 1 0 0;-1 0 0 0;1 -1 0 -1];
% fitted_normals = [0 0 -1 0;1 -1 0 -1;0 1 0 0;-1 0 0 0];
% fitted_normals(:,4) = fitted_normals(:,4)-sum(fitted_normals(:,1:3),2);

normals = bsxfun(@rdivide, normals, sqrt(sum(normals(:,1:3).^2,2)));
fitted_normals = bsxfun(@rdivide, fitted_normals, sqrt(sum(fitted_normals(:,1:3).^2,2)));

% run RANSAC to find normals in the pointcloud
num_faces = size(normals,1);
%fitted_normals = getNormalsFromPointCloud(num_faces, points);
seen_faces = size(fitted_normals,1);
if seen_faces < 3,
    'ERROR: not enough planes found in the camera point cloud'
end

best_fit = [1:num_faces];
best_r = eye(3);
best_error = Inf;

p = nchoosek(1:num_faces,seen_faces);
combos = [];
for i=1:size(p,1)
    combos = [combos; perms(p(i,:))];
end
for i=1:size(combos,1)
    
    pts_obj_ind = combos(i,:);%nchoosek(1:num_faces,seen_faces);
    pts_cam_ind = [1:seen_faces];
    %pts_obj_ind = [2,3,4,5];
    
    pts_cam = fitted_normals(pts_cam_ind,1:3)';
    pts_obj = normals(pts_obj_ind,1:3)';
    
    % find rotation
    [U, ~, V] = svd(pts_obj * pts_cam');
    R = V * [1 0 0; 0 1 0; 0 0 det(V*U')] * U';
    %new_normals = (R*normals(pts_obj_ind,1:3)')'
    %new_normals = [(R*normals(:,1:3)')' normals(:,4)];
    normals;
    new_normals = (R*normals(:,1:3)')';
    fitted_normals;
    new_seen_normals = new_normals(pts_obj_ind,:);
    
    new_seen_normals-fitted_normals(:,1:3);
    error = sum(sqrt(sum((new_seen_normals-fitted_normals(:,1:3)).^2,2)));
    
    %if all(pts_obj_ind == [4 2 5 3]) || all(pts_obj_ind == [3 1 5 4])
%     if all(pts_obj_ind == [4 5 3]) || all(pts_obj_ind == [3 5 4])
%         pts_obj_ind
%         error
%         
%     end
    
    if error < best_error
        best_error = error;
        best_r = R;
        best_fit = pts_obj_ind;
    end
    
end

best_error;
best_r;
best_fit
new_seen_normals;
fitted_normals;

seen_intersections = [];
for i=1:size(intersections,1)
    new_int = [];
    for j=1:size(intersections(i,:),2)
        index = find(intersections(i,j)==best_fit);
        if ~isempty(index)
            new_int = [new_int index];
        else
            new_int = [];
            break
        end
    end
    seen_intersections = [seen_intersections;new_int];
end
            
% find the intersection points of the object
obj_intersections = zeros(size(seen_intersections));
for i=1:size(seen_intersections,1),
    planes = normals(best_fit(seen_intersections(i,:)),:);
    obj_intersections(i,:) = -inv(planes(:,1:3))*planes(:,4);
end    
    


% % find the intersection points of the object
% obj_intersections = zeros(size(seen_intersections));
% for i=1:size(seen_intersections,1),
%     planes = normals(seen_intersections(i,:),:);
%     obj_intersections(i,:) = -inv(planes(:,1:3))*planes(:,4);
% end

% rotate the intersection pts
rot_intersections = (best_r*obj_intersections')';

% % find seen intersections
% not_seen = setdiff([1:num_faces],best_fit);
% int_not_seen = zeros(size(intersections,1),1);
% for i=1:length(not_seen)
%     int_not_seen = int_not_seen  + (sum((intersections==not_seen(i)),2)==0);
% end
% seen_intersections = intersections(logical(int_not_seen),:);

%padded_fitted_normals = zeros(size(normals));
%padded_fitted_normals(best_fit,:) = fitted_normals;
%seen_intersection_pts = rot_intersections(logical(int_not_seen),:);
t = zeros(3,1);
for j=1:size(seen_intersections,1)
    current_intersection = seen_intersections(j,:)
    
    fint = fitted_normals(seen_intersections(j,:),:)
    current_intersection_pts = rot_intersections(j,:)
    fint_pt = -inv(fint(:,1:3))*fint(:,4);
    %t = t + (fint_pt - nint_pt);
    %seen_rot_intersections = rot_intersections(seen_intersections(j,:),:);
    part_t = (fint_pt - rot_intersections(j,:)')
    t = t + (fint_pt - rot_intersections(j,:)');
end
size(seen_intersections,1)
t = t/size(seen_intersections,1);

H = [best_r t; 0 0 0 1];

end



% function error = findT(R,t,fitted_normals,normals)
%         T = [eye(3) t; 0 0 0 1];
%         R = [R zeros(3,1);0 0 0 1];
%         H = T*inv(R)*inv(T);
%         new_normals = (normals*H);
%         new_normals = bsxfun(@rdivide,new_normals,new_normals(:,4))
%     
%         error = sum(sqrt(sum((new_normals-fitted_normals).^2,2)))
%  
% end

        
%         % find translation
%         % get intersections
%         for j=1:size(intersections,1),
%             int = normals(intersections(j,:));
%             int_pt = inv(int(:,1:3))*int(:,4);
%             
%             
%         end
%         t = mean(points,2)
%         T = [eye(3) t; 0 0 0 1];
%         H = T*inv(R)*inv(T);
%         new_normals = (normals*H);
%         new_normals = bsxfun(@rdivide,new_normals,new_normals(:,4))
%         error = sum(sqrt(sum((new_normals-fitted_normals).^2,2)))
        
%         normals(:,4) = normals(:,4)-sum(normals(:,1:3),2);
%         fitted_normals(:,4) = fitted_normals(:,4)-sum(fitted_normals(:,1:3),2);
%         normals = bsxfun(@rdivide,normals,normals(:,4));
%         fitted_normals = bsxfun(@rdivide,fitted_normals,fitted_normals(:,4))
%         t0 = [0;0;0];        
%         [t,ferror] = fminunc(@(t)findT(R,t,fitted_normals,normals),t0)
        
%         normals
%         new_normals
%         fitted_normals
%    end

% 	normals = [0 0 1 -1;0 0 -1 0;-1 0 0 0;0 -1 0 0;1 1 0 -1];
% 	fitted_normals = [0 0 1 -1;0 0 -1 0;0 1 0 0;-1 0 0 0;1 -1 0 -1];
%     % perturb so no d=0
%     normals(:,4) = normals(:,4)-sum(normals(:,1:3),2);
%     fitted_normals(:,4) = fitted_normals(:,4)-sum(fitted_normals(:,1:3),2);
%     % normalize by d
%     normals = bsxfun(@rdivide,normals,normals(:,4));
%     fitted_normals = bsxfun(@rdivide,fitted_normals,fitted_normals(:,4));
%     % plotPlanes(normals, fitted_normals)


%         
%                 normals(:,4) = normals(:,4)-sum(normals(:,1:3),2);
%         fitted_normals(:,4) = fitted_normals(:,4)-sum(fitted_normals(:,1:3),2);
%         % normalize by d
%         normals = bsxfun(@rdivide,normals,normals(:,4));
%         fitted_normals = bsxfun(@rdivide,fitted_normals,fitted_normals(:,4));
%         pts_cam_ind = [3,4,5];
%         pts_obj_ind = [3,4,5];
%         pts_cam = fitted_normals(pts_cam_ind,1:3)';
%         pts_obj = normals(pts_obj_ind,1:3)';
%         
%         tt = zeros(3);
%         ts = zeros(3,1);
%         for j=1:size(pts_obj,2)
%             opt = pts_obj(:,j);
%             cpt = pts_cam(:,j);
%             tt = tt + opt*opt';
%             
%             ts = ts + ((opt'*R*cpt)/(cpt'*cpt))*opt;
%         end
%         
%         ts = ts*2;
%         T = pinv(tt)*ts
%  
        
        
%         % find centroids
%         m_pc = mean(pts_cam,2);
%         m_po = mean(pts_obj,2);
%         
%         % transform all the pts relativ to centroid
%         trans_pc = bsxfun(@minus, pts_cam, m_pc);
%         trans_po = bsxfun(@minus, pts_obj, m_po);
        
%         % find the svd of the correlation matrix
%         %[U, ~, V] = svd(trans_po * trans_pc');
%         [U, ~, V] = svd(trans_po * trans_pc');
%         
%         % Compute the rotation matrix, making sure that the orthogonal matrix is a
%         % rotation and not a reflection
%         R = V * [1 0 0; 0 1 0; 0 0 det(V*U')] * U';
%         % Compute our translation matrix now that we know the optimal rotation
%         t = m_pc - R*m_po;
%         
%         H = [[R t];0 0 0 1]
%         T = [eye(3) ones(3,1); 0 0 0 1];
%         R = [0 1 0 0;-1 0 0 0;0 0 1 0;0 0 0 1];
%         H = T*inv(R)*inv(T)
%         % calculate error of this transform 
%         %new_normals = (H*normals')'
%         new_normals = (normals*H);
%         new_normals = bsxfun(@rdivide,new_normals,new_normals(:,4))
%%%%%%%%%%%%%%%%

%         figure
%         hold on
%         for i=1:2%size(normals,1)
%             [xx,yy] = ndgrid(1:2,1:2);
%             normal = new_normals(i,:);
%             z = (-normal(1)*xx - normal(2)*yy - normal(4))/normal(3);
%             surf(xx,yy,z)
%         end
%         for i=4:4
%             [yy,zz] = ndgrid(1:3,1:3);
%             normal = new_normals(i,:);
%             x = (-normal(2)*yy - normal(3)*zz - normal(4))/normal(1);
%             surf(x,yy,zz)
%         end
%         
%         for i=[3 5]
%             [xx,zz] = ndgrid(1:2,1:2);
%             normal = new_normals(i,:);
%             y = (-normal(1)*xx - normal(3)*zz - normal(4))/normal(2);
%             surf(xx,y,zz)
%         end
        
        
        %normals
        %fitted_normals
        %error = 
%    end
    
    
    
%     combos = [combntns(1:num_faces,2); [1:num_faces;1:num_faces]'];
%     for i=1:1%size(combos,1),
%         % find a rotation
%         i=15
%         
%         ind = randperm(size(normals,1));
%         %H*po = pc
%         pc = fitted_normals';
%         po = normals';
%         H = po'*pc;
%         error = 
%         
%         v1 = normals(combos(i,1),:)'
%         v2 = fitted_normals(combos(i,2),:)'
%         angle = acos(v1'*v2)
%         axis = cross(v1,v2)/norm(cross(v1,v2))
%         axis_skewed = [0 -axis(3) axis(2) ; axis(3) 0 -axis(1) ; -axis(2) axis(1) 0];
%         r = eye(3) + sin(angle)*axis_skewed + (1-cos(angle))*axis_skewed*axis_skewed;
%         %r = vrrotvec(normals(1,:),fitted_normals(i,:));
%         %m = vrrotvec2mat(r);
%         
%         for j=1:1%factorial(num_faces)
%             %ind = randperm(size(normals,1));
%             ind = [1:num_faces];
%             
%             % see if it fits
%             pn = normals(ind,:);
%             rot_n = (r*pn')';
%             fitted_normals
%             rot_n
%             error = sum(cross(fitted_normals,rot_n),2);
%             
%             if error < best_error
%                 best_error = error;
%                 best_r = r;
%                 best_fit = ind;
%             end
%         end
%     end
%        
%     best_error
%     best_r
%     best_fit

%     
%     % find correspondences between normals & fitted_normals
%     % assumes correspondence possible
%     error = Inf;
%     epsilon = 0.001;
%     while error > epsilon,
%         % choose a random assignment of correspondences
%         ind = randperm(size(normals,1));
%         pn = normals(ind,:);
%         error = sum(cross(fitted_normals,pn),2);
%     end
%     
%     % find transform 
%     for i=1:num_faces,
%         
%         norm_comb = [normals(i:end,:);normals(1:(i-1),:)];
%         h = [eye(3) zeros(3,1)];%fit_n(1) -> n(1)
%         sum(cross())
%         
%         [D,I] = min(sum(cross(fit_n,norm_comb),2));
%     
%     end
%    
