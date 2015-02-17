function [transforms, M, valid_transforms] = findObjMulti(normals, intersections, fitted_normals)
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

p = nchoosek(1:num_faces,seen_faces);
combos = [];
for i=1:size(p,1)
    combos = [combos; perms(p(i,:))];
end

M = size(combos,1);

R_list = cell(M,1);
errors = zeros(M,1);

for i=1:size(combos,1)
    
    pts_obj_ind = combos(i,:);%nchoosek(1:num_faces,seen_faces);
    pts_cam_ind = [1:seen_faces];
    %pts_obj_ind = [2,3,4,5];
    
    pts_cam = fitted_normals(pts_cam_ind,1:3)';
    pts_obj = normals(pts_obj_ind,1:3)';
    
    % find rotation
    [U, ~, V] = svd(pts_obj * pts_cam');
    R = V * [1 0 0; 0 1 0; 0 0 det(V*U')] * U';

    new_normals = (R*normals(:,1:3)')';
    new_seen_normals = new_normals(pts_obj_ind,:);
    
    new_seen_normals-fitted_normals(:,1:3);
    error = sum(sqrt(sum((new_seen_normals-fitted_normals(:,1:3)).^2,2)));

    R_list{i} = R;
    errors(i) = error;
end

% Look at our list of errors, and find 
[se, e_idx] = sort(errors);
diff = se(2:end) - se(1:end-1);
cdiff = cumsum(diff);
cutoff = find(cdiff > 0.02, 1);
best_idxs = e_idx(1:cutoff);
errors(best_idxs)

combos(best_idxs,:)


M = length(best_idxs);

transforms = cell(M,1);
valid_transforms = ones(M,1);

for kk=1:M

    best_fit = combos(best_idxs(kk),:)

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

    if isempty(seen_intersections)
        'ERROR: No intersections seen for this set of planes'
        valid_transforms(kk) = 0;
        
        H = [R_list{best_idxs(kk)} zeros(3,1); 0 0 0 1];
        transforms{kk} = H;
        
        continue
    end

    
    % find the intersection points of the object
    obj_intersections = zeros(size(seen_intersections));
    for i=1:size(seen_intersections,1),
        planes = normals(best_fit(seen_intersections(i,:)),:);
        obj_intersections(i,:) = -inv(planes(:,1:3))*planes(:,4);
    end    


    % rotate the intersection pts
    rot_intersections = (R_list{best_idxs(kk)}*obj_intersections')';


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

    H = [R_list{best_idxs(kk)} t; 0 0 0 1];

    transforms{kk} = H;

end

M

end


