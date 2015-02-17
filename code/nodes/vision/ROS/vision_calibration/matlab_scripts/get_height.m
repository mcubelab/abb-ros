function z = get_height(obj_points, robot_poses)
HOLE_LOCATIONS = ...
    [-0.046, -0.046, 0.085;
    -0.046, 0.046, 0.085;
    0.0, 0.0, 0.085];

dz = -0.004:0.0005:0.02;

N=length(dz);

inlier_list = zeros(N,1);
t_list = zeros(3,N);
q_list = zeros(4,N);

for i=1:N
    i
    M=length(robot_poses);
    world_points = cell(M,1);
    for j=1:M
        H = toHomo(robot_poses(j,:));
        temp = H * [bsxfun(@plus,HOLE_LOCATIONS',[0;0;dz(i)]); ones(1,size(HOLE_LOCATIONS,1))];
        world_points{j} = temp(1:3,:)';
    end
    
    [t_c, q_c, num_inliers] = get_transform(obj_points, world_points);
    inlier_list(i) = num_inliers;
    t_list(:,i) = t_c;
    q_list(:,i) = q_c;
end

figure(1);clf;
plot(dz+0.085, inlier_list');
[~,best_idx] = max(inlier_list);
z=dz(best_idx)+0.085;
t_list
q_list
[t_list(:,best_idx);q_list(:,best_idx)]
end