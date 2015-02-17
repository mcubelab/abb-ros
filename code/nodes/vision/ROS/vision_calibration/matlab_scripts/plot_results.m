function plot_results(point_clouds, robot_poses, Tc, object_points)

M = length(point_clouds);

h1=figure(2);clf;set(h1, 'Visible', 'off');
h2=figure(3);clf;set(h2, 'Visible', 'off');
h3=figure(4);clf;set(h3, 'Visible', 'off');
h4=figure(5);clf;set(h4, 'Visible', 'off');
NUM_PTS = 500;

for i=1:M
    points = point_clouds{i};
    idxs = randperm(size(points,1));

    oc = toHomo(Tc) * [object_points{i}'; ones(1,size(object_points{i},1))];
    %roc = inv(toHomo(To)) * inv(toHomo(robot_poses(i,:))) * oc;
    roc = inv(toHomo(robot_poses(i,:))) * oc;
    
    wc = toHomo(Tc) * [points(idxs(1:NUM_PTS),:)'; ones(1,NUM_PTS)];
    %rc = inv(toHomo(To)) * inv(toHomo(robot_poses(i,:))) * wc;
    rc = inv(toHomo(robot_poses(i,:))) * wc;

    
    set(0, 'currentfigure', h1);hold on;
    plot3(wc(1,:),wc(2,:), wc(3,:), '.');
    set(0, 'currentfigure', h2);hold on;
    plot3(rc(1,:),rc(2,:), rc(3,:), '.');
    set(0, 'currentfigure', h3);hold on;
    plot3([oc(1,:) oc(1,1)], [oc(2,:) oc(2,1)], [oc(3,:) oc(3,1)]);
    set(0, 'currentfigure', h4);hold on;
    plot3([roc(1,:) roc(1,1)], [roc(2,:) roc(2,1)], [roc(3,:) roc(3,1)]);
end
set(h1, 'Visible', 'on');
set(h2, 'Visible', 'on');
set(h3, 'Visible', 'on');
set(h4, 'Visible', 'on');
drawnow;
end