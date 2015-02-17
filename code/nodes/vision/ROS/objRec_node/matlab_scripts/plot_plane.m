function plot_plane(points, plane, fidx)%, i,all_points)

% Plot our new plane with the data
figure(fidx);%clf;
hold on;
x_min = min(points(:,1));
x_max = max(points(:,1));
y_min = min(points(:,2));
y_max = max(points(:,2));
[x y] = meshgrid(x_min:0.01:x_max+0.01, y_min:0.01:y_max+0.01);
z = (-plane(4) - plane(1)*x - plane(2)*y)/plane(3);
surf(x,y,z);

plot3(points(:,1), points(:,2), points(:,3),'o')

% cc=hsv(6);
% plot3(all_points(:,1), all_points(:,2), all_points(:,3),'o','color',cc(i+1,:))
% plot3(points(:,1), points(:,2), points(:,3),'o','color',cc(i,:))


end