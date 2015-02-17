%2011 October 20
%Collision checking demo

%Add to MATLAB path the folder where mexCollide binaries are
addpath('../bin');

%Load 3D models
[tri1,vert1] = mexCollideLoadModel('3Dmodels/tetrahedron.wrl');
[tri2,vert2] = mexCollideLoadModel('3Dmodels/robot.wrl');


%Location of the models
rot1 = [1 0 0; 0 1 0; 0 0 1];
rot2 = [1 0 0; 0 1 0; 0 0 1];
trans1 = [0; 0; 0.23];
trans2 = [0; 0; 0];


%Visualization
transformedVert1 = repmat(trans1,1,size(vert1,2)) + rot1*vert1;
trisurf(tri1',transformedVert1(1,:),transformedVert1(2,:), transformedVert1(3,:),'EdgeColor', [0.0 0.0 0.0], 'EdgeAlpha', 0.3, 'EdgeLighting', 'gouraud', 'FaceAlpha', 0.5, 'FaceColor', [0.2, 0.6, 0.4], 'FaceLighting', 'gouraud','SpecularStrength', 0.9, 'AmbientStrength', 0.7);
hold on;

transformedVert2 = repmat(trans2,1,size(vert2,2)) + rot2*vert2;
trisurf(tri2',transformedVert2(1,:),transformedVert2(2,:), transformedVert2(3,:),'EdgeColor', [0.0 0.0 0.0], 'EdgeAlpha', 0.3, 'EdgeLighting', 'gouraud', 'FaceAlpha', 0.5, 'FaceColor', [0.8, 0.2, 0.2], 'FaceLighting', 'gouraud','SpecularStrength', 0.9, 'AmbientStrength', 0.7);
axis equal;


%Collision detection
transformation1 = [rot1(:);trans1];
transformation2 = [rot2(:);trans2];
if(mexCollideDetect(tri1,vert1,tri2,vert2,transformation1, transformation2))
   title('Objects collide'); 
else
   title('Objects do not collide');
end


%Speed test

%1 - Check for collision 1000 independent times.
%for i=1:1000
%    mexCollideDetect(tri1,vert1,tri2,vert2,transformation1, transformation2);
%end

%2 - Check for collision 1000 combined times (much faster).
%transformationMult1 = repmat(transformation1,1,1000);
%transformationMult2 = repmat(transformation2,1,1000);
%out = mexCollideDetect(tri1,vert1,tri2,vert2,transformationMult1, transformationMult2);




