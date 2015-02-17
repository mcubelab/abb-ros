% This function randomly generates points on the surface of all objects in
% our object library, and saves these points, along with the object
% surface area. These values will be used later to enable us to compute
% distances between poses of objects
function init_posedist()
  global obj_data;

  dat = load('obj_data.mat');

  obj_data = dat.obj_data;

end
