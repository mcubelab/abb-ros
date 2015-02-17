% Takes in a homogeneous matrix and converts it into a translation and a
% quaternion
function [t, q] = toTransQuat(H)

q = rot2quat(H(1:3, 1:3));
t = H(1:3, 4);

end