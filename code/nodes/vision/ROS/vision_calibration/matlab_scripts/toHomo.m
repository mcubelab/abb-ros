
% Takes in a 7 element array [x y z q0 qx qy qz] and converts it into a
% homogeneous transform
function H = toHomo(v)

H = [[quat2rot(v(4:7)) [v(1); v(2); v(3)]]; 0 0 0 1];

end