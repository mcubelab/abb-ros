% Given lists of poses p and q, compute the "squared" distance between them 

% p - N x 7 matrix of poses (x, y, z, q0, qx, qy, qz)
% q - N x 7 matrix of poses (x, y, z, q0, qx, qy, qz)
% objNum - a number representing which object we are trying to compute the
% pose distance between. Note that the size of the object will end up
% determining how we trade off orientation and position. Note that this
% number is 0-indexed and corresponds to the list in RecObj::Type found in
% objRec_comm.h

function D = posedist2(t1, t2, objNum)

global obj_data;

n = obj_data{objNum+1}.numPts;
pts = obj_data{objNum+1}.pts;
sa = obj_data{objNum+1}.surfaceArea;

% Find the number of poses we're trying to compare
m = size(t1,1);
% For each pose, let's make sure we take any symmetries into account first
for i=1:m
  t1(i,:) = getClosestTransf(t1(i,:), t2(i,4:7), objNum);
end


% We will create a stack of homogeneous transforms that we can then
% multiply our points by
T_1 = zeros(4*m, 4);
T_2 = zeros(4*m, 4);

% Convert each row of our input into a homogeneous transform and add it to
% our stack
for i=1:m
    T_1((i-1)*4+1:i*4,:) = toHomo(t1(i,:));
    T_2((i-1)*4+1:i*4,:) = toHomo(t2(i,:));
end

% Our computation will be p'*(T_1 - T_2)' * (T_1 - T_2) * p
middle = (T_1 - T_2)';

% Let's compute half of the sum, and then square it to get the other half.
% Finally, let's sum over all of the points
temp = ([pts ones(n,1)] * middle);
d = sum(temp.^2);


% At this point, we have a 1 by 4*N array which represents the following:
% [sum(dx1.^2) sum(dy1.^2) sum(dz1.^2) n*middle(4,4) sum(dx1.^2) ...]
%
% We need to add each block of 4 columns together, and then subtract out
% the garbage to get our final squared distance. To do this, we'll take the
% cumulative sum of the array, and then look at every 4th point to get the
% sum of each block
ds = cumsum(d);

diff = ds(4:4:end) - [0,ds(4:4:end-1)] - middle(4,4)*n;

% Finally, let's normalize our squared difference
D = diff / (n*sa);
toc
end

% Function to convert a 1x7 or 7x1 array [x y z q0 qx qy qz] into a
% homogeneous transform
function H = toHomo(t)
trans = t(1:3);
quat = t(4:7);

H = [[quat2rot(quat) trans(:)]; 0 0 0 1];
end

% Function to convert a quaternion into a rotation matrix
function R = quat2rot(q)

q = q/norm(q);

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

R =[q0^2+q1^2-q2^2-q3^2,    2*(q1*q2-q0*q3),        2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3),        q0^2-q1^2+q2^2-q3^2,    2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2),        2*(q2*q3+q0*q1),        q0^2-q1^2-q2^2+q3^2];
end




