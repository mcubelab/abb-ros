%
% Finds the best rigid body transform between 2 corresponding sets of
% points in a least squares sense
%
% Input: P, a 3 x N matrix containing the initial points in 3d
%        Q, a 3 x N matrix containing the transformed points in 3d
%
% Output: R, the best 3 x 3 rotation matrix
%         t, the best 3 x 1 translation vector
%
% Note that we find the R and t that minimizes the following:
% 
% With no noise, Q = R * P + t. With noise, we minimize || R*P+t - Q ||ˆ2.
function [R, t] = findTransform(P, Q)
% Find the centroids of our data
p_bar = mean(P, 2);
q_bar = mean(Q, 2);
% Transform all of the points relative to their centroid
Pp = bsxfun(@minus, P, p_bar);
Qp = bsxfun(@minus, Q, q_bar);
% Find the svd of the correlation matrix
[U, ~, V] = svd(Pp * Qp');
% Compute our rotation matrix according to our algorithm
R = V * U';
% Compute our translation matrix according to our algorithm
t = q_bar - R * p_bar;
end