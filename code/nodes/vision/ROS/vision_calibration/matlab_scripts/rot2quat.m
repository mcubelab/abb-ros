% Convert rotation matrix into quaternion
function q=rot2quat(R)

q2 = zeros(4,1);

q2(1) = 1/4*(1+R(1,1)+R(2,2)+R(3,3));
q2(2) = 1/4*(1+R(1,1)-R(2,2)-R(3,3));
q2(3) = 1/4*(1-R(1,1)+R(2,2)-R(3,3));
q2(4) = 1/4*(1-R(1,1)-R(2,2)+R(3,3));

[~,i] = max(q2);

switch i
    case 1
        q0 = sqrt(q2(1));
        q1 = 1/(4*q0)*(R(3,2)-R(2,3));
        q2 = 1/(4*q0)*(R(1,3)-R(3,1));
        q3 = 1/(4*q0)*(R(2,1)-R(1,2));
    case 2
        q1 = sqrt(q2(2));
        q0 = 1/(4*q1)*(R(3,2)-R(2,3));
        q2 = 1/(4*q1)*(R(1,2)+R(2,1)); 
        q3 = 1/(4*q1)*(R(1,3)+R(3,1)); 
    case 3
        q2 = sqrt(q2(3));
        q0 = 1/(4*q2)*(R(1,3)-R(3,1));
        q1 = 1/(4*q2)*(R(1,2)+R(2,1));
        q3 = 1/(4*q2)*(R(2,3)+R(3,2));
    case 4
        q3 = sqrt(q2(4));
        q0 = 1/(4*q3)*(R(2,1)-R(1,2));
        q1 = 1/(4*q3)*(R(1,3)+R(3,1));
        q2 = 1/(4*q3)*(R(2,3)+R(3,2));
    otherwise
        error('something went wrong');
end

q = [q0;q1;q2;q3];

end