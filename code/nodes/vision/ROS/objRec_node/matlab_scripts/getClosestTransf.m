% This function returns the closest transform for a given object taking
% into account symmetry given an initial object pose (x,y,z,q0,qx,qy,qz)
% and a desired orientation, along with what object we're doing this with

% Note that the object number is base 0 and follows the order in
% RecObj::Type, which is defined in objRec_comm.h

function Tf = getClosestTransf(Ti, quat, objNum)
  global obj_data;

  obj_cfg = obj_data{objNum+1};

  switch(obj_cfg.symType)

    % No Symmetry
    case 0
      % Can't adjust the pose even if we wanted to.
      Tf = Ti;

    % Order Symmetry
    case 1
      % For order symmetry, we'll just go through all of our pre-computed
      % symmetric transforms, and return the one that gets us closest to
      % our desired orientation
      orig = toHomo(Ti);
      bestDist = realmax;
      bestTransf = orig;
      for i=1:obj_cfg.numSymTransf
        tempTransf = orig * obj_cfg.symTransf{i};
        dist = quatDist(rot2quat(tempTransf(1:3,1:3)), quat);
        if (dist < bestDist)
          bestTransf = tempTransf;
          bestDist = dist;
        end
      end
      Tf = toPose(bestTransf);

    % Cylindrical Symmetry
    case 2
      % For cylindrical symmetry, we'll find the orientation closest to the
      % desired orientation by transforming into the cylinder frame and
      % finding the closest quaternion that represents a rotation soley
      % about the cylindrical axis

      % First let's compute our desired orientation in the cylinder frame
      r = quat2rot(quat);
      tr = obj_cfg.cylT(1:3, 1:3);
      orig = toHomo(Ti);

      desired_rot = inv(tr) * inv(orig(1:3,1:3)) * r * tr;
      desired_q = rot2quat(desired_rot);

      % Now that we have our desired orientation, let's compute the closest
      % possible orientation to that with the constraint that we can only
      % have rotations about the z-axis
      q02 = desired_q(1) * desired_q(1);
      qz2 = desired_q(4) * desired_q(4);

      best_q = [desired_q(1) / sqrt(q02 + qz2);
                0;
                0;
                desired_q(4) / sqrt(q02 + qz2)];

      % If our object also happens to have an order symmetry, (for example,
      % it's a cylinder), then let's transform to the other possible
      % cylinder frame and do the same thing
      if (obj_cfg.cylOrderSym == 1)
        % Transforming to the other cylinder frame (if the axis is the
        % z-axis), just represents a 180 degree rotation about the
        % x-axis
        rotx = [1 0 0; 0 -1 0; 0 0 -1];
        desired_rot2 = desired_rot * inv(rotx);
        desired_q2 = rot2quat(desired_rot2);
        
        q02 = desired_q2(1) * desired_q2(1);
        qz2 = desired_q2(4) * desired_q2(4);

        best_q2 = [desired_q2(1) / sqrt(q02 + qz2);
                   0;
                   0;
                   desired_q2(4) / sqrt(q02 + qz2)];

        % Compute how far we are in each case
        dist1 = quatDist(desired_q, best_q);
        dist2 = quatDist(desired_q2, best_q2);

        % If transforming to the other cylinder frame helped, then let's
        % save this as our best orientation
        if (dist2 < dist1)
          best_q = quatMult(best_q2, rot2quat(rotx));
        end
      end

      % Now convert our best quaternion back into our original frame
      Tf = toPose(orig * obj_cfg.cylT * toHomo([0; 0; 0; best_q]) * inv(obj_cfg.cylT));

    % Spherical Symmetry
    case 3
      % If we have spherical symmetry, we are guaranteed to achieve the
      % exact preferred orientation. Let's transform correctly, and we're
      % done.
      r = quat2rot(quat);
      orig = toHomo(Ti);
      desired_rot = r * inv(orig(1:3,1:3));

      sphereT = eye(4);
      sphereT(1:3,4) = obj_cfg.sphereCenter;

      Tf = toPose(orig * sphereT * [[desired_rot [0;0;0]]; 0 0 0 1] * inv(sphereT));
    
    otherwise
      warn('Unknown Symmetry. Simply returning original transform');
      Tf = Ti;
  end
end

function D = quatDist(q, p)
  D = min(norm(q-p), norm(q+p));
end

% T = (x,y,z,q0,qx,qy,qz)
function H = toHomo(T)

  H = [[quat2rot(T(4:7)) [T(1); T(2); T(3)]]; 0 0 0 1];

end

% T = (x,y,z,q0,qx,qy,qz)
function T = toPose(H)
  T = zeros(1,7);
  T(1:3) = H(1:3, 4);
  T(4:7) = rot2quat(H(1:3,1:3));
end
