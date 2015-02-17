function [ score ] = ff( l, t )

    global n;
    global traj;
    global dtraj;
    global ddtraj;

    [~,ind] = max(t(2,:));   
    %l = 5;%t(2,ind);
    m = size(t,2);
    lift = find(t(2,:)==l);
    lift = lift(1);
    
    % initialize
    s = [[l;0] zeros(2,m-1)];
    s_vel = zeros(1,m); % only vel in x
    p_vel = zeros(1,m);
    angular_vel = zeros(1,m);
    theta = zeros(1,lift);
    bsi=0;
    ii =0;
    
    for i=1:lift
        s(:,i) = model(l,t(:,i));
        if (i>ind)
            theta(i) = -acos((t(2,i)/l));
        else
            theta(i) = acos((t(2,i)/l));
        end
    end
    if (lift~=1)
        dtraj_init = t(:,lift-1)-t(:,lift);
        if (lift~=2)
            ddtraj_init = (t(:,lift-2)-t(:,lift-1)) - dtraj_init;
        else
            ddtraj_init = [0;0];
        end
    else
        dtraj_init = [0;0];
        ddtraj_init = [0;0];
    end
    angular_vel(2:lift) = theta(2:lift)-theta(1:lift-1);
        
    if (lift~=m)
        traj = t(:,lift:m);
        n = size(traj,2);
        traj(2,:) = traj(2,:)-l;
        dtraj = [dtraj_init traj(:,2:n)-traj(:,1:n-1)];
        ddtraj = [ddtraj_init dtraj(:,2:n)-dtraj(:,1:n-1)];
        ss = sim('sim_for_matlab');
        ta = ss.get('theta');
        dta = ss.get('dtheta');
        ddta = ss.get('ddtheta');   
        tx = ss.get('x');
        ty = ss.get('y');
        tx = tx(2:length(tx))
        ty = ty(2:length(ty))
        ta = ta(2:length(ta));
        dta = dta(2:length(dta));
        ddta = ddta(2:length(ddta));
                
        for ii=2:length(ta)%size(traj,2)
            %if ((traj(2,i)+l)/cos(ta(i))<=l)
            if ((ty(ii)+l)/cos(ta(ii))<l)
                break;
            end
        end
        ii;
        scatter(tx,ty);
        
        % check theta for monotonically decreasing
        back_swing = find(diff(ta)>0);
        bsi = back_swing(1);
        bsi;
        
        theta(lift:lift+(i-1)) = ta(1:i);
       	angular_vel(lift:lift+(i-1)) = dta(1:i);
        
    end
    
    s_vel(2:m) = s(1,2:m) - s(1,1:m-1);
    p_vel(2:m) = t(1,2:m) - t(1,1:m-1);
    
    pi = 3.14;
    goal = -pi/2;
    (theta(end))*(180/pi);
    end_theta = (goal-theta(end))*(180/pi)
    end_speed = angular_vel(end)*(180/pi)
    (ii-bsi)
    score = -abs(end_theta) - abs(end_speed) - 5*(ii-bsi);
    
    if sum(t(2,:)<0)
        score = -inf;
    end
    
    % work = sum ( force at fingertips X vel at fingertips )
    
end

