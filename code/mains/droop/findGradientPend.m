function [ newt ] = findGradientPend( l,t )

    % find the trajectory as a pendulum
    m = size(t,2);
    lift = find(t(2,:)==l);
    lift = lift(1);

    if (lift==m)
        newt = t;
        return;
    end
        
    traj = t(:,lift:m)
    n = size(traj,2);

    max_score = ff(l,traj);
    newt = traj;
    grad = zeros(size(traj));
    n = length(traj);
    l = 5;
    a = 0.1;
    
    for i=1:n
        dirs = nchoosek([2:n],i);
        
        for j=1:size(dirs,1)
            dir = dirs(j,:);
            nn = length(dir);
            
            % find all the combos of path adjustments
            discretization = 9;%17; % choose something divisible by 4 +1
            tmps = repmat(traj,discretization^nn,1);
            time = linspace(0,2*pi,discretization);
            
            for k=1:length(dir)
                pt = traj(:,dir(k));
                x = pt(1) + a*cos(time);
                y = pt(2) + a*sin(time);
                
                tmps(1:2:end,dir(k)) = repmat(kron(x,ones(1,discretization^(k-1))),1,(discretization^(nn-k)));
                tmps(2:2:end,dir(k)) = repmat(kron(y,ones(1,discretization^(k-1))),1,(discretization^(nn-k)));
            end
            
            % find the score for each of these
            for k=1:2:size(tmps,1),
                score = ff(l,tmps(k:k+1,:));
                
                if score>max_score
                    max_score = score
                    newt = tmps(k:k+1,:)
                end 
            end
        end
    end
end

