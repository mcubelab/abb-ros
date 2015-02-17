function [ grad ] = planar_gradient( traj, ind )

    % find the partial of the traj with respect to the pt at ind
    % compare the current point and all the points in a circle surrounding
    % it. if no variance, increase the size of the circle.

    % initialize
    pt = traj(:,ind);
    max_score = f(traj);
    grad = [0;0];
    found = false;
    iter = 1;
    max_iter = 5;
    a = 0.1;
    
    % look for gradient in progressively bigger circles
    while ((iter<max_iter) && (~found))
        % make a circle around the pt: (x-pt(1))^2 + (y-pt(2))^2 = 1
        t = linspace(0,2*pi,21);
        x = pt(1) + a*cos(t);
        y = pt(2) + a*sin(t);
    
        % look for gradient in this circle
        for i=1:length(t)
            temp_traj = traj;
            temp_traj(:,ind) = [x(i); y(i)];
            
            score = f(temp_traj);
            if (score>max_score)
                grad = [(x(i)-pt(1));(y(i)-pt(2))];
                max_score = score;
                found = true;
            end
        end
        
        a = a*5;
        iter=iter+1;
    end

end

