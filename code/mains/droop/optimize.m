function [ traj ] = optimize( init )
    
    avg = zeros(size(init));
    nn = 1;
    l=5;
    
    for k=1:nn

    traj = init;
    figure(1); hold on;
    plot(traj(1,:),traj(2,:),'b')
    iter = 5;%30;
    i =1;
    grad = inf;
    
    %for i=1:iter
    while ((sum(sum(grad~=0))~=0) && (i<iter))
        % find gradient direction
        gradt = findGradientPend(l,traj);
        grad = gradt - traj;

        % line search along that direction
        max_ln = lineSearch(l,grad,gradt);
        
        traj = max_ln;
        i = i+1;
        
        plot(traj(1,:),traj(2,:),'r');
        (sum(sum(grad~=0))~=0)
        (i<iter)
        (sum(sum(grad~=0))~=0) && (i<iter)
    end
    
    avg = (avg+traj)
    plot(avg(1,:),avg(2,:),'r');
    end
    
    avg = avg./nn;
    plot(avg(1,:),avg(2,:),'r');
end

