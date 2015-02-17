function [ s ] = model( l, p )

    s = zeros(2,1);
    
    size = 0.0;
    noise = rand()*(size*2)-size;
    s(1) = sqrt(l^2 - p(2)^2) + p(1)+noise;
    
    if (p(2)<0)
        'bad traj: into the table'
    end
    
end

