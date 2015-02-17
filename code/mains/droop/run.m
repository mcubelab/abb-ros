function [ finished, time, dtheta ] = run( iax,iay,ifx,ify )

    global ax; global ay; global fx; global fy;
    ax = iax;
    ay = iay;
    fx = ifx;
    fy = ify;
    simout = sim('openloop_hand');
    
    t = simout.get('theta');
    im = t<=(-pi+0.05);
    ip = t>=(pi-0.05);
    i = or(im,ip);

    ind = find(i);
    if (length(ind)==0)
        finished = false;
        time = Inf;
        dtheta = Inf;
    else
        finished = true;
        time = ind(1);
        dthetas = simout.get('dtheta');
        dtheta = dthetas(time);
    end
    
end

