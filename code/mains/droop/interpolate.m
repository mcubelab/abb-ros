function [ new,v ] = interpolate( p )

    global l;
    global curr_s;

    ep = 0.25;
    x1 = p(1)-ep;
    x2 = p(1)+ep;
    y1 = p(2)-ep;
    y2 = p(2)+ep;
    % choose 4 pts 
    q11 = [x1;y1];
    q12 = [x1;y2];
    q21 = [x2;y1];
    q22 = [x2;y2];
    q = [p q11 q12 q21 q22];

    ds = evaluate(p);
    ds11= evaluate(q11);
    ds12 = evaluate(q12);
    ds21= evaluate(q21);
    ds22 = evaluate(q22);
    
    % TODO: a circle 
    % move in direction max dsx and min dsy
    dss = [ds ds11 ds12 ds21 ds22];
    m = max(dss,[],2);
    maxIndex = find(dss == m(1));
    [i,j] = ind2sub(size(dss),maxIndex);
    maxv = sum(q(:,j),2)/length(j);
    
    m = min(dss,[],2);
    maxIndex = find(dss == m(2));
    [i,j] = ind2sub(size(dss),maxIndex);
    minv = sum(q(:,j),2)/length(j);
    
    
    vx = (maxv-p)*1.0;
    vy = (minv-p)*1.0;
    v = [(vx(1)+vy(1))/2; (vx(2)+vy(2))/2];
    
    new = p + v; %[(p(1)+q(1,maxx)+q(1,miny))/3 ; (p(2)+q(2,maxx)+q(2,miny))/3];
    new(2) = max(0,new(2));
    
    
end

