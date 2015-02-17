function [ max_pt ] = lineSearch( l,ln, pt  )

    % find the max along line ln where each pt is a trajectory and the 
    % "line" is in n dimensional space where n is the length of the traj
    
    maxk =20;
    k=0;
    prev = zeros(size(pt));
    
    while ((k<maxk) && any(any(prev~=pt)))
        
        % find dir of max
        %pts = [pt(1,:)-ln; pt(1,:); pt(1,:)+ln];
        %scores = [ff([pts(1,:);pt(2,:)]) ff([pts(2,:);pt(2,:)]) ff([pts(3,:);pt(2,:)])];
        pts = [pt-ln; pt; pt+ln];
        scores = [ff(l,pts(1:2,:)) ff(l,pts(3:4,:)) ff(l,pts(5:6,:))];
        [~,i] = max(scores);
        prev = pt;  
        %pt = [pts(i,:);pt(2,:)];
        pt = pts(2*(i-1)+1:2*(i-1)+2,:);
        k=k+1;
    end

    max_pt = pt;
end

