function [ newt ] = findGradient( traj )

    max_score = ff(traj);
    newt = traj;
    grad = zeros(size(traj));
    n = length(traj);
    
    for i=1:n
        dirs = nchoosek([2:n],i);
        
        for j=1:size(dirs,1)
            dir = dirs(j,:);
            
            % at each pt in dirs, try combos
            % assume pts y val is fixed, x can vary
            tmps = repmat(traj(1,:),(2*length(dir)), 1);
            for k=1:length(dir)
                kk = length(dir);
                nn = (2*length(dir));
                indp = repmat([ones(floor(nn/(2^k)),1); zeros(floor(nn/(2^k)),1)],k,1);
                ii = logical(repmat(indp,1,size(tmps,2)));
                rowsp = tmps(ii);
                rowsm = tmps(~ii);
                rowsp = reshape(rowsp,length(rowsp)/n,n);
                rowsm = reshape(rowsm,length(rowsm)/n,n);
                rowsp(:,dir(k)) = rowsp(:,dir(k)) + 0.1;
                rowsm(:,dir(k)) = rowsm(:,dir(k)) - 0.1;
                
                tmps(ii) = rowsp;
                tmps(~ii) = rowsm;
            end
            
            for k=1:size(tmps,1)
                tmpt = [tmps(k,:) ; traj(2,:)];
                score = ff(tmpt);
                
                if score>max_score
                    max_score = score;
                    newt = tmpt;
                end
            end
        end
    end

end

