function [ score] = f( traj )

    l = traj(size(traj,1),size(traj,2));
    n = size(traj,2);

    % initialize
    s = [[l;0] zeros(2,n-1)];
    s_vel = zeros(1,n); % only vel in x
    s_accel = zeros(1,n);
    p_vel = zeros(2,length(traj)-1);
    
    for i=1:n
        s(:,i) = model(l,traj(:,i));
    end
    
    s_vel(2:n) = s(1,2:n) - s(1,1:n-1);
    s 
    s_vel
    
%     % distance travelled by hand (first pt always at (0,0)
%     hand_dist = sqrt(traj(1,1)^2 + traj(2,1)^2);
%     
%     for i=1:size(traj,2)
%         slider = model(l,traj(:,i));
%         s(i+1) = slider(1);
%         
%         if (i>1)
%             p_vel(:,i-1) = traj(:,i) - traj(:,i-1);
%             ddt = sqrt(p_vel(1,:).^2 + p_vel(2,:).^2);
%             s_vel(i+1) = s(i) - s(i-1);
%             hand_dist = hand_dist + sqrt((traj(1,i)-traj(1,i-1))^2 + (traj(2,i)-traj(2,i-1))^2);
%         end
%     end
%     
%     % smoothness of motion (so try to move same amt on each step)
%     smoothness = sum(abs(ddt - sum(ddt)/length(ddt))); % minimize this
%     
%     
%     % try to make the hand move the same dist at each time step
%     avg_dist = hand_dist/(length(traj)-1);
%     wypt_variance = sum(abs(ddt-avg_dist)); % minimize this
%     
%     % try to make the motion of the slider monotonically increasing
% %     diff = dds(2:length(dds)) - dds(1:length(dds)-1);
% %     diff2 = dds(2:length(diff)) > dds(1:length(diff)-1);
% %     increasing = sum(diff2); % maximize this
%     
%     %diff
%     %wypt_variance % should be as close as possible to zero
%     (hand_dist-l) % should also be as close to zero as possible
%     sum(abs(s_vel)) % max this by a lot
%     score = -wypt_variance - 10*(hand_dist-l) + sum(abs(s_vel));
%     
    %score = -hand_dist;
    
end

