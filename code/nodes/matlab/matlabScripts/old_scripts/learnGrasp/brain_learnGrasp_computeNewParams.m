function brain_learnGrasp_computeNewParams()
    global learnGrasp_global;
    
    MIN_POINTS = 30;
    COV_EST_START = 1000;
    VALIDATION_SET = 0.1;
    
    % If we haven't seen enough data yet
    if (learnGrasp_global.n < MIN_POINTS)
        % Return a random set of parameters to try next
        learnGrasp_global.nextP = rand(1,learnGrasp_global.d);
        
        % If we haven't gotten any positive examples yet, 
        %  the max value is just set to a random point
        if (sum(learnGrasp_global.y) == 0)
            learnGrasp_global.maxP = learnGrasp_global.nextP;
        else
            % Otherwise, return the weighted average of all of the positive 
            % examples we've seen so far
            learnGrasp_global.maxP = sum(bsxfun(@times, learnGrasp_global.x, learnGrasp_global.y)) / sum(learnGrasp_global.y);
        end
    else
        % Otherwise, use IEmax to compute bounds on our function
        
        % Set up the type of covariances to use
        covfunc = {'covSum', {'covSEiso','covNoise'}};
        
        % If 
        if (learnGrasp_global.n >= COV_EST_START)
            indexes = randperm(learnGrasp_global.n);
            VN = round(VALIDATION_SET*learnGrasp_global.n);
            test_idx = indexes(1:VN);
            train_idx = indexes(VN+1:end);
            
            min_diff = realmax;
            
            for i=0.1:0.1:1,
                for j=0.1:0.1:1,
                    for k=0.1:0.1:1,
                        [testVals, stdev] = gpr([log(i);log(j);log(k)], @infExact, covfunc, [], [], learnGrasp_global.x(train_idx, :), learnGrasp_global.y(train_idx, :), learnGrasp_global.x(test_idx, :));
                        diff = sum(sum((learnGrasp_global.y(test_idx, :) - testVals).^2));
                        if (diff < min_diff)
                            min_diff = diff;
                            bestH = [log(i);log(j);log(k)];
                        end
                    end
                end
            end
            disp(sprintf('previous hyper: %f, %f, %f', learnGrasp_global.loghyper(1), learnGrasp_global.loghyper(2), learnGrasp_global.loghyper(3)));
            learnGrasp_global.loghyper = bestH;
            disp(sprintf('new hyper: %f, %f, %f', learnGrasp_global.loghyper(1), learnGrasp_global.loghyper(2), learnGrasp_global.loghyper(3)));
        else
            learnGrasp_global.loghyper = [log(0.15); log(0.30); log(0.35)];
        end

        % Use our guess of hyperparameters from last time
        
        %if (learnGrasp_global.n >= COV_EST_START)
        %    disp(sprintf('initial hyper: %f, %f, %f', learnGrasp_global.loghyper(1), learnGrasp_global.loghyper(2), learnGrasp_global.loghyper(3)));
        %    %
        %    % Now find an estimate of the best hyperparameters to use
        %    learnGrasp_global.loghyper = minimize(learnGrasp_global.loghyper, 'gpr', -20, covfunc, learnGrasp_global.x, learnGrasp_global.y);
        %    disp(sprintf('final hyper: %f, %f, %f', learnGrasp_global.loghyper(1), learnGrasp_global.loghyper(2), learnGrasp_global.loghyper(3)));
        %end
        %
        
        % Use gaussian process regression to estimate our function and the
        % variance at each point
        [learnGrasp_global.fTest, learnGrasp_global.s2] = gp(learnGrasp_global.loghyper, @infExact, [], covfunc, [], learnGrasp_global.x, learnGrasp_global.y, learnGrasp_global.xTest);
    
        % Subtract the noise variance from total variance to get the real
        % variance
        learnGrasp_global.s2 = learnGrasp_global.s2 - exp(2*learnGrasp_global.loghyper(3));

        % Our upper bound is 3 standard deviations above our function
        % estimate
        learnGrasp_global.bound = learnGrasp_global.fTest + 3*sqrt(learnGrasp_global.s2);

        % Get maximum of the function
        [val indMax] = max(learnGrasp_global.fTest);
        learnGrasp_global.maxP = learnGrasp_global.xTest(indMax, :);

        % Get maximum of the bound and set the next exploration value
        [val indMax] = max(learnGrasp_global.bound);
        learnGrasp_global.nextP = learnGrasp_global.xTest(indMax, :);
    end
    % Remember that we estimated the function at this point
    learnGrasp_global.last_computed = learnGrasp_global.n;
end
