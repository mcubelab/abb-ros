function maxP = brain_learnGrasp_getMaxParams()
    global learnGrasp_global;
    
    % If we haven't already, compute our new function estimates
    if (learnGrasp_global.last_computed ~= learnGrasp_global.n)
        brain_learnGrasp_computeNewParams();
    end

    % Scale our values appropriately
    maxP = learnGrasp_global.maxP .* (learnGrasp_global.max - learnGrasp_global.min) + learnGrasp_global.min;
end