

function brain_learnGrasp_trainParams(newX, newY)
    global learnGrasp_global;
    
    SAVE_FREQ = 1;
    
    % Scale our values appropriately
    newX = (newX - learnGrasp_global.min)./(learnGrasp_global.max - learnGrasp_global.min);
    
    % Save our new values    
    learnGrasp_global.x = [learnGrasp_global.x; newX];
    learnGrasp_global.y = [learnGrasp_global.y; newY];
    learnGrasp_global.n = learnGrasp_global.n + 1;
    
    % Make sure we save this model to memory every so often
    if (mod(learnGrasp_global.n, SAVE_FREQ) == 0)
        brain_learnGrasp_saveModel();
    end
        
end