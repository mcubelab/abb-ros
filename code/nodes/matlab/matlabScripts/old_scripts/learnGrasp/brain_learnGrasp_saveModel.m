%Mlab - Simple Hands
%Brain_node
%Class: Singulation Detection

% Save current IEMAX model to file  

function brain_learnGrasp_saveModel()
    global learnGrasp_global;
    save(learnGrasp_global.modelName,'learnGrasp_global','-mat');
end


