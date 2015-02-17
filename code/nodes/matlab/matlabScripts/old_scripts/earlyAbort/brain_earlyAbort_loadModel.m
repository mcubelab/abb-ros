%Mlab - Simple Hands
%Brain_node
%Class: Early Abort

%Function to load an early abort svm model into the global
%parameter earlyAbort_global
%
% Input Parameters:
%        modelName - Name of the model to load.   
%

function brain_earlyAbort_loadModel(modelName)


%Global varaibles
global earlyAbort_global;

load(modelName,'-mat');

%Output
disp(sprintf('BRAIN_NODE: Early Abort model loaded from %s.mat',modelName));
end


