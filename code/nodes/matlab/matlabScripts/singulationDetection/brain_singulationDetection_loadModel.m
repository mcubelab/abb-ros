
%Mlab - Simple Hands
%Brain_node
%Class: Singulation Detection

%Function to load a singulation detection svm model into the global
%parameter singulationDetection_global
%
% Input Parameters:
%        modelName - Name of the model to load.   
%

function brain_singulationDetection_loadModel(modelName)


%Global varaibles
global singulationDetection_global;

load(modelName,'-mat');

%Output
disp(sprintf('BRAIN_NODE: Singulation Detection model loaded from %s.mat',modelName));
end


