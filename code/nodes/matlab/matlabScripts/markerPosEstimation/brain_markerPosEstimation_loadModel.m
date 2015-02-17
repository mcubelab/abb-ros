
%Mlab - Simple Hands
%Brain_node
%Class: Marker Position Estimation

%Function to load a singulation detection svm model into the global
%parameter singulationDetection_global
%
% Input Parameters:
%        modelName - Name of the model to load.   
%

function brain_markerPosEstimation_loadModel(modelName)


%Global varaibles
global markerPosEstimation_global;

load(modelName,'-mat');

%Output
disp(sprintf('BRAIN_NODE: Marker Position Estimation model loaded from %s.mat',modelName));
end


