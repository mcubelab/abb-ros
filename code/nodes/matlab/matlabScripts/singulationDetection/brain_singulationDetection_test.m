%Mlab - Simple Hands
%Brain_node
%Class: Singulation Detection

%Function to test an example agains a singulation svm model
%Description: Learns a model from all signatures in the provided folder
%
% Input Parameters:
%        fileName - Name of the file with the Log.
%
% Return Values:
%         label - Classification svm output

function [label] = brain_singulationDetection_test(fileName)

%Global variables
global singulationDetection_global;

%Load data
[features, labels, nTimeStamps, nFeatPerTime] = loadLogFile(fileName);

%Normalization of the number of time stamps
if(nTimeStamps >= singulationDetection_global.nTimeStamps)
    features = features(1:singulationDetection_global.nTimeStamps*singulationDetection_global.nFeaturesPerTimeStamp);
else
    features = [features repmat(features(end-singulationDetection_global.nFeaturesPerTimeStamp+1:end),1,singulationDetection_global.nTimeStamps-nTimeStamps)];
end

%Feature Selection
testFeatures = features(:,singulationDetection_global.idFeatures);

%Feature Normalization
testFeatures = testFeatures - singulationDetection_global.mean;
testFeatures = testFeatures./singulationDetection_global.std;

%Prediction
label = svmpredict(0, testFeatures, singulationDetection_global.model);

%Output
disp(sprintf('BRAIN_NODE: Singulation Detection - Predicted label %d.\n',label));
end


