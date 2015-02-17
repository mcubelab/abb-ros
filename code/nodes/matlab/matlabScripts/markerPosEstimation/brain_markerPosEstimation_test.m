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
%         y - predicted marker position estimate
%         s - variance for each prediction

function [y, s] = brain_markerPosEstimation_test(fileName)

%Global variables
global markerPosEstimation_global;

%Load data
[features, labels, nTimeStamps, nFeatPerTime] = loadLogFile(fileName);

%Normalization of the number of time stamps


% 
% if(nTimeStamps >= markerPosEstimation_global.nTimeStamps)
%     features = features(1:markerPosEstimation_global.nTimeStamps*markerPosEstimation_global.nFeaturesPerTimeStamp);
% else
%     features = [features repmat(features(end-markerPosEstimation_global.nFeaturesPerTimeStamp+1:end),1,markerPosEstimation_global.nTimeStamps-nTimeStamps)];
% end

%Feature Selection
%testFeatures = features(:,markerPosEstimation_global.idFeatures);
testFeatures = features(end-markerPosEstimation_global.nFeaturesPerTimeStamp+1:end);

%Feature Normalization
testFeatures = testFeatures - markerPosEstimation_global.xmean;
testFeatures = testFeatures./markerPosEstimation_global.xstd

% Prediction
[y(1) s(1)] = gp(markerPosEstimation_global.hyp1, @infExact, ...
    [], ...
    markerPosEstimation_global.covfunc, ...
    markerPosEstimation_global.likfunc, ...
    markerPosEstimation_global.x, ...
    markerPosEstimation_global.y(:,1), ...
    testFeatures);

[y(2) s(2)] = gp(markerPosEstimation_global.hyp2, @infExact, ...
    [], ...
    markerPosEstimation_global.covfunc, ...
    markerPosEstimation_global.likfunc, ...
    markerPosEstimation_global.x, ...
    markerPosEstimation_global.y(:,2), ...
    testFeatures);

% Make our variances into standard deviations
s = sqrt(s);

% Unnormalize our predicted values
y = (y .* markerPosEstimation_global.ystd) + markerPosEstimation_global.ymean;
s = s .* markerPosEstimation_global.ystd;

%Output
%disp(sprintf('BRAIN_NODE: Singulation Detection - Predicted label %d.\n',label));
end


