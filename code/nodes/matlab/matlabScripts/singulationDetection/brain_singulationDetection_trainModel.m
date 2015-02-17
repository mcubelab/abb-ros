%Mlab - Simple Hands
%Brain_node
%Class: Singulation Detection

%Function to train singulation svm model
%Description: Learns a model from all signatures in the provided folder
%
% Input Parameters:
%        folder - Name of the folder to to look for Log files.
%        modelName - Name of the model to save.   
%
% Return Values:
%         error - Estimated cross-validation error.
%                 (-1 If there was an error).    

function [error] = brain_singulationDetection_trainModel(folder,modelName)

%Global variables
global singulationDetection_global;

% if (exist(modelName, 'file'))
%     disp(sprintf('BRAIN_NODE: Model already exists! Loading old model.'));
%     brain_singulationDetection_loadModel(modelName);
%     error = 0.0;
%     return;
% end
    
weight = '';%' -w1 50';

%Load data
%NOTE: load data returns as features the four encoders
%for each time stamp
data_name = sprintf('%s/all_data.mat',folder);
if (exist(data_name))
    disp('hi');
    C = load(data_name);
    features = C.features;
    labels = C.labels;
    nTime = C.nTimeStamps;
    nFeat = C.nFeatures; 
else
    [features, labels,nTime,nFeat] = loadSaveVisionPose(folder);%loadLogFolder(folder);
end

%nFeat = 3;
%features = features(:,4:6);


singulationDetection_global.nTimeStamps = nTime;
singulationDetection_global.nFeaturesPerTimeStamp = nFeat;

%Feature selection
%Only end pose of the hand
idFeatures = [size(features,2) - nFeat+1 : size(features,2)];
%To add more features
%slice1 = nTime;
%slice2 = nTime - 15;
%slice3 = nTime - 30;
%idFeatures = [1 + 3*(slice1-1): 3 + 3*(slice1-1)];
%idFeatures = [1 + 3*(slice2-1): 3 + 3*(slice2-1),idFeatures];
%idFeatures = [1 + 3*(slice3-1): 3 + 3*(slice3-1),idFeatures];

trainFeatures = features(:,idFeatures);
singulationDetection_global.idFeatures = idFeatures;

% We select as label the first column, which is 1/0 depending on
% wether the grasp was singulated or not.
idLabels = 1;
trainLabels = labels(:,idLabels);
singulationDetection_global.idLabels = idLabels;


%%Feature Normalization
nSamples = size(trainFeatures,1);
m = mean(trainFeatures,1);
s = std(trainFeatures,0,1);
trainFeatures = trainFeatures - repmat(m,nSamples,1);
trainFeatures = trainFeatures./repmat(s,nSamples,1);
singulationDetection_global.mean = m;
singulationDetection_global.std = s;

length(trainFeatures)

%Hyper parameter fitting (cross validation)
bestcv = 0;
%for log2c = -10:1:10
for log2c = 5:0.5:10
%   for log2g = -10:1:1
    for log2g = -5:0.5:5
       params = ['-q -t 2 -v 5 -c ', num2str(2^log2c), ' -g ', num2str(2^log2g), weight];
       cvAccuracy = svmtrain(trainLabels, trainFeatures, params);
       if (cvAccuracy >= bestcv),
           bestcv = cvAccuracy; bestc = 2^log2c; bestg = 2^log2g;
       end
   end
end

%Final error
params = ['-q -t 2 -v 5 -c ', num2str(bestc), ' -g ', num2str(bestg), weight];
accuracy = svmtrain(trainLabels, trainFeatures, params);
error = 100.0 - accuracy;

%Save hyperparameters to file;
singulationDetection_global.c = bestc;
singulationDetection_global.g = bestg;

    
%Compute model and save global parameters
%params = ['-q -t 2 -b 0 -c ', num2str(bestc), ' -g ', num2str(bestg)];
params = ['-q -t 2 -c ', num2str(bestc), ' -g ', num2str(bestg), weight];
model = svmtrain(trainLabels, trainFeatures, params);
singulationDetection_global.model = model;
fileName = sprintf('%s',modelName);
save(fileName,'singulationDetection_global','-mat');

falsePos = 0;
falseNeg = 0;
for i=1:length(trainFeatures)
     label = svmpredict(0, trainFeatures(i,:), singulationDetection_global.model);
     if (trainLabels(i) == 0 && label == 1)
         falsePos = falsePos + 1;
     elseif (trainLabels(i) == 1 && label == 0)
         falseNeg = falseNeg + 1;
     end
end

falsePosErr = falsePos/sum(trainLabels==1)
falseNegErr = falseNeg/sum(trainLabels==0)

bestc
bestg


%Output
disp(sprintf('BRAIN_NODE: Singulation Detection model learned. Cross Validation error = %0.5g',error));
disp(sprintf('BRAIN_NODE: Model saved to learnedModels/%s',modelName));
end


