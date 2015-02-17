%Mlab - Simple Hands
%Brain_node
%Class: Early Abort

%Function to train singulation svm model
%Description: Learns a model from all signatures in the provided folder
%
% Input Parameters:
%        folder - Name of the folder to to look for Log files.
%        modelName - Name of the model to save.
%        nSlices - Number of slices in the model
% Return Values:
%         error - Estimated cross-validation error.
%                 (-1 If there was an error).

function [error] = brain_earlyAbort_trainModel(folder,modelName,nSlices)

%Global variables
global earlyAbort_global;
%Load data
%NOTE: load data returns as features the four encoders
%for each time stamp
[features, labels,nTime,nFeat] = loadLogFolder(folder);
earlyAbort_global.nTimeStamps = nTime;
earlyAbort_global.nFeaturesPerTimeStamp = nFeat;
earlyAbort_global.nSlices = nSlices;

nExper = size(features, 1);
nTrain = floor(nExper / 2);

if (mod(nTime, nSlices) ~= 0)
    lastSlice = true;
    timeStampsPerSlice = floor(nTime/nSlices);
else
    lastSlice = false;
    timeStampsPerSlice = nTime / nSlices;
end

probs = [];

%Repeat for each slice
for slice=1:1:nSlices
    disp(sprintf('Creating model for slice: %d', slice));
    nFeatures =  slice * timeStampsPerSlice * nFeat;
    %Feature selection
    %Feature up to selection point
    idFeatures = [1 : nFeatures];
    trainFeatures = features(1:nTrain,idFeatures);
    testFeatures = features(nTrain+1:end,idFeatures);
    earlyAbort_global.idFeatures{slice} = idFeatures;
    
    % We select as label the first column, which is 1/0 depending on
    % wether the grasp was singulated or not.
    idLabels = 1;
    trainLabels = labels(1:nTrain,idLabels);
    trainLabels = trainLabels*2 - 1;
    testLabels = labels(nTrain+1:end, idLabels);
    testLabels = testLabels*2 - 1;
    earlyAbort_global.idLabels{slice} = idLabels;
    
    
    %%Feature Normalization
    nSamples = size(trainFeatures,1);
    m = mean(trainFeatures,1);
    s = std(trainFeatures,0,1);
    trainFeatures = trainFeatures - repmat(m,nSamples,1);
    trainFeatures = trainFeatures./repmat(s,nSamples,1);
    earlyAbort_global.mean{slice} = m;
    earlyAbort_global.std{slice} = s;
    
    % Preprocess test features
    nSamplesTest = size(testFeatures, 1);
    testFeatures = testFeatures - repmat(m, nSamplesTest, 1);
    testFeatures = testFeatures ./ repmat(s, nSamplesTest, 1);
    
    disp('Hyper parameter fitting with cross validation');
    %Hyper parameter fitting (cross validation)
    bestcv = 0;
    for log2c = -10:1:10
        for log2g = -10:1:1
            params = ['-q -t 2 -v 5 -c ', num2str(2^log2c), ' -g ', num2str(2^log2g)];
            cvAccuracy = svmtrain(trainLabels, trainFeatures, params);
            if (cvAccuracy >= bestcv),
                bestcv = cvAccuracy; bestc = 2^log2c; bestg = 2^log2g;
            end
        end
    end
    disp(sprintf('Best C: %ld    Best G: %ld', bestc, bestg));
    %Final error
    params = ['-q -t 2 -v 5 -c ', num2str(bestc), ' -g ', num2str(bestg)];
    accuracy = svmtrain(trainLabels, trainFeatures, params);
    error = 100.0 - accuracy;
    disp(sprintf('Error: %d', error));
    %Compute model and save global parameters
    %params = ['-q -t 2 -b 0 -c ', num2str(bestc), ' -g ', num2str(bestg)];
    params = ['-q -t 2 -b 1 -c ', num2str(bestc), ' -g ', num2str(bestg)];
    model = svmtrain(trainLabels, trainFeatures, params);
    [predictedLabels,accuracyTest, prob] = svmpredict(testLabels, testFeatures, model,'-b 1');
    probs = [probs, prob(:,1)]; %success probabilities of test labels
    
    earlyAbort_global.model{slice} = model;
    earlyAbort_global.error{slice} = error;

%     
%     %Output
     disp(sprintf('BRAIN_NODE: Early Abort model learned. Cross Validation error = %0.5g',error));
     disp(sprintf('BRAIN_NODE: Model saved to %s%d.mat',modelName, slice));
end

testLabels = labels(nTrain+1:end, 1) * 2 - 1;
%Expected time is a method that computes the expected time based on the
%probabilities (explained in the paper)
cost = @(probThreshold) (expectedTimeGeneral(nSlices,probs,testLabels,probThreshold));

%Optimization of cost using GA (genetic algorithm)
lb = repmat(0.0,1,nSlices-1);
ub = repmat(1.0,1,nSlices-1);
gaOptions = gaoptimset('Generations', 200, 'PopulationSize', 10000,'TimeLimit',20000,'StallGenLimit',50,'StallTimeLimit',1000,'Display', 'iter');

probThreshold1 = ga(cost,nSlices-1,[],[],[],[],lb,ub,[],gaOptions);
cost(probThreshold1)

earlyAbort_global.probThreshold = probThreshold1;

fileName = sprintf('%s',modelName);
save(fileName,'earlyAbort_global','-mat');

end


