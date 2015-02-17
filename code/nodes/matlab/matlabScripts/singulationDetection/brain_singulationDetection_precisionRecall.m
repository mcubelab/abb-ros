%Mlab - Simple Hands
%Brain_node
%Class: Singulation Detection

%Function to compute the precission/recall plot of a singulation detection
% svm model
%
% Input Parameters:
%        folder - Precission Recall
%
% Output Paremeters:
%        w1 - array with weights of class 1.
%        w0 - array with weights of class 0.
%        precission - array with precission.
%        recall - array with recall.
%        c - array with hyperparameter c.
%        g - array with hyperparameter g.

function [w1, w0, precission, recall, c, g] = brain_singulationDetection_precisionRecall(folder)

%Global varaibles
global singulationDetection_global;

%Load data
data_name = sprintf('%s/all_data.mat',folder);
if (exist(data_name))
    C = load(data_name);
    features = C.features;
    labels = C.labels;
    nTime = C.nTimeStamps;
    nFeat = C.nFeatures; 
else
    [features, labels,nTime,nFeat] = loadLogFolder(folder);
end

%Feature selection (Only end pose of the hand)
idFeatures = [size(features,2) - nFeat+1 : size(features,2)];
singulationDetection_global.idFeatures = idFeatures;
features = features(:,idFeatures);

%Label selection (Grasp success)
idLabels = 1;
singulationDetection_global.idLabels = idLabels;
labels = labels(:,idLabels);

%Feature Normalization
nSamples = size(features,1);
m = mean(features,1);
s = std(features,0,1);
features = features - repmat(m,nSamples,1);
features = features./repmat(s,nSamples,1);
singulationDetection_global.mean = m;
singulationDetection_global.std = s;

%Split data into training and testing
nData = size(labels,1);
nTrain = round(nData*0.50);
ids = randperm(nData);
idTrain = ids(1:nTrain);
idTest = ids(nTrain+1:end);

trainFeatures = features(idTrain,:);
testFeatures = features(idTest,:);
trainLabels = labels(idTrain,:);
testLabels = labels(idTest,:);

%Weight selection
maxWeight=20;
nPoints=40;
weights = linspace(1,maxWeight,nPoints+1);
w1 = [ones(1,nPoints),1,weights(2:end)];
w0 = [weights(end:-1:2),1,ones(1,nPoints)];

%Nothing for now.
%bestc = singulationDetection_global.c;
%bestg = singulationDetection_global.g;


%Precission Recall
global res;
res = [];
c = [];
g = [];
for i=1:length(w0)
    %Hyper parameter fitting (cross validation)
    bestcv = 0;
    for log2c = 5:0.5:10
        for log2g = -5:0.5:5
            w = sprintf(' -w1 %d -w0 %d',w1(i),w0(i));
            params = ['-q -t 2 -v 5 -c ', num2str(2^log2c), ' -g ', num2str(2^log2g), w];
            cvAccuracy = svmtrain(trainLabels, trainFeatures, params);
            if (cvAccuracy >= bestcv),
                bestcv = cvAccuracy; bestc = 2^log2c; bestg = 2^log2g;
            end
        end
    end
    w = sprintf(' -w1 %d -w0 %d',w1(i),w0(i));
    params = ['-q -t 2 -c ', num2str(bestc), ' -g ', num2str(bestg), w];
    model = svmtrain(trainLabels, trainFeatures, params);
    
    %Save hyperparameters
    c = [c,bestc];
    g = [g,bestg];
    
    %Compute precission and recall
    predLabels = svmpredict(testLabels, testFeatures, model);
    pp = length(find(predLabels == 1));
    tp = length(find(testLabels(find(predLabels == 1)) == 1));
    ap = length(find(testLabels ==1));
    precission(i) = tp/pp;
    recall(i) = tp/ap;
    res = [res;[i,precission(i),recall(i),bestc,bestg]];
    disp(res(i,:));
end
save('PrecisionRecall20Fine');
plot(recall,precission,'o');

end