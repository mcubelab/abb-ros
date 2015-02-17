%Mlab - Simple Hands
%Brain_node
%Class: Early Abort

%Function to test an example against an early abort svm model
%
% Input Parameters:
%        fileName - Name of the file with the Log.
%	 slice - Which slice to test against
%
% Return Values:
%         abort - Classification svm output: 1, continue or singulated. 0, abort or not-singulated.

function [abort] = brain_earlyAbort_test(fileName)

%Global variables
global earlyAbort_global;

%Load data
[features, labels, nTimeStamps, nFeatPerTime] = loadLogFile(fileName);

%Normalization of the number of time stamps
if(nTimeStamps >= earlyAbort_global.nTimeStamps)
    features = features(1:earlyAbort_global.nTimeStamps*earlyAbort_global.nFeaturesPerTimeStamp);
else
    features = [features repmat(features(end-earlyAbort_global.nFeaturesPerTimeStamp+1:end),1,earlyAbort_global.nTimeStamps-nTimeStamps)];
end

for i=1:earlyAbort_global.nSlices

	%Feature Selection
	testFeatures = features(:,earlyAbort_global.idFeatures{i});

	%Feature Normalization
	testFeatures = testFeatures - earlyAbort_global.mean{i};
	testFeatures = testFeatures./earlyAbort_global.std{i};

	%Prediction
	[predictedLabels, accuracy, prob] = svmpredict(0, testFeatures, earlyAbort_global.model{i},'-b 1');
	disp(sprintf('BRAIN_NODE: Slice %d, probability: %d, cutoff: %d. \n', i, prob, earlyAbort_global.probThreshold[i]);

	%Output
	if (prob < earlyAbort_global.probThreshold[i])
		abort = 0;
		disp(sprintf('BRAIN_NODE: Early abort: aborts at slice %d.\n',i));
	else
		abort = 1;
		disp(sprintf('BRAIN_NODE: Early abort: continues at slice %d.\n'));
	end

end

end
