%Mlab - Simple Hands
%Brain_node
%Function: loadLogFolder

%Function to load all log files in a folder
% In the current implementation:
%  - It loads the 5 encoder signatures as a single signature as:
%    [enc1(t1) enc2(t1) enc3(t1) enc4(t1) enc5(t1) enc1(t2) ...]
%  - It loads as many labels (k) as the log files have appended at the end
%    of the file.
%  - Normalize the number of timeStamps of each signature to the median
%    number of timeStamps in the log files. If the log is larger, the 
%    function crops it. If the log is shorter, the log repeats the last
%    value.
%
% Parameters:
% folder - String poiting to the folder with the log signatures. 
%          For now the funtion assumes that inside folder there is a file
%          called fileList.txt with the list of files to load.
%
% Return values:
% features - Matrix m*n where the n is the number of features and m the
%            number of training examples. Each row is a training example.
% labels - Vector m*k where m is the number of training examples and k is
%          the number of labels appended to the end of the log file.

function [features, labels, nTimeStamps, nFeatPerTime] = loadLogFolder(folder)

labels = [];
features = [];
featuresAux = [];
nTimeStamps = [];
fileNames = [];

%Find all Log files in the provided folder
files = dir(folder);
nLogs = 0;
for i=1:length(files)
    %We only pay attention to files that begin with L
    if files(i).name(1)=='L'
        fileNames = [fileNames;files(i).name];
        nLogs = nLogs + 1;
    end
end

%Read in the Log files
for i=1:nLogs
    %Import the log signature
    signatureFileName = sprintf('%s/%s',folder,fileNames(i,:));
    
    %Read file
    [featuresFile, labelsFile, nTime, nFeatPerTime] = loadLogFile(signatureFileName);
    
    labels = [labels ; labelsFile];
    
    nTimeStamps = [nTimeStamps; nTime];    
    featuresAux{i} = featuresFile;
end

% Normalize the number of timeStamps in each Log file
% We will make the length of all logs equal to the median length
desiredTimeStamps = round(median(nTimeStamps));

%*** We will make the length of all logs equal to the minimum length
%desiredTimeStamps = floor(min(nTimeStamps));

for i=1:nLogs
    if(nTimeStamps(i) >= desiredTimeStamps)
        % In this case we cut the feature vector 
        len = desiredTimeStamps*nFeatPerTime;
        newFeatures = featuresAux{i}(1:len);

    else
        % We copy the last timeStamp
        nCopy = desiredTimeStamps-nTimeStamps(i);
        newFeatures = [featuresAux{i} repmat(featuresAux{i}(end-nFeatPerTime+1:end),1,nCopy)];    
    end
        
    %Every row of features is a training example    
    features = [features ; newFeatures];
end

nTimeStamps = desiredTimeStamps;

end



