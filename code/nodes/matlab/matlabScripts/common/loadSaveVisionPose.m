function loadSaveVisionPose(folderName)
labels = [];
features = [];
fileNames = [];
nTimeStamps = [];    

%Find all Log files in the provided folder
files = dir(folderName);
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
    signatureFileName = sprintf('%s/%s',folderName,fileNames(i,:));
    
    %Read file
    [featuresFile, labelsFile] = loadFile(signatureFileName);
    labels = [labels ; labelsFile];
    
    nTimeStamps = [nTimeStamps; 1];    
    features = [features;featuresFile];
end
nFeatures = size(features, 2);
% a = strfind(folderName, '/');
% if (~isempty(a))
%     folderName = folderName(a(end)+1:end);
% end
save_name = sprintf('%s/all_data.mat', folderName);
save(save_name, 'labels', 'features', 'nTimeStamps','nFeatures');
end

function [features, labels] = loadFile(fileName)
featuresId = [22,23,24];
text=fileread(fileName);
lines = regexp(text, '\n', 'split');
for i=length(lines):-1:1
    if (lines{i}(1) == '#')
        if (lines{i}(2) == 'S')
           aux = textscan(lines{i},'#S,%*d,%d,%f,%d,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%*s',1);
           labels = [double(aux{1}),double(aux{2}),double(aux{3}),double(aux{4}),double(aux{5}),double(aux{6}),double(aux{7}),double(aux{8})];
           features = [double(aux{10}),double(aux{11}),double(aux{12}), double(aux{13}), double(aux{14}), double(aux{15})];
%            aux = textscan(lines{i},'#S,%*d,%d,%f,%d,%f,%f,%f,%f,%d,%d,%d,%d,%*s',1);
%            labels = [double(aux{1}),double(aux{2}),double(aux{3}),double(aux{4}),double(aux{5}),double(aux{6}),double(aux{7})];
%            features = [double(aux{9}),double(aux{10}),double(aux{11})];
           break;
        else
            disp('skipping...');
        end
    else
        disp('skipping...');
    end
end
end