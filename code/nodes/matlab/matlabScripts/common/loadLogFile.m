%Mlab - Simple Hands
%Brain_node
%Function: loadLog

%Function to load log file in memory
% In the current implementation:
%  - It loads the 4 encoder signatures as a single signature as:
%    [enc1(t1) enc2(t1) enc3(t1) enc4(t1) enc1(t2) ...]
%  - It loads as many labels (k) as the log files have appended at the end
%    of the file.
%
% Parameters:
% fileName - String with the log file name.
%            The file is supposed to be a log file created by the system
%            logger with an extra line at the end with some features.
%
% Return values:
% features - Vector 1*n where the n is the number of features.
%
% labels - Vector 1*k where k is the number of labels appended to the end
%          of the log file.

function [features, labels, nTimeStamps, nFeaturesPerTimeStamp] = loadLogFile(fileName)

%Feature selection (we only care about motor and finger encoders)
%For 3 fingers
featuresId = [22,23,24];
%For 4 fingers
%featuredId = [21,22,23,24,25];

%Open file
fid = fopen(fileName);

%First line: Id
line = fgetl(fid);

%Second line: headers
line = fgetl(fid);

%Read log
features = [];
labels = [];
line = fgetl(fid);
while line ~= -1
    %If it is an appended line
    if(line(1) == '#')
        % If Output from sensor
        if(line(2) == 'S')
           aux = textscan(line,'#S,%*d,%d,%f,%d,%f,%f,%f,%f,%*s',1);
           labels = [double(aux{1}),double(aux{2}),double(aux{3}),double(aux{4}),double(aux{5}),double(aux{6}),double(aux{7})];
        end
    %If it is a regular log line    
    else
        % This are features to import
        timeStamp = textscan(line,'%f,');
        features = [features; timeStamp{1}(featuresId)'];
    end
    line = fgetl(fid);
end

features = features';
features = features(:)';
nFeaturesPerTimeStamp = length(featuresId);
nTimeStamps = length(features)/nFeaturesPerTimeStamp;
fclose(fid);
end



