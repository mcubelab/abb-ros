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

function brain_markerPosEstimation_trainModel(folder,modelName, numTest)

if (nargin < 3)
    numTest = 0;
end

%Global variables
global markerPosEstimation_global;

% K-Fold cross validation will be used
%K = 10;

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
    [features, labels,nTime,nFeat] = loadLogFolder(folder);
end
markerPosEstimation_global.nTimeStamps = nTime;
markerPosEstimation_global.nFeaturesPerTimeStamp = nFeat;

%Feature selection
%Only end pose of the hand
idFeatures = [size(features,2) - nFeat+1 : size(features,2)];
markerPosEstimation_global.idFeatures = idFeatures;

% We select as label the first column, which is 1/0 depending on
% wether the grasp was singulated or not.
idLabels = 1;
trainLabels = labels(:,idLabels);
yLabels = labels(:, [7 8]); % Extract r and alpha

alpha = yLabels(trainLabels == 1, 1);
r = yLabels(trainLabels == 1, 2);


% x = labels(trainLabels == 1,4);
% y = labels(trainLabels == 1,5);
% theta = labels(trainLabels == 1,6);
% cx = sin(theta).*(-y.*cos(theta) + x.*sin(theta));
% cy = cos(theta).*(y.*cos(theta) - x.*sin(theta));
% r = sqrt(cx.^2 + cy.^2);
% alpha = atan2(cy,cx);




% Parameterize alpha as a unit vector, so we don't have a singularity
% around 0 and 2pi

all_x = features(trainLabels == 1, idFeatures);
all_y = [r, cos(alpha), sin(alpha)];

idx = randperm(size(all_x, 1));
train_idx = idx(1:(size(all_x,1)-numTest));
test_idx = idx((size(all_x,1)-numTest+1):end);



markerPosEstimation_global.x = all_x(train_idx,:);
markerPosEstimation_global.testX = all_x(test_idx,:);
markerPosEstimation_global.y = all_y(train_idx,:);
markerPosEstimation_global.testY = all_y(test_idx,:);


%%Feature Normalization
mx = mean(markerPosEstimation_global.x,1);
sx = std(markerPosEstimation_global.x,0,1);
my = mean(markerPosEstimation_global.y,1);
sy = std(markerPosEstimation_global.y,0,1);

markerPosEstimation_global.x = bsxfun(@minus, markerPosEstimation_global.x, mx);
markerPosEstimation_global.x = bsxfun(@rdivide, markerPosEstimation_global.x, sx);
markerPosEstimation_global.y = bsxfun(@minus, markerPosEstimation_global.y, my);
markerPosEstimation_global.y = bsxfun(@rdivide, markerPosEstimation_global.y, sy);

markerPosEstimation_global.xmean = mx;
markerPosEstimation_global.xstd = sx;
markerPosEstimation_global.ymean = my;
markerPosEstimation_global.ystd = sy;

[n,d]=size(markerPosEstimation_global.x)
size(markerPosEstimation_global.y)

disp('loaded features!');


% Set up to estimate hyper parameters
markerPosEstimation_global.likfunc = @likGauss; 
markerPosEstimation_global.covfunc = {'covSum', {'covSEiso','covNoise'}};
markerPosEstimation_global.hyp.cov = [log(0.15); log(0.30); log(0.35)]; 
markerPosEstimation_global.hyp.lik = log(0.1);

test_x = bsxfun(@minus, markerPosEstimation_global.testX, mx);
test_x = bsxfun(@rdivide, test_x, sx);

y_est = zeros(size(markerPosEstimation_global.testY));
s_est = zeros(size(y_est));

y_enc_est = zeros(size(y_est));
s_enc_est = zeros(size(y_est));

y_palm_est = zeros(size(y_est));
s_palm_est = zeros(size(y_est));


for i=1:size(markerPosEstimation_global.y,2)
    % Choose hyper parameters that maximize our likelihood
markerPosEstimation_global.hyp1 = minimize(markerPosEstimation_global.hyp, @gp, -100, @infExact, ...
    [], ...
    markerPosEstimation_global.covfunc, ...
    markerPosEstimation_global.likfunc, ...
    markerPosEstimation_global.x, markerPosEstimation_global.y(:,i));


% Now regress the function
[f, s] = gp(markerPosEstimation_global.hyp1, @infExact, ...
    [], ...
    markerPosEstimation_global.covfunc, ...
    markerPosEstimation_global.likfunc, ...
    markerPosEstimation_global.x, ...
    markerPosEstimation_global.y(:,i), test_x);

y_est(:,i) = f;
s_est(:,i) = s;

markerPosEstimation_global.hyp2 = minimize(markerPosEstimation_global.hyp, @gp, -100, @infExact, ...
    [], ...
    markerPosEstimation_global.covfunc, ...
    markerPosEstimation_global.likfunc, ...
    markerPosEstimation_global.x(:,1:3), markerPosEstimation_global.y(:,i));


% Now regress the function
[f, s] = gp(markerPosEstimation_global.hyp2, @infExact, ...
    [], ...
    markerPosEstimation_global.covfunc, ...
    markerPosEstimation_global.likfunc, ...
    markerPosEstimation_global.x(:,1:3), ...
    markerPosEstimation_global.y(:,i), test_x(:,1:3));

y_enc_est(:,i) = f;
s_enc_est(:,i) = s;

markerPosEstimation_global.hyp3 = minimize(markerPosEstimation_global.hyp, @gp, -100, @infExact, ...
    [], ...
    markerPosEstimation_global.covfunc, ...
    markerPosEstimation_global.likfunc, ...
    markerPosEstimation_global.x(:,4:6), markerPosEstimation_global.y(:,i));


% Now regress the function
[f, s] = gp(markerPosEstimation_global.hyp3, @infExact, ...
    [], ...
    markerPosEstimation_global.covfunc, ...
    markerPosEstimation_global.likfunc, ...
    markerPosEstimation_global.x(:,4:6), ...
    markerPosEstimation_global.y(:,i), test_x(:,4:6));

y_palm_est(:,i) = f;
s_palm_est(:,i) = s;

end


% Now let's recover our actual estimated output
y_est = bsxfun(@times, y_est, sy);
y_est = bsxfun(@plus, y_est, my);

y_enc_est = bsxfun(@times, y_est, sy);
y_enc_est = bsxfun(@plus, y_enc_est, my);

y_palm_est = bsxfun(@times, y_est, sy);
y_palm_est = bsxfun(@plus, y_palm_est, my);

figure(1);clf;
mag = sqrt(y_est(:,3).^2 + y_est(:,2).^2);
plot(mag,'o');

markerPosEstimation_global.mag = mag;

enc_mag = sqrt(y_enc_est(:,3).^2 + y_enc_est(:,2).^2);
markerPosEstimation_global.enc_mag = enc_mag;

palm_mag = sqrt(y_palm_est(:,3).^2 + y_palm_est(:,2).^2);
markerPosEstimation_global.palm_mag = palm_mag;


est_alpha = atan2(y_est(:,3), y_est(:,2));
est_r = y_est(:,1);

enc_est_alpha = atan2(y_enc_est(:,3), y_enc_est(:,2));
enc_est_r = y_enc_est(:,1);

palm_est_alpha = atan2(y_palm_est(:,3), y_palm_est(:,2));
palm_est_r = y_palm_est(:,1);


test_alpha = alpha(test_idx);
test_r = r(test_idx);

for i=1:numTest
    if (abs(est_alpha(i) - test_alpha(i)) > pi)
        if (est_alpha(i) > 0)
            est_alpha(i) = est_alpha(i) - 2*pi;
        else
            est_alpha(i) = est_alpha(i) + 2*pi;
        end
    end
end

for i=1:numTest
    if (abs(enc_est_alpha(i) - test_alpha(i)) > pi)
        if (enc_est_alpha(i) > 0)
            enc_est_alpha(i) = enc_est_alpha(i) - 2*pi;
        else
            enc_est_alpha(i) = enc_est_alpha(i) + 2*pi;
        end
    end
end

for i=1:numTest
    if (abs(palm_est_alpha(i) - test_alpha(i)) > pi)
        if (palm_est_alpha(i) > 0)
            palm_est_alpha(i) = palm_est_alpha(i) - 2*pi;
        else
            palm_est_alpha(i) = palm_est_alpha(i) + 2*pi;
        end
    end
end


err_r = est_r - test_r;
err_alpha = est_alpha - test_alpha;

enc_err_r = enc_est_r - test_r;
enc_err_alpha = enc_est_alpha - test_alpha;

palm_err_r = palm_est_r - test_r;
palm_err_alpha = palm_est_alpha - test_alpha;

markerPosEstimation_global.errors = [est_r test_r est_alpha test_alpha err_r err_alpha];
markerPosEstimation_global.enc_errors = [enc_est_r test_r enc_est_alpha test_alpha enc_err_r enc_err_alpha];
markerPosEstimation_global.palm_errors = [palm_est_r test_r palm_est_alpha test_alpha palm_err_r palm_err_alpha];

figure(2);clf;
hold on;
plot(est_r, 'bo');
plot(test_r, 'ro');

figure(3);clf;
hold on;
plot(est_alpha, 'bo');
plot(test_alpha, 'ro');

std_r = std(err_r)
std_alpha = std(err_alpha)

std_enc_r = std(enc_err_r)
std_enc_alpha = std(enc_err_alpha)

std_palm_r = std(palm_err_r)
std_palm_alpha = std(palm_err_alpha)

std_r = std(err_r(abs(mag - 1) < 0.05))
std_alpha = std(err_alpha(abs(mag - 1) < 0.05))

std_enc_r = std(enc_err_r(abs(enc_mag - 1) < 0.05))
std_enc_alpha = std(enc_err_alpha(abs(enc_mag - 1) < 0.05))

std_palm_r = std(palm_err_r(abs(palm_mag - 1) < 0.05))
std_palm_alpha = std(palm_err_alpha(abs(palm_mag - 1) < 0.05))


figure(4); clf;
scatter(err_r, err_alpha);
xlabel('err_r');
ylabel('err_alpha');

figure(5);clf;
plot(sqrt(y_est(:,3).^2 + y_est(:,2).^2), err_alpha, 'o');

figure(6);clf;
hist(err_alpha,10);

% Save our model
fileName = sprintf('%s',modelName);
save(fileName,'markerPosEstimation_global','-mat');

%Output
disp(sprintf('BRAIN_NODE: Model saved to models/%s.mat',modelName));
end


