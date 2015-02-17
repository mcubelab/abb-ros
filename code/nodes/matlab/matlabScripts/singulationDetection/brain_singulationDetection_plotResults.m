function brain_singulationDetection_plotResults(folderName)

global singulationDetection_global;

all_data = sprintf('%s/all_data.mat',folderName);
load(all_data)

normFeat = bsxfun(@rdivide, bsxfun(@minus,features,singulationDetection_global.mean),singulationDetection_global.std);
lbls = svmpredict(zeros(length(normFeat),1), normFeat, singulationDetection_global.model);
lbls = (lbls==1);

sing = (labels(:,1)==1);

figure(1);clf
plot3(features(lbls,1),features(lbls,2),features(lbls,3),'ro');
hold on;
plot3(features(~lbls,1),features(~lbls,2),features(~lbls,3),'bo');
title('All Results');

figure(2);clf
plot3(features((lbls & sing),1),features((lbls & sing),2),features((lbls & sing),3),'ro');
hold on;
plot3(features((~lbls & sing),1),features((~lbls & sing),2),features((~lbls & sing),3),'bo');
title('Singulated Results');

figure(3);clf
plot3(features((lbls & ~sing),1),features((lbls & ~sing),2),features((lbls & ~sing),3),'ro');
hold on;
plot3(features((~lbls & ~sing),1),features((~lbls & ~sing),2),features((~lbls & ~sing),3),'bo');
title('Non-Singulated Results');
end