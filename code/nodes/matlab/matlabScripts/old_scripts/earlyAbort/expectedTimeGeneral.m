function time = expectedTimeGeneral(nSlices,probs,testLabels,probThreshold)
%Estimates the expected time based on the transition probabilities.

inds{1} = 1:1:size(probs,1);
for i=2:nSlices
    inds{i} = inds{i-1}(find(probs(inds{i-1},i-1)>=probThreshold(i-1)));
end
inds{nSlices+1} = find(testLabels(inds{nSlices}) ==1);
    
p = [];
for i =1:nSlices
    p = [p length(inds{i+1})/length(inds{i})];
end    

a = 1;
for i=nSlices-1:-1:1
    a = a*p(i) + 1; 
end

b = 1;
for i=1:nSlices
    b = b*p(i);
end

time = 1/nSlices*a/b;