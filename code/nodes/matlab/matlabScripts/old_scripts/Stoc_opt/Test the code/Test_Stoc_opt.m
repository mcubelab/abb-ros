% Test Stoc_opt
clear;
loadParams;InitializeExperiment;
nMin =10;
nMax =20;
for num = 1 :nMax
Run_Gpr;
end
