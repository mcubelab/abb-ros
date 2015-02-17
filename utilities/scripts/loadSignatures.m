%This places each signature in a column of motor, enc1, enc2, enc3 and enc4
%nCols = 200
%nRows = 76 (number of samples in the signature)
motor=zeros(0);
enc1=zeros(0);
enc2=zeros(0);
enc3=zeros(0);
enc4=zeros(0);
for i=1:200
    fileName = sprintf('../../2-collectedData/processed/signatures/graspingMarkersP2_2010_05_28_signature%03d.txt',i);
    sample = load(fileName);
    newMotor = sample(:,2)';
    newEnc1 = sample(:,3)';
    newEnc2 = sample(:,4)';
    newEnc3 = sample(:,5)';
    newEnc4 = sample(:,6)';
    motor = [motor newMotor(:)];
    enc1 = [enc1 newEnc1(:)];
    enc2 = [enc2 newEnc2(:)];
    enc3 = [enc3 newEnc3(:)];
    enc4 = [enc4 newEnc4(:)];
end
%Loads the targets values (1 == singulated, 0 == not singulated)
targets = load('targets.txt');