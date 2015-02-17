% INITIALIZEEXPERIMENT ()
% This function will create all the matrices needed for Optimization


if( 0 == exist('nMin','var') || 0 == exist('grid','var') || 0 == exist('nParams','var') )
    loadParams;
end

rand('state',2000);
saver = [];
MaxVal = [];

i =1;

%This is where the program performs uniform search
x_orig = rand(nMin,nParams);

x_0s = zeros(nMin/4,1);
x_1s = 0.5*ones(nMin/4,1);

x1 = x_orig((0*nMin/4 +1 : 1*nMin/4),:)./2 + [x_0s x_0s];
x2 = x_orig((1*nMin/4 +1 : 2*nMin/4),:)./2 + [x_0s x_1s];
x3 = x_orig((2*nMin/4 +1 : 3*nMin/4),:)./2 + [x_1s x_0s];
x4 = x_orig((3*nMin/4 +1 : 4*nMin/4),:)./2 + [x_1s x_1s];


x = [x1;x2;x3;x4];



if(7 ~= exist('tmp_1','dir'))
    mkdir tmp_1;
end

disp('here');

if(2 == nParams)
    xlin = linspace(0,1,grid);
    ylin = linspace(0,1,grid);
    [X,Y] = meshgrid(xlin,ylin);
    nextP = [0;0]; % just initializing
    normP = [0;0];
    maxP = [0;0];
    xTest = [X(:),Y(:)];
    Param = [MinA1 ,MaxA1;MinA2, MaxA2];
end

if(3 == nParams)
    xlin = linspace(0,1,grid);
    ylin = linspace(0,1,grid);
    zlin = linspace(0,1,grid);
    [X,Y,Z] = meshgrid(xlin,ylin,zlin);
    nextP = [0;0;0]; % just initializing
    normP = [0;0;0];
    xTest = [X(:),Y(:),Z(:)];
    Param = [MinA1 ,MaxA1;MinA2, MaxA2;MinA3,MaxA3];
end

