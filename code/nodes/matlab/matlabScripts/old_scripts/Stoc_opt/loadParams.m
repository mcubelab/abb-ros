% Function - loadParams(no_params)
% This function will load the parameters which are set before the start of
% the experiment. This function can also be called to restart the
% experiment if it gets stopped in the middle for any reason.

clear;
close all;
clc;
PI = 3.14159265;

nParams =2;


nMin = 32;
nMax  =200;
%this is for ShieldCan
%MinA1 = 5
%MaxA1 = 20

%MinA2 = 5
%MaxA2 = 20


%this is for battery insertion
MinA1  = 30;
MaxA1  =45;

MinA2  =-4;
MaxA2  =4;

MinA3  =0 ;
MaxA3  = 0;

grid  = 40;
