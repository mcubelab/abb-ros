% Functions saveParams(filename, iter_number)
% This function will save the current state
% Input1 = filename =>basename of the file
% Input2 = iter_number =>iteration number for saving file during
% experimentation

ParamLimits = [MinA1 , MaxA2;MinA2 ,Max A2;Min A3,MaxA3];
save_norm = [save_norm; normP(1) normP(2) userin];
save_actual = [save_actual;nextP(1) nextP(2)  userin val1];
