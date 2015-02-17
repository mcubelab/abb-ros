
% Mlab - Simple Hands
% Brain_node
% Class: Learning to Grasp

% Function to load a IEmax model into memory
%
% Input Parameters:
%        modelName - Name of the model to load.   
%

function brain_learnGrasp_loadModel(modelName, bounds, grid)

%Global varaibles
global learnGrasp_global;

% Check if we already have the current model loaded in memory
% (This includes checking if all of the input arguments are the same as
% well)
if (~sameModel(modelName, bounds, grid))
    % Otherwise, attempt to load the model
    try
        load(modelName,'-mat');
        
        % If it did exist, make sure all of the bounds are the same
        if (~sameModel(modelName, bounds, grid))
            learnGrasp_global.modelName = sprintf('%s_bkp.mat', strtok(learnGrasp_global.modelName, '.'));
            brain_learnGrasp_saveModel();
            disp(sprintf('BRAIN_NODE: Model had same name but different parameters. Saving old model to: %s',learnGrasp_global.modelName));
            createNewModel(modelName, bounds, grid);
        else
            disp(sprintf('BRAIN_NODE: Loaded grasp learning model from %s',modelName));
        end
    catch
        % If it didn't exist, create a new model
        createNewModel(modelName, bounds, grid);
    end
else
    disp(sprintf('BRAIN_NODE: Grasp Learning model %s already loaded',modelName));
end

end

function tf = modelNamePresent()
global learnGrasp_global;
tf = ~isempty(strmatch('modelName', fieldnames(learnGrasp_global), 'exact'));
end

function tf = sameModel(modelName, bounds, grid)
global learnGrasp_global;
tf =  (isstruct(learnGrasp_global) && modelNamePresent() && ...
    strcmp(learnGrasp_global.modelName, modelName) && ...
    all(bounds(1,:) == learnGrasp_global.min) && ...
    all(bounds(2,:) == learnGrasp_global.max) && ...
    grid ==learnGrasp_global.grid);
end

function createNewModel(modelName, bounds, grid)
    global learnGrasp_global;
    
    % Save the model name so we can compare later
    learnGrasp_global.modelName = modelName;

    % Save the grid size
    learnGrasp_global.grid = grid;

    % Keep track of all of the points we've tried so far
    learnGrasp_global.x = [];
    learnGrasp_global.y = [];
    learnGrasp_global.n = 0;

    % This will hold the index at which we last estimated our function
    learnGrasp_global.last_computed = -1;

    % Remember the bounds on x parameters
    learnGrasp_global.min = bounds(1,:);
    learnGrasp_global.max = bounds(2,:);

    % Make sure we know the dimension of x values to expect
    learnGrasp_global.d = length(learnGrasp_global.min);

    % This will be our initial hyperparameter guess (for use during IEMAX)
    learnGrasp_global.loghyper = [log(0.15); log(0.30); log(0.35)];
    
    % Create a grid of points which we will estimate our function and
    % upper bounds on at each iteration
    lin = linspace(0,1,grid);
    [outArgs{1:learnGrasp_global.d}] = ndgrid(lin);
    learnGrasp_global.xTest = [];
    for i=1:learnGrasp_global.d
        learnGrasp_global.xTest = [learnGrasp_global.xTest, outArgs{i}(:)];
    end

    disp(sprintf('BRAIN_NODE: Created new grasp learning model: %s',modelName));
end