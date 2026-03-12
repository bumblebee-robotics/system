% --- Add Current Folder and ALL Subfolders to Path ---

% 1. Get the path to the folder where this script (startup.m) is located.
mainProjectRoot = fileparts(mfilename('fullpath'));

% 2. Use genpath to generate a list of all folders and subfolders 
%    starting from the mainProjectRoot.
allPaths = genpath(mainProjectRoot);

% 3. Add all generated paths to the MATLAB search path.
addpath(allPaths);

disp('Successfully added project root and all subfolders to the MATLAB path.');