% Initialize the Matlab path for the validation scripts to work

% Moritz Schappler, schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

this_path = fileparts( mfilename('fullpath') );
addpath(fullfile(this_path, '..', 'helper'));
addpath(fullfile(this_path, '..', 'KAS5m3', 'matlabfcn'));
addpath(fullfile(this_path, '..', 'KAS5m5', 'matlabfcn'));
addpath(fullfile(this_path, '..', 'KAS5m7OL', 'matlabfcn'));
addpath(fullfile(this_path, '..', 'KAS5m7OL', 'testfcn'));
addpath(fullfile(this_path, '..', 'KAS5m7TE', 'matlabfcn'));
addpath(fullfile(this_path, '..', 'KAS5m7TE', 'testfcn'));
addpath(fullfile(this_path, '..', 'KAS5m7DE1', 'matlabfcn'));
addpath(fullfile(this_path, '..', 'KAS5m7DE1', 'testfcn'));
addpath(fullfile(this_path, '..', 'KAS5m7DE2', 'matlabfcn'));
addpath(fullfile(this_path, '..', 'KAS5m7DE2', 'testfcn'));