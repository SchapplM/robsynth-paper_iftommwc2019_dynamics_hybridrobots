% Pfade hinzufügen
% 
% For this file to work, it has to be executed as a complete file, not
% line-wise (because of the function `mfilename`)
% 
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

%% Pfade hinzufügen

this_path = fileparts( mfilename('fullpath') );
% In diesem Pfad liegen die Ordner matlabfcn, testfcn und simulink,
% die die entsprechenden Dateien der generierten Umgebung des Roboters beinhalten
main_path_codeexport = clean_absolute_path(fullfile(this_path, '..', '..'));
main_path_this_robot = clean_absolute_path(fullfile(this_path, '..'));


% Verzeichnis mit generierten Matlab-Funktionen hinzufügen
matlabfcn_path = fullfile(main_path_this_robot, 'matlabfcn');
addpath(matlabfcn_path);

% Verzeichnis der Abhängigkeiten hinzufügen: Mit Eliminiation (TE) und ohne 
% Zwangsbedingungen (OL); testfcn benötigt für Initialisierung 
addpath(fullfile(main_path_codeexport, 'KAS5m7TE', 'matlabfcn'));
addpath(fullfile(main_path_codeexport, 'KAS5m7TE', 'testfcn'));
addpath(fullfile(main_path_codeexport, 'KAS5m7OL', 'matlabfcn'));
addpath(fullfile(main_path_codeexport, 'KAS5m7OL', 'testfcn'));
