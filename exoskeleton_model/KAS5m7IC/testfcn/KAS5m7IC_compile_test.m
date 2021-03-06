% Compile all Functions as mex to check if asserts are correct

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (C) Institut für Regelungstechnik, Universität Hannover

clear
clc

%% Init
KAS5m7IC_testfunctions_path_init % erzeugt matlabfcn_path.m

mex_all_matlabfcn_in_dir(matlabfcn_path, 1);

fprintf('Alle m-Funktionen als mex kompiliert!\n');
