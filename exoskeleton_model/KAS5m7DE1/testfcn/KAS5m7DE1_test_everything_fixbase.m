% Test script for all fixed-base Matlab/Simulink functions for 
% KAS5m7DE1

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (C) Institut für Regelungstechnik, Universität Hannover

clc
clear

KAS5m7DE1_testfunctions_path_init

KAS5m7DE1_varpar_fixbase_kinematics_test
KAS5m7DE1_varpar_fixbase_invdyn_test
KAS5m7DE1_varpar_fixbase_num_test

% Regressorform
KAS5m7DE1_varpar_fixbase_paramlin_test

KAS5m7DE1_compile_test

fprintf('Tests der Fixed-Base Funktionen für KAS5m7DE1 abgeschlossen.\n');
