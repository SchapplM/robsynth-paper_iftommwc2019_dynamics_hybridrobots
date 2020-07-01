% Test script for all fixed-base Matlab/Simulink functions for 
% KAS5m7OL

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (C) Institut für Regelungstechnik, Universität Hannover

clc
clear

KAS5m7OL_testfunctions_path_init

KAS5m7OL_varpar_fixbase_kinematics_test
KAS5m7OL_varpar_fixbase_invdyn_test
KAS5m7OL_varpar_fixbase_num_test

% Regressorform
KAS5m7OL_varpar_fixbase_paramlin_test

% Simulink
% KAS5m7OL_fdyn_fixb_test_mp_start
% KAS5m7OL_fdyn_fixb_test_vp1_start

KAS5m7OL_compile_test

fprintf('Tests der Fixed-Base Funktionen für KAS5m7OL abgeschlossen.\n');
