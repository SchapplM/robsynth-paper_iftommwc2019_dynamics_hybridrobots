% Test script for all fixed-base Matlab/Simulink functions for 
% KAS5m7TE

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (C) Institut für Regelungstechnik, Universität Hannover

clc
clear

KAS5m7TE_testfunctions_path_init

KAS5m7TE_varpar_fixbase_kinematics_test
KAS5m7TE_varpar_fixbase_invdyn_test
KAS5m7TE_varpar_fixbase_num_test

% Regressorform
KAS5m7TE_varpar_fixbase_paramlin_test

% Simulink
% KAS5m7TE_fdyn_fixb_test_mp_start
% KAS5m7TE_fdyn_fixb_test_vp1_start

KAS5m7TE_compile_test

fprintf('Tests der Fixed-Base Funktionen für KAS5m7TE abgeschlossen.\n');
