% Test script for all Matlab/Simulink functions for kinematics of
% KAS5m7IC (implicit definitions of kinematic constraints)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
% (C) Institut für Regelungstechnik, Universität Hannover

clc
clear

KAS5m7IC_testfunctions_path_init

KAS5m7IC_test_constr_kinematics
KAS5m7IC_compile_test

fprintf('Tests aller Funktionen für KAS5m7IC abgeschlossen.\n');
