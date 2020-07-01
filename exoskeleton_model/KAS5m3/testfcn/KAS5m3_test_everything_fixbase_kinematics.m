% Test script for fixed-base Matlab/Simulink kinematic-functions for 
% KAS5m3

% Tim-David Job, 2019-11, HiWi bei
% Moritz Schappler, schappler@irt.uni-hannover.de
% (C) Institut für mechatronische Systeme, Universität Hannover

clc
clear

KAS5m3_testfunctions_path_init

KAS5m3_varpar_fixbase_kinematics_test
KAS5m3_compile_test

fprintf('Tests der Fixed-Base Kinematik-Funktionen für KAS5m3 abgeschlossen.\n');
