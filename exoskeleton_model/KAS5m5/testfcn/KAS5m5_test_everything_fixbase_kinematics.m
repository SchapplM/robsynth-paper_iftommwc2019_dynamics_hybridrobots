% Test script for fixed-base Matlab/Simulink kinematic-functions for 
% KAS5m5

% Tim-David Job, 2019-11, HiWi bei
% Moritz Schappler, schappler@irt.uni-hannover.de
% (C) Institut für mechatronische Systeme, Universität Hannover

clc
clear

KAS5m5_testfunctions_path_init

KAS5m5_varpar_fixbase_kinematics_test
KAS5m5_compile_test

fprintf('Tests der Fixed-Base Kinematik-Funktionen für KAS5m5 abgeschlossen.\n');
