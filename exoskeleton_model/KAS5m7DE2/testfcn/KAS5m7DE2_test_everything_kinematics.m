% Test script for Matlab/Simulink kinematic-functions for 
% KAS5m7DE2

% Tim-David Job, 2019-11, HiWi bei
% Moritz Schappler, schappler@irt.uni-hannover.de
% (C) Institut für mechatronische Systeme, Universität Hannover

clc
clear

KAS5m7DE2_testfunctions_path_init

KAS5m7DE2_varpar_fixbase_kinematics_test
KAS5m7DE2_varpar_floatbase_kinematics_test
KAS5m7DE2_compile_test

fprintf('Tests aller Kinematik-Funktionen für KAS5m7DE2 abgeschlossen.\n');
