% Test script for Matlab/Simulink kinematic-functions for 
% KAS5m7OL

% Tim-David Job, 2019-11, HiWi bei
% Moritz Schappler, schappler@irt.uni-hannover.de
% (C) Institut für mechatronische Systeme, Universität Hannover

clc
clear

KAS5m7OL_testfunctions_path_init

KAS5m7OL_varpar_fixbase_kinematics_test
KAS5m7OL_varpar_floatbase_kinematics_test
KAS5m7OL_compile_test

fprintf('Tests aller Kinematik-Funktionen für KAS5m7OL abgeschlossen.\n');
