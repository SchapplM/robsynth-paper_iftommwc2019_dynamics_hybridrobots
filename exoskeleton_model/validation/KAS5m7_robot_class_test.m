% Test the exoskeleton system KAS5m7 with the robot class
% The Matlab dependencies have to be installed, see README.MD
% 
% Result:
% * Figure with frames like Fig. 2 in the paper
% * Animation with demonstration of all single DoFs of the exoskeleton

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

if isempty(which('hybroblib_path_init.m'))
  error('the hybrid robot library is not in the Matlab path');
end

%% Initialize robot class.
% Initialize the matlab class from the matlab robotics repository
% use the hybrid robots library for the initialization routine
R = hybroblib_create_robot_class('KAS5m7', 'TE');

% Initialize Parameters
TSS = KAS5m7TE_varpar_testfunctions_parameter();
pkin = TSS.pkin;
% Scale the length parameters so that the joints are smaller and the plots
% have better visibility
I_lpar = (R.pkin_types==2 | R.pkin_types==4 | R.pkin_types==6);
pkin(I_lpar) = 1.8*pkin(I_lpar); % 80% longer
R.update_mdh(pkin);

%% Plot the exoskeleton
q0_m5 = zeros(5,1);
q0_m5(3) = 30*pi/180;
q0_m5(4) = 30*pi/180;
jvar_test = R.jointvar(q0_m5);
T_test = R.jtraf(q0_m5);
Tc_test = R.fkine(q0_m5);
figure(2);clf;
s_plot = struct( 'ks', [1:R.NJ, R.NJ+2], 'straight', 0);
hold on;
grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view(3);
title('Exoskeleton KAS5m7 in reference pose');
R.plot( q0_m5, s_plot );
view([15 15])

%% plot a trajektory for the exoskeleton
QE = [0, 0, 30, 30 , 0; ...
      0, 0, 30, 45 , 0; ... % elbow rotation
      0, 0, 30, 30 , 0; ...
      0, 0, 15, 30 , 0; ... % shoulder lifting (flexion/extension)
      0, 0, 30, 30 , 0; ...
      0, -30, 30, 30 , 0; ... % shoulder sideways movement
      0, 0, 30, 30 , 0; ...
      -30, 0, 30, 30 , 0; ... % shoulder outward rotation
      0, 0, 30, 30 , 0] * pi/180;
Q = traj_trapez2_multipoint(QE, 1, 0.1, 0.01, 1e-3, 1e-1);
s_anim = struct( 'gif_name', '');
figure(3);clf;
hold on;
grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view(3);
title('Exoskeleton KAS5m7 animation');
set(3, 'units','normalized','outerposition',[0 0 1 1]);
R.anim( Q(1:50:end,:), [], s_anim, s_plot);
