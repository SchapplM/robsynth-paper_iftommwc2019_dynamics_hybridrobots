% Ausgabe der MDH-Parameter für KAS5
% 
% Input:
% 
% Output:
% a_mdh, d_mdh, alpha_mdh, q_offset_mdh [13x1]
%   Numeric Values of Kinematic Parameters, MDH-Notation
% v
%   Liste mit Vorgänger-Nummern der einzelnen Körper
% rSges_num [14x3]
%   Center of Gravity of all indepedent bodies (arm only) in urdf frames
% m_num [14x1]
%   masses of all indepedent bodies (arm only)
% Iges_num  [14x6]
%   inertia of all indepedent bodies (arm only)
%     rows: bodies
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz
% 
% Sources:
% [1] [KhalilKle1986]

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Universität Hannover

function [a_mdh, d_mdh, alpha_mdh, q_offset_mdh, v, ...
  rSges_num_mdh, m_num_mdh, Iges_num_mdh] = KAS5_m4_parameter_mdh()

%% Kinematik Parameter
l = KAS5_parameter_kinematic();

%% Calculate MDH kinematic Parameter
alpha_mdh = zeros(13,1);
a_mdh = zeros(13,1);
q_offset_mdh = zeros(13,1);
d_mdh = zeros(13,1);

% Vektor mit Nummern der Vorgänger-Körper
v = uint8([0:6, ...
  2, 8, ...
  3, 10, 11, 12]);


%% Hauptstruktur
q_offset_mdh(1) = -pi/2;
d_mdh(1) = l(1);

alpha_mdh(2) = -pi/2;
q_offset_mdh(2) = pi/2;
d_mdh(2) = l(2);

alpha_mdh(3) = pi/2;
d_mdh(3) = l(3);

a_mdh(4) = l(5);

a_mdh(5) = l(11);

a_mdh(6) = l(12);
q_offset_mdh(6) = pi/2;

alpha_mdh(7) = pi/2;
d_mdh(7) = l(15);

%% Parallelstruktur
alpha_mdh(8) = pi/2;
q_offset_mdh(8) = pi/2;
d_mdh(8) = l(3);

a_mdh(9) = -l(4);

a_mdh(10) = l(6);

a_mdh(11) = l(22);
q_offset_mdh(11) = pi;

a_mdh(12) = l(13);
q_offset_mdh(12) = 0;

q_offset_mdh(13) = pi;


%% Dynamik-Parameter 
% erstellt von: KAS5_get_dyn_param

% Massen-Parameter
m_num_mdh = [0.03936; 0.13395; 0.41301; 0.1476; 0.01582; 0.01582; 0.21958; ...
             0.1123; 0.004985; 0.01309; 0.01061; 0.04215; 0.008785; 0.06291; ...
             ];
m_num_mdh(8) = 4;
% Schwerpunktsparameter
rSges_num_mdh = ...
  [0 0 0.02407;
   0 0.025895059798432252 -0.010432866836132884;
   6.8160269727125254E-6 -0.0026613174015157016 -0.044237728452095594;
   0.086124715447154457 -0.00024837317073170734 -0.006434471544715447;
   0.02151 2.866E-6 0.006;
   0.02149 -2.866E-6 0.002;
   -0.0016244831041078427 -0.17496493123235266 -0.0035694614627926036;
   0.0007919 -0.0008683 0.02558;
   -0.0104 0.0002365 0.003;
   0.08169 -0.006971 0.0085;
   0.01588 -0.002299 0.003;
   0.09279 -0.008702 0.004;
   -0.04 0 0.0105;
   0.05916 0 0.0043];

% Trägheitsmomente
Iges_num_mdh = ...
  [3.99E-5 3.99E-5 1.901E-5 0 0 0;
   0.000294290377401999 0.00017524037740199898 0.00016775999999999998 0 ...
   0 -1.451E-7;
   0.0017427890248235091 0.0018197092438066466 0.00035825025735855955 3.3777601430491666E-8 ...
   1.4191367140727584E-7 -7.3120190662724146E-8;
   0.0012427530638247451 0.0026543818034647456 0.0038758005718839022 0.00051515370367804885 ...
   -4.9850039698673174E-5 -1.4496190950731705E-6;
   3.605E-6 1.738E-5 1.98E-5 0 2.042E-6 0;
   3.099E-6 1.686E-5 1.979E-5 0 6.798E-7 0;
   0.0084836718973456161 0.00010611161007161536 0.0085369949725123354 2.0384154962832324E-5 ...
   -3.4809472399676843E-6 0.00016782973640923232;
   0.0001391 0.0001535 3.923E-5 -7.754E-8 2.275E-6 5.398E-6;
   3.248E-7 1.243E-6 1.448E-6 -1.182E-8 -1.556E-7 0;
   2.882E-6 0.0001175 0.0001184 -6.034E-6 9.087E-6 -7.755E-7;
   2.733E-6 8.021E-6 1.05E-5 -1.996E-6 5.055E-7 -7.319E-8;
   6.308E-6 0.0004741 0.000478 -3.427E-5 1.564E-5 -1.467E-6;
   1.288E-6 2.113E-5 2.045E-5 0 -3.69E-6 0;
   6.779E-6 0.0002883 0.0002873 0 1.6E-5 0];
% Höheres Massenträgheitsmoment des Werkzeugs (4kg)
Iges_num_mdh(8,1:3) = 2/3 * m_num_mdh(8) * 0.1^2;
Iges_num_mdh(8,4:6) = 0;
