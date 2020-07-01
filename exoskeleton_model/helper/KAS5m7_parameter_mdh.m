% Ausgabe der MDH-Parameter für KAS5m7
% 
% Output:
% a_mdh, d_mdh, alpha_mdh, q_offset_mdh [21x1]
%   Numeric Values of Kinematic Parameters, MDH-Notation
% v
%   Liste mit Vorgänger-Nummern der einzelnen Körper
% rSges_num [21x3]
%   Center of Gravity of all indepedent bodies (arm only) in urdf frames
% m_num [21x1]
%   masses of all indepedent bodies (arm only)
% Iges_num  [21x6]
%   inertia of all indepedent bodies (arm only)
%     rows: bodies
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz
% 
% Sources:
% [1] [KhalilKle1986]

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [sigma_mdh, beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, q_offset_mdh, v] = KAS5m7_parameter_mdh()

%% Kinematik Parameter
[l,w] = KAS5_parameter_kinematic();
delta9 = w(1);
delta20 = w(7);
%% Calculate MDH kinematic Parameter
beta_mdh = zeros(21,1);
b_mdh = zeros(21,1);

alpha_mdh = zeros(21,1);
a_mdh = zeros(21,1);
q_offset_mdh = zeros(21,1);
theta_mdh = zeros(21,1);
d_mdh = zeros(21,1);

sigma_mdh = zeros(21,1);
sigma_mdh(15) = 1;

% Vektor mit Nummern der Vorgänger-Körper
v = uint8([0:6, ...
  2, 8, 3, 10, 4, 12, 13, 14, ...
  10, 13, 6, 9, 11, 15])';


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

a_mdh(12) = l(11);
q_offset_mdh(12) = -pi/2;

q_offset_mdh(13) = -pi/2;

a_mdh(14) = l(14);
q_offset_mdh(14) = 3*pi/2;

alpha_mdh(15) = pi/2;

%% Schnitt-Gelenke
beta_mdh(16) = delta9;
a_mdh(16) = l(21);
a_mdh(17) = l(14);
q_offset_mdh(18) = pi/2;
beta_mdh(18) = -pi/2 - delta20;
a_mdh(18) = l(23);
a_mdh(19) = l(20);
a_mdh(20) = l(13);
alpha_mdh(21) = -pi/2;
