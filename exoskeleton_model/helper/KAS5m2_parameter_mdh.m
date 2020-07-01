% Ausgabe der MDH-Parameter für KAS5m2
% 
% Input:
% 
% Output:
% a_mdh, d_mdh, alpha_mdh, q_offset_mdh [7x1]
%   Numeric Values of Kinematic Parameters, MDH-Notation
% rSges_num [8x3]
%   Center of Gravity of all indepedent bodies (arm only) in urdf frames
% m_num [8x1]
%   masses of all indepedent bodies (arm only)
% Iges_num  [8x6]
%   inertia of all indepedent bodies (arm only)
%     rows: bodies (base, moveable links)
%     columns: inertia tensor entries. Order: xx, yy, zz, xy, xz, yz
% 
% Sources:
% [1] [KhalilKle1986]

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Universität Hannover

function [a_mdh, d_mdh, alpha_mdh, q_offset_mdh, ...
  rSges_num_mdh, m_num_mdh, Iges_num_mdh] = KAS5m2_parameter_mdh()

%% Kinematik Parameter
l = KAS5_parameter_kinematic();

%% Calculate MDH kinematic Parameter
alpha_mdh = zeros(7,1);
a_mdh = zeros(7,1);
q_offset_mdh = zeros(7,1);
d_mdh = zeros(7,1);

% Fülle MDH-Parameter

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

%% Dynamik-Parameter schätzen
% geraten (Summe für den Arm ca. 3 kg)
m_num_mdh = 0.4*ones(8,1);
m_num_mdh(8) = 4.2; % Bohrmaschine
% Nehme alle Körper als Stäbe an und die Schwerpunkte in der Mitte
rSges_num_mdh = NaN(8,3);
% Basis
rSges_num_mdh(1,:) = [0, 0, 0];
% Segment 1
rSges_num_mdh(2,:) = [0, 0, -l(1)/2];
% Segment 2
rSges_num_mdh(3,:) = [0, 0, -l(2)/2];
% Segment 3
rSges_num_mdh(4,:) = [l(5)/2, 0, 0];
% Segment 4
rSges_num_mdh(5,:) = [l(11)/2, 0, 0];
% Segment 5
rSges_num_mdh(6,:) = [l(12)/2, 0, 0];
% Segment 6
rSges_num_mdh(7,:) = [0, -l(15)/2, 0];
% Segment 7 (Endeffektor, Bohrmaschine
rSges_num_mdh(8,:) = [0, 0, l(19)];

% Trägheitsmomente der Stäbe mit Näherungsformel für dünnen Stab
% TM FS S. 44
Iges_num_mdh = zeros(8,6);

% Basis, Näherung als Kugel
Iges_num_mdh(1, 1:3) = 2/5 * m_num_mdh(1) * 0.100^2;

% Segment 1: Stab
Iges_num_mdh(2, [1,2]) = 1/12 * m_num_mdh(2) * l(1)^2;
Iges_num_mdh(2, 3) = 0.01*Iges_num_mdh(2, 1);
% Segment 2: Stab (in z-Richtung)
Iges_num_mdh(3, [1,2]) = 1/12 * m_num_mdh(2) * l(2)^2;
Iges_num_mdh(3, 3) = 0.01*Iges_num_mdh(3, 1);
% Segment 3: Stab (in x-Richtung)
Iges_num_mdh(4, [2,3]) = 1/12 * m_num_mdh(2) * l(5)^2;
Iges_num_mdh(4, 1) = 0.01*Iges_num_mdh(4, 2);
% Segment 4: Stab (in x-Richtung)
Iges_num_mdh(5, [2,3]) = 1/12 * m_num_mdh(2) * l(11)^2;
Iges_num_mdh(5, 1) = 0.01*Iges_num_mdh(5, 2);
% Segment 5: Stab (in x-Richtung)
Iges_num_mdh(6, [2,3]) = 1/12 * m_num_mdh(2) * l(12)^2;
Iges_num_mdh(6, 1) = 0.01*Iges_num_mdh(6, 2);
% Segment 6: Stab (in y-Richtung)
Iges_num_mdh(7, [1,3]) = 1/12 * m_num_mdh(2) * l(15)^2;
Iges_num_mdh(7, 2) = 0.01*Iges_num_mdh(7, 1);
% Segment 7: Kugel (wegen schwerem Werkzeug)
Iges_num_mdh(8, 1:3) = 2/5 * m_num_mdh(8) * 0.100^2;