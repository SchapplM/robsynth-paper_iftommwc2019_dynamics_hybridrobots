% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Die Zwangsbedingungen geben die Abweichung zwischen einer Soll-Pose in
% EE-Koordinaten und der Ist-Pose aus gegebenen Gelenk-Koordinaten an.
% Vollständige Rotations- und Translationskomponenten
% Variante 2:
% * Vektor vom Basis- zum EE-KS (kein Unterschied zu SKM-Variante 1)
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln (entspricht PKM
%   Variante 2)
% * Rotationsfehler ausgedrückt in Euler-Winkeln (um raumfeste Achsen), je
%   nach Eingabeargument `reci` (entspricht teilweise PKM-Variante 2)
%
% Eingabe:
% q
%   Gelenkwinkel des Roboters
% xE
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% pkin
%   Kinematik-Parameter
% T_N_E
%   Transformationsmatrix EE-Segment-KS -> EE-KS
% phiconv_W_E
%   Winkelkonvention der Euler-Winkel Welt->End-Effektor. Siehe eul2r.m
% I_EElink
%   Nummer des Segmentes, an dem der EE befestigt ist (0=Basis)
% reci
%   true: Nehme reziproke Euler-Winkel für Orientierungsfehler (z.B.
%   ZYX-Orientierungsfehler für XYZ-Absolutorientierung)
%   false: Gleiche Euler-Winkel für Fehler und Absolut [Standard]
%
% Ausgabe:
% Phi
%   Kinematische Zwangsbedingungen des Roboters: Maß für den Positions- und
%   Orientierungsfehler zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x

% Quelle:
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi = KAS5m7TE_constr2(q, xE, pkin, T_N_E, phiconv_W_E, I_EElink, reci)

%% Init
%#codegen
%$cgargs {zeros(5,1),zeros(6,1),zeros(24,1),zeros(4,4),uint8(2),uint8(0),true}

%% Allgemein
Tc_ges = KAS5m7TE_fkine_fixb_rotmat_mdh_sym_varpar(q, pkin);
T_0_E_q = Tc_ges(:,:,I_EElink+1) * T_N_E;

%% Translatorischer Teil
r_0_E_x = xE(1:3); % [SchapplerTapOrt2019, Gl. (3)

% Direkte Kinematik der Beinkette
r_0_E_q = T_0_E_q(1:3,4); % [SchapplerTapOrt2019, Gl. (1)

% [SchapplerTapOrt2019, Gl. (8)
Phix = r_0_E_q - r_0_E_x;

%% Rotatorischer Teil
R_0_E_x = eul2r(xE(4:6), phiconv_W_E); % [SchapplerTapOrt2019, Gl. (4)
if reci
  % Wahl reziproker Winkel, siehe [SchapplerTapOrt2019, Fig. 1
  [~,phiconv_delta] = euler_angle_properties(phiconv_W_E);
else
  phiconv_delta = phiconv_W_E;
end

R_0_E_q = T_0_E_q(1:3,1:3);

% Differenz-Rotation z.B. mit ZYX-Euler-Winkel
% [SchapplerTapOrt2019], Gl. (9)
R_Ex_Eq = R_0_E_x' * R_0_E_q;
phiR = r2eul(R_Ex_Eq, phiconv_delta);

% [SchapplerTapOrt2019, Gl. (7)
Phi = [Phix; phiR];
