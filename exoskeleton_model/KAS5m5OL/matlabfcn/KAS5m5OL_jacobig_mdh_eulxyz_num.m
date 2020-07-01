% Jacobimatrix zu beliebigen Punkten eines Körpers für
% KAS5m5OL
%
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
% qJ [13x1]
%   Joint Angles [rad]
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt. (0=Basis).
%   Siehe auch: KAS5m5OL_fkine_fixb_rotmat_mdh_sym_varpar.m
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% pkin [12x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8]';
%
% Output:
% Jg_C [6x(6+13)]
%   geometric body jacobian for the defined point with respect to
%   generalized coordinates:
%
% Quellen:
% [1] Aufzeichnungen Schappler (vom 10.06.2016)
% [2] BouyarmaneKhe2012: On the dynamics modeling of free-floating-base
%     articulated mechanisms and applications to humanoid whole-body dynamics
%     and control

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover
% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (C) Institut für Regelungstechnik, Leibniz Universität Hannover

function Jg_C = KAS5m5OL_jacobig_mdh_eulxyz_num(phi_base, qJ, link_index, r_i_i_C, pkin)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(13,1),uint8(zeros(1,1)),zeros(3,1),zeros(12,1)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'KAS5m5OL_jacobig_mdh_eulxyz_num: Base RPY angles have to be [3x1] (double)');
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_jacobig_mdh_eulxyz_num: Joint angles qJ have to be [13x1] (double)');
assert(isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ...
  'KAS5m5OL_jacobig_mdh_eulxyz_num: Position vector r_i_i_C has to be [3x1] (double)');
assert(all(size(link_index) == [1 1]), ...
  'KAS5m5OL_jacobig_mdh_eulxyz_num: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_jacobig_mdh_eulxyz_num: Kinematic parameters pkin have to be [12x1] (double)');

%% Kinematik berechnen
T_c_mdh = KAS5m5OL_fkine_fixb_rotmat_mdh_sym_varpar(qJ, pkin);
v_mdh = KAS5m5OL_structural_kinematic_parameters();
T_B_i = T_c_mdh(:,:,link_index+1);
% Vektor von der Basis (B) zum Punkt C (auf Körper i)
r_B_B_C = T_B_i(1:3,4) + T_B_i(1:3,1:3) * r_i_i_C; % r_B_B_i + R_B_i_C

% Rotationsmatrix vom Welt- ins Basis-KS
R_W_B = eulxyz2r(phi_base);

% Transformation zwischen RPY-Winkel-Zeitableitung und
% Winkelgeschwindigkeit der Basis im Welt-KS
T_angvel = eulxyzjac(phi_base);
%% Basis-Jacobi-Matrix
% Siehe [1]
% [2], equ. (14)
Jg_BTC = [eye(3),     -skew(R_W_B*r_B_B_C)*T_angvel];
% [2], equ. (17)
Jg_BRC = [zeros(3,3), T_angvel];

%% Gelenk-Jacobi-Matrix
Jg_JC_B = KAS5m5OL_jacobig_mdh_num(qJ, link_index, r_i_i_C, pkin);
Jg_JTC = R_W_B * Jg_JC_B(1:3,:);
Jg_JRC = R_W_B * Jg_JC_B(4:6,:);

%% Gesamt-Jacobi-Matrix zusammensetzen
Jg_C = [Jg_BTC, Jg_JTC; ... % [2], equ. (14)
        Jg_BRC, Jg_JRC]; % [2], equ. (17)
