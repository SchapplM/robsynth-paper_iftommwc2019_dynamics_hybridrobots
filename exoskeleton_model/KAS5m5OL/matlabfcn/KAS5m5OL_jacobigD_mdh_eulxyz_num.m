% Jacobimatrix-Zeitableitung zu beliebigen Punkten eines Körpers für
% KAS5m5OL
% Floating-Base-Notation: Jacobi-Matrix bezogen auf Basis und Gelenke
%
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
% phiD_base [3x1]
%   Time Derivative of Base Orientation in world frame.
%   Expressed with XYZ-Euler angles ("rpy")
% qJ [13x1]
%   Joint Angles [rad]
% qJD [13x1]
%   Joint Velocities [rad/s]
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
% JgD_C [6x(6+13)]
%   time derivative of geometric body jacobian for the defined point with respect to
%   generalized coordinates:
%
% Quellen:
% [1] Aufzeichnungen Schappler (vom 22.06.2016)
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

function JgD_C = KAS5m5OL_jacobigD_mdh_eulxyz_num(phi_base, phiD_base, qJ, qJD, link_index, r_i_i_C, ...
  pkin)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1),zeros(13,1),zeros(13,1),
%$cgargs  uint8(zeros(1,1)),zeros(3,1),zeros(12,1)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'KAS5m5OL_jacobigD_mdh_eulxyz_num: Base Euler angles have to be [3x1] (double)');
assert(isreal(phiD_base) && all(size(phiD_base) == [3 1]), ...
  'KAS5m5OL_jacobigD_mdh_eulxyz_num: Base Euler angles time derivatives have to be [3x1] (double)');
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_jacobigD_mdh_eulxyz_num: Joint angles qJ have to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m5OL_jacobigD_mdh_eulxyz_num: Joint velocities qJD have to be [13x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
  'KAS5m5OL_jacobigD_mdh_eulxyz_num: link_index has to be [1x1] uint8');
assert(isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ...
  'KAS5m5OL_jacobigD_mdh_eulxyz_num: Position vector r_i_i_C has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_jacobigD_mdh_eulxyz_num: Kinematic parameters pkin have to be [12x1] (double)');

if link_index > 14-1
  error('Index exceeds number of bodies');
end

%% Kinematik berechnen
T_c_mdh = KAS5m5OL_fkine_fixb_rotmat_mdh_sym_varpar(qJ, pkin);
v_mdh = KAS5m5OL_structural_kinematic_parameters();
T_B_i = T_c_mdh(:,:,link_index+1);
% Vektor von der Basis (B) zum Punkt C (auf Körper i)
r_B_B_C = T_B_i(1:3,4) + T_B_i(1:3,1:3) * r_i_i_C; % r_B_B_i + R_B_i_C

% Rotationsmatrix vom Welt- ins Basis-KS
R_W_B = eulxyz2r(phi_base);

% Transformation zwischen Euler-Winkel-Zeitableitung und
% Winkelgeschwindigkeit der Basis im Welt-KS
T_angvel = eulxyzjac(phi_base);
TD_angvel = eulxyzjacD(phi_base, phiD_base);

omega_W_B = T_angvel*phiD_base;

%% Gelenk-Jacobi-Matrix
Jg_JC_B = KAS5m5OL_jacobig_mdh_num(qJ, link_index, r_i_i_C, pkin);
JgD_JC_B = KAS5m5OL_jacobigD_mdh_num(qJ, qJD, link_index, r_i_i_C, pkin);
% Rotation ins Welt-KS und Zeitableitung der Rotation
JgD_JTC = skew(omega_W_B)*R_W_B*Jg_JC_B(1:3,:) + R_W_B*JgD_JC_B(1:3,:);
JgD_JRC = skew(omega_W_B)*R_W_B*Jg_JC_B(4:6,:) + R_W_B*JgD_JC_B(4:6,:);

%% Basis-Jacobi-Matrix
% Siehe [1]
% [2], equ. (14). Zeitableitung durch Produktregel und Eulersche
% Differentiationsregel
Term5_Teil1 = skew(Jg_JC_B(1:3,:)*qJD) * (R_W_B'); % erster Term aus [1] Gl. (5)
Term5_Teil2 = skew(r_B_B_C)*((skew(omega_W_B)*R_W_B)'); % zweiter Term aus [1] Gl. (5)
Term5 = Term5_Teil1 + Term5_Teil2;
Term4_Teil1 = skew(omega_W_B) * R_W_B * skew(r_B_B_C) * (R_W_B'); % erster Term aus [1] Gl. (4)
Term4 = Term4_Teil1 + R_W_B*Term5;
Term3_Teil2 = R_W_B*skew(r_B_B_C)*R_W_B'*TD_angvel; % zweiter Term aus [1] Gl. (3)
Term3 = Term4*T_angvel + Term3_Teil2;

JgD_BTC = [zeros(3,3),     -Term3];
% [2], equ. (17). Zeitableitung
JgD_BRC = [zeros(3,3), TD_angvel];

%% Gesamt-Jacobi-Matrix zusammensetzen
JgD_C = [JgD_BTC, JgD_JTC; ... % [2], equ. (14)
         JgD_BRC, JgD_JRC]; % [2], equ. (17)
