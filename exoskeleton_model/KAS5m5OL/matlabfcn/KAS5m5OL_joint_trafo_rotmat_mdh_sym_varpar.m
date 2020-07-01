% Calculate homogenous joint transformation matrices for
% KAS5m5OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [12x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8]';
% 
% Output:
% T_mdh [4x4x13]
%   homogenous transformation matrices for joint transformation (MDH)
%   Transformation matrices from one joint to the next (not: from base to joints)
% T_stack [(13+1)*3 x 4]
%   stacked matrices from T_mdh into one 2D array, last row left out.
%   Last row only contains [0 0 0 1].

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [T_mdh, T_stack] = KAS5m5OL_joint_trafo_rotmat_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(12,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_joint_trafo_rotmat_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_joint_trafo_rotmat_mdh_sym_varpar: pkin has to be [12x1] (double)');

%% Symbolic Calculation
% From joint_transformation_mdh_rotmat_matlab.m
% OptimizationMode: 2
% StartTime: 2020-06-27 19:20:23
% EndTime: 2020-06-27 19:20:23
% DurationCPUTime: 0.08s
% Computational Cost: add. (26->26), mult. (0->0), div. (0->0), fcn. (52->26), ass. (0->27)
t118 = cos(qJ(1));
t117 = cos(qJ(2));
t116 = cos(qJ(3));
t115 = cos(qJ(4));
t114 = cos(qJ(5));
t113 = cos(qJ(6));
t112 = cos(qJ(7));
t111 = cos(qJ(8));
t110 = cos(qJ(9));
t109 = sin(qJ(1));
t108 = sin(qJ(2));
t107 = sin(qJ(3));
t106 = sin(qJ(4));
t105 = sin(qJ(5));
t104 = sin(qJ(6));
t103 = sin(qJ(7));
t102 = sin(qJ(8));
t101 = sin(qJ(9));
t100 = cos(qJ(10));
t99 = cos(qJ(11));
t98 = cos(qJ(12));
t97 = cos(qJ(13));
t96 = sin(qJ(10));
t95 = sin(qJ(11));
t94 = sin(qJ(12));
t93 = sin(qJ(13));
t1 = [t109, t118, 0, 0; -t118, t109, 0, 0; 0, 0, 1, pkin(8); -t108, -t117, 0, 0; 0, 0, 1, pkin(9); -t117, t108, 0, 0; t116, -t107, 0, 0; 0, 0, -1, -pkin(10); t107, t116, 0, 0; t115, -t106, 0, pkin(4); t106, t115, 0, 0; 0, 0, 1, 0; t114, -t105, 0, pkin(5); t105, t114, 0, 0; 0, 0, 1, 0; -t104, -t113, 0, pkin(6); t113, -t104, 0, 0; 0, 0, 1, 0; t112, -t103, 0, 0; 0, 0, -1, -pkin(11); t103, t112, 0, 0; -t102, -t111, 0, 0; 0, 0, -1, -pkin(12); t111, -t102, 0, 0; t110, -t101, 0, pkin(7); t101, t110, 0, 0; 0, 0, 1, 0; t100, -t96, 0, pkin(1); t96, t100, 0, 0; 0, 0, 1, 0; -t99, t95, 0, pkin(2); -t95, -t99, 0, 0; 0, 0, 1, 0; t98, -t94, 0, pkin(3); t94, t98, 0, 0; 0, 0, 1, 0; -t97, t93, 0, 0; -t93, -t97, 0, 0; 0, 0, 1, 0;];
T_stack = t1;
%% Postprocessing: Reshape Output
% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)
% Fallunterscheidung der Initialisierung für symbolische Eingabe
if isa([qJ; pkin], 'double'), T_mdh = NaN(4,4,13);             % numerisch
else,                         T_mdh = sym('xx', [4,4,13]); end % symbolisch

for i = 1:13
  T_mdh(:,:,i) = [T_stack((i-1)*3+1 : 3*i, :);[0 0 0 1]];
end
