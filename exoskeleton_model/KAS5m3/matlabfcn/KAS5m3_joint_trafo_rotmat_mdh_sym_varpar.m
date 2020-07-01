% Calculate homogenous joint transformation matrices for
% KAS5m3
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [6x1]
%   Generalized joint coordinates (joint angles)
% pkin [30x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8,delta8s,delta9s,l11,l12,l13,l14,l17,l18,l20,l21,l22,l4,l5,l6,delta10s,delta12s,delta17s,delta18s]';
% 
% Output:
% T_mdh [4x4x13]
%   homogenous transformation matrices for joint transformation (MDH)
%   Transformation matrices from one joint to the next (not: from base to joints)
% T_stack [(13+1)*3 x 4]
%   stacked matrices from T_mdh into one 2D array, last row left out.
%   Last row only contains [0 0 0 1].

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 17:47
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [T_mdh, T_stack] = KAS5m3_joint_trafo_rotmat_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(6,1),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [6 1]), ...
  'KAS5m3_joint_trafo_rotmat_mdh_sym_varpar: qJ has to be [6x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m3_joint_trafo_rotmat_mdh_sym_varpar: pkin has to be [30x1] (double)');

%% Symbolic Calculation
% From joint_transformation_mdh_rotmat_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-27 16:35:40
% EndTime: 2020-06-27 16:35:46
% DurationCPUTime: 6.42s
% Computational Cost: add. (405293->98), mult. (523040->135), div. (8540->7), fcn. (209764->26), ass. (0->259)
unknown=NaN(39,4);
t1 = sin(qJ(1));
t2 = cos(qJ(1));
t3 = sin(qJ(2));
t4 = cos(qJ(2));
t5 = cos(qJ(3));
t6 = -pkin(25) + pkin(26);
t7 = t5 * t6;
t8 = pkin(29) + qJ(3);
t9 = cos(t8);
t10 = t9 * pkin(18);
t11 = -pkin(15) + t7 + t10;
t12 = pkin(17) ^ 2;
t13 = pkin(23) ^ 2;
t14 = t9 ^ 2;
t15 = pkin(18) ^ 2;
t17 = sin(t8);
t18 = t17 ^ 2;
t21 = (-pkin(15) + t7) ^ 2;
t22 = sin(qJ(3));
t23 = t22 ^ 2;
t24 = t6 ^ 2;
t28 = t22 * t6;
t29 = t17 * pkin(18);
t30 = -t28 + t29;
t33 = 0.2e1 * t11 * t9 * pkin(18) + 0.2e1 * t30 * t17 * pkin(18) - t14 * t15 - t18 * t15 + t23 * t24 + t12 - t13 + t21;
t37 = 0.4e1 * t11 ^ 2 + 0.4e1 * t30 ^ 2;
t39 = t33 ^ 2;
t41 = sqrt(t12 * t37 - t39);
t44 = 0.1e1 / t37;
t45 = (0.2e1 * t11 * t33 - 0.2e1 * t30 * t41) * t44;
t46 = pkin(15) - t7 - t10 + t45;
t51 = (0.2e1 * t11 * t41 + 0.2e1 * t33 * t30) * t44;
t52 = t28 - t29 + t51;
t54 = -t22 * t52 + t5 * t46;
t55 = 0.1e1 / pkin(23);
t56 = t54 * t55;
t57 = cos(pkin(14));
t58 = t57 * pkin(22);
t59 = t56 * t58;
t62 = t22 * t46 + t5 * t52;
t63 = t62 * t55;
t64 = sin(pkin(14));
t65 = t64 * pkin(22);
t66 = t63 * t65;
t67 = -pkin(26) - t59 + t66;
t68 = pkin(21) ^ 2;
t69 = pkin(24) ^ 2;
t70 = t59 - t66;
t71 = t70 ^ 2;
t72 = t63 * t58;
t73 = t56 * t65;
t74 = t72 + t73;
t75 = t74 ^ 2;
t76 = pkin(26) ^ 2;
t79 = -0.2e1 * t67 * t70 + 0.2e1 * t74 ^ 2 + t68 - t69 - t71 - t75 + t76;
t83 = 0.4e1 * t67 ^ 2 + 0.4e1 * t74 ^ 2;
t85 = t79 ^ 2;
t87 = sqrt(t68 * t83 - t85);
t90 = 0.1e1 / t83;
t93 = 0.1e1 / pkin(24);
t94 = (-t59 + t66 - (0.2e1 * t67 * t79 + 0.2e1 * t74 * t87) * t90 - pkin(26)) * t93;
t95 = sin(pkin(13));
t102 = (-t72 - t73 - (0.2e1 * t67 * t87 - 0.2e1 * t79 * t74) * t90) * t93;
t103 = cos(pkin(13));
t105 = t102 * t103 - t94 * t95;
t108 = -t102 * t95 - t94 * t103;
t109 = cos(qJ(4));
t110 = sin(qJ(4));
t111 = sin(qJ(5));
t112 = cos(qJ(5));
t113 = cos(qJ(6));
t114 = sin(qJ(6));
t117 = t105 * t103 - t108 * t95;
t119 = t117 * pkin(24) + t72 + t73;
t121 = 0.1e1 / pkin(21);
t125 = t108 * t103 + t105 * t95;
t127 = -t125 * pkin(24) + pkin(26) + t59 - t66;
t130 = t117 * t119 * t121 - t125 * t127 * t121;
t135 = t117 * t127 * t121 + t119 * t121 * t125;
t140 = t22 * t62 * t55 + t5 * t54 * t55;
t142 = t140 * pkin(23) - pkin(15) + t10 + t7;
t144 = 0.1e1 / pkin(17);
t150 = t22 * t54 * t55 - t5 * t62 * t55;
t152 = t150 * pkin(23) + t28 - t29;
t155 = -t140 * t142 * t144 - t150 * t152 * t144;
t160 = -t140 * t152 * t144 + t142 * t144 * t150;
t161 = t144 * t9;
t163 = t144 * t17;
t165 = t45 * t161 + t51 * t163;
t168 = t51 * t161 - t45 * t163;
t171 = pkin(29) + qJ(3) - qJ(4);
t172 = cos(t171);
t174 = -t172 * pkin(18) - t111 * pkin(19) - t112 * pkin(20) - pkin(16);
t175 = t174 ^ 2;
t178 = sin(t171);
t180 = -t178 * pkin(18) + t112 * pkin(19) - t111 * pkin(20);
t181 = t180 ^ 2;
t183 = sqrt(t175 + t181);
t184 = 0.1e1 / t183;
t185 = t174 * t184;
t187 = -t180 * t184;
t189 = t185 * t172 - t187 * t178;
t192 = t187 * t172 + t185 * t178;
unknown(1,1) = t1;
unknown(1,2) = t2;
unknown(1,3) = 0.0e0;
unknown(1,4) = 0.0e0;
unknown(2,1) = -t2;
unknown(2,2) = t1;
unknown(2,3) = 0.0e0;
unknown(2,4) = 0.0e0;
unknown(3,1) = 0.0e0;
unknown(3,2) = 0.0e0;
unknown(3,3) = 0.1e1;
unknown(3,4) = pkin(8);
unknown(4,1) = -t3;
unknown(4,2) = -t4;
unknown(4,3) = 0.0e0;
unknown(4,4) = 0.0e0;
unknown(5,1) = 0.0e0;
unknown(5,2) = 0.0e0;
unknown(5,3) = 0.1e1;
unknown(5,4) = pkin(9);
unknown(6,1) = -t4;
unknown(6,2) = t3;
unknown(6,3) = 0.0e0;
unknown(6,4) = 0.0e0;
unknown(7,1) = t105;
unknown(7,2) = t108;
unknown(7,3) = 0.0e0;
unknown(7,4) = 0.0e0;
unknown(8,1) = 0.0e0;
unknown(8,2) = 0.0e0;
unknown(8,3) = -0.1e1;
unknown(8,4) = -pkin(10);
unknown(9,1) = -t108;
unknown(9,2) = t105;
unknown(9,3) = 0.0e0;
unknown(9,4) = 0.0e0;
unknown(10,1) = t5;
unknown(10,2) = -t22;
unknown(10,3) = 0.0e0;
unknown(10,4) = pkin(4);
unknown(11,1) = t22;
unknown(11,2) = t5;
unknown(11,3) = 0.0e0;
unknown(11,4) = 0.0e0;
unknown(12,1) = 0.0e0;
unknown(12,2) = 0.0e0;
unknown(12,3) = 0.1e1;
unknown(12,4) = 0.0e0;
unknown(13,1) = t109;
unknown(13,2) = -t110;
unknown(13,3) = 0.0e0;
unknown(13,4) = pkin(5);
unknown(14,1) = t110;
unknown(14,2) = t109;
unknown(14,3) = 0.0e0;
unknown(14,4) = 0.0e0;
unknown(15,1) = 0.0e0;
unknown(15,2) = 0.0e0;
unknown(15,3) = 0.1e1;
unknown(15,4) = 0.0e0;
unknown(16,1) = -t111;
unknown(16,2) = -t112;
unknown(16,3) = 0.0e0;
unknown(16,4) = pkin(6);
unknown(17,1) = t112;
unknown(17,2) = -t111;
unknown(17,3) = 0.0e0;
unknown(17,4) = 0.0e0;
unknown(18,1) = 0.0e0;
unknown(18,2) = 0.0e0;
unknown(18,3) = 0.1e1;
unknown(18,4) = 0.0e0;
unknown(19,1) = t113;
unknown(19,2) = -t114;
unknown(19,3) = 0.0e0;
unknown(19,4) = 0.0e0;
unknown(20,1) = 0.0e0;
unknown(20,2) = 0.0e0;
unknown(20,3) = -0.1e1;
unknown(20,4) = -pkin(11);
unknown(21,1) = t114;
unknown(21,2) = t113;
unknown(21,3) = 0.0e0;
unknown(21,4) = 0.0e0;
unknown(22,1) = -t95;
unknown(22,2) = -t103;
unknown(22,3) = 0.0e0;
unknown(22,4) = 0.0e0;
unknown(23,1) = 0.0e0;
unknown(23,2) = 0.0e0;
unknown(23,3) = -0.1e1;
unknown(23,4) = -pkin(12);
unknown(24,1) = t103;
unknown(24,2) = -t95;
unknown(24,3) = 0.0e0;
unknown(24,4) = 0.0e0;
unknown(25,1) = t130;
unknown(25,2) = t135;
unknown(25,3) = 0.0e0;
unknown(25,4) = pkin(7);
unknown(26,1) = -t135;
unknown(26,2) = t130;
unknown(26,3) = 0.0e0;
unknown(26,4) = 0.0e0;
unknown(27,1) = 0.0e0;
unknown(27,2) = 0.0e0;
unknown(27,3) = 0.1e1;
unknown(27,4) = 0.0e0;
unknown(28,1) = t56;
unknown(28,2) = -t63;
unknown(28,3) = 0.0e0;
unknown(28,4) = pkin(1);
unknown(29,1) = t63;
unknown(29,2) = t56;
unknown(29,3) = 0.0e0;
unknown(29,4) = 0.0e0;
unknown(30,1) = 0.0e0;
unknown(30,2) = 0.0e0;
unknown(30,3) = 0.1e1;
unknown(30,4) = 0.0e0;
unknown(31,1) = t155;
unknown(31,2) = t160;
unknown(31,3) = 0.0e0;
unknown(31,4) = pkin(2);
unknown(32,1) = -t160;
unknown(32,2) = t155;
unknown(32,3) = 0.0e0;
unknown(32,4) = 0.0e0;
unknown(33,1) = 0.0e0;
unknown(33,2) = 0.0e0;
unknown(33,3) = 0.1e1;
unknown(33,4) = 0.0e0;
unknown(34,1) = t165;
unknown(34,2) = t168;
unknown(34,3) = 0.0e0;
unknown(34,4) = pkin(3);
unknown(35,1) = -t168;
unknown(35,2) = t165;
unknown(35,3) = 0.0e0;
unknown(35,4) = 0.0e0;
unknown(36,1) = 0.0e0;
unknown(36,2) = 0.0e0;
unknown(36,3) = 0.1e1;
unknown(36,4) = 0.0e0;
unknown(37,1) = t189;
unknown(37,2) = t192;
unknown(37,3) = 0.0e0;
unknown(37,4) = 0.0e0;
unknown(38,1) = -t192;
unknown(38,2) = t189;
unknown(38,3) = 0.0e0;
unknown(38,4) = 0.0e0;
unknown(39,1) = 0.0e0;
unknown(39,2) = 0.0e0;
unknown(39,3) = 0.1e1;
unknown(39,4) = 0.0e0;
T_stack = unknown;
%% Postprocessing: Reshape Output
% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)
% Fallunterscheidung der Initialisierung für symbolische Eingabe
if isa([qJ; pkin], 'double'), T_mdh = NaN(4,4,13);             % numerisch
else,                         T_mdh = sym('xx', [4,4,13]); end % symbolisch

for i = 1:13
  T_mdh(:,:,i) = [T_stack((i-1)*3+1 : 3*i, :);[0 0 0 1]];
end
