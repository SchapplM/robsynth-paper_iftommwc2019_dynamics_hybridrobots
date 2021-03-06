% Calculate homogenous joint transformation matrices for
% KAS5m7TE
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% T_mdh [4x4x21]
%   homogenous transformation matrices for joint transformation (MDH)
%   Transformation matrices from one joint to the next (not: from base to joints)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T_mdh = KAS5m7TE_joint_trafo_rotmat_mdh_sym_varpar(qJ, ...
  pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_joint_trafo_rotmat_mdh_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_joint_trafo_rotmat_mdh_sym_varpar: pkin has to be [24x1] (double)');

%% Symbolic Calculation
% From joint_transformation_mdh_rotmat_matlab.m
% OptimizationMode: 1
% StartTime: 2020-04-13 18:10:47
% EndTime: 2020-04-13 18:10:53
% DurationCPUTime: 6.52s
% Computational Cost: add. (405423->107), mult. (523053->137), div. (8540->7), fcn. (209792->30), ass. (0->444)
unknown=NaN(84,4);
t1 = sin(qJ(1));
t2 = cos(qJ(1));
t3 = sin(qJ(2));
t4 = cos(qJ(2));
t5 = cos(qJ(3));
t6 = -pkin(23) + pkin(24);
t7 = t5 * t6;
t8 = pkin(3) + qJ(3);
t9 = cos(t8);
t10 = t9 * pkin(12);
t11 = -pkin(9) + t7 + t10;
t12 = pkin(11) ^ 2;
t13 = pkin(19) ^ 2;
t14 = t9 ^ 2;
t15 = pkin(12) ^ 2;
t17 = sin(t8);
t18 = t17 ^ 2;
t21 = (-pkin(9) + t7) ^ 2;
t22 = sin(qJ(3));
t23 = t22 ^ 2;
t24 = t6 ^ 2;
t28 = t22 * t6;
t29 = t17 * pkin(12);
t30 = -t28 + t29;
t33 = 0.2e1 * t11 * t9 * pkin(12) + 0.2e1 * t30 * t17 * pkin(12) - t14 * t15 - t18 * t15 + t23 * t24 + t12 - t13 + t21;
t37 = 0.4e1 * t11 ^ 2 + 0.4e1 * t30 ^ 2;
t39 = t33 ^ 2;
t41 = sqrt(t12 * t37 - t39);
t44 = 0.1e1 / t37;
t45 = (0.2e1 * t11 * t33 - 0.2e1 * t30 * t41) * t44;
t46 = pkin(9) - t7 - t10 + t45;
t51 = (0.2e1 * t11 * t41 + 0.2e1 * t33 * t30) * t44;
t52 = t28 - t29 + t51;
t54 = -t22 * t52 + t5 * t46;
t55 = 0.1e1 / pkin(19);
t56 = t54 * t55;
t57 = cos(pkin(7));
t58 = t57 * pkin(18);
t59 = t56 * t58;
t62 = t22 * t46 + t5 * t52;
t63 = t62 * t55;
t64 = sin(pkin(7));
t65 = t64 * pkin(18);
t66 = t63 * t65;
t67 = -pkin(24) - t59 + t66;
t68 = pkin(17) ^ 2;
t69 = pkin(22) ^ 2;
t70 = t59 - t66;
t71 = t70 ^ 2;
t72 = t63 * t58;
t73 = t56 * t65;
t74 = t72 + t73;
t75 = t74 ^ 2;
t76 = pkin(24) ^ 2;
t79 = -0.2e1 * t67 * t70 + 0.2e1 * t74 ^ 2 + t68 - t69 - t71 - t75 + t76;
t83 = 0.4e1 * t67 ^ 2 + 0.4e1 * t74 ^ 2;
t85 = t79 ^ 2;
t87 = sqrt(t68 * t83 - t85);
t90 = 0.1e1 / t83;
t93 = 0.1e1 / pkin(22);
t94 = (-t59 + t66 - (0.2e1 * t67 * t79 + 0.2e1 * t74 * t87) * t90 - pkin(24)) * t93;
t95 = sin(pkin(6));
t102 = (-t72 - t73 - (0.2e1 * t67 * t87 - 0.2e1 * t79 * t74) * t90) * t93;
t103 = cos(pkin(6));
t105 = t102 * t103 - t94 * t95;
t108 = -t102 * t95 - t94 * t103;
t109 = cos(qJ(4));
t110 = sin(qJ(4));
t111 = qJ(4) - qJ(3) + pkin(4);
t112 = sin(t111);
t113 = cos(t111);
t114 = cos(qJ(5));
t115 = sin(qJ(5));
t118 = t105 * t103 - t108 * t95;
t120 = t118 * pkin(22) + t72 + t73;
t122 = 0.1e1 / pkin(17);
t126 = t108 * t103 + t105 * t95;
t128 = -t126 * pkin(22) + pkin(24) + t59 - t66;
t131 = t118 * t120 * t122 - t126 * t128 * t122;
t136 = t118 * t128 * t122 + t120 * t122 * t126;
t141 = t22 * t62 * t55 + t5 * t54 * t55;
t143 = t141 * pkin(19) - pkin(9) + t10 + t7;
t145 = 0.1e1 / pkin(11);
t151 = t22 * t54 * t55 - t5 * t62 * t55;
t153 = t151 * pkin(19) + t28 - t29;
t156 = -t141 * t143 * t145 - t151 * t153 * t145;
t161 = -t141 * t153 * t145 + t143 * t145 * t151;
t162 = sin(pkin(3));
t163 = cos(pkin(3));
t165 = pkin(3) + qJ(3) - qJ(4);
t166 = sin(t165);
t169 = t166 * pkin(12) - t113 * pkin(14) + t112 * pkin(15);
t172 = cos(t165);
t174 = -t172 * pkin(12) - t112 * pkin(14) - t113 * pkin(15) - pkin(10);
t175 = t174 ^ 2;
t176 = t169 ^ 2;
t178 = sqrt(t175 + t176);
t179 = 0.1e1 / t178;
t180 = t169 * t179;
t182 = t174 * t179;
t184 = t182 * t166 + t180 * t172;
t187 = t180 * t166 - t182 * t172;
t188 = t145 * t9;
t190 = t145 * t17;
t192 = t45 * t188 + t51 * t190;
t195 = -t51 * t188 + t45 * t190;
t196 = cos(pkin(5));
t197 = sin(pkin(5));
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
unknown(4,1) = 0.0e0;
unknown(4,2) = 0.0e0;
unknown(4,3) = 0.0e0;
unknown(4,4) = 0.1e1;
unknown(5,1) = -t3;
unknown(5,2) = -t4;
unknown(5,3) = 0.0e0;
unknown(5,4) = 0.0e0;
unknown(6,1) = 0.0e0;
unknown(6,2) = 0.0e0;
unknown(6,3) = 0.1e1;
unknown(6,4) = pkin(16);
unknown(7,1) = -t4;
unknown(7,2) = t3;
unknown(7,3) = 0.0e0;
unknown(7,4) = 0.0e0;
unknown(8,1) = 0.0e0;
unknown(8,2) = 0.0e0;
unknown(8,3) = 0.0e0;
unknown(8,4) = 0.1e1;
unknown(9,1) = t105;
unknown(9,2) = t108;
unknown(9,3) = 0.0e0;
unknown(9,4) = 0.0e0;
unknown(10,1) = 0.0e0;
unknown(10,2) = 0.0e0;
unknown(10,3) = -0.1e1;
unknown(10,4) = -pkin(21);
unknown(11,1) = -t108;
unknown(11,2) = t105;
unknown(11,3) = 0.0e0;
unknown(11,4) = 0.0e0;
unknown(12,1) = 0.0e0;
unknown(12,2) = 0.0e0;
unknown(12,3) = 0.0e0;
unknown(12,4) = 0.1e1;
unknown(13,1) = t5;
unknown(13,2) = -t22;
unknown(13,3) = 0.0e0;
unknown(13,4) = pkin(23);
unknown(14,1) = t22;
unknown(14,2) = t5;
unknown(14,3) = 0.0e0;
unknown(14,4) = 0.0e0;
unknown(15,1) = 0.0e0;
unknown(15,2) = 0.0e0;
unknown(15,3) = 0.1e1;
unknown(15,4) = 0.0e0;
unknown(16,1) = 0.0e0;
unknown(16,2) = 0.0e0;
unknown(16,3) = 0.0e0;
unknown(16,4) = 0.1e1;
unknown(17,1) = t109;
unknown(17,2) = -t110;
unknown(17,3) = 0.0e0;
unknown(17,4) = pkin(9);
unknown(18,1) = t110;
unknown(18,2) = t109;
unknown(18,3) = 0.0e0;
unknown(18,4) = 0.0e0;
unknown(19,1) = 0.0e0;
unknown(19,2) = 0.0e0;
unknown(19,3) = 0.1e1;
unknown(19,4) = 0.0e0;
unknown(20,1) = 0.0e0;
unknown(20,2) = 0.0e0;
unknown(20,3) = 0.0e0;
unknown(20,4) = 0.1e1;
unknown(21,1) = -t112;
unknown(21,2) = -t113;
unknown(21,3) = 0.0e0;
unknown(21,4) = pkin(10);
unknown(22,1) = t113;
unknown(22,2) = -t112;
unknown(22,3) = 0.0e0;
unknown(22,4) = 0.0e0;
unknown(23,1) = 0.0e0;
unknown(23,2) = 0.0e0;
unknown(23,3) = 0.1e1;
unknown(23,4) = 0.0e0;
unknown(24,1) = 0.0e0;
unknown(24,2) = 0.0e0;
unknown(24,3) = 0.0e0;
unknown(24,4) = 0.1e1;
unknown(25,1) = t114;
unknown(25,2) = -t115;
unknown(25,3) = 0.0e0;
unknown(25,4) = 0.0e0;
unknown(26,1) = 0.0e0;
unknown(26,2) = 0.0e0;
unknown(26,3) = -0.1e1;
unknown(26,4) = -pkin(13);
unknown(27,1) = t115;
unknown(27,2) = t114;
unknown(27,3) = 0.0e0;
unknown(27,4) = 0.0e0;
unknown(28,1) = 0.0e0;
unknown(28,2) = 0.0e0;
unknown(28,3) = 0.0e0;
unknown(28,4) = 0.1e1;
unknown(29,1) = -t95;
unknown(29,2) = -t103;
unknown(29,3) = 0.0e0;
unknown(29,4) = 0.0e0;
unknown(30,1) = 0.0e0;
unknown(30,2) = 0.0e0;
unknown(30,3) = -0.1e1;
unknown(30,4) = -pkin(21);
unknown(31,1) = t103;
unknown(31,2) = -t95;
unknown(31,3) = 0.0e0;
unknown(31,4) = 0.0e0;
unknown(32,1) = 0.0e0;
unknown(32,2) = 0.0e0;
unknown(32,3) = 0.0e0;
unknown(32,4) = 0.1e1;
unknown(33,1) = t131;
unknown(33,2) = t136;
unknown(33,3) = 0.0e0;
unknown(33,4) = -pkin(22);
unknown(34,1) = -t136;
unknown(34,2) = t131;
unknown(34,3) = 0.0e0;
unknown(34,4) = 0.0e0;
unknown(35,1) = 0.0e0;
unknown(35,2) = 0.0e0;
unknown(35,3) = 0.1e1;
unknown(35,4) = 0.0e0;
unknown(36,1) = 0.0e0;
unknown(36,2) = 0.0e0;
unknown(36,3) = 0.0e0;
unknown(36,4) = 0.1e1;
unknown(37,1) = t56;
unknown(37,2) = -t63;
unknown(37,3) = 0.0e0;
unknown(37,4) = pkin(24);
unknown(38,1) = t63;
unknown(38,2) = t56;
unknown(38,3) = 0.0e0;
unknown(38,4) = 0.0e0;
unknown(39,1) = 0.0e0;
unknown(39,2) = 0.0e0;
unknown(39,3) = 0.1e1;
unknown(39,4) = 0.0e0;
unknown(40,1) = 0.0e0;
unknown(40,2) = 0.0e0;
unknown(40,3) = 0.0e0;
unknown(40,4) = 0.1e1;
unknown(41,1) = t156;
unknown(41,2) = t161;
unknown(41,3) = 0.0e0;
unknown(41,4) = pkin(19);
unknown(42,1) = -t161;
unknown(42,2) = t156;
unknown(42,3) = 0.0e0;
unknown(42,4) = 0.0e0;
unknown(43,1) = 0.0e0;
unknown(43,2) = 0.0e0;
unknown(43,3) = 0.1e1;
unknown(43,4) = 0.0e0;
unknown(44,1) = 0.0e0;
unknown(44,2) = 0.0e0;
unknown(44,3) = 0.0e0;
unknown(44,4) = 0.1e1;
unknown(45,1) = t22;
unknown(45,2) = t5;
unknown(45,3) = 0.0e0;
unknown(45,4) = pkin(9);
unknown(46,1) = -t5;
unknown(46,2) = t22;
unknown(46,3) = 0.0e0;
unknown(46,4) = 0.0e0;
unknown(47,1) = 0.0e0;
unknown(47,2) = 0.0e0;
unknown(47,3) = 0.1e1;
unknown(47,4) = 0.0e0;
unknown(48,1) = 0.0e0;
unknown(48,2) = 0.0e0;
unknown(48,3) = 0.0e0;
unknown(48,4) = 0.1e1;
unknown(49,1) = t162;
unknown(49,2) = t163;
unknown(49,3) = 0.0e0;
unknown(49,4) = 0.0e0;
unknown(50,1) = -t163;
unknown(50,2) = t162;
unknown(50,3) = 0.0e0;
unknown(50,4) = 0.0e0;
unknown(51,1) = 0.0e0;
unknown(51,2) = 0.0e0;
unknown(51,3) = 0.1e1;
unknown(51,4) = 0.0e0;
unknown(52,1) = 0.0e0;
unknown(52,2) = 0.0e0;
unknown(52,3) = 0.0e0;
unknown(52,4) = 0.1e1;
unknown(53,1) = t184;
unknown(53,2) = t187;
unknown(53,3) = 0.0e0;
unknown(53,4) = pkin(12);
unknown(54,1) = -t187;
unknown(54,2) = t184;
unknown(54,3) = 0.0e0;
unknown(54,4) = 0.0e0;
unknown(55,1) = 0.0e0;
unknown(55,2) = 0.0e0;
unknown(55,3) = 0.1e1;
unknown(55,4) = 0.0e0;
unknown(56,1) = 0.0e0;
unknown(56,2) = 0.0e0;
unknown(56,3) = 0.0e0;
unknown(56,4) = 0.1e1;
unknown(57,1) = 0.1e1;
unknown(57,2) = 0.0e0;
unknown(57,3) = 0.0e0;
unknown(57,4) = 0.0e0;
unknown(58,1) = 0.0e0;
unknown(58,2) = 0.0e0;
unknown(58,3) = -0.1e1;
unknown(58,4) = -t178;
unknown(59,1) = 0.0e0;
unknown(59,2) = 0.1e1;
unknown(59,3) = 0.0e0;
unknown(59,4) = 0.0e0;
unknown(60,1) = 0.0e0;
unknown(60,2) = 0.0e0;
unknown(60,3) = 0.0e0;
unknown(60,4) = 0.1e1;
unknown(61,1) = t57;
unknown(61,2) = -t64;
unknown(61,3) = 0.0e0;
unknown(61,4) = t58;
unknown(62,1) = t64;
unknown(62,2) = t57;
unknown(62,3) = 0.0e0;
unknown(62,4) = t65;
unknown(63,1) = 0.0e0;
unknown(63,2) = 0.0e0;
unknown(63,3) = 0.1e1;
unknown(63,4) = 0.0e0;
unknown(64,1) = 0.0e0;
unknown(64,2) = 0.0e0;
unknown(64,3) = 0.0e0;
unknown(64,4) = 0.1e1;
unknown(65,1) = t192;
unknown(65,2) = t195;
unknown(65,3) = 0.0e0;
unknown(65,4) = pkin(12);
unknown(66,1) = -t195;
unknown(66,2) = t192;
unknown(66,3) = 0.0e0;
unknown(66,4) = 0.0e0;
unknown(67,1) = 0.0e0;
unknown(67,2) = 0.0e0;
unknown(67,3) = 0.1e1;
unknown(67,4) = 0.0e0;
unknown(68,1) = 0.0e0;
unknown(68,2) = 0.0e0;
unknown(68,3) = 0.0e0;
unknown(68,4) = 0.1e1;
unknown(69,1) = t196;
unknown(69,2) = t197;
unknown(69,3) = 0.0e0;
unknown(69,4) = -t197 * pkin(20);
unknown(70,1) = -t197;
unknown(70,2) = t196;
unknown(70,3) = 0.0e0;
unknown(70,4) = -t196 * pkin(20);
unknown(71,1) = 0.0e0;
unknown(71,2) = 0.0e0;
unknown(71,3) = 0.1e1;
unknown(71,4) = 0.0e0;
unknown(72,1) = 0.0e0;
unknown(72,2) = 0.0e0;
unknown(72,3) = 0.0e0;
unknown(72,4) = 0.1e1;
unknown(73,1) = 0.1e1;
unknown(73,2) = 0.0e0;
unknown(73,3) = 0.0e0;
unknown(73,4) = pkin(17);
unknown(74,1) = 0.0e0;
unknown(74,2) = 0.1e1;
unknown(74,3) = 0.0e0;
unknown(74,4) = 0.0e0;
unknown(75,1) = 0.0e0;
unknown(75,2) = 0.0e0;
unknown(75,3) = 0.1e1;
unknown(75,4) = 0.0e0;
unknown(76,1) = 0.0e0;
unknown(76,2) = 0.0e0;
unknown(76,3) = 0.0e0;
unknown(76,4) = 0.1e1;
unknown(77,1) = 0.1e1;
unknown(77,2) = 0.0e0;
unknown(77,3) = 0.0e0;
unknown(77,4) = pkin(11);
unknown(78,1) = 0.0e0;
unknown(78,2) = 0.1e1;
unknown(78,3) = 0.0e0;
unknown(78,4) = 0.0e0;
unknown(79,1) = 0.0e0;
unknown(79,2) = 0.0e0;
unknown(79,3) = 0.1e1;
unknown(79,4) = 0.0e0;
unknown(80,1) = 0.0e0;
unknown(80,2) = 0.0e0;
unknown(80,3) = 0.0e0;
unknown(80,4) = 0.1e1;
unknown(81,1) = 0.1e1;
unknown(81,2) = 0.0e0;
unknown(81,3) = 0.0e0;
unknown(81,4) = 0.0e0;
unknown(82,1) = 0.0e0;
unknown(82,2) = 0.0e0;
unknown(82,3) = 0.1e1;
unknown(82,4) = 0.0e0;
unknown(83,1) = 0.0e0;
unknown(83,2) = -0.1e1;
unknown(83,3) = 0.0e0;
unknown(83,4) = 0.0e0;
unknown(84,1) = 0.0e0;
unknown(84,2) = 0.0e0;
unknown(84,3) = 0.0e0;
unknown(84,4) = 0.1e1;
T_ges = unknown;
%% Postprocessing: Reshape Output
% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)
% Fallunterscheidung der Initialisierung für symbolische Eingabe
if isa([qJ; pkin], 'double'), T_mdh = NaN(4,4,21);             % numerisch
else,                         T_mdh = sym('xx', [4,4,21]); end % symbolisch

for i = 1:21
  T_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);
end
