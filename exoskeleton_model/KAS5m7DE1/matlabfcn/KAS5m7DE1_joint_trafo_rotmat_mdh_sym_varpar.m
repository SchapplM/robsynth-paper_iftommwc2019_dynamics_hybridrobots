% Calculate homogenous joint transformation matrices for
% KAS5m7DE1
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
% T_stack [(21+1)*3 x 4]
%   stacked matrices from T_mdh into one 2D array, last row left out.
%   Last row only contains [0 0 0 1].

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [T_mdh, T_stack] = KAS5m7DE1_joint_trafo_rotmat_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_joint_trafo_rotmat_mdh_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_joint_trafo_rotmat_mdh_sym_varpar: pkin has to be [24x1] (double)');

%% Symbolic Calculation
% From joint_transformation_mdh_rotmat_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-25 11:26:58
% EndTime: 2020-05-25 11:27:06
% DurationCPUTime: 4.85s
% Computational Cost: add. (272346->112), mult. (351325->109), div. (5688->4), fcn. (140952->48), ass. (0->362)
unknown=NaN(63,4);
t1 = sin(qJ(1));
t2 = cos(qJ(1));
t3 = sin(qJ(2));
t4 = cos(qJ(2));
t5 = sin(qJ(3));
t6 = cos(qJ(3));
t7 = -pkin(23) + pkin(24);
t8 = t6 * t7;
t9 = pkin(3) + qJ(3);
t10 = cos(t9);
t11 = t10 * pkin(12);
t12 = -pkin(9) + t8 + t11;
t13 = pkin(11) ^ 2;
t14 = pkin(19) ^ 2;
t15 = t10 ^ 2;
t16 = pkin(12) ^ 2;
t18 = sin(t9);
t19 = t18 ^ 2;
t22 = (-pkin(9) + t8) ^ 2;
t23 = t5 ^ 2;
t24 = t7 ^ 2;
t28 = t5 * t7;
t29 = t18 * pkin(12);
t30 = -t28 + t29;
t33 = 0.2e1 * t12 * t10 * pkin(12) + 0.2e1 * t30 * t18 * pkin(12) - t15 * t16 - t19 * t16 + t23 * t24 + t13 - t14 + t22;
t37 = 0.4e1 * t12 ^ 2 + 0.4e1 * t30 ^ 2;
t39 = t33 ^ 2;
t41 = sqrt(t13 * t37 - t39);
t44 = 0.1e1 / t37;
t45 = (0.2e1 * t12 * t33 - 0.2e1 * t30 * t41) * t44;
t46 = pkin(9) - t8 - t11 + t45;
t51 = (0.2e1 * t12 * t41 + 0.2e1 * t33 * t30) * t44;
t52 = t28 - t29 + t51;
t54 = t5 * t46 + t6 * t52;
t55 = 0.1e1 / pkin(19);
t56 = t54 * t55;
t57 = cos(pkin(7));
t58 = t57 * pkin(18);
t59 = t56 * t58;
t62 = t6 * t46 - t5 * t52;
t63 = t62 * t55;
t64 = sin(pkin(7));
t65 = t64 * pkin(18);
t66 = t63 * t65;
t67 = pkin(17) ^ 2;
t68 = pkin(22) ^ 2;
t69 = t63 * t58;
t70 = t56 * t65;
t71 = t69 - t70;
t72 = t71 ^ 2;
t73 = t59 + t66;
t74 = t73 ^ 2;
t75 = pkin(24) ^ 2;
t76 = -pkin(24) - t69 + t70;
t79 = -0.2e1 * t76 * t71 + 0.2e1 * t73 ^ 2 + t67 - t68 - t72 - t74 + t75;
t83 = 0.4e1 * t73 ^ 2 + 0.4e1 * t76 ^ 2;
t85 = t79 ^ 2;
t87 = sqrt(t67 * t83 - t85);
t90 = 0.1e1 / t83;
t92 = -t59 - t66 - (-0.2e1 * t79 * t73 + 0.2e1 * t76 * t87) * t90;
t97 = t69 - t70 + (0.2e1 * t73 * t87 + 0.2e1 * t76 * t79) * t90 + pkin(24);
t98 = atan2(t92, t97);
t99 = t98 + pkin(6);
t100 = sin(t99);
t101 = cos(t99);
t102 = cos(qJ(4));
t103 = sin(qJ(4));
t104 = qJ(4) - qJ(3) + pkin(4);
t105 = sin(t104);
t106 = cos(t104);
t107 = cos(qJ(5));
t108 = sin(qJ(5));
t109 = sin(pkin(6));
t110 = cos(pkin(6));
t111 = 0.1e1 / pkin(22);
t112 = t92 * t111;
t114 = -t97 * t111;
t116 = t112 * t109 + t114 * t110;
t120 = -t114 * t109 + t112 * t110;
t130 = atan2(-(t120 * t109 - t116 * t110) * pkin(22) + pkin(24) + t69 - t70, (t116 * t109 + t120 * t110) * pkin(22) + t59 + t66);
t131 = t98 - t130;
t132 = sin(t131);
t133 = cos(t131);
t134 = atan2(-t54, t62);
t135 = cos(t134);
t136 = sin(t134);
t151 = atan2((t5 * t62 * t55 - t6 * t54 * t55) * pkin(19) + t28 - t29, (t5 * t54 * t55 + t6 * t62 * t55) * pkin(19) - pkin(9) + t8 + t11);
t152 = -t151 + qJ(3) + t134;
t153 = cos(t152);
t154 = sin(t152);
t155 = sin(pkin(3));
t156 = cos(pkin(3));
t158 = pkin(3) + qJ(3) - qJ(4);
t159 = sin(t158);
t162 = t159 * pkin(12) - t106 * pkin(14) + t105 * pkin(15);
t165 = cos(t158);
t167 = -t165 * pkin(12) - t105 * pkin(14) - t106 * pkin(15) - pkin(10);
t168 = atan2(t162, t167);
t169 = t168 + pkin(3) + qJ(3) - qJ(4);
t170 = sin(t169);
t171 = cos(t169);
t172 = t167 ^ 2;
t173 = t162 ^ 2;
t175 = sqrt(t172 + t173);
t176 = atan2(t51, t45);
t177 = -t176 + pkin(3) + qJ(3);
t178 = cos(t177);
t179 = sin(t177);
t180 = cos(pkin(5));
t181 = sin(pkin(5));
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
unknown(5,4) = pkin(16);
unknown(6,1) = -t4;
unknown(6,2) = t3;
unknown(6,3) = 0.0e0;
unknown(6,4) = 0.0e0;
unknown(7,1) = t100;
unknown(7,2) = t101;
unknown(7,3) = 0.0e0;
unknown(7,4) = 0.0e0;
unknown(8,1) = 0.0e0;
unknown(8,2) = 0.0e0;
unknown(8,3) = -0.1e1;
unknown(8,4) = -pkin(21);
unknown(9,1) = -t101;
unknown(9,2) = t100;
unknown(9,3) = 0.0e0;
unknown(9,4) = 0.0e0;
unknown(10,1) = t6;
unknown(10,2) = -t5;
unknown(10,3) = 0.0e0;
unknown(10,4) = pkin(23);
unknown(11,1) = t5;
unknown(11,2) = t6;
unknown(11,3) = 0.0e0;
unknown(11,4) = 0.0e0;
unknown(12,1) = 0.0e0;
unknown(12,2) = 0.0e0;
unknown(12,3) = 0.1e1;
unknown(12,4) = 0.0e0;
unknown(13,1) = t102;
unknown(13,2) = -t103;
unknown(13,3) = 0.0e0;
unknown(13,4) = pkin(9);
unknown(14,1) = t103;
unknown(14,2) = t102;
unknown(14,3) = 0.0e0;
unknown(14,4) = 0.0e0;
unknown(15,1) = 0.0e0;
unknown(15,2) = 0.0e0;
unknown(15,3) = 0.1e1;
unknown(15,4) = 0.0e0;
unknown(16,1) = -t105;
unknown(16,2) = -t106;
unknown(16,3) = 0.0e0;
unknown(16,4) = pkin(10);
unknown(17,1) = t106;
unknown(17,2) = -t105;
unknown(17,3) = 0.0e0;
unknown(17,4) = 0.0e0;
unknown(18,1) = 0.0e0;
unknown(18,2) = 0.0e0;
unknown(18,3) = 0.1e1;
unknown(18,4) = 0.0e0;
unknown(19,1) = t107;
unknown(19,2) = -t108;
unknown(19,3) = 0.0e0;
unknown(19,4) = 0.0e0;
unknown(20,1) = 0.0e0;
unknown(20,2) = 0.0e0;
unknown(20,3) = -0.1e1;
unknown(20,4) = -pkin(13);
unknown(21,1) = t108;
unknown(21,2) = t107;
unknown(21,3) = 0.0e0;
unknown(21,4) = 0.0e0;
unknown(22,1) = -t109;
unknown(22,2) = -t110;
unknown(22,3) = 0.0e0;
unknown(22,4) = 0.0e0;
unknown(23,1) = 0.0e0;
unknown(23,2) = 0.0e0;
unknown(23,3) = -0.1e1;
unknown(23,4) = -pkin(21);
unknown(24,1) = t110;
unknown(24,2) = -t109;
unknown(24,3) = 0.0e0;
unknown(24,4) = 0.0e0;
unknown(25,1) = t132;
unknown(25,2) = t133;
unknown(25,3) = 0.0e0;
unknown(25,4) = -pkin(22);
unknown(26,1) = -t133;
unknown(26,2) = t132;
unknown(26,3) = 0.0e0;
unknown(26,4) = 0.0e0;
unknown(27,1) = 0.0e0;
unknown(27,2) = 0.0e0;
unknown(27,3) = 0.1e1;
unknown(27,4) = 0.0e0;
unknown(28,1) = t135;
unknown(28,2) = t136;
unknown(28,3) = 0.0e0;
unknown(28,4) = pkin(24);
unknown(29,1) = -t136;
unknown(29,2) = t135;
unknown(29,3) = 0.0e0;
unknown(29,4) = 0.0e0;
unknown(30,1) = 0.0e0;
unknown(30,2) = 0.0e0;
unknown(30,3) = 0.1e1;
unknown(30,4) = 0.0e0;
unknown(31,1) = -t153;
unknown(31,2) = t154;
unknown(31,3) = 0.0e0;
unknown(31,4) = pkin(19);
unknown(32,1) = -t154;
unknown(32,2) = -t153;
unknown(32,3) = 0.0e0;
unknown(32,4) = 0.0e0;
unknown(33,1) = 0.0e0;
unknown(33,2) = 0.0e0;
unknown(33,3) = 0.1e1;
unknown(33,4) = 0.0e0;
unknown(34,1) = t5;
unknown(34,2) = t6;
unknown(34,3) = 0.0e0;
unknown(34,4) = pkin(9);
unknown(35,1) = -t6;
unknown(35,2) = t5;
unknown(35,3) = 0.0e0;
unknown(35,4) = 0.0e0;
unknown(36,1) = 0.0e0;
unknown(36,2) = 0.0e0;
unknown(36,3) = 0.1e1;
unknown(36,4) = 0.0e0;
unknown(37,1) = t155;
unknown(37,2) = t156;
unknown(37,3) = 0.0e0;
unknown(37,4) = 0.0e0;
unknown(38,1) = -t156;
unknown(38,2) = t155;
unknown(38,3) = 0.0e0;
unknown(38,4) = 0.0e0;
unknown(39,1) = 0.0e0;
unknown(39,2) = 0.0e0;
unknown(39,3) = 0.1e1;
unknown(39,4) = 0.0e0;
unknown(40,1) = t170;
unknown(40,2) = -t171;
unknown(40,3) = 0.0e0;
unknown(40,4) = pkin(12);
unknown(41,1) = t171;
unknown(41,2) = t170;
unknown(41,3) = 0.0e0;
unknown(41,4) = 0.0e0;
unknown(42,1) = 0.0e0;
unknown(42,2) = 0.0e0;
unknown(42,3) = 0.1e1;
unknown(42,4) = 0.0e0;
unknown(43,1) = 0.1e1;
unknown(43,2) = 0.0e0;
unknown(43,3) = 0.0e0;
unknown(43,4) = 0.0e0;
unknown(44,1) = 0.0e0;
unknown(44,2) = 0.0e0;
unknown(44,3) = -0.1e1;
unknown(44,4) = -t175;
unknown(45,1) = 0.0e0;
unknown(45,2) = 0.1e1;
unknown(45,3) = 0.0e0;
unknown(45,4) = 0.0e0;
unknown(46,1) = t57;
unknown(46,2) = -t64;
unknown(46,3) = 0.0e0;
unknown(46,4) = t58;
unknown(47,1) = t64;
unknown(47,2) = t57;
unknown(47,3) = 0.0e0;
unknown(47,4) = t65;
unknown(48,1) = 0.0e0;
unknown(48,2) = 0.0e0;
unknown(48,3) = 0.1e1;
unknown(48,4) = 0.0e0;
unknown(49,1) = t178;
unknown(49,2) = t179;
unknown(49,3) = 0.0e0;
unknown(49,4) = pkin(12);
unknown(50,1) = -t179;
unknown(50,2) = t178;
unknown(50,3) = 0.0e0;
unknown(50,4) = 0.0e0;
unknown(51,1) = 0.0e0;
unknown(51,2) = 0.0e0;
unknown(51,3) = 0.1e1;
unknown(51,4) = 0.0e0;
unknown(52,1) = t180;
unknown(52,2) = t181;
unknown(52,3) = 0.0e0;
unknown(52,4) = -t181 * pkin(20);
unknown(53,1) = -t181;
unknown(53,2) = t180;
unknown(53,3) = 0.0e0;
unknown(53,4) = -t180 * pkin(20);
unknown(54,1) = 0.0e0;
unknown(54,2) = 0.0e0;
unknown(54,3) = 0.1e1;
unknown(54,4) = 0.0e0;
unknown(55,1) = 0.1e1;
unknown(55,2) = 0.0e0;
unknown(55,3) = 0.0e0;
unknown(55,4) = pkin(17);
unknown(56,1) = 0.0e0;
unknown(56,2) = 0.1e1;
unknown(56,3) = 0.0e0;
unknown(56,4) = 0.0e0;
unknown(57,1) = 0.0e0;
unknown(57,2) = 0.0e0;
unknown(57,3) = 0.1e1;
unknown(57,4) = 0.0e0;
unknown(58,1) = 0.1e1;
unknown(58,2) = 0.0e0;
unknown(58,3) = 0.0e0;
unknown(58,4) = pkin(11);
unknown(59,1) = 0.0e0;
unknown(59,2) = 0.1e1;
unknown(59,3) = 0.0e0;
unknown(59,4) = 0.0e0;
unknown(60,1) = 0.0e0;
unknown(60,2) = 0.0e0;
unknown(60,3) = 0.1e1;
unknown(60,4) = 0.0e0;
unknown(61,1) = 0.1e1;
unknown(61,2) = 0.0e0;
unknown(61,3) = 0.0e0;
unknown(61,4) = 0.0e0;
unknown(62,1) = 0.0e0;
unknown(62,2) = 0.0e0;
unknown(62,3) = 0.1e1;
unknown(62,4) = 0.0e0;
unknown(63,1) = 0.0e0;
unknown(63,2) = -0.1e1;
unknown(63,3) = 0.0e0;
unknown(63,4) = 0.0e0;
T_stack = unknown;
%% Postprocessing: Reshape Output
% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)
% Fallunterscheidung der Initialisierung für symbolische Eingabe
if isa([qJ; pkin], 'double'), T_mdh = NaN(4,4,21);             % numerisch
else,                         T_mdh = sym('xx', [4,4,21]); end % symbolisch

for i = 1:21
  T_mdh(:,:,i) = [T_stack((i-1)*3+1 : 3*i, :);[0 0 0 1]];
end
