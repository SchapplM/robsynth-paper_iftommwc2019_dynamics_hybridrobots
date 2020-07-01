% Calculate homogenous joint transformation matrices for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% T_mdh [4x4x21]
%   homogenous transformation matrices for joint transformation (MDH)
%   Transformation matrices from one joint to the next (not: from base to joints)
% T_stack [(21+1)*3 x 4]
%   stacked matrices from T_mdh into one 2D array, last row left out.
%   Last row only contains [0 0 0 1].

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [T_mdh, T_stack] = KAS5m7OL_joint_trafo_rotmat_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_joint_trafo_rotmat_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_joint_trafo_rotmat_mdh_sym_varpar: pkin has to be [19x1] (double)');

%% Symbolic Calculation
% From joint_transformation_mdh_rotmat_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:17
% EndTime: 2020-06-30 17:46:18
% DurationCPUTime: 0.16s
% Computational Cost: add. (33->33), mult. (4->4), div. (0->0), fcn. (68->32), ass. (0->284)
unknown=NaN(63,4);
t1 = sin(qJ(1));
t2 = cos(qJ(1));
t3 = sin(qJ(2));
t4 = cos(qJ(2));
t5 = cos(qJ(3));
t6 = sin(qJ(3));
t7 = cos(qJ(4));
t8 = sin(qJ(4));
t9 = cos(qJ(5));
t10 = sin(qJ(5));
t11 = sin(qJ(6));
t12 = cos(qJ(6));
t13 = cos(qJ(7));
t14 = sin(qJ(7));
t15 = sin(pkin(3));
t16 = cos(pkin(3));
t17 = cos(qJ(8));
t18 = sin(qJ(8));
t19 = cos(qJ(9));
t20 = sin(qJ(9));
t21 = cos(qJ(10));
t22 = sin(qJ(10));
t23 = sin(qJ(11));
t24 = cos(qJ(11));
t25 = sin(pkin(1));
t26 = cos(pkin(1));
t27 = sin(qJ(12));
t28 = cos(qJ(12));
t29 = cos(pkin(4));
t30 = sin(pkin(4));
t33 = sin(pkin(2));
t34 = cos(pkin(2));
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
unknown(3,4) = pkin(5);
unknown(4,1) = -t3;
unknown(4,2) = -t4;
unknown(4,3) = 0.0e0;
unknown(4,4) = 0.0e0;
unknown(5,1) = 0.0e0;
unknown(5,2) = 0.0e0;
unknown(5,3) = 0.1e1;
unknown(5,4) = pkin(11);
unknown(6,1) = -t4;
unknown(6,2) = t3;
unknown(6,3) = 0.0e0;
unknown(6,4) = 0.0e0;
unknown(7,1) = t5;
unknown(7,2) = -t6;
unknown(7,3) = 0.0e0;
unknown(7,4) = 0.0e0;
unknown(8,1) = 0.0e0;
unknown(8,2) = 0.0e0;
unknown(8,3) = -0.1e1;
unknown(8,4) = -pkin(16);
unknown(9,1) = t6;
unknown(9,2) = t5;
unknown(9,3) = 0.0e0;
unknown(9,4) = 0.0e0;
unknown(10,1) = t7;
unknown(10,2) = -t8;
unknown(10,3) = 0.0e0;
unknown(10,4) = pkin(18);
unknown(11,1) = t8;
unknown(11,2) = t7;
unknown(11,3) = 0.0e0;
unknown(11,4) = 0.0e0;
unknown(12,1) = 0.0e0;
unknown(12,2) = 0.0e0;
unknown(12,3) = 0.1e1;
unknown(12,4) = 0.0e0;
unknown(13,1) = t9;
unknown(13,2) = -t10;
unknown(13,3) = 0.0e0;
unknown(13,4) = pkin(6);
unknown(14,1) = t10;
unknown(14,2) = t9;
unknown(14,3) = 0.0e0;
unknown(14,4) = 0.0e0;
unknown(15,1) = 0.0e0;
unknown(15,2) = 0.0e0;
unknown(15,3) = 0.1e1;
unknown(15,4) = 0.0e0;
unknown(16,1) = -t11;
unknown(16,2) = -t12;
unknown(16,3) = 0.0e0;
unknown(16,4) = pkin(7);
unknown(17,1) = t12;
unknown(17,2) = -t11;
unknown(17,3) = 0.0e0;
unknown(17,4) = 0.0e0;
unknown(18,1) = 0.0e0;
unknown(18,2) = 0.0e0;
unknown(18,3) = 0.1e1;
unknown(18,4) = 0.0e0;
unknown(19,1) = t13;
unknown(19,2) = -t14;
unknown(19,3) = 0.0e0;
unknown(19,4) = 0.0e0;
unknown(20,1) = 0.0e0;
unknown(20,2) = 0.0e0;
unknown(20,3) = -0.1e1;
unknown(20,4) = -pkin(10);
unknown(21,1) = t14;
unknown(21,2) = t13;
unknown(21,3) = 0.0e0;
unknown(21,4) = 0.0e0;
unknown(22,1) = -t15;
unknown(22,2) = -t16;
unknown(22,3) = 0.0e0;
unknown(22,4) = 0.0e0;
unknown(23,1) = 0.0e0;
unknown(23,2) = 0.0e0;
unknown(23,3) = -0.1e1;
unknown(23,4) = -pkin(16);
unknown(24,1) = t16;
unknown(24,2) = -t15;
unknown(24,3) = 0.0e0;
unknown(24,4) = 0.0e0;
unknown(25,1) = t17;
unknown(25,2) = -t18;
unknown(25,3) = 0.0e0;
unknown(25,4) = -pkin(17);
unknown(26,1) = t18;
unknown(26,2) = t17;
unknown(26,3) = 0.0e0;
unknown(26,4) = 0.0e0;
unknown(27,1) = 0.0e0;
unknown(27,2) = 0.0e0;
unknown(27,3) = 0.1e1;
unknown(27,4) = 0.0e0;
unknown(28,1) = t19;
unknown(28,2) = -t20;
unknown(28,3) = 0.0e0;
unknown(28,4) = pkin(19);
unknown(29,1) = t20;
unknown(29,2) = t19;
unknown(29,3) = 0.0e0;
unknown(29,4) = 0.0e0;
unknown(30,1) = 0.0e0;
unknown(30,2) = 0.0e0;
unknown(30,3) = 0.1e1;
unknown(30,4) = 0.0e0;
unknown(31,1) = -t21;
unknown(31,2) = t22;
unknown(31,3) = 0.0e0;
unknown(31,4) = pkin(14);
unknown(32,1) = -t22;
unknown(32,2) = -t21;
unknown(32,3) = 0.0e0;
unknown(32,4) = 0.0e0;
unknown(33,1) = 0.0e0;
unknown(33,2) = 0.0e0;
unknown(33,3) = 0.1e1;
unknown(33,4) = 0.0e0;
unknown(34,1) = t23;
unknown(34,2) = t24;
unknown(34,3) = 0.0e0;
unknown(34,4) = pkin(6);
unknown(35,1) = -t24;
unknown(35,2) = t23;
unknown(35,3) = 0.0e0;
unknown(35,4) = 0.0e0;
unknown(36,1) = 0.0e0;
unknown(36,2) = 0.0e0;
unknown(36,3) = 0.1e1;
unknown(36,4) = 0.0e0;
unknown(37,1) = t25;
unknown(37,2) = t26;
unknown(37,3) = 0.0e0;
unknown(37,4) = 0.0e0;
unknown(38,1) = -t26;
unknown(38,2) = t25;
unknown(38,3) = 0.0e0;
unknown(38,4) = 0.0e0;
unknown(39,1) = 0.0e0;
unknown(39,2) = 0.0e0;
unknown(39,3) = 0.1e1;
unknown(39,4) = 0.0e0;
unknown(40,1) = t27;
unknown(40,2) = t28;
unknown(40,3) = 0.0e0;
unknown(40,4) = pkin(9);
unknown(41,1) = -t28;
unknown(41,2) = t27;
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
unknown(44,4) = -qJ(13);
unknown(45,1) = 0.0e0;
unknown(45,2) = 0.1e1;
unknown(45,3) = 0.0e0;
unknown(45,4) = 0.0e0;
unknown(46,1) = t29;
unknown(46,2) = -t30;
unknown(46,3) = 0.0e0;
unknown(46,4) = t29 * pkin(13);
unknown(47,1) = t30;
unknown(47,2) = t29;
unknown(47,3) = 0.0e0;
unknown(47,4) = t30 * pkin(13);
unknown(48,1) = 0.0e0;
unknown(48,2) = 0.0e0;
unknown(48,3) = 0.1e1;
unknown(48,4) = 0.0e0;
unknown(49,1) = 0.0e0;
unknown(49,2) = -0.1e1;
unknown(49,3) = 0.0e0;
unknown(49,4) = pkin(9);
unknown(50,1) = 0.1e1;
unknown(50,2) = 0.0e0;
unknown(50,3) = 0.0e0;
unknown(50,4) = 0.0e0;
unknown(51,1) = 0.0e0;
unknown(51,2) = 0.0e0;
unknown(51,3) = 0.1e1;
unknown(51,4) = 0.0e0;
unknown(52,1) = -t33;
unknown(52,2) = t34;
unknown(52,3) = 0.0e0;
unknown(52,4) = -t33 * pkin(15);
unknown(53,1) = -t34;
unknown(53,2) = -t33;
unknown(53,3) = 0.0e0;
unknown(53,4) = -t34 * pkin(15);
unknown(54,1) = 0.0e0;
unknown(54,2) = 0.0e0;
unknown(54,3) = 0.1e1;
unknown(54,4) = 0.0e0;
unknown(55,1) = 0.1e1;
unknown(55,2) = 0.0e0;
unknown(55,3) = 0.0e0;
unknown(55,4) = pkin(12);
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
unknown(58,4) = pkin(8);
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
