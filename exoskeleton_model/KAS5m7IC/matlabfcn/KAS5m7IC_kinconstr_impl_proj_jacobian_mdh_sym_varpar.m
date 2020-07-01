% Jacobian of implicit kinematic constraints of
% KAS5m7IC
% projection from active to passive joints coordinates
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [20x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% B21 [(no of passive joints)x(no of active joints)] 

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:50
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function B21 = KAS5m7IC_kinconstr_impl_proj_jacobian_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(20,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7IC_kinconstr_impl_proj_jacobian_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7IC_kinconstr_impl_proj_jacobian_mdh_sym_varpar: pkin has to be [20x1] (double)');

%% Symbolic Calculation
% From kinconstr_impl_projection_jacobian_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 18:48:51
% EndTime: 2020-06-30 18:48:51
% DurationCPUTime: 0.12s
% Computational Cost: add. (219->119), mult. (101->95), div. (12->7), fcn. (87->37), ass. (0->97)
unknown=NaN(8,5);
t1 = 2 * qJ(9);
t3 = sin((qJ(4) + qJ(11) + pkin(1) - t1 - qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3)));
t5 = qJ(4) + qJ(11) + pkin(1);
t6 = sin(t5);
t8 = sin(qJ(4));
t10 = -pkin(7) * t8 + pkin(10) * t6;
t11 = -t1 - qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3);
t12 = cos(t11);
t14 = cos(qJ(4));
t16 = cos(t5);
t17 = pkin(10) * t16;
t18 = -pkin(7) * t14 + t17;
t19 = sin(t11);
t22 = sin((qJ(4) + qJ(11) + pkin(1) - qJ(10) + pkin(5) - pkin(4) - qJ(8) + qJ(3)));
t24 = qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3);
t25 = cos(t24);
t27 = sin(t24);
t31 = 0.1e1 / pkin(15);
t33 = sin((-qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3) - qJ(9)));
t34 = pkin(14) * t33;
t36 = sin((qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3) - qJ(9)));
t37 = pkin(14) * t36;
t39 = sin((pkin(4) + qJ(8) - qJ(3) - qJ(10)));
t41 = sin((pkin(4) + qJ(8) - qJ(3) + qJ(10)));
t49 = t1 + pkin(5) + qJ(10);
t50 = cos(t49);
t55 = -qJ(10) + pkin(5);
t56 = cos(t55);
t61 = sin(t55);
t66 = sin(t49);
t72 = cos((pkin(5) + qJ(4) + qJ(11) + pkin(1) - qJ(10)));
t75 = cos((-t1 - pkin(5) + qJ(4) + qJ(11) + pkin(1) - qJ(10)));
t87 = qJ(9) + qJ(10);
t88 = sin(t87);
t93 = cos(t87);
t99 = sin((qJ(4) + qJ(11) + pkin(1) - qJ(9) - qJ(10)));
t102 = sin(qJ(10));
t103 = 0.1e1 / t102;
t109 = sin((-qJ(9) + qJ(4) + qJ(11) + pkin(1)));
t113 = sin(qJ(9));
t118 = cos(qJ(9));
t138 = qJ(11) + pkin(1);
t139 = sin(t138);
t140 = qJ(11) + pkin(1) + qJ(12);
t141 = sin(t140);
t144 = cos(t138);
t145 = cos(t140);
t148 = -qJ(5) - qJ(6) + pkin(3);
t149 = cos(t148);
t153 = sin(t148);
t156 = 0.1e1 / qJ(13);
t158 = qJ(11) + pkin(1) + qJ(12) - qJ(5) - qJ(6) + pkin(3);
t159 = cos(t158);
t162 = qJ(11) + pkin(1) + qJ(12) - qJ(5);
t163 = cos(t162);
t176 = sin(t158);
t179 = sin(t162);
unknown(1,1) = 0;
unknown(1,2) = 0;
unknown(1,3) = (0.1e1 / (t34 - t37 + (t39 - t41) * pkin(20)) * t31 * (t22 * pkin(10) + t3 * pkin(10) + t12 * t10 + t25 * t10 + t19 * t18 - t18 * t27) * pkin(14));
unknown(1,4) = 0;
unknown(1,5) = 0;
unknown(2,1) = 0;
unknown(2,2) = 0;
unknown(2,3) = -1;
unknown(2,4) = 1;
unknown(2,5) = 0;
unknown(3,1) = 0;
unknown(3,2) = 0;
unknown(3,3) = (0.1e1 / (-pkin(20) * t39 + pkin(20) * t41 - t34 + t37) * t31 / pkin(13) * (-pkin(7) * t14 * t50 + pkin(7) * t14 * t56 - pkin(7) * t8 * t61 - pkin(7) * t8 * t66 + pkin(10) * t16 * t50 - pkin(10) * t16 * t56 + pkin(10) * t6 * t61 + pkin(10) * t6 * t66 - t72 * pkin(10) + t75 * pkin(10)) * pkin(14) * pkin(20));
unknown(3,4) = 0;
unknown(3,5) = 0;
unknown(4,1) = 0;
unknown(4,2) = 0;
unknown(4,3) = (t31 * t103 * (pkin(7) * t14 * t88 - pkin(7) * t8 * t93 - pkin(10) * t16 * t88 + pkin(10) * t6 * t93 + t99 * pkin(10)));
unknown(4,4) = 0;
unknown(4,5) = 0;
unknown(5,1) = 0;
unknown(5,2) = 0;
unknown(5,3) = (t103 * t31 / pkin(9) * (-pkin(10) * pkin(9) * t99 + pkin(15) * pkin(10) * t109 - (-pkin(9) * t88 + pkin(15) * t113) * t17 + t6 * (-pkin(9) * t93 + pkin(15) * t118) * pkin(10) + (t93 * pkin(9) * t8 - t88 * t14 * pkin(9) + (t14 * t113 - t8 * t118) * pkin(15)) * pkin(7)));
unknown(5,4) = 0;
unknown(5,5) = 0;
unknown(6,1) = 0;
unknown(6,2) = 0;
unknown(6,3) = 1;
unknown(6,4) = 0;
unknown(6,5) = 0;
unknown(7,1) = 0;
unknown(7,2) = 0;
unknown(7,3) = (t156 * (pkin(10) * t141 * t139 + pkin(10) * t145 * t144 + t153 * pkin(16) * t141 - t145 * t149 * pkin(16) - qJ(13)));
unknown(7,4) = (t156 * (t163 * pkin(8) + 0.2e1 * t159 * pkin(16)));
unknown(7,5) = 0;
unknown(8,1) = 0;
unknown(8,2) = 0;
unknown(8,3) = (-pkin(10) * t145 * t139 + pkin(10) * t141 * t144 - pkin(16) * t149 * t141 - pkin(16) * t153 * t145);
unknown(8,4) = (t179 * pkin(8) + 0.2e1 * t176 * pkin(16));
unknown(8,5) = 0;
B21 = unknown;
