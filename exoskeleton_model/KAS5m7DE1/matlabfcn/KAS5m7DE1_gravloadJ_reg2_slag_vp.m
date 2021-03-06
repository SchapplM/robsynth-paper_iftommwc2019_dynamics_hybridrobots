% Calculate inertial parameters regressor of gravitation load for
% KAS5m7DE1
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% g_base [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% taug_reg [5x(5*10)]
%   inertial parameter regressor of gravitation joint torque vector

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function taug_reg = KAS5m7DE1_gravloadJ_reg2_slag_vp(qJ, g, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_gravloadJ_reg2_slag_vp: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE1_gravloadJ_reg2_slag_vp: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_gravloadJ_reg2_slag_vp: pkin has to be [24x1] (double)');

%% Symbolic Calculation
% From gravload_joint_fixb_regressor_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-21 12:01:06
% EndTime: 2020-05-21 14:48:31
% DurationCPUTime: 3272.37s
% Computational Cost: add. (226789187->864), mult. (295070475->1217), div. (4030336->24), fcn. (118110215->43), ass. (0->1258)
unknown=NaN(5,150);
t1 = sin(qJ(1));
t2 = g(2) * t1;
t3 = cos(qJ(1));
t4 = g(1) * t3;
t6 = g(2) * t3;
t7 = g(1) * t1;
t8 = -t6 + t7;
t9 = sin(qJ(2));
t13 = cos(qJ(2));
t16 = t13 * t2 + t13 * t4;
t20 = t1 * t9;
t21 = sin(qJ(3));
t22 = cos(qJ(3));
t23 = -pkin(23) + pkin(24);
t24 = t22 * t23;
t25 = pkin(3) + qJ(3);
t26 = cos(t25);
t27 = t26 * pkin(12);
t28 = -pkin(9) + t24 + t27;
t29 = pkin(11) ^ 2;
t30 = pkin(19) ^ 2;
t31 = t26 ^ 2;
t32 = pkin(12) ^ 2;
t34 = sin(t25);
t35 = t34 ^ 2;
t37 = -pkin(9) + t24;
t38 = t37 ^ 2;
t39 = t21 ^ 2;
t40 = t23 ^ 2;
t44 = t21 * t23;
t45 = t34 * pkin(12);
t46 = -t44 + t45;
t49 = 0.2e1 * pkin(12) * t26 * t28 + 0.2e1 * pkin(12) * t34 * t46 - t31 * t32 - t32 * t35 + t39 * t40 + t29 - t30 + t38;
t53 = 0.4e1 * t28 ^ 2 + 0.4e1 * t46 ^ 2;
t55 = t49 ^ 2;
t57 = sqrt(t29 * t53 - t55);
t59 = 0.2e1 * t28 * t49 - 0.2e1 * t46 * t57;
t60 = 0.1e1 / t53;
t62 = t59 * t60 + pkin(9) - t24 - t27;
t63 = t21 * t62;
t66 = 0.2e1 * t28 * t57 + 0.2e1 * t46 * t49;
t68 = t60 * t66 + t44 - t45;
t69 = t22 * t68;
t70 = t63 + t69;
t71 = 0.1e1 / pkin(19);
t72 = t70 * t71;
t73 = cos(pkin(7));
t74 = t73 * pkin(18);
t75 = t72 * t74;
t76 = t22 * t62;
t77 = t21 * t68;
t78 = t76 - t77;
t79 = t78 * t71;
t80 = sin(pkin(7));
t81 = t80 * pkin(18);
t82 = t79 * t81;
t83 = pkin(17) ^ 2;
t84 = pkin(22) ^ 2;
t85 = t79 * t74;
t86 = t72 * t81;
t87 = t85 - t86;
t88 = t87 ^ 2;
t89 = t75 + t82;
t90 = t89 ^ 2;
t91 = pkin(24) ^ 2;
t92 = -pkin(24) - t85 + t86;
t95 = -0.2e1 * t87 * t92 + 0.2e1 * t89 ^ 2 + t83 - t84 - t88 - t90 + t91;
t99 = 0.4e1 * t89 ^ 2 + 0.4e1 * t92 ^ 2;
t101 = t95 ^ 2;
t103 = sqrt(t83 * t99 - t101);
t105 = 0.2e1 * t103 * t92 - 0.2e1 * t89 * t95;
t106 = 0.1e1 / t99;
t108 = -t105 * t106 - t75 - t82;
t111 = 0.2e1 * t103 * t89 + 0.2e1 * t92 * t95;
t113 = t106 * t111 + pkin(24) + t85 - t86;
t114 = atan2(t108, t113);
t115 = t114 + pkin(6);
t116 = sin(t115);
t118 = cos(t115);
t120 = -t116 * t20 - t118 * t3;
t122 = t3 * t9;
t125 = t1 * t118 - t116 * t122;
t130 = t116 * t3 - t118 * t20;
t134 = -t1 * t116 - t118 * t122;
t137 = t1 * t13;
t138 = t137 * pkin(21);
t139 = t3 * pkin(16);
t142 = t3 * t13;
t143 = t142 * pkin(21);
t144 = t1 * pkin(16);
t147 = -g(2) * (t138 + t139) - g(1) * (t143 - t144);
t148 = t120 * t22;
t149 = t130 * t21;
t150 = t148 + t149;
t154 = t125 * t22 + t134 * t21;
t157 = t120 * t21;
t158 = t130 * t22;
t159 = -t157 + t158;
t163 = -t125 * t21 + t134 * t22;
t166 = t120 * pkin(23);
t169 = t125 * pkin(23);
t173 = cos(qJ(4));
t175 = sin(qJ(4));
t177 = t150 * t173 + t159 * t175;
t181 = t154 * t173 + t163 * t175;
t186 = -t150 * t175 + t159 * t173;
t190 = -t154 * t175 + t163 * t173;
t193 = t150 * pkin(9);
t196 = t154 * pkin(9);
t199 = -g(2) * (t193 + t166 + t138 + t139) - g(1) * (t196 + t169 + t143 - t144);
t200 = qJ(4) - qJ(3) + pkin(4);
t201 = sin(t200);
t202 = t177 * t201;
t203 = cos(t200);
t204 = t186 * t203;
t205 = -t202 + t204;
t209 = -t181 * t201 + t190 * t203;
t212 = t177 * t203;
t213 = t186 * t201;
t214 = -t212 - t213;
t218 = -t181 * t203 - t190 * t201;
t221 = t177 * pkin(10);
t224 = t181 * pkin(10);
t228 = cos(qJ(5));
t230 = sin(qJ(5));
t232 = t137 * t230 + t205 * t228;
t235 = t142 * t230;
t241 = t137 * t228 - t205 * t230;
t244 = t142 * t228;
t258 = sin(pkin(6));
t260 = cos(pkin(6));
t262 = t20 * t258 + t260 * t3;
t266 = -t1 * t260 + t122 * t258;
t271 = t20 * t260 - t258 * t3;
t275 = t1 * t258 + t122 * t260;
t278 = 0.1e1 / pkin(22);
t279 = t108 * t278;
t281 = -t113 * t278;
t283 = t258 * t279 + t260 * t281;
t287 = -t258 * t281 + t260 * t279;
t291 = -(t258 * t287 - t260 * t283) * pkin(22) + pkin(24) + t85 - t86;
t296 = (t258 * t283 + t260 * t287) * pkin(22) + t75 + t82;
t297 = atan2(t291, t296);
t298 = t114 - t297;
t299 = sin(t298);
t301 = cos(t298);
t326 = atan2(-t70, t78);
t327 = cos(t326);
t329 = sin(t326);
t331 = t120 * t327 - t130 * t329;
t335 = t125 * t327 - t134 * t329;
t340 = t120 * t329 + t130 * t327;
t344 = t125 * t329 + t134 * t327;
t347 = t120 * pkin(24);
t350 = t125 * pkin(24);
t355 = t21 * t78 * t71;
t357 = t22 * t70 * t71;
t360 = (t355 - t357) * pkin(19) + t44 - t45;
t362 = t22 * t78 * t71;
t364 = t21 * t70 * t71;
t367 = (t362 + t364) * pkin(19) - pkin(9) + t24 + t27;
t368 = atan2(t360, t367);
t369 = -t368 + qJ(3) + t326;
t370 = cos(t369);
t372 = sin(t369);
t397 = t150 * t21;
t398 = t159 * t22;
t399 = t397 - t398;
t403 = t154 * t21 - t163 * t22;
t406 = t150 * t22;
t407 = t159 * t21;
t408 = t406 + t407;
t412 = t154 * t22 + t163 * t21;
t415 = sin(pkin(3));
t417 = cos(pkin(3));
t419 = t399 * t415 - t408 * t417;
t423 = t403 * t415 - t412 * t417;
t428 = t399 * t417 + t408 * t415;
t432 = t403 * t417 + t412 * t415;
t435 = t201 * pkin(15);
t436 = pkin(3) + qJ(3) - qJ(4);
t437 = sin(t436);
t438 = t437 * pkin(12);
t439 = t203 * pkin(14);
t440 = t435 + t438 - t439;
t441 = t201 * pkin(14);
t442 = t203 * pkin(15);
t443 = cos(t436);
t444 = t443 * pkin(12);
t445 = -t441 - pkin(10) - t442 - t444;
t446 = atan2(t440, t445);
t447 = t446 + pkin(3) + qJ(3) - qJ(4);
t448 = sin(t447);
t450 = cos(t447);
t458 = -g(2) * (t419 * t448 + t428 * t450) - g(1) * (t423 * t448 + t432 * t450);
t461 = -t419 * t450 + t428 * t448;
t465 = -t423 * t450 + t432 * t448;
t468 = t419 * pkin(12);
t471 = t423 * pkin(12);
t478 = t445 ^ 2;
t479 = t440 ^ 2;
t481 = sqrt(t478 + t479);
t489 = g(3) * t9;
t493 = g(3) * t13;
t496 = t6 * t9 - t7 * t9 - t493;
t498 = t13 * t116;
t503 = t13 * t118;
t508 = t9 * pkin(21);
t511 = pkin(21) * t493 - t508 * t6 + t508 * t7;
t512 = t9 * t116;
t514 = t9 * t118;
t516 = t21 * t514 + t22 * t512;
t518 = t116 * t22;
t520 = t118 * t21;
t522 = t142 * t518 + t142 * t520;
t526 = -t137 * t518 - t137 * t520;
t531 = -t21 * t512 + t22 * t514;
t533 = t116 * t21;
t535 = t118 * t22;
t537 = -t142 * t533 + t142 * t535;
t541 = t137 * t533 - t137 * t535;
t544 = t512 * pkin(23);
t545 = t13 * pkin(21);
t548 = t116 * pkin(23);
t549 = t142 * t548;
t550 = t122 * pkin(21);
t553 = t137 * t548;
t554 = t20 * pkin(21);
t560 = t173 * t516 + t175 * t531;
t564 = t173 * t522 + t175 * t537;
t568 = t173 * t526 + t175 * t541;
t573 = t173 * t531 - t175 * t516;
t577 = t173 * t537 - t175 * t522;
t581 = t173 * t541 - t175 * t526;
t584 = t516 * pkin(9);
t587 = t522 * pkin(9);
t590 = t526 * pkin(9);
t593 = -g(3) * (t584 + t544 - t545) - g(2) * (t587 + t549 + t550) - g(1) * (t590 - t553 - t554);
t596 = -t201 * t560 + t203 * t573;
t600 = -t201 * t564 + t203 * t577;
t604 = -t201 * t568 + t203 * t581;
t609 = -t201 * t573 - t203 * t560;
t613 = -t201 * t577 - t203 * t564;
t617 = -t201 * t581 - t203 * t568;
t620 = t560 * pkin(10);
t623 = t564 * pkin(10);
t626 = t568 * pkin(10);
t671 = t13 * t258;
t676 = t13 * t260;
t680 = t9 * t258;
t682 = t9 * t260;
t686 = t258 * t299;
t688 = t260 * t301;
t701 = t258 * t301;
t703 = t260 * t299;
t715 = t258 * pkin(22);
t725 = t327 * t512 - t329 * t514;
t727 = t116 * t327;
t729 = t118 * t329;
t731 = t142 * t727 - t142 * t729;
t735 = -t137 * t727 + t137 * t729;
t740 = t327 * t514 + t329 * t512;
t742 = t116 * t329;
t744 = t118 * t327;
t746 = t142 * t742 + t142 * t744;
t750 = -t137 * t742 - t137 * t744;
t753 = t512 * pkin(24);
t756 = t116 * pkin(24);
t757 = t142 * t756;
t760 = t137 * t756;
t802 = t21 * t516 - t22 * t531;
t806 = t21 * t522 - t22 * t537;
t810 = t21 * t526 - t22 * t541;
t815 = t21 * t531 + t22 * t516;
t819 = t21 * t537 + t22 * t522;
t823 = t21 * t541 + t22 * t526;
t828 = t415 * t802 - t417 * t815;
t832 = t415 * t806 - t417 * t819;
t836 = t415 * t810 - t417 * t823;
t841 = t415 * t815 + t417 * t802;
t845 = t415 * t819 + t417 * t806;
t849 = t415 * t823 + t417 * t810;
t864 = -g(3) * (t448 * t828 + t450 * t841) - g(2) * (t448 * t832 + t450 * t845) - g(1) * (t448 * t836 + t450 * t849);
t867 = t448 * t841 - t450 * t828;
t871 = t448 * t845 - t450 * t832;
t875 = t448 * t849 - t450 * t836;
t878 = t828 * pkin(12);
t881 = t832 * pkin(12);
t884 = t836 * pkin(12);
t902 = -t44 - t45;
t914 = -t24 + t27;
t919 = 0.2e1 * pkin(12) * t26 * t46 + 0.2e1 * pkin(12) * t26 * t902 - 0.2e1 * pkin(12) * t28 * t34 + 0.2e1 * pkin(12) * t34 * t914 + 0.2e1 * t21 * t22 * t40 - 0.2e1 * t21 * t23 * t37;
t922 = 0.1e1 / t57;
t926 = 0.4e1 * t28 * t902 + 0.4e1 * t46 * t914;
t930 = 0.2e1 * t29 * t926 - 0.2e1 * t49 * t919;
t935 = t53 ^ 2;
t936 = 0.1e1 / t935;
t939 = t44 + t45 + (-t46 * t922 * t930 + 0.2e1 * t28 * t919 + 0.2e1 * t49 * t902 - 0.2e1 * t57 * t914) * t60 - 0.2e1 * t59 * t936 * t926;
t951 = t24 - t27 + (t28 * t922 * t930 + 0.2e1 * t46 * t919 + 0.2e1 * t49 * t914 + 0.2e1 * t57 * t902) * t60 - 0.2e1 * t66 * t936 * t926;
t953 = t21 * t939 + t22 * t951 + t76 - t77;
t954 = t953 * t71;
t955 = t954 * t74;
t958 = -t21 * t951 + t22 * t939 - t63 - t69;
t959 = t958 * t71;
t960 = t959 * t81;
t961 = t959 * t74;
t962 = t954 * t81;
t963 = t961 - t962;
t966 = t955 + t960;
t972 = 0.2e1 * t89 * t966 - 0.2e1 * t92 * t963;
t976 = 0.1e1 / t103;
t980 = 0.4e1 * t89 * t966 - 0.4e1 * t92 * t963;
t984 = 0.2e1 * t83 * t980 - 0.2e1 * t95 * t972;
t989 = t99 ^ 2;
t990 = 0.1e1 / t989;
t993 = -t955 - t960 - (t92 * t976 * t984 - 0.2e1 * t103 * t963 - 0.2e1 * t89 * t972 - 0.2e1 * t95 * t966) * t106 + 0.2e1 * t105 * t990 * t980;
t996 = t108 ^ 2;
t997 = t113 ^ 2;
t998 = 0.1e1 / t997;
t1001 = 0.1e1 / (t996 * t998 + 0.1e1);
t1002 = t993 / t113 * t1001;
t1013 = t961 - t962 + (t89 * t976 * t984 + 0.2e1 * t103 * t966 + 0.2e1 * t92 * t972 - 0.2e1 * t95 * t963) * t106 - 0.2e1 * t111 * t990 * t980;
t1016 = t1013 * t108 * t998 * t1001;
t1017 = t1002 - t1016;
t1018 = t1017 * t118;
t1021 = t1 * t1017;
t1023 = t1018 * t122 + t1021 * t116;
t1024 = g(2) * t1023;
t1026 = t3 * t1017;
t1028 = -t1018 * t20 + t1026 * t116;
t1029 = g(1) * t1028;
t1031 = t1017 * t116;
t1035 = t1021 * t118 - t1031 * t122;
t1039 = t1026 * t118 + t1031 * t20;
t1042 = t13 * t1017;
t1044 = t498 * t21;
t1046 = t503 * t22;
t1047 = t1042 * t533 - t1042 * t535 + t1044 - t1046;
t1050 = -t125 * t21;
t1052 = -t134 * t22;
t1053 = t1023 * t22 + t1035 * t21 - t1050 + t1052;
t1057 = t1028 * t22 + t1039 * t21 - t157 + t158;
t1061 = t498 * t22;
t1063 = t503 * t21;
t1064 = t1042 * t518 + t1042 * t520 + t1061 + t1063;
t1067 = -t125 * t22;
t1069 = -t134 * t21;
t1070 = -t1023 * t21 + t1035 * t22 - t1067 - t1069;
t1074 = -t1028 * t21 + t1039 * t22 - t148 - t149;
t1084 = t1047 * t173 + t1064 * t175;
t1088 = t1053 * t173 + t1070 * t175;
t1092 = t1057 * t173 + t1074 * t175;
t1097 = -t1047 * t175 + t1064 * t173;
t1101 = -t1053 * t175 + t1070 * t173;
t1105 = -t1057 * t175 + t1074 * t173;
t1108 = t1047 * pkin(9);
t1110 = t1042 * t118 * pkin(23);
t1113 = t1053 * pkin(9);
t1114 = t1023 * pkin(23);
t1117 = t1057 * pkin(9);
t1118 = t1028 * pkin(23);
t1121 = -g(3) * (t1108 - t1110) - g(2) * (t1113 + t1114) - g(1) * (t1117 + t1118);
t1123 = -t1061 - t1063;
t1125 = t1044 - t1046;
t1127 = t1123 * t173 + t1125 * t175;
t1128 = t1127 * t203;
t1132 = -t1123 * t175 + t1125 * t173;
t1133 = t1132 * t201;
t1135 = g(3) * (-t1084 * t201 + t1097 * t203 + t1128 + t1133);
t1137 = t1067 + t1069;
t1139 = -t1050 + t1052;
t1141 = t1137 * t173 + t1139 * t175;
t1142 = t1141 * t203;
t1146 = -t1137 * t175 + t1139 * t173;
t1147 = t1146 * t201;
t1149 = g(2) * (-t1088 * t201 + t1101 * t203 + t1142 + t1147);
t1153 = g(1) * (-t1092 * t201 + t1105 * t203 + t212 + t213);
t1156 = t1127 * t201;
t1158 = t1132 * t203;
t1159 = -t1084 * t203 - t1097 * t201 - t1156 + t1158;
t1162 = t1141 * t201;
t1164 = t1146 * t203;
t1165 = -t1088 * t203 - t1101 * t201 - t1162 + t1164;
t1169 = -t1092 * t203 - t1105 * t201 - t202 + t204;
t1172 = t1084 * pkin(10);
t1175 = t1088 * pkin(10);
t1178 = t1092 * pkin(10);
t1204 = t993 * t278;
t1206 = -t1013 * t278;
t1208 = t1204 * t258 + t1206 * t260;
t1212 = t1204 * t260 - t1206 * t258;
t1219 = t291 ^ 2;
t1220 = t296 ^ 2;
t1221 = 0.1e1 / t1220;
t1224 = 0.1e1 / (t1219 * t1221 + 0.1e1);
t1234 = t1002 - t1016 - (-(-t1208 * t260 + t1212 * t258) * pkin(22) + t961 - t962) / t296 * t1224 + ((t1208 * t258 + t1212 * t260) * pkin(22) + t955 + t960) * t291 * t1221 * t1224;
t1235 = t1234 * t301;
t1237 = t1234 * t299;
t1241 = -t266 * t1234;
t1243 = -t275 * t1234;
t1247 = t262 * t1234;
t1249 = t271 * t1234;
t1270 = t70 ^ 2;
t1271 = t78 ^ 2;
t1272 = 0.1e1 / t1271;
t1275 = 0.1e1 / (t1270 * t1272 + 0.1e1);
t1276 = -t953 / t78 * t1275;
t1279 = -t958 * t70 * t1272 * t1275;
t1280 = t1276 - t1279;
t1281 = t1280 * t329;
t1284 = t1280 * t327;
t1286 = -t1042 * t742 - t1042 * t744 + t1281 * t498 + t1284 * t503;
t1289 = -t125 * t1280;
t1292 = -t134 * t1280;
t1294 = t1023 * t327 - t1035 * t329 - t1289 * t329 - t1292 * t327;
t1297 = t120 * t1280;
t1300 = t130 * t1280;
t1302 = t1028 * t327 - t1039 * t329 - t1297 * t329 - t1300 * t327;
t1309 = t1042 * t727 - t1042 * t729 + t1281 * t503 - t1284 * t498;
t1315 = t1023 * t329 + t1035 * t327 + t1289 * t327 - t1292 * t329;
t1321 = t1028 * t329 + t1039 * t327 + t1297 * t327 - t1300 * t329;
t1342 = t360 ^ 2;
t1343 = t367 ^ 2;
t1344 = 0.1e1 / t1343;
t1347 = 0.1e1 / (t1342 * t1344 + 0.1e1);
t1359 = -((t21 * t71 * t958 - t22 * t71 * t953 + t362 + t364) * pkin(19) + t24 - t27) / t367 * t1347 + ((t21 * t71 * t953 + t22 * t71 * t958 - t355 + t357) * pkin(19) - t44 - t45) * t360 * t1344 * t1347 + 0.1e1 + t1276 - t1279;
t1360 = (-t327 * t498 + t329 * t503) * t1359;
t1366 = (-t327 * t503 - t329 * t498) * t1359;
t1374 = (-t125 * t327 + t134 * t329) * t1359;
t1380 = (-t125 * t329 - t134 * t327) * t1359;
t1385 = t331 * t1359;
t1388 = t340 * t1359;
t1427 = t1123 * t22;
t1429 = t1125 * t21;
t1430 = t1047 * t21 - t1064 * t22 + t1427 + t1429;
t1433 = t1137 * t22;
t1435 = t1139 * t21;
t1436 = t1053 * t21 - t1070 * t22 + t1433 + t1435;
t1440 = t1057 * t21 - t1074 * t22 + t406 + t407;
t1444 = t1123 * t21;
t1446 = t1125 * t22;
t1447 = t1047 * t22 + t1064 * t21 - t1444 + t1446;
t1450 = t1137 * t21;
t1452 = t1139 * t22;
t1453 = t1053 * t22 + t1070 * t21 - t1450 + t1452;
t1457 = t1057 * t22 + t1074 * t21 - t397 + t398;
t1462 = t1430 * t415 - t1447 * t417;
t1466 = t1436 * t415 - t1453 * t417;
t1470 = t1440 * t415 - t1457 * t417;
t1475 = t1430 * t417 + t1447 * t415;
t1479 = t1436 * t417 + t1453 * t415;
t1483 = t1440 * t417 + t1457 * t415;
t1487 = t1444 - t1446;
t1489 = t1427 + t1429;
t1491 = t1487 * t415 - t1489 * t417;
t1492 = -t442 + t444 - t441;
t1493 = 0.1e1 / t445;
t1495 = 0.1e1 / t478;
t1498 = 0.1e1 / (t1495 * t479 + 0.1e1);
t1500 = t439 - t435 + t438;
t1502 = t1495 * t1498;
t1504 = t1492 * t1493 * t1498 - t1500 * t1502 * t440 + 0.1e1;
t1505 = t1491 * t1504;
t1510 = t1487 * t417 + t1489 * t415;
t1511 = t1510 * t1504;
t1516 = t1450 - t1452;
t1518 = t1433 + t1435;
t1520 = t1516 * t415 - t1518 * t417;
t1521 = t1520 * t1504;
t1526 = t1516 * t417 + t1518 * t415;
t1527 = t1526 * t1504;
t1532 = t419 * t1504;
t1535 = t428 * t1504;
t1539 = -g(3) * (t1462 * t448 + t1475 * t450 + t1505 * t450 - t1511 * t448) - g(2) * (t1466 * t448 + t1479 * t450 + t1521 * t450 - t1527 * t448) - g(1) * (t1470 * t448 + t1483 * t450 + t1532 * t450 - t1535 * t448);
t1544 = -t1462 * t450 + t1475 * t448 + t1505 * t448 + t1511 * t450;
t1550 = -t1466 * t450 + t1479 * t448 + t1521 * t448 + t1527 * t450;
t1556 = -t1470 * t450 + t1483 * t448 + t1532 * t448 + t1535 * t450;
t1559 = t1462 * pkin(12);
t1562 = t1466 * pkin(12);
t1565 = t1470 * pkin(12);
t1577 = 0.1e1 / t481;
t1578 = (-t1491 * t450 + t1510 * t448) * t1577;
t1581 = t1492 * t440 + t1500 * t445;
t1590 = (-t1520 * t450 + t1526 * t448) * t1577;
t1596 = t461 * t1577;
t1602 = g(3) * t1132;
t1603 = g(2) * t1146;
t1604 = g(1) * t186;
t1613 = g(3) * (-t1127 * t203 - t1128 - 0.2e1 * t1133);
t1617 = g(2) * (-t1141 * t203 - t1142 - 0.2e1 * t1147);
t1621 = g(1) * (-t177 * t203 - t212 - 0.2e1 * t213);
t1625 = t1127 * t201 + t1156 - 0.2e1 * t1158;
t1629 = t1141 * t201 + t1162 - 0.2e1 * t1164;
t1633 = t177 * t201 + t202 - 0.2e1 * t204;
t1669 = -t1492 * t1493 * t1498 + t1500 * t1502 * t440 - 0.1e1;
t1670 = t1491 * t1669;
t1672 = t1510 * t1669;
t1676 = t1520 * t1669;
t1678 = t1526 * t1669;
t1682 = t419 * t1669;
t1684 = t428 * t1669;
t1688 = -g(3) * (t1670 * t450 - t1672 * t448) - g(2) * (t1676 * t450 - t1678 * t448) - g(1) * (t1682 * t450 - t1684 * t448);
t1691 = t1670 * t448 + t1672 * t450;
t1695 = t1676 * t448 + t1678 * t450;
t1699 = t1682 * t448 + t1684 * t450;
t1709 = -t1492 * t440 - t1500 * t445;
t1725 = -t1156 + t1158;
t1730 = -t1162 + t1164;
unknown(1,1) = 0;
unknown(1,2) = 0;
unknown(1,3) = 0;
unknown(1,4) = 0;
unknown(1,5) = 0;
unknown(1,6) = 0;
unknown(1,7) = (-t2 - t4);
unknown(1,8) = t8;
unknown(1,9) = 0;
unknown(1,10) = 0;
unknown(1,11) = 0;
unknown(1,12) = 0;
unknown(1,13) = 0;
unknown(1,14) = 0;
unknown(1,15) = 0;
unknown(1,16) = 0;
unknown(1,17) = (t2 * t9 + t4 * t9);
unknown(1,18) = t16;
unknown(1,19) = t8;
unknown(1,20) = (-pkin(16) * t6 + pkin(16) * t7);
unknown(1,21) = 0;
unknown(1,22) = 0;
unknown(1,23) = 0;
unknown(1,24) = 0;
unknown(1,25) = 0;
unknown(1,26) = 0;
unknown(1,27) = (-g(1) * t125 - g(2) * t120);
unknown(1,28) = (-g(1) * t134 - g(2) * t130);
unknown(1,29) = -t16;
unknown(1,30) = t147;
unknown(1,31) = 0;
unknown(1,32) = 0;
unknown(1,33) = 0;
unknown(1,34) = 0;
unknown(1,35) = 0;
unknown(1,36) = 0;
unknown(1,37) = (-g(1) * t154 - g(2) * t150);
unknown(1,38) = (-g(1) * t163 - g(2) * t159);
unknown(1,39) = -t16;
unknown(1,40) = (-g(2) * (t166 + t138 + t139) - g(1) * (t169 + t143 - t144));
unknown(1,41) = 0;
unknown(1,42) = 0;
unknown(1,43) = 0;
unknown(1,44) = 0;
unknown(1,45) = 0;
unknown(1,46) = 0;
unknown(1,47) = (-g(1) * t181 - g(2) * t177);
unknown(1,48) = (-g(1) * t190 - g(2) * t186);
unknown(1,49) = -t16;
unknown(1,50) = t199;
unknown(1,51) = 0;
unknown(1,52) = 0;
unknown(1,53) = 0;
unknown(1,54) = 0;
unknown(1,55) = 0;
unknown(1,56) = 0;
unknown(1,57) = (-g(1) * t209 - g(2) * t205);
unknown(1,58) = (-g(1) * t218 - g(2) * t214);
unknown(1,59) = -t16;
unknown(1,60) = (-g(2) * (t221 + t193 + t166 + t138 + t139) - g(1) * (t224 + t196 + t169 + t143 - t144));
unknown(1,61) = 0;
unknown(1,62) = 0;
unknown(1,63) = 0;
unknown(1,64) = 0;
unknown(1,65) = 0;
unknown(1,66) = 0;
unknown(1,67) = (-g(2) * t232 - g(1) * (t209 * t228 + t235));
unknown(1,68) = (-g(2) * t241 - g(1) * (-t209 * t230 + t244));
unknown(1,69) = (g(1) * t218 + g(2) * t214);
unknown(1,70) = (-g(2) * (-pkin(13) * t214 + t138 + t139 + t166 + t193 + t221) - g(1) * (-pkin(13) * t218 + t143 - t144 + t169 + t196 + t224));
unknown(1,71) = 0;
unknown(1,72) = 0;
unknown(1,73) = 0;
unknown(1,74) = 0;
unknown(1,75) = 0;
unknown(1,76) = 0;
unknown(1,77) = (-g(1) * t266 - g(2) * t262);
unknown(1,78) = (-g(1) * t275 - g(2) * t271);
unknown(1,79) = -t16;
unknown(1,80) = t147;
unknown(1,81) = 0;
unknown(1,82) = 0;
unknown(1,83) = 0;
unknown(1,84) = 0;
unknown(1,85) = 0;
unknown(1,86) = 0;
unknown(1,87) = (-g(2) * (t262 * t299 - t271 * t301) - g(1) * (t266 * t299 - t275 * t301));
unknown(1,88) = (-g(2) * (t262 * t301 + t271 * t299) - g(1) * (t266 * t301 + t275 * t299));
unknown(1,89) = -t16;
unknown(1,90) = (-g(2) * (-pkin(22) * t262 + t138 + t139) - g(1) * (-pkin(22) * t266 + t143 - t144));
unknown(1,91) = 0;
unknown(1,92) = 0;
unknown(1,93) = 0;
unknown(1,94) = 0;
unknown(1,95) = 0;
unknown(1,96) = 0;
unknown(1,97) = (-g(1) * t335 - g(2) * t331);
unknown(1,98) = (-g(1) * t344 - g(2) * t340);
unknown(1,99) = -t16;
unknown(1,100) = (-g(2) * (t347 + t138 + t139) - g(1) * (t350 + t143 - t144));
unknown(1,101) = 0;
unknown(1,102) = 0;
unknown(1,103) = 0;
unknown(1,104) = 0;
unknown(1,105) = 0;
unknown(1,106) = 0;
unknown(1,107) = (-g(2) * (-t331 * t370 - t340 * t372) - g(1) * (-t335 * t370 - t344 * t372));
unknown(1,108) = (-g(2) * (t331 * t372 - t340 * t370) - g(1) * (t335 * t372 - t344 * t370));
unknown(1,109) = -t16;
unknown(1,110) = (-g(2) * (pkin(19) * t331 + t138 + t139 + t347) - g(1) * (pkin(19) * t335 + t143 - t144 + t350));
unknown(1,111) = 0;
unknown(1,112) = 0;
unknown(1,113) = 0;
unknown(1,114) = 0;
unknown(1,115) = 0;
unknown(1,116) = 0;
unknown(1,117) = (-g(1) * t403 - g(2) * t399);
unknown(1,118) = (-g(1) * t412 - g(2) * t408);
unknown(1,119) = -t16;
unknown(1,120) = t199;
unknown(1,121) = 0;
unknown(1,122) = 0;
unknown(1,123) = 0;
unknown(1,124) = 0;
unknown(1,125) = 0;
unknown(1,126) = 0;
unknown(1,127) = (-g(1) * t423 - g(2) * t419);
unknown(1,128) = (-g(1) * t432 - g(2) * t428);
unknown(1,129) = -t16;
unknown(1,130) = t199;
unknown(1,131) = 0;
unknown(1,132) = 0;
unknown(1,133) = 0;
unknown(1,134) = 0;
unknown(1,135) = 0;
unknown(1,136) = 0;
unknown(1,137) = t458;
unknown(1,138) = (-g(1) * t465 - g(2) * t461);
unknown(1,139) = -t16;
unknown(1,140) = (-g(2) * (t468 + t193 + t166 + t138 + t139) - g(1) * (t471 + t196 + t169 + t143 - t144));
unknown(1,141) = 0;
unknown(1,142) = 0;
unknown(1,143) = 0;
unknown(1,144) = 0;
unknown(1,145) = 0;
unknown(1,146) = 0;
unknown(1,147) = t458;
unknown(1,148) = -t16;
unknown(1,149) = (g(1) * t465 + g(2) * t461);
unknown(1,150) = (-g(2) * (-t461 * t481 + t138 + t139 + t166 + t193 + t468) - g(1) * (-t465 * t481 + t143 - t144 + t169 + t196 + t471));
unknown(2,1) = 0;
unknown(2,2) = 0;
unknown(2,3) = 0;
unknown(2,4) = 0;
unknown(2,5) = 0;
unknown(2,6) = 0;
unknown(2,7) = 0;
unknown(2,8) = 0;
unknown(2,9) = 0;
unknown(2,10) = 0;
unknown(2,11) = 0;
unknown(2,12) = 0;
unknown(2,13) = 0;
unknown(2,14) = 0;
unknown(2,15) = 0;
unknown(2,16) = 0;
unknown(2,17) = (-t13 * t6 + t13 * t7 - t489);
unknown(2,18) = t496;
unknown(2,19) = 0;
unknown(2,20) = 0;
unknown(2,21) = 0;
unknown(2,22) = 0;
unknown(2,23) = 0;
unknown(2,24) = 0;
unknown(2,25) = 0;
unknown(2,26) = 0;
unknown(2,27) = (-t116 * t489 - t498 * t6 + t498 * t7);
unknown(2,28) = (-t118 * t489 - t503 * t6 + t503 * t7);
unknown(2,29) = -t496;
unknown(2,30) = t511;
unknown(2,31) = 0;
unknown(2,32) = 0;
unknown(2,33) = 0;
unknown(2,34) = 0;
unknown(2,35) = 0;
unknown(2,36) = 0;
unknown(2,37) = (-g(1) * t526 - g(2) * t522 - g(3) * t516);
unknown(2,38) = (-g(1) * t541 - g(2) * t537 - g(3) * t531);
unknown(2,39) = -t496;
unknown(2,40) = (-g(3) * (t544 - t545) - g(2) * (t549 + t550) - g(1) * (-t553 - t554));
unknown(2,41) = 0;
unknown(2,42) = 0;
unknown(2,43) = 0;
unknown(2,44) = 0;
unknown(2,45) = 0;
unknown(2,46) = 0;
unknown(2,47) = (-g(1) * t568 - g(2) * t564 - g(3) * t560);
unknown(2,48) = (-g(1) * t581 - g(2) * t577 - g(3) * t573);
unknown(2,49) = -t496;
unknown(2,50) = t593;
unknown(2,51) = 0;
unknown(2,52) = 0;
unknown(2,53) = 0;
unknown(2,54) = 0;
unknown(2,55) = 0;
unknown(2,56) = 0;
unknown(2,57) = (-g(1) * t604 - g(2) * t600 - g(3) * t596);
unknown(2,58) = (-g(1) * t617 - g(2) * t613 - g(3) * t609);
unknown(2,59) = -t496;
unknown(2,60) = (-g(3) * (t620 + t584 + t544 - t545) - g(2) * (t623 + t587 + t549 + t550) - g(1) * (t626 + t590 - t553 - t554));
unknown(2,61) = 0;
unknown(2,62) = 0;
unknown(2,63) = 0;
unknown(2,64) = 0;
unknown(2,65) = 0;
unknown(2,66) = 0;
unknown(2,67) = (-g(3) * (-t13 * t230 + t228 * t596) - g(2) * (t122 * t230 + t228 * t600) - g(1) * (-t20 * t230 + t228 * t604));
unknown(2,68) = (-g(3) * (-t13 * t228 - t230 * t596) - g(2) * (t122 * t228 - t230 * t600) - g(1) * (-t20 * t228 - t230 * t604));
unknown(2,69) = (g(1) * t617 + g(2) * t613 + g(3) * t609);
unknown(2,70) = (-g(3) * (-pkin(13) * t609 + t544 - t545 + t584 + t620) - g(2) * (-pkin(13) * t613 + t549 + t550 + t587 + t623) - g(1) * (-pkin(13) * t617 - t553 - t554 + t590 + t626));
unknown(2,71) = 0;
unknown(2,72) = 0;
unknown(2,73) = 0;
unknown(2,74) = 0;
unknown(2,75) = 0;
unknown(2,76) = 0;
unknown(2,77) = (t258 * t489 + t6 * t671 - t671 * t7);
unknown(2,78) = (t260 * t489 + t6 * t676 - t676 * t7);
unknown(2,79) = -t496;
unknown(2,80) = t511;
unknown(2,81) = 0;
unknown(2,82) = 0;
unknown(2,83) = 0;
unknown(2,84) = 0;
unknown(2,85) = 0;
unknown(2,86) = 0;
unknown(2,87) = (-g(3) * (-t299 * t680 + t301 * t682) - g(2) * (-t142 * t686 + t142 * t688) - g(1) * (t137 * t686 - t137 * t688));
unknown(2,88) = (-g(3) * (-t299 * t682 - t301 * t680) - g(2) * (-t142 * t701 - t142 * t703) - g(1) * (t137 * t701 + t137 * t703));
unknown(2,89) = -t496;
unknown(2,90) = (-g(3) * (pkin(22) * t680 - t545) - g(2) * (t142 * t715 + t550) - g(1) * (-t137 * t715 - t554));
unknown(2,91) = 0;
unknown(2,92) = 0;
unknown(2,93) = 0;
unknown(2,94) = 0;
unknown(2,95) = 0;
unknown(2,96) = 0;
unknown(2,97) = (-g(1) * t735 - g(2) * t731 - g(3) * t725);
unknown(2,98) = (-g(1) * t750 - g(2) * t746 - g(3) * t740);
unknown(2,99) = -t496;
unknown(2,100) = (-g(3) * (t753 - t545) - g(2) * (t757 + t550) - g(1) * (-t760 - t554));
unknown(2,101) = 0;
unknown(2,102) = 0;
unknown(2,103) = 0;
unknown(2,104) = 0;
unknown(2,105) = 0;
unknown(2,106) = 0;
unknown(2,107) = (-g(3) * (-t370 * t725 - t372 * t740) - g(2) * (-t370 * t731 - t372 * t746) - g(1) * (-t370 * t735 - t372 * t750));
unknown(2,108) = (-g(3) * (-t370 * t740 + t372 * t725) - g(2) * (-t370 * t746 + t372 * t731) - g(1) * (-t370 * t750 + t372 * t735));
unknown(2,109) = -t496;
unknown(2,110) = (-g(3) * (pkin(19) * t725 - t545 + t753) - g(2) * (pkin(19) * t731 + t550 + t757) - g(1) * (pkin(19) * t735 - t554 - t760));
unknown(2,111) = 0;
unknown(2,112) = 0;
unknown(2,113) = 0;
unknown(2,114) = 0;
unknown(2,115) = 0;
unknown(2,116) = 0;
unknown(2,117) = (-g(1) * t810 - g(2) * t806 - g(3) * t802);
unknown(2,118) = (-g(1) * t823 - g(2) * t819 - g(3) * t815);
unknown(2,119) = -t496;
unknown(2,120) = t593;
unknown(2,121) = 0;
unknown(2,122) = 0;
unknown(2,123) = 0;
unknown(2,124) = 0;
unknown(2,125) = 0;
unknown(2,126) = 0;
unknown(2,127) = (-g(1) * t836 - g(2) * t832 - g(3) * t828);
unknown(2,128) = (-g(1) * t849 - g(2) * t845 - g(3) * t841);
unknown(2,129) = -t496;
unknown(2,130) = t593;
unknown(2,131) = 0;
unknown(2,132) = 0;
unknown(2,133) = 0;
unknown(2,134) = 0;
unknown(2,135) = 0;
unknown(2,136) = 0;
unknown(2,137) = t864;
unknown(2,138) = (-g(1) * t875 - g(2) * t871 - g(3) * t867);
unknown(2,139) = -t496;
unknown(2,140) = (-g(3) * (t878 + t584 + t544 - t545) - g(2) * (t881 + t587 + t549 + t550) - g(1) * (t884 + t590 - t553 - t554));
unknown(2,141) = 0;
unknown(2,142) = 0;
unknown(2,143) = 0;
unknown(2,144) = 0;
unknown(2,145) = 0;
unknown(2,146) = 0;
unknown(2,147) = t864;
unknown(2,148) = -t496;
unknown(2,149) = (g(1) * t875 + g(2) * t871 + g(3) * t867);
unknown(2,150) = (-g(3) * (-t481 * t867 + t544 - t545 + t584 + t878) - g(2) * (-t481 * t871 + t549 + t550 + t587 + t881) - g(1) * (-t481 * t875 - t553 - t554 + t590 + t884));
unknown(3,1) = 0;
unknown(3,2) = 0;
unknown(3,3) = 0;
unknown(3,4) = 0;
unknown(3,5) = 0;
unknown(3,6) = 0;
unknown(3,7) = 0;
unknown(3,8) = 0;
unknown(3,9) = 0;
unknown(3,10) = 0;
unknown(3,11) = 0;
unknown(3,12) = 0;
unknown(3,13) = 0;
unknown(3,14) = 0;
unknown(3,15) = 0;
unknown(3,16) = 0;
unknown(3,17) = 0;
unknown(3,18) = 0;
unknown(3,19) = 0;
unknown(3,20) = 0;
unknown(3,21) = 0;
unknown(3,22) = 0;
unknown(3,23) = 0;
unknown(3,24) = 0;
unknown(3,25) = 0;
unknown(3,26) = 0;
unknown(3,27) = (t1018 * t493 - t1024 - t1029);
unknown(3,28) = (-g(1) * t1039 - g(2) * t1035 - t1031 * t493);
unknown(3,29) = 0;
unknown(3,30) = 0;
unknown(3,31) = 0;
unknown(3,32) = 0;
unknown(3,33) = 0;
unknown(3,34) = 0;
unknown(3,35) = 0;
unknown(3,36) = 0;
unknown(3,37) = (-g(1) * t1057 - g(2) * t1053 - g(3) * t1047);
unknown(3,38) = (-g(1) * t1074 - g(2) * t1070 - g(3) * t1064);
unknown(3,39) = 0;
unknown(3,40) = (pkin(23) * t1018 * t493 - pkin(23) * t1024 - pkin(23) * t1029);
unknown(3,41) = 0;
unknown(3,42) = 0;
unknown(3,43) = 0;
unknown(3,44) = 0;
unknown(3,45) = 0;
unknown(3,46) = 0;
unknown(3,47) = (-g(1) * t1092 - g(2) * t1088 - g(3) * t1084);
unknown(3,48) = (-g(1) * t1105 - g(2) * t1101 - g(3) * t1097);
unknown(3,49) = 0;
unknown(3,50) = t1121;
unknown(3,51) = 0;
unknown(3,52) = 0;
unknown(3,53) = 0;
unknown(3,54) = 0;
unknown(3,55) = 0;
unknown(3,56) = 0;
unknown(3,57) = (-t1135 - t1149 - t1153);
unknown(3,58) = (-g(1) * t1169 - g(2) * t1165 - g(3) * t1159);
unknown(3,59) = 0;
unknown(3,60) = (-g(3) * (t1172 + t1108 - t1110) - g(2) * (t1175 + t1113 + t1114) - g(1) * (t1178 + t1117 + t1118));
unknown(3,61) = 0;
unknown(3,62) = 0;
unknown(3,63) = 0;
unknown(3,64) = 0;
unknown(3,65) = 0;
unknown(3,66) = 0;
unknown(3,67) = (-t1135 * t228 - t1149 * t228 - t1153 * t228);
unknown(3,68) = (t1135 * t230 + t1149 * t230 + t1153 * t230);
unknown(3,69) = (g(1) * t1169 + g(2) * t1165 + g(3) * t1159);
unknown(3,70) = (-g(3) * (-pkin(13) * t1159 + t1108 - t1110 + t1172) - g(2) * (-pkin(13) * t1165 + t1113 + t1114 + t1175) - g(1) * (-pkin(13) * t1169 + t1117 + t1118 + t1178));
unknown(3,71) = 0;
unknown(3,72) = 0;
unknown(3,73) = 0;
unknown(3,74) = 0;
unknown(3,75) = 0;
unknown(3,76) = 0;
unknown(3,77) = 0;
unknown(3,78) = 0;
unknown(3,79) = 0;
unknown(3,80) = 0;
unknown(3,81) = 0;
unknown(3,82) = 0;
unknown(3,83) = 0;
unknown(3,84) = 0;
unknown(3,85) = 0;
unknown(3,86) = 0;
unknown(3,87) = (-g(3) * (t1235 * t671 + t1237 * t676) - g(2) * (t1241 * t301 + t1243 * t299) - g(1) * (t1247 * t301 + t1249 * t299));
unknown(3,88) = (-g(3) * (t1235 * t676 - t1237 * t671) - g(2) * (-t1241 * t299 + t1243 * t301) - g(1) * (-t1247 * t299 + t1249 * t301));
unknown(3,89) = 0;
unknown(3,90) = 0;
unknown(3,91) = 0;
unknown(3,92) = 0;
unknown(3,93) = 0;
unknown(3,94) = 0;
unknown(3,95) = 0;
unknown(3,96) = 0;
unknown(3,97) = (-g(1) * t1302 - g(2) * t1294 - g(3) * t1286);
unknown(3,98) = (-g(1) * t1321 - g(2) * t1315 - g(3) * t1309);
unknown(3,99) = 0;
unknown(3,100) = (pkin(24) * t1018 * t493 - pkin(24) * t1024 - pkin(24) * t1029);
unknown(3,101) = 0;
unknown(3,102) = 0;
unknown(3,103) = 0;
unknown(3,104) = 0;
unknown(3,105) = 0;
unknown(3,106) = 0;
unknown(3,107) = (-g(3) * (-t1286 * t370 - t1309 * t372 + t1360 * t372 - t1366 * t370) - g(2) * (-t1294 * t370 - t1315 * t372 + t1374 * t372 - t1380 * t370) - g(1) * (-t1302 * t370 - t1321 * t372 + t1385 * t372 - t1388 * t370));
unknown(3,108) = (-g(3) * (t1286 * t372 - t1309 * t370 + t1360 * t370 + t1366 * t372) - g(2) * (t1294 * t372 - t1315 * t370 + t1374 * t370 + t1380 * t372) - g(1) * (t1302 * t372 - t1321 * t370 + t1385 * t370 + t1388 * t372));
unknown(3,109) = 0;
unknown(3,110) = (-g(3) * (-pkin(24) * t1042 * t118 + pkin(19) * t1286) - g(2) * (pkin(19) * t1294 + pkin(24) * t1023) - g(1) * (pkin(19) * t1302 + pkin(24) * t1028));
unknown(3,111) = 0;
unknown(3,112) = 0;
unknown(3,113) = 0;
unknown(3,114) = 0;
unknown(3,115) = 0;
unknown(3,116) = 0;
unknown(3,117) = (-g(1) * t1440 - g(2) * t1436 - g(3) * t1430);
unknown(3,118) = (-g(1) * t1457 - g(2) * t1453 - g(3) * t1447);
unknown(3,119) = 0;
unknown(3,120) = t1121;
unknown(3,121) = 0;
unknown(3,122) = 0;
unknown(3,123) = 0;
unknown(3,124) = 0;
unknown(3,125) = 0;
unknown(3,126) = 0;
unknown(3,127) = (-g(1) * t1470 - g(2) * t1466 - g(3) * t1462);
unknown(3,128) = (-g(1) * t1483 - g(2) * t1479 - g(3) * t1475);
unknown(3,129) = 0;
unknown(3,130) = t1121;
unknown(3,131) = 0;
unknown(3,132) = 0;
unknown(3,133) = 0;
unknown(3,134) = 0;
unknown(3,135) = 0;
unknown(3,136) = 0;
unknown(3,137) = t1539;
unknown(3,138) = (-g(1) * t1556 - g(2) * t1550 - g(3) * t1544);
unknown(3,139) = 0;
unknown(3,140) = (-g(3) * (t1559 + t1108 - t1110) - g(2) * (t1562 + t1113 + t1114) - g(1) * (t1565 + t1117 + t1118));
unknown(3,141) = 0;
unknown(3,142) = 0;
unknown(3,143) = 0;
unknown(3,144) = 0;
unknown(3,145) = 0;
unknown(3,146) = 0;
unknown(3,147) = t1539;
unknown(3,148) = 0;
unknown(3,149) = (g(1) * t1556 + g(2) * t1550 + g(3) * t1544);
unknown(3,150) = (-g(3) * (-t1544 * t481 - t1578 * t1581 + t1108 - t1110 + t1559) - g(2) * (-t1550 * t481 - t1581 * t1590 + t1113 + t1114 + t1562) - g(1) * (-t1556 * t481 - t1581 * t1596 + t1117 + t1118 + t1565));
unknown(4,1) = 0;
unknown(4,2) = 0;
unknown(4,3) = 0;
unknown(4,4) = 0;
unknown(4,5) = 0;
unknown(4,6) = 0;
unknown(4,7) = 0;
unknown(4,8) = 0;
unknown(4,9) = 0;
unknown(4,10) = 0;
unknown(4,11) = 0;
unknown(4,12) = 0;
unknown(4,13) = 0;
unknown(4,14) = 0;
unknown(4,15) = 0;
unknown(4,16) = 0;
unknown(4,17) = 0;
unknown(4,18) = 0;
unknown(4,19) = 0;
unknown(4,20) = 0;
unknown(4,21) = 0;
unknown(4,22) = 0;
unknown(4,23) = 0;
unknown(4,24) = 0;
unknown(4,25) = 0;
unknown(4,26) = 0;
unknown(4,27) = 0;
unknown(4,28) = 0;
unknown(4,29) = 0;
unknown(4,30) = 0;
unknown(4,31) = 0;
unknown(4,32) = 0;
unknown(4,33) = 0;
unknown(4,34) = 0;
unknown(4,35) = 0;
unknown(4,36) = 0;
unknown(4,37) = 0;
unknown(4,38) = 0;
unknown(4,39) = 0;
unknown(4,40) = 0;
unknown(4,41) = 0;
unknown(4,42) = 0;
unknown(4,43) = 0;
unknown(4,44) = 0;
unknown(4,45) = 0;
unknown(4,46) = 0;
unknown(4,47) = (-t1602 - t1603 - t1604);
unknown(4,48) = (g(1) * t177 + g(2) * t1141 + g(3) * t1127);
unknown(4,49) = 0;
unknown(4,50) = 0;
unknown(4,51) = 0;
unknown(4,52) = 0;
unknown(4,53) = 0;
unknown(4,54) = 0;
unknown(4,55) = 0;
unknown(4,56) = 0;
unknown(4,57) = (-t1613 - t1617 - t1621);
unknown(4,58) = (-g(1) * t1633 - g(2) * t1629 - g(3) * t1625);
unknown(4,59) = 0;
unknown(4,60) = (-pkin(10) * t1602 - pkin(10) * t1603 - pkin(10) * t1604);
unknown(4,61) = 0;
unknown(4,62) = 0;
unknown(4,63) = 0;
unknown(4,64) = 0;
unknown(4,65) = 0;
unknown(4,66) = 0;
unknown(4,67) = (-t1613 * t228 - t1617 * t228 - t1621 * t228);
unknown(4,68) = (t1613 * t230 + t1617 * t230 + t1621 * t230);
unknown(4,69) = (g(1) * t1633 + g(2) * t1629 + g(3) * t1625);
unknown(4,70) = (-g(3) * (pkin(10) * t1132 - pkin(13) * t1625) - g(2) * (pkin(10) * t1146 - pkin(13) * t1629) - g(1) * (pkin(10) * t186 - pkin(13) * t1633));
unknown(4,71) = 0;
unknown(4,72) = 0;
unknown(4,73) = 0;
unknown(4,74) = 0;
unknown(4,75) = 0;
unknown(4,76) = 0;
unknown(4,77) = 0;
unknown(4,78) = 0;
unknown(4,79) = 0;
unknown(4,80) = 0;
unknown(4,81) = 0;
unknown(4,82) = 0;
unknown(4,83) = 0;
unknown(4,84) = 0;
unknown(4,85) = 0;
unknown(4,86) = 0;
unknown(4,87) = 0;
unknown(4,88) = 0;
unknown(4,89) = 0;
unknown(4,90) = 0;
unknown(4,91) = 0;
unknown(4,92) = 0;
unknown(4,93) = 0;
unknown(4,94) = 0;
unknown(4,95) = 0;
unknown(4,96) = 0;
unknown(4,97) = 0;
unknown(4,98) = 0;
unknown(4,99) = 0;
unknown(4,100) = 0;
unknown(4,101) = 0;
unknown(4,102) = 0;
unknown(4,103) = 0;
unknown(4,104) = 0;
unknown(4,105) = 0;
unknown(4,106) = 0;
unknown(4,107) = 0;
unknown(4,108) = 0;
unknown(4,109) = 0;
unknown(4,110) = 0;
unknown(4,111) = 0;
unknown(4,112) = 0;
unknown(4,113) = 0;
unknown(4,114) = 0;
unknown(4,115) = 0;
unknown(4,116) = 0;
unknown(4,117) = 0;
unknown(4,118) = 0;
unknown(4,119) = 0;
unknown(4,120) = 0;
unknown(4,121) = 0;
unknown(4,122) = 0;
unknown(4,123) = 0;
unknown(4,124) = 0;
unknown(4,125) = 0;
unknown(4,126) = 0;
unknown(4,127) = 0;
unknown(4,128) = 0;
unknown(4,129) = 0;
unknown(4,130) = 0;
unknown(4,131) = 0;
unknown(4,132) = 0;
unknown(4,133) = 0;
unknown(4,134) = 0;
unknown(4,135) = 0;
unknown(4,136) = 0;
unknown(4,137) = t1688;
unknown(4,138) = (-g(1) * t1699 - g(2) * t1695 - g(3) * t1691);
unknown(4,139) = 0;
unknown(4,140) = 0;
unknown(4,141) = 0;
unknown(4,142) = 0;
unknown(4,143) = 0;
unknown(4,144) = 0;
unknown(4,145) = 0;
unknown(4,146) = 0;
unknown(4,147) = t1688;
unknown(4,148) = 0;
unknown(4,149) = (g(1) * t1699 + g(2) * t1695 + g(3) * t1691);
unknown(4,150) = (-g(3) * (-t1578 * t1709 - t1691 * t481) - g(2) * (-t1590 * t1709 - t1695 * t481) - g(1) * (-t1596 * t1709 - t1699 * t481));
unknown(5,1) = 0;
unknown(5,2) = 0;
unknown(5,3) = 0;
unknown(5,4) = 0;
unknown(5,5) = 0;
unknown(5,6) = 0;
unknown(5,7) = 0;
unknown(5,8) = 0;
unknown(5,9) = 0;
unknown(5,10) = 0;
unknown(5,11) = 0;
unknown(5,12) = 0;
unknown(5,13) = 0;
unknown(5,14) = 0;
unknown(5,15) = 0;
unknown(5,16) = 0;
unknown(5,17) = 0;
unknown(5,18) = 0;
unknown(5,19) = 0;
unknown(5,20) = 0;
unknown(5,21) = 0;
unknown(5,22) = 0;
unknown(5,23) = 0;
unknown(5,24) = 0;
unknown(5,25) = 0;
unknown(5,26) = 0;
unknown(5,27) = 0;
unknown(5,28) = 0;
unknown(5,29) = 0;
unknown(5,30) = 0;
unknown(5,31) = 0;
unknown(5,32) = 0;
unknown(5,33) = 0;
unknown(5,34) = 0;
unknown(5,35) = 0;
unknown(5,36) = 0;
unknown(5,37) = 0;
unknown(5,38) = 0;
unknown(5,39) = 0;
unknown(5,40) = 0;
unknown(5,41) = 0;
unknown(5,42) = 0;
unknown(5,43) = 0;
unknown(5,44) = 0;
unknown(5,45) = 0;
unknown(5,46) = 0;
unknown(5,47) = 0;
unknown(5,48) = 0;
unknown(5,49) = 0;
unknown(5,50) = 0;
unknown(5,51) = 0;
unknown(5,52) = 0;
unknown(5,53) = 0;
unknown(5,54) = 0;
unknown(5,55) = 0;
unknown(5,56) = 0;
unknown(5,57) = 0;
unknown(5,58) = 0;
unknown(5,59) = 0;
unknown(5,60) = 0;
unknown(5,61) = 0;
unknown(5,62) = 0;
unknown(5,63) = 0;
unknown(5,64) = 0;
unknown(5,65) = 0;
unknown(5,66) = 0;
unknown(5,67) = (-g(3) * (-t1725 * t230 - t228 * t9) - g(2) * (-t1730 * t230 - t244) - g(1) * t241);
unknown(5,68) = (-g(3) * (-t1725 * t228 + t230 * t9) - g(2) * (-t1730 * t228 + t235) + g(1) * t232);
unknown(5,69) = 0;
unknown(5,70) = 0;
unknown(5,71) = 0;
unknown(5,72) = 0;
unknown(5,73) = 0;
unknown(5,74) = 0;
unknown(5,75) = 0;
unknown(5,76) = 0;
unknown(5,77) = 0;
unknown(5,78) = 0;
unknown(5,79) = 0;
unknown(5,80) = 0;
unknown(5,81) = 0;
unknown(5,82) = 0;
unknown(5,83) = 0;
unknown(5,84) = 0;
unknown(5,85) = 0;
unknown(5,86) = 0;
unknown(5,87) = 0;
unknown(5,88) = 0;
unknown(5,89) = 0;
unknown(5,90) = 0;
unknown(5,91) = 0;
unknown(5,92) = 0;
unknown(5,93) = 0;
unknown(5,94) = 0;
unknown(5,95) = 0;
unknown(5,96) = 0;
unknown(5,97) = 0;
unknown(5,98) = 0;
unknown(5,99) = 0;
unknown(5,100) = 0;
unknown(5,101) = 0;
unknown(5,102) = 0;
unknown(5,103) = 0;
unknown(5,104) = 0;
unknown(5,105) = 0;
unknown(5,106) = 0;
unknown(5,107) = 0;
unknown(5,108) = 0;
unknown(5,109) = 0;
unknown(5,110) = 0;
unknown(5,111) = 0;
unknown(5,112) = 0;
unknown(5,113) = 0;
unknown(5,114) = 0;
unknown(5,115) = 0;
unknown(5,116) = 0;
unknown(5,117) = 0;
unknown(5,118) = 0;
unknown(5,119) = 0;
unknown(5,120) = 0;
unknown(5,121) = 0;
unknown(5,122) = 0;
unknown(5,123) = 0;
unknown(5,124) = 0;
unknown(5,125) = 0;
unknown(5,126) = 0;
unknown(5,127) = 0;
unknown(5,128) = 0;
unknown(5,129) = 0;
unknown(5,130) = 0;
unknown(5,131) = 0;
unknown(5,132) = 0;
unknown(5,133) = 0;
unknown(5,134) = 0;
unknown(5,135) = 0;
unknown(5,136) = 0;
unknown(5,137) = 0;
unknown(5,138) = 0;
unknown(5,139) = 0;
unknown(5,140) = 0;
unknown(5,141) = 0;
unknown(5,142) = 0;
unknown(5,143) = 0;
unknown(5,144) = 0;
unknown(5,145) = 0;
unknown(5,146) = 0;
unknown(5,147) = 0;
unknown(5,148) = 0;
unknown(5,149) = 0;
unknown(5,150) = 0;
taug_reg = unknown;
