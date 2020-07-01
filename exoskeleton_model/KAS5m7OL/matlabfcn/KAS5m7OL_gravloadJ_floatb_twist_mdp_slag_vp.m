% Calculate Gravitation load on the joints for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% MDP [88x1]
%   Minimal dynamic parameter vector (fixed base model)
%   see KAS5m7OL_convert_par2_MPV_fixb.m
% 
% Output:
% taug [13x1]
%   joint torques required to compensate gravitation load

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function taug = KAS5m7OL_gravloadJ_floatb_twist_mdp_slag_vp(qJ, g, pkin, MDP)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(19,1),zeros(88,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_mdp_slag_vp: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_mdp_slag_vp: pkin has to be [19x1] (double)');
assert(isreal(MDP) && all(size(MDP) == [88 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_mdp_slag_vp: MDP has to be [88x1] (double)'); 

%% Symbolic Calculation
% From gravload_joint_fixb_mdp_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:59:57
% EndTime: 2020-06-30 17:59:58
% DurationCPUTime: 0.34s
% Computational Cost: add. (2468->465), mult. (1819->579), div. (0->0), fcn. (1836->26), ass. (0->213)
unknown=NaN(13,1);
t1 = sin(qJ(1));
t2 = sin(qJ(2));
t3 = t1 * t2;
t4 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t5 = cos(t4);
t7 = cos(qJ(1));
t8 = sin(t4);
t10 = t3 * t5 - t7 * t8;
t12 = qJ(3) + qJ(4);
t13 = cos(t12);
t14 = pkin(6) * t13;
t15 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t16 = cos(t15);
t17 = pkin(9) * t16;
t18 = cos(qJ(3));
t19 = t18 * pkin(18);
t20 = t14 - t17 + t19;
t21 = t3 * t20;
t22 = cos(qJ(2));
t23 = t1 * t22;
t24 = t23 * pkin(16);
t25 = sin(t12);
t26 = pkin(6) * t25;
t27 = sin(t15);
t28 = pkin(9) * t27;
t29 = sin(qJ(3));
t30 = t29 * pkin(18);
t31 = t26 - t28 + t30;
t32 = t7 * t31;
t33 = t7 * pkin(11);
t36 = t7 * t2;
t39 = t1 * t8 + t36 * t5;
t41 = t36 * t20;
t42 = t7 * t22;
t43 = t42 * pkin(16);
t44 = t1 * t31;
t45 = t1 * pkin(11);
t50 = qJ(3) + qJ(9) + qJ(10);
t51 = cos(t50);
t53 = sin(t50);
t55 = t3 * t51 - t7 * t53;
t59 = t1 * t53 + t36 * t51;
t65 = -t3 * t53 - t7 * t51;
t69 = t1 * t51 - t36 * t53;
t73 = qJ(3) + qJ(4) + qJ(11);
t74 = sin(t73);
t76 = cos(t73);
t78 = -t3 * t74 - t7 * t76;
t82 = t1 * t76 - t36 * t74;
t88 = -t3 * t76 + t7 * t74;
t92 = -t1 * t74 - t36 * t76;
t96 = t14 + t19;
t98 = t26 + t30;
t110 = t3 * t8 + t7 * t5;
t114 = -t1 * t5 + t36 * t8;
t116 = -g(1) * t114 - g(2) * t110;
t129 = g(2) * t1;
t131 = g(1) * t7;
t133 = -t129 * t22 - t131 * t22;
t139 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t140 = cos(t139);
t142 = sin(t139);
t144 = t3 * t140 - t7 * t142;
t148 = t1 * t142 + t36 * t140;
t154 = t7 * t140 + t3 * t142;
t155 = cos(qJ(7));
t157 = sin(qJ(7));
t159 = t154 * t155 + t23 * t157;
t163 = -t1 * t140 + t36 * t142;
t165 = t42 * t157;
t172 = -t154 * t157 + t23 * t155;
t175 = t42 * t155;
t184 = (-g(2) * (-t10 * qJ(13) - t21 + t24 + t32 + t33) - g(1) * (-t39 * qJ(13) - t41 + t43 - t44 - t45)) * MDP(88) + (-g(1) * t59 - g(2) * t55) * MDP(67) + (-g(1) * t69 - g(2) * t65) * MDP(68) + (-g(1) * t82 - g(2) * t78) * MDP(74) + (-g(1) * t92 - g(2) * t88) * MDP(75) + (-g(2) * (-t3 * t96 + t7 * t98 + t24 + t33) - g(1) * (-t1 * t98 - t36 * t96 + t43 - t45)) * MDP(76) + t116 * MDP(82) + (-g(1) * t39 - g(2) * t10) * MDP(83) + (-g(2) * (-t21 + t24 + t32 + t33) - g(1) * (-t41 + t43 - t44 - t45)) * MDP(84) + t116 * MDP(85) + t133 * MDP(86) + (g(1) * t39 + g(2) * t10) * MDP(87) + (-g(1) * t148 - g(2) * t144) * MDP(38) + (-g(2) * t159 - g(1) * (t163 * t155 + t165)) * MDP(44) + (-g(2) * t172 - g(1) * (-t163 * t157 + t175)) * MDP(45) + (g(1) * t148 + g(2) * t144) * MDP(46);
t186 = qJ(3) + qJ(4) + qJ(5);
t187 = cos(t186);
t188 = pkin(7) * t187;
t189 = t188 + t14 + t19;
t191 = sin(t186);
t192 = pkin(7) * t191;
t193 = t192 + t26 + t30;
t204 = pkin(3) + qJ(8);
t205 = sin(t204);
t207 = cos(t204);
t209 = t3 * t205 + t7 * t207;
t213 = -t1 * t207 + t36 * t205;
t219 = -t7 * t205 + t3 * t207;
t223 = t1 * t205 + t36 * t207;
t227 = qJ(3) + qJ(9);
t228 = cos(t227);
t230 = sin(t227);
t232 = -t3 * t228 + t7 * t230;
t236 = -t1 * t230 - t36 * t228;
t242 = t7 * t228 + t3 * t230;
t246 = -t1 * t228 + t36 * t230;
t252 = g(2) * t7;
t253 = g(1) * t1;
t263 = -t3 * t18 + t7 * t29;
t267 = -t1 * t29 - t36 * t18;
t273 = t7 * t18 + t3 * t29;
t277 = -t1 * t18 + t36 * t29;
t283 = -t3 * t13 + t7 * t25;
t287 = -t1 * t25 - t36 * t13;
t293 = t7 * t13 + t3 * t25;
t297 = -t1 * t13 + t36 * t25;
t303 = -t3 * t187 + t7 * t191;
t307 = -t1 * t191 - t36 * t187;
t313 = t7 * t187 + t3 * t191;
t317 = -t1 * t187 + t36 * t191;
t325 = (-g(2) * (-t144 * pkin(10) - t3 * t189 + t7 * t193 + t24 + t33) - g(1) * (-t148 * pkin(10) - t1 * t193 - t36 * t189 + t43 - t45)) * MDP(47) + (-g(1) * t213 - g(2) * t209) * MDP(53) + (-g(1) * t223 - g(2) * t219) * MDP(54) + (-g(1) * t236 - g(2) * t232) * MDP(60) + (-g(1) * t246 - g(2) * t242) * MDP(61) + (-t129 - t131) * MDP(2) + (-t252 + t253) * MDP(3) + (t129 * t2 + t131 * t2) * MDP(9) - t133 * MDP(10) + (-g(1) * t267 - g(2) * t263) * MDP(16) + (-g(1) * t277 - g(2) * t273) * MDP(17) + (-g(1) * t287 - g(2) * t283) * MDP(23) + (-g(1) * t297 - g(2) * t293) * MDP(24) + (-g(1) * t307 - g(2) * t303) * MDP(30) + (-g(1) * t317 - g(2) * t313) * MDP(31) + (-g(1) * t163 - g(2) * t154) * MDP(37);
t327 = g(3) * t2;
t332 = g(3) * t22;
t335 = t252 * t2 - t253 * t2 - t332;
t338 = t22 * t18;
t344 = t22 * t29;
t350 = t22 * t13;
t356 = t22 * t25;
t362 = t22 * t187;
t368 = t22 * t191;
t374 = t22 * t142;
t380 = t22 * t140;
t383 = t327 * t140 + t252 * t380 - t253 * t380;
t385 = t2 * t142;
t390 = t142 * t155;
t405 = t142 * t157;
t420 = t22 * pkin(16);
t423 = t140 * pkin(10);
t426 = t36 * pkin(16);
t431 = t3 * pkin(16);
t437 = t22 * t205;
t442 = (-t252 * t22 + t253 * t22 - t327) * MDP(9) + t335 * MDP(10) + (-t327 * t18 - t252 * t338 + t253 * t338) * MDP(16) + (t252 * t344 - t253 * t344 + t327 * t29) * MDP(17) + (-t327 * t13 - t252 * t350 + t253 * t350) * MDP(23) + (t327 * t25 + t252 * t356 - t253 * t356) * MDP(24) + (-t327 * t187 - t252 * t362 + t253 * t362) * MDP(30) + (t327 * t191 + t252 * t368 - t253 * t368) * MDP(31) + (t327 * t142 + t252 * t374 - t253 * t374) * MDP(37) + t383 * MDP(38) + (-g(3) * (-t385 * t155 - t22 * t157) - g(2) * (t36 * t157 - t42 * t390) - g(1) * (-t3 * t157 + t23 * t390)) * MDP(44) + (-g(3) * (-t22 * t155 + t385 * t157) - g(2) * (t36 * t155 + t42 * t405) - g(1) * (-t3 * t155 - t23 * t405)) * MDP(45) - t383 * MDP(46) + (-g(3) * (t2 * t140 * pkin(10) + t2 * t189 - t420) - g(2) * (t42 * t189 + t42 * t423 + t426) - g(1) * (-t23 * t189 - t23 * t423 - t431)) * MDP(47) + (t327 * t205 + t252 * t437 - t253 * t437) * MDP(53);
t444 = t22 * t207;
t450 = t22 * t228;
t456 = t22 * t230;
t462 = t22 * t51;
t468 = t22 * t53;
t474 = t22 * t74;
t480 = t22 * t76;
t497 = t22 * t8;
t500 = t252 * t497 - t253 * t497 + t327 * t8;
t503 = t22 * t5;
t506 = t252 * t503 - t253 * t503 + t327 * t5;
t508 = t2 * t20;
t511 = t42 * t20;
t514 = t23 * t20;
t526 = t5 * qJ(13);
t535 = (t327 * t207 + t252 * t444 - t253 * t444) * MDP(54) + (-t327 * t228 - t252 * t450 + t253 * t450) * MDP(60) + (t327 * t230 + t252 * t456 - t253 * t456) * MDP(61) + (t252 * t462 - t253 * t462 + t327 * t51) * MDP(67) + (-t252 * t468 + t253 * t468 - t327 * t53) * MDP(68) + (-t252 * t474 + t253 * t474 - t327 * t74) * MDP(74) + (-t252 * t480 + t253 * t480 - t327 * t76) * MDP(75) + (-g(3) * (t2 * t96 - t420) - g(2) * (t42 * t96 + t426) - g(1) * (-t23 * t96 - t431)) * MDP(76) + t500 * MDP(82) + t506 * MDP(83) + (-g(3) * (t508 - t420) - g(2) * (t511 + t426) - g(1) * (-t514 - t431)) * MDP(84) + t500 * MDP(85) - t335 * MDP(86) - t506 * MDP(87) + (-g(3) * (t2 * t5 * qJ(13) - t420 + t508) - g(2) * (t42 * t526 + t426 + t511) - g(1) * (-t23 * t526 - t431 - t514)) * MDP(88);
t537 = t497 * qJ(13);
t541 = t114 * qJ(13);
t542 = -t36 * t31;
t543 = t1 * t20;
t546 = -t110 * qJ(13);
t547 = -t3 * t31;
t548 = t7 * t20;
t553 = t332 * t8;
t557 = (-g(1) * t110 + g(2) * t114 - t553) * MDP(87);
t558 = t332 * t5;
t561 = -g(1) * t10 + g(2) * t39 - t558;
t562 = t561 * MDP(85);
t570 = g(2) * t114;
t571 = -g(1) * t110;
t573 = (t553 - t570 - t571) * MDP(83);
t574 = t561 * MDP(82);
t590 = (-g(1) * t313 + g(2) * t317 - t332 * t191) * MDP(30);
t595 = (g(1) * t303 - g(2) * t307 - t332 * t187) * MDP(31);
t597 = -g(2) * t148;
t598 = g(1) * t144;
t600 = (-t332 * t140 - t597 - t598) * MDP(37);
t601 = t332 * t142;
t602 = g(2) * t163;
t603 = -g(1) * t154;
t605 = (t601 - t602 - t603) * MDP(38);
t611 = (-t332 * t140 * t155 - t597 * t155 - t598 * t155) * MDP(44);
t612 = (-g(3) * (t22 * t31 + t537) - g(2) * (-t541 + t542 + t543) - g(1) * (-t546 - t547 + t548)) * MDP(88) + t557 + t562 + (-t332 * t31 - g(2) * (t542 + t543) - g(1) * (-t547 + t548)) * MDP(84) + t573 + t574 + (-t332 * t98 - g(2) * (t1 * t96 - t36 * t98) - g(1) * (t3 * t98 + t7 * t96)) * MDP(76) + t590 + t595 + t600 + t605 + t611;
t618 = (t332 * t140 * t157 + t597 * t157 + t598 * t157) * MDP(45);
t622 = (-g(1) * t154 + g(2) * t163 - t601) * MDP(46);
t623 = t374 * pkin(10);
t627 = t163 * pkin(10);
t632 = -t154 * pkin(10);
t643 = (-g(1) * t242 + g(2) * t246 - t332 * t230) * MDP(60);
t648 = (g(1) * t232 - g(2) * t236 - t332 * t228) * MDP(61);
t653 = (-g(1) * t65 + g(2) * t69 + t332 * t53) * MDP(67);
t658 = (g(1) * t55 - g(2) * t59 + t332 * t51) * MDP(68);
t663 = (-g(1) * t88 + g(2) * t92 + t332 * t76) * MDP(74);
t668 = (g(1) * t78 - g(2) * t82 - t332 * t74) * MDP(75);
t683 = (-g(1) * t293 + g(2) * t297 - t332 * t25) * MDP(23);
t688 = (g(1) * t283 - g(2) * t287 - t332 * t13) * MDP(24);
t689 = t618 + t622 + (-g(3) * (t22 * t193 + t623) - g(2) * (t1 * t189 - t36 * t193 - t627) - g(1) * (t7 * t189 + t3 * t193 - t632)) * MDP(47) + t643 + t648 + t653 + t658 + t663 + t668 + (-g(1) * t273 + g(2) * t277 - t332 * t29) * MDP(16) + (g(1) * t263 - g(2) * t267 - t332 * t18) * MDP(17) + t683 + t688;
t691 = -t192 - t26;
t696 = t188 + t14;
t719 = -t26 + t28;
t721 = t36 * t719;
t722 = t14 - t17;
t723 = t1 * t722;
t726 = t3 * t719;
t727 = t7 * t722;
t741 = t683 + t688 + t590 + t595 + t600 + t605 + t611 + t618 + t622 + (-g(3) * (-t22 * t691 + t623) - g(2) * (t1 * t696 + t36 * t691 - t627) - g(1) * (-t3 * t691 + t7 * t696 - t632)) * MDP(47) + t663 + t668 + (-t332 * t26 - g(2) * (t1 * pkin(6) * t13 - t36 * t26) - g(1) * (t7 * pkin(6) * t13 + t3 * t26)) * MDP(76) + t574 + t573 + (t332 * t719 - g(2) * (t721 + t723) - g(1) * (-t726 + t727)) * MDP(84) + t562 + t557 + (-g(3) * (-t22 * t719 + t537) - g(2) * (-t541 + t721 + t723) - g(1) * (-t546 - t726 + t727)) * MDP(88);
t801 = t36 * t28;
t803 = t1 * pkin(9) * t16;
t806 = t3 * t28;
t808 = t7 * pkin(9) * t16;
unknown(1,1) = t184 + t325;
unknown(2,1) = t442 + t535;
unknown(3,1) = t612 + t689;
unknown(4,1) = t741;
unknown(5,1) = t590 + t595 + t600 + t605 + t611 + t618 + t622 + (-g(3) * (t22 * pkin(7) * t191 + t623) - g(2) * (t1 * pkin(7) * t187 - t36 * t192 - t627) - g(1) * (t7 * pkin(7) * t187 + t3 * t192 - t632)) * MDP(47);
unknown(6,1) = t600 + t605 + t611 + t618 + t622 + (-t332 * t142 * pkin(10) + t602 * pkin(10) + t603 * pkin(10)) * MDP(47);
unknown(7,1) = (-g(3) * (-t2 * t155 - t374 * t157) - g(2) * (t163 * t157 - t175) - g(1) * t172) * MDP(44) + (-g(3) * (-t374 * t155 + t2 * t157) - g(2) * (t163 * t155 + t165) + g(1) * t159) * MDP(45);
unknown(8,1) = (-g(1) * t219 + g(2) * t223 - t332 * t207) * MDP(53) + (g(1) * t209 - g(2) * t213 + t332 * t205) * MDP(54);
unknown(9,1) = t643 + t648 + t653 + t658;
unknown(10,1) = t653 + t658;
unknown(11,1) = t663 + t668 + t574 + t573 + (t332 * t28 - g(2) * (t801 - t803) - g(1) * (-t806 - t808)) * MDP(84) + t562 + t557 + (-g(3) * (-t22 * pkin(9) * t27 + t537) - g(2) * (-t541 + t801 - t803) - g(1) * (-t546 - t806 - t808)) * MDP(88);
unknown(12,1) = t574 + t573 + t562 + t557 + (-t332 * t8 * qJ(13) + t570 * qJ(13) + t571 * qJ(13)) * MDP(88);
unknown(13,1) = (g(1) * t10 - g(2) * t39 + t558) * MDP(88);
taug = unknown;
