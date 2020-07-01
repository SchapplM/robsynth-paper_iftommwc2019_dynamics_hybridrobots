% Calculate kinetic energy for
% KAS5m7DE1
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% qJD [5x1]
%   Generalized joint velocities
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% m [16x1]
%   mass of all robot links (including the base)
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% Ifges [16x6]
%   inertia of all robot links about their respective body frame origins, in body frames
%   rows: links of the robot (starting with base)
%   columns: xx, yy, zz, xy, xz, yz (see inertial_parameters_convert_par1_par2.m)
% 
% Output:
% T [1x1]
%   kinetic energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = KAS5m7DE1_energykin_floatb_twist_slag_vp2(qJ, qJD, V_base, ...
  pkin, m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),zeros(6,1),zeros(24,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_energykin_floatb_twist_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m7DE1_energykin_floatb_twist_slag_vp2: qJD has to be [5x1] (double)');
assert(isreal(V_base) && all(size(V_base) == [6 1]), ...
  'KAS5m7DE1_energykin_floatb_twist_slag_vp2: V_base has to be [6x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_energykin_floatb_twist_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE1_energykin_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE1_energykin_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7DE1_energykin_floatb_twist_slag_vp2: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_floatb_twist_linkframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-14 20:15:51
% EndTime: 2020-05-14 20:34:01
% DurationCPUTime: 1088.98s
% Computational Cost: add. (78701481->602), mult. (103917246->883), div. (1437062->24), fcn. (40948174->43), ass. (0->354)
t1 = sin(qJ(3));
t2 = cos(qJ(3));
t3 = -pkin(23) + pkin(24);
t4 = t2 * t3;
t5 = pkin(3) + qJ(3);
t6 = cos(t5);
t7 = t6 * pkin(12);
t8 = -pkin(9) + t4 + t7;
t9 = pkin(11) ^ 2;
t10 = pkin(19) ^ 2;
t11 = t6 ^ 2;
t12 = pkin(12) ^ 2;
t14 = sin(t5);
t15 = t14 ^ 2;
t17 = -pkin(9) + t4;
t18 = t17 ^ 2;
t19 = t1 ^ 2;
t20 = t3 ^ 2;
t24 = t1 * t3;
t25 = t14 * pkin(12);
t26 = -t24 + t25;
t29 = 0.2e1 * t26 * t14 * pkin(12) + 0.2e1 * t8 * t6 * pkin(12) - t11 * t12 - t15 * t12 + t19 * t20 - t10 + t18 + t9;
t33 = 0.4e1 * t26 ^ 2 + 0.4e1 * t8 ^ 2;
t35 = t29 ^ 2;
t37 = sqrt(t9 * t33 - t35);
t39 = -0.2e1 * t26 * t37 + 0.2e1 * t8 * t29;
t40 = 0.1e1 / t33;
t42 = t39 * t40 + pkin(9) - t4 - t7;
t46 = 0.2e1 * t29 * t26 + 0.2e1 * t8 * t37;
t48 = t46 * t40 + t24 - t25;
t50 = -t1 * t42 - t2 * t48;
t53 = -t1 * t48 + t2 * t42;
t54 = atan2(t50, t53);
t55 = cos(t54);
t56 = 0.1e1 / pkin(19);
t57 = -t50 * t56;
t58 = cos(pkin(7));
t59 = t58 * pkin(18);
t60 = t57 * t59;
t61 = t53 * t56;
t62 = sin(pkin(7));
t63 = t62 * pkin(18);
t64 = t61 * t63;
t65 = pkin(17) ^ 2;
t66 = pkin(22) ^ 2;
t67 = t61 * t59;
t68 = t57 * t63;
t69 = t67 - t68;
t70 = t69 ^ 2;
t71 = t60 + t64;
t72 = t71 ^ 2;
t73 = pkin(24) ^ 2;
t74 = -pkin(24) - t67 + t68;
t77 = -0.2e1 * t74 * t69 + 0.2e1 * t71 ^ 2 + t65 - t66 - t70 - t72 + t73;
t81 = 0.4e1 * t71 ^ 2 + 0.4e1 * t74 ^ 2;
t83 = t77 ^ 2;
t85 = sqrt(t65 * t81 - t83);
t87 = -0.2e1 * t77 * t71 + 0.2e1 * t74 * t85;
t88 = 0.1e1 / t81;
t90 = -t87 * t88 - t60 - t64;
t93 = 0.2e1 * t71 * t85 + 0.2e1 * t74 * t77;
t95 = t93 * t88 + pkin(24) + t67 - t68;
t96 = atan2(t90, t95);
t97 = t96 + pkin(6);
t98 = sin(t97);
t99 = sin(qJ(2));
t100 = sin(qJ(1));
t102 = pkin(8) * V_base(5) + V_base(1);
t103 = t100 * t102;
t104 = cos(qJ(1));
t106 = -pkin(8) * V_base(4) + V_base(2);
t107 = t104 * t106;
t108 = V_base(6) + qJD(1);
t110 = -t108 * pkin(16) + t103 - t107;
t111 = t99 * t110;
t112 = cos(qJ(2));
t115 = t100 * V_base(4) - t104 * V_base(5);
t117 = t115 * pkin(16) + V_base(3);
t118 = t112 * t117;
t119 = t104 * V_base(4);
t120 = t100 * V_base(5);
t121 = t119 + t120 + qJD(2);
t123 = t121 * pkin(21) - t111 - t118;
t125 = cos(t97);
t126 = t104 * t102;
t127 = t100 * t106;
t130 = -t112 * t108 - t99 * t115;
t132 = -t130 * pkin(21) + t126 + t127;
t134 = t98 * t123 - t125 * t132;
t136 = sin(t54);
t137 = t125 * t123;
t138 = t98 * t132;
t139 = t112 * t115;
t140 = t99 * t108;
t141 = qJD(3) * t2;
t143 = qJD(3) * t1;
t144 = t143 * t3;
t146 = qJD(3) * t14 * pkin(12);
t147 = -t144 - t146;
t159 = t141 * t3;
t161 = qJD(3) * t6 * pkin(12);
t162 = -t159 + t161;
t167 = 0.2e1 * t162 * t14 * pkin(12) + 0.2e1 * t147 * t6 * pkin(12) - 0.2e1 * t17 * qJD(3) * t24 - 0.2e1 * t8 * qJD(3) * t25 + 0.2e1 * t26 * qJD(3) * t7 + 0.2e1 * t1 * t20 * t141;
t170 = 0.1e1 / t37;
t174 = 0.4e1 * t8 * t147 + 0.4e1 * t26 * t162;
t178 = -0.2e1 * t29 * t167 + 0.2e1 * t9 * t174;
t183 = t33 ^ 2;
t184 = 0.1e1 / t183;
t187 = t144 + t146 + (-t26 * t170 * t178 + 0.2e1 * t147 * t29 - 0.2e1 * t162 * t37 + 0.2e1 * t8 * t167) * t40 - 0.2e1 * t39 * t184 * t174;
t200 = t159 - t161 + (t8 * t170 * t178 + 0.2e1 * t147 * t37 + 0.2e1 * t29 * t162 + 0.2e1 * t167 * t26) * t40 - 0.2e1 * t46 * t184 * t174;
t202 = t1 * t187 + t141 * t42 - t143 * t48 + t2 * t200;
t203 = t202 * t56;
t204 = t203 * t59;
t209 = -t1 * t200 - t141 * t48 - t143 * t42 + t2 * t187;
t210 = t209 * t56;
t211 = t210 * t63;
t212 = t210 * t59;
t213 = t203 * t63;
t214 = t212 - t213;
t217 = t204 + t211;
t223 = -0.2e1 * t74 * t214 + 0.2e1 * t71 * t217;
t227 = 0.1e1 / t85;
t231 = -0.4e1 * t74 * t214 + 0.4e1 * t71 * t217;
t235 = -0.2e1 * t77 * t223 + 0.2e1 * t65 * t231;
t240 = t81 ^ 2;
t241 = 0.1e1 / t240;
t244 = -t204 - t211 - (t74 * t227 * t235 - 0.2e1 * t214 * t85 - 0.2e1 * t77 * t217 - 0.2e1 * t223 * t71) * t88 + 0.2e1 * t87 * t241 * t231;
t247 = t90 ^ 2;
t248 = t95 ^ 2;
t249 = 0.1e1 / t248;
t252 = 0.1e1 / (t247 * t249 + 0.1e1);
t253 = t244 / t95 * t252;
t264 = t212 - t213 + (t71 * t227 * t235 - 0.2e1 * t214 * t77 + 0.2e1 * t217 * t85 + 0.2e1 * t74 * t223) * t88 - 0.2e1 * t93 * t241 * t231;
t267 = t264 * t90 * t249 * t252;
t268 = t139 - t140 + t253 - t267;
t270 = t268 * pkin(24) + t137 + t138;
t272 = t55 * t134 - t136 * t270;
t275 = -t125 * t121 + t98 * t130;
t279 = t98 * t121 + t125 * t130;
t281 = t136 * t275 + t55 * t279;
t285 = t50 ^ 2;
t286 = t53 ^ 2;
t287 = 0.1e1 / t286;
t290 = 0.1e1 / (t285 * t287 + 0.1e1);
t295 = t139 - t140 + t253 - t267 + t202 / t53 * t290 + t209 * t50 * t287 * t290;
t305 = (t1 * t53 * t56 + t2 * t50 * t56) * pkin(19) + t24 - t25;
t312 = (-t1 * t50 * t56 + t2 * t53 * t56) * pkin(19) - pkin(9) + t4 + t7;
t313 = atan2(t305, t312);
t314 = -t313 + qJ(3) + t54;
t315 = cos(t314);
t317 = sin(t314);
t318 = t136 * t134;
t319 = t55 * t270;
t321 = t295 * pkin(19) + t318 + t319;
t323 = -t315 * t272 - t317 * t321;
t324 = t323 ^ 2;
t327 = t317 * t272 - t315 * t321;
t328 = t327 ^ 2;
t329 = t112 * t110;
t330 = t99 * t117;
t331 = t279 * pkin(24);
t333 = -t281 * pkin(19) + t329 - t330 - t331;
t334 = t333 ^ 2;
t338 = t272 ^ 2;
t339 = t318 + t319;
t340 = t339 ^ 2;
t341 = t329 - t330 - t331;
t342 = t341 ^ 2;
t346 = 0.1e1 / pkin(22);
t347 = t90 * t346;
t348 = sin(pkin(6));
t350 = -t95 * t346;
t351 = cos(pkin(6));
t353 = t347 * t348 + t350 * t351;
t357 = t347 * t351 - t350 * t348;
t361 = -(t357 * t348 - t353 * t351) * pkin(22) + pkin(24) + t67 - t68;
t366 = (t353 * t348 + t357 * t351) * pkin(22) + t60 + t64;
t367 = atan2(t361, t366);
t368 = t96 - t367;
t369 = sin(t368);
t372 = -t348 * t123 + t351 * t132;
t374 = cos(t368);
t375 = t351 * t123;
t376 = t348 * t132;
t377 = t139 - t140;
t379 = -t377 * pkin(22) - t375 - t376;
t381 = t369 * t372 - t374 * t379;
t384 = t351 * t121 - t348 * t130;
t388 = -t348 * t121 - t351 * t130;
t390 = t369 * t388 + t374 * t384;
t392 = t244 * t346;
t394 = -t264 * t346;
t396 = t392 * t348 + t394 * t351;
t400 = -t394 * t348 + t392 * t351;
t407 = t361 ^ 2;
t408 = t366 ^ 2;
t409 = 0.1e1 / t408;
t412 = 0.1e1 / (t407 * t409 + 0.1e1);
t422 = t139 - t140 + t253 - t267 - (-(t400 * t348 - t396 * t351) * pkin(22) + t212 - t213) / t366 * t412 + ((t396 * t348 + t400 * t351) * pkin(22) + t204 + t211) * t361 * t409 * t412;
t428 = t268 * pkin(23) + t137 + t138;
t430 = t1 * t428 + t2 * t134;
t432 = t1 * t134;
t433 = t2 * t428;
t434 = t139 - t140 + t253 - t267 + qJD(3);
t436 = t434 * pkin(9) - t432 + t433;
t438 = t1 * t430 - t2 * t436;
t439 = t438 ^ 2;
t442 = t1 * t436 + t2 * t430;
t443 = t442 ^ 2;
t444 = t279 * pkin(23);
t447 = -t1 * t275 + t2 * t279;
t448 = t447 * pkin(9);
t449 = t329 - t330 - t444 - t448;
t450 = t449 ^ 2;
t454 = sin(pkin(3));
t456 = cos(pkin(3));
t458 = t454 * t438 - t456 * t442;
t461 = t1 * t279 + t2 * t275;
t464 = t1 * t461 - t2 * t447;
t468 = t1 * t447 + t2 * t461;
t470 = t454 * t468 + t456 * t464;
t473 = t139 - t140 + t253 - t267 + 0.2e1 * qJD(3);
t477 = t456 * t438;
t478 = t454 * t442;
t479 = t477 + t478;
t482 = t454 * t464 - t456 * t468;
t487 = t458 ^ 2;
t488 = t479 ^ 2;
t494 = t369 * t379 + t374 * t372;
t497 = t369 * t384 - t374 * t388;
t502 = t430 ^ 2;
t503 = -t432 + t433;
t504 = t503 ^ 2;
t505 = t329 - t330 - t444;
t506 = t505 ^ 2;
t514 = t137 + t138;
t521 = -t136 * t279 + t55 * t275;
t524 = -t317 * t281 - t315 * t521;
t528 = -t315 * t281 + t317 * t521;
t541 = t305 ^ 2;
t542 = t312 ^ 2;
t543 = 0.1e1 / t542;
t546 = 0.1e1 / (t541 * t543 + 0.1e1);
t560 = t139 - t140 + t253 - t267 - ((t1 * t209 * t56 - t2 * t202 * t56 + t141 * t61 + t143 * t57) * pkin(19) + t159 - t161) / t312 * t546 + ((t1 * t202 * t56 + t2 * t209 * t56 + t141 * t57 - t143 * t61) * pkin(19) - t144 - t146) * t305 * t543 * t546 + qJD(3);
t571 = t272 * (-t295 * mrSges(11,2) + t281 * mrSges(11,3)) + m(12) * (t324 + t328 + t334) / 0.2e1 + m(11) * (t338 + t340 + t342) / 0.2e1 + t381 * (-t422 * mrSges(10,2) + t390 * mrSges(10,3)) + m(13) * (t439 + t443 + t450) / 0.2e1 + t458 * (-t473 * mrSges(14,2) + t470 * mrSges(14,3)) + t479 * (t473 * mrSges(14,1) - t482 * mrSges(14,3)) + m(14) * (t487 + t488 + t450) / 0.2e1 + t494 * (t422 * mrSges(10,1) - t497 * mrSges(10,3)) + m(5) * (t502 + t504 + t506) / 0.2e1 + t134 * (-t268 * mrSges(4,2) + t279 * mrSges(4,3)) + t514 * (t268 * mrSges(4,1) - t275 * mrSges(4,3)) + t524 * (Ifges(12,1) * t524 + Ifges(12,4) * t528 + Ifges(12,5) * t560) / 0.2e1 + t528 * (Ifges(12,4) * t524 + Ifges(12,2) * t528 + Ifges(12,6) * t560) / 0.2e1;
t616 = t126 + t127;
t621 = -t111 - t118;
t622 = t621 ^ 2;
t623 = -t329 + t330;
t624 = t623 ^ 2;
t625 = t616 ^ 2;
t635 = qJ(4) - qJ(3) + pkin(4);
t636 = sin(t635);
t638 = pkin(3) + qJ(3) - qJ(4);
t639 = sin(t638);
t641 = cos(t635);
t643 = t639 * pkin(12) - t641 * pkin(14) + t636 * pkin(15);
t646 = cos(t638);
t648 = -t646 * pkin(12) - t636 * pkin(14) - t641 * pkin(15) - pkin(10);
t649 = atan2(t643, t648);
t650 = t649 + pkin(3) + qJ(3) - qJ(4);
t651 = cos(t650);
t653 = sin(t650);
t655 = t653 * t470 - t651 * t482;
t658 = t651 * t470 + t653 * t482;
t661 = qJD(4) - qJD(3);
t662 = t661 * t641;
t666 = t661 * t636;
t668 = -t661 * t646 * pkin(12) + t666 * pkin(14) + t662 * pkin(15);
t671 = t643 ^ 2;
t672 = t648 ^ 2;
t673 = 0.1e1 / t672;
t676 = 0.1e1 / (t671 * t673 + 0.1e1);
t682 = -t661 * t639 * pkin(12) - t662 * pkin(14) + t666 * pkin(15);
t686 = t139 - t140 + t253 - t267 + qJD(3) - t668 / t648 * t676 + t682 * t643 * t673 * t676 + qJD(4);
t697 = t653 * t458;
t699 = t473 * pkin(12) + t477 + t478;
t700 = t651 * t699;
t702 = sqrt(t672 + t671);
t704 = t686 * t702 + t697 + t700;
t709 = t323 * (-t560 * mrSges(12,2) + t528 * mrSges(12,3)) + t327 * (t560 * mrSges(12,1) - t524 * mrSges(12,3)) + t464 * (Ifges(13,1) * t464 + Ifges(13,4) * t468 + Ifges(13,5) * t473) / 0.2e1 + t468 * (Ifges(13,4) * t464 + Ifges(13,2) * t468 + Ifges(13,6) * t473) / 0.2e1 + t473 * (Ifges(13,5) * t464 + Ifges(13,6) * t468 + Ifges(13,3) * t473) / 0.2e1 + t130 * (Ifges(3,1) * t130 - Ifges(3,4) * t377 + Ifges(3,5) * t121) / 0.2e1 - t377 * (Ifges(3,4) * t130 - Ifges(3,2) * t377 + Ifges(3,6) * t121) / 0.2e1 + t121 * (Ifges(3,5) * t130 - Ifges(3,6) * t377 + Ifges(3,3) * t121) / 0.2e1 + t616 * (t377 * mrSges(3,1) + t130 * mrSges(3,2)) + m(3) * (t622 + t624 + t625) / 0.2e1 + t377 * (Ifges(9,5) * t384 + Ifges(9,6) * t388 + Ifges(9,3) * t377) / 0.2e1 + t655 * (Ifges(15,4) * t658 + Ifges(15,2) * t655 + Ifges(15,6) * t686) / 0.2e1 + t686 * (Ifges(15,5) * t658 + Ifges(15,6) * t655 + Ifges(15,3) * t686) / 0.2e1 + t704 * (t655 * mrSges(16,2) + t686 * mrSges(16,3));
t711 = cos(qJ(4));
t713 = sin(qJ(4));
t715 = t711 * t430 + t713 * t436;
t716 = t636 * t715;
t717 = t713 * t430;
t718 = t711 * t436;
t719 = t139 - t140 + t253 - t267 + qJD(3) + qJD(4);
t721 = t719 * pkin(10) - t717 + t718;
t722 = t641 * t721;
t723 = -t716 + t722;
t726 = t713 * t447 + t711 * t461;
t727 = t641 * t726;
t730 = t711 * t447 - t713 * t461;
t731 = t636 * t730;
t732 = -t727 - t731;
t735 = t139 - t140 + t253 - t267 + 0.2e1 * qJD(4);
t757 = t651 * t458;
t758 = t653 * t699;
t765 = t757 - t758 + 0.1e1 / t702 * (t643 * t668 + t648 * t682);
t770 = t715 ^ 2;
t771 = -t717 + t718;
t772 = t771 ^ 2;
t792 = -t375 - t376;
t801 = t372 ^ 2;
t802 = t792 ^ 2;
t814 = t723 * (-t735 * mrSges(7,2) + t732 * mrSges(7,3)) + t726 * (Ifges(6,1) * t726 + Ifges(6,4) * t730 + Ifges(6,5) * t719) / 0.2e1 + t730 * (Ifges(6,4) * t726 + Ifges(6,2) * t730 + Ifges(6,6) * t719) / 0.2e1 + t719 * (Ifges(6,5) * t726 + Ifges(6,6) * t730 + Ifges(6,3) * t719) / 0.2e1 + t765 * (-t686 * mrSges(16,1) + t658 * mrSges(16,2)) + m(6) * (t770 + t772 + t450) / 0.2e1 + t449 * (-t730 * mrSges(6,1) + t726 * mrSges(6,2)) + t384 * (Ifges(9,1) * t384 + Ifges(9,4) * t388 + Ifges(9,5) * t377) / 0.2e1 + t388 * (Ifges(9,4) * t384 + Ifges(9,2) * t388 + Ifges(9,6) * t377) / 0.2e1 + t792 * (t377 * mrSges(9,1) - t384 * mrSges(9,3)) - t623 * (-t388 * mrSges(9,1) + t384 * mrSges(9,2)) + m(9) * (t801 + t802 + t624) / 0.2e1 + t372 * (-t377 * mrSges(9,2) + t388 * mrSges(9,3)) + t621 * (-t121 * mrSges(3,2) - t377 * mrSges(3,3));
t819 = t103 - t107;
t820 = t819 ^ 2;
t821 = V_base(3) ^ 2;
t826 = t119 + t120;
t886 = t623 * (t121 * mrSges(3,1) - t130 * mrSges(3,3)) + m(2) * (t820 + t625 + t821) / 0.2e1 + t115 * (Ifges(2,1) * t115 + Ifges(2,4) * t826 + Ifges(2,5) * t108) / 0.2e1 + t826 * (Ifges(2,4) * t115 + Ifges(2,2) * t826 + Ifges(2,6) * t108) / 0.2e1 + t108 * (Ifges(2,5) * t115 + Ifges(2,6) * t826 + Ifges(2,3) * t108) / 0.2e1 + V_base(3) * (-t826 * mrSges(2,1) + t115 * mrSges(2,2)) + t819 * (-t108 * mrSges(2,2) + t826 * mrSges(2,3)) + t616 * (t108 * mrSges(2,1) - t115 * mrSges(2,3)) + V_base(4) * (Ifges(1,1) * V_base(4) + Ifges(1,4) * V_base(5) + Ifges(1,5) * V_base(6)) / 0.2e1 + V_base(5) * (Ifges(1,4) * V_base(4) + Ifges(1,2) * V_base(5) + Ifges(1,6) * V_base(6)) / 0.2e1 + V_base(6) * (Ifges(1,5) * V_base(4) + Ifges(1,6) * V_base(5) + Ifges(1,3) * V_base(6)) / 0.2e1 + V_base(2) * (mrSges(1,1) * V_base(6) - mrSges(1,3) * V_base(4)) + V_base(3) * (-mrSges(1,1) * V_base(5) + mrSges(1,2) * V_base(4)) + V_base(1) * (-mrSges(1,2) * V_base(6) + mrSges(1,3) * V_base(5));
t889 = V_base(1) ^ 2;
t890 = V_base(2) ^ 2;
t894 = t470 * pkin(12);
t896 = -t658 * t702 + t329 - t330 - t444 - t448 - t894;
t901 = t730 * pkin(10);
t902 = t329 - t330 - t444 - t448 - t901;
t905 = -t636 * t726 + t641 * t730;
t910 = t329 - t330 - t444 - t448 - t894;
t915 = t697 + t700;
t916 = t915 ^ 2;
t917 = -t757 + t758;
t918 = t917 ^ 2;
t919 = t910 ^ 2;
t923 = t727 + t731 + qJD(5);
t924 = cos(qJ(5));
t926 = sin(qJ(5));
t928 = t926 * t735 + t924 * t905;
t932 = t924 * t735 - t926 * t905;
t939 = t735 * pkin(13) - t716 + t722;
t942 = -t905 * pkin(13) + t329 - t330 - t444 - t448 - t901;
t944 = t924 * t942 - t926 * t939;
t963 = t924 * t939 + t926 * t942;
t970 = t636 * t721 + t641 * t715;
t975 = t963 ^ 2;
t976 = t944 ^ 2;
t977 = t970 ^ 2;
t993 = m(1) * (t889 + t890 + t821) / 0.2e1 + t896 * (-t655 * mrSges(16,1) - t658 * mrSges(16,3)) + t902 * (-t732 * mrSges(7,1) + t905 * mrSges(7,2)) + t910 * (-t655 * mrSges(15,1) + t658 * mrSges(15,2)) + m(15) * (t916 + t918 + t919) / 0.2e1 + t923 * (Ifges(8,5) * t928 + Ifges(8,6) * t932 + Ifges(8,3) * t923) / 0.2e1 + t944 * (t923 * mrSges(8,1) - t928 * mrSges(8,3)) + t928 * (Ifges(8,1) * t928 + Ifges(8,4) * t932 + Ifges(8,5) * t923) / 0.2e1 + t932 * (Ifges(8,4) * t928 + Ifges(8,2) * t932 + Ifges(8,6) * t923) / 0.2e1 + t963 * (-t923 * mrSges(8,2) + t932 * mrSges(8,3)) + t970 * (-t932 * mrSges(8,1) + t928 * mrSges(8,2)) + m(8) * (t975 + t976 + t977) / 0.2e1 + t905 * (Ifges(7,1) * t905 + Ifges(7,4) * t732 + Ifges(7,5) * t735) / 0.2e1 + t732 * (Ifges(7,4) * t905 + Ifges(7,2) * t732 + Ifges(7,6) * t735) / 0.2e1;
t1022 = t723 ^ 2;
t1023 = t902 ^ 2;
t1045 = t704 ^ 2;
t1046 = t896 ^ 2;
t1047 = t765 ^ 2;
t1067 = t686 * (Ifges(16,4) * t658 + Ifges(16,2) * t686 - Ifges(16,6) * t655) / 0.2e1 - t655 * (Ifges(16,5) * t658 + Ifges(16,6) * t686 - Ifges(16,3) * t655) / 0.2e1 + t658 * (Ifges(15,1) * t658 + Ifges(15,4) * t655 + Ifges(15,5) * t686) / 0.2e1 + t735 * (Ifges(7,5) * t905 + Ifges(7,6) * t732 + Ifges(7,3) * t735) / 0.2e1 + t771 * (t719 * mrSges(6,1) - t726 * mrSges(6,3)) + m(7) * (t1022 + t977 + t1023) / 0.2e1 + t715 * (-t719 * mrSges(6,2) + t730 * mrSges(6,3)) - t970 * (t735 * mrSges(7,1) - t905 * mrSges(7,3)) + t658 * (Ifges(16,1) * t658 + Ifges(16,4) * t686 - Ifges(16,5) * t655) / 0.2e1 + t917 * (t686 * mrSges(15,1) - t658 * mrSges(15,3)) + m(16) * (t1045 + t1046 + t1047) / 0.2e1 + t915 * (-t686 * mrSges(15,2) + t655 * mrSges(15,3)) + t482 * (Ifges(14,1) * t482 + Ifges(14,4) * t470 + Ifges(14,5) * t473) / 0.2e1 + t470 * (Ifges(14,4) * t482 + Ifges(14,2) * t470 + Ifges(14,6) * t473) / 0.2e1;
t1093 = t381 ^ 2;
t1094 = t494 ^ 2;
t1096 = t388 * pkin(22) + t329 - t330;
t1097 = t1096 ^ 2;
t1113 = t134 ^ 2;
t1114 = t514 ^ 2;
t1136 = t473 * (Ifges(14,5) * t482 + Ifges(14,6) * t470 + Ifges(14,3) * t473) / 0.2e1 + t275 * (Ifges(4,1) * t275 + Ifges(4,4) * t279 + Ifges(4,5) * t268) / 0.2e1 + t341 * (-t281 * mrSges(11,1) + t521 * mrSges(11,2)) + t505 * (-t447 * mrSges(5,1) + t461 * mrSges(5,2)) + t333 * (-t528 * mrSges(12,1) + t524 * mrSges(12,2)) + m(10) * (t1093 + t1094 + t1097) / 0.2e1 + t449 * (-t470 * mrSges(14,1) + t482 * mrSges(14,2)) + t1096 * (-t390 * mrSges(10,1) + t497 * mrSges(10,2)) - t623 * (-t279 * mrSges(4,1) + t275 * mrSges(4,2)) + m(4) * (t1113 + t1114 + t624) / 0.2e1 + t449 * (-t468 * mrSges(13,1) + t464 * mrSges(13,2)) + t438 * (-t473 * mrSges(13,2) + t468 * mrSges(13,3)) + t442 * (t473 * mrSges(13,1) - t464 * mrSges(13,3)) + t279 * (Ifges(4,4) * t275 + Ifges(4,2) * t279 + Ifges(4,6) * t268) / 0.2e1;
t1215 = t268 * (Ifges(4,5) * t275 + Ifges(4,6) * t279 + Ifges(4,3) * t268) / 0.2e1 + t560 * (Ifges(12,5) * t524 + Ifges(12,6) * t528 + Ifges(12,3) * t560) / 0.2e1 + t339 * (t295 * mrSges(11,1) - t521 * mrSges(11,3)) + t461 * (Ifges(5,1) * t461 + Ifges(5,4) * t447 + Ifges(5,5) * t434) / 0.2e1 + t447 * (Ifges(5,4) * t461 + Ifges(5,2) * t447 + Ifges(5,6) * t434) / 0.2e1 + t434 * (Ifges(5,5) * t461 + Ifges(5,6) * t447 + Ifges(5,3) * t434) / 0.2e1 + t521 * (Ifges(11,1) * t521 + Ifges(11,4) * t281 + Ifges(11,5) * t295) / 0.2e1 + t281 * (Ifges(11,4) * t521 + Ifges(11,2) * t281 + Ifges(11,6) * t295) / 0.2e1 + t430 * (-t434 * mrSges(5,2) + t447 * mrSges(5,3)) + t503 * (t434 * mrSges(5,1) - t461 * mrSges(5,3)) + t295 * (Ifges(11,5) * t521 + Ifges(11,6) * t281 + Ifges(11,3) * t295) / 0.2e1 + t422 * (Ifges(10,5) * t497 + Ifges(10,6) * t390 + Ifges(10,3) * t422) / 0.2e1 + t497 * (Ifges(10,1) * t497 + Ifges(10,4) * t390 + Ifges(10,5) * t422) / 0.2e1 + t390 * (Ifges(10,4) * t497 + Ifges(10,2) * t390 + Ifges(10,6) * t422) / 0.2e1;
t1218 = t571 + t709 + t814 + t886 + t993 + t1067 + t1136 + t1215;
T = t1218;
