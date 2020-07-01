% Calculate kinetic energy for
% KAS5m7DE2
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
% Datum: 2020-06-03 15:49
% Revision: caa0dbda1e8a16d11faaa29ba3bbef6afcd619f7 (2020-05-25)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = KAS5m7DE2_energykin_floatb_twist_slag_vp2(qJ, qJD, V_base, ...
  pkin, m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),zeros(6,1),zeros(24,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE2_energykin_floatb_twist_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m7DE2_energykin_floatb_twist_slag_vp2: qJD has to be [5x1] (double)');
assert(isreal(V_base) && all(size(V_base) == [6 1]), ...
  'KAS5m7DE2_energykin_floatb_twist_slag_vp2: V_base has to be [6x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE2_energykin_floatb_twist_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE2_energykin_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE2_energykin_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7DE2_energykin_floatb_twist_slag_vp2: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_floatb_twist_linkframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-25 12:22:02
% EndTime: 2020-05-25 12:42:32
% DurationCPUTime: 1163.75s
% Computational Cost: add. (78701481->602), mult. (103917246->870), div. (1437062->24), fcn. (40948174->43), ass. (0->354)
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
t29 = 0.2e1 * pkin(12) * t14 * t26 + 0.2e1 * pkin(12) * t6 * t8 - t11 * t12 - t15 * t12 + t19 * t20 - t10 + t18 + t9;
t33 = 0.4e1 * t26 ^ 2 + 0.4e1 * t8 ^ 2;
t35 = t29 ^ 2;
t37 = sqrt(t9 * t33 - t35);
t39 = -0.2e1 * t26 * t37 + 0.2e1 * t8 * t29;
t40 = 0.1e1 / t33;
t42 = t39 * t40 + pkin(9) - t4 - t7;
t46 = 0.2e1 * t29 * t26 + 0.2e1 * t8 * t37;
t48 = t46 * t40 + t24 - t25;
t50 = -t1 * t48 + t2 * t42;
t52 = 0.1e1 / pkin(19);
t56 = t1 * t42 + t2 * t48;
t61 = (t1 * t50 * t52 - t2 * t52 * t56) * pkin(19) + t24 - t25;
t68 = (t1 * t52 * t56 + t2 * t50 * t52) * pkin(19) - pkin(9) + t4 + t7;
t69 = atan2(t61, t68);
t70 = atan2(-t56, t50);
t71 = -t69 + qJ(3) + t70;
t72 = cos(t71);
t73 = cos(t70);
t74 = t56 * t52;
t75 = cos(pkin(7));
t76 = t75 * pkin(18);
t77 = t74 * t76;
t78 = t50 * t52;
t79 = sin(pkin(7));
t80 = t79 * pkin(18);
t81 = t78 * t80;
t82 = pkin(17) ^ 2;
t83 = pkin(22) ^ 2;
t84 = t78 * t76;
t85 = t74 * t80;
t86 = t84 - t85;
t87 = t86 ^ 2;
t88 = t77 + t81;
t89 = t88 ^ 2;
t90 = pkin(24) ^ 2;
t91 = -pkin(24) - t84 + t85;
t94 = -0.2e1 * t91 * t86 + 0.2e1 * t88 ^ 2 + t82 - t83 - t87 - t89 + t90;
t98 = 0.4e1 * t88 ^ 2 + 0.4e1 * t91 ^ 2;
t100 = t94 ^ 2;
t102 = sqrt(t82 * t98 - t100);
t104 = 0.2e1 * t91 * t102 - 0.2e1 * t94 * t88;
t105 = 0.1e1 / t98;
t107 = -t104 * t105 - t77 - t81;
t110 = 0.2e1 * t88 * t102 + 0.2e1 * t91 * t94;
t112 = t110 * t105 + pkin(24) + t84 - t85;
t113 = atan2(t107, t112);
t114 = t113 + pkin(6);
t115 = sin(t114);
t116 = sin(qJ(2));
t117 = sin(qJ(1));
t119 = pkin(8) * V_base(5) + V_base(1);
t120 = t117 * t119;
t121 = cos(qJ(1));
t123 = -pkin(8) * V_base(4) + V_base(2);
t124 = t121 * t123;
t125 = V_base(6) + qJD(1);
t127 = -pkin(16) * t125 + t120 - t124;
t128 = t116 * t127;
t129 = cos(qJ(2));
t132 = t117 * V_base(4) - t121 * V_base(5);
t134 = pkin(16) * t132 + V_base(3);
t135 = t129 * t134;
t136 = t121 * V_base(4);
t137 = t117 * V_base(5);
t138 = t136 + t137 + qJD(2);
t140 = pkin(21) * t138 - t128 - t135;
t142 = cos(t114);
t143 = t121 * t119;
t144 = t117 * t123;
t147 = -t116 * t132 - t129 * t125;
t149 = -pkin(21) * t147 + t143 + t144;
t151 = t115 * t140 - t142 * t149;
t153 = sin(t70);
t154 = t142 * t140;
t155 = t115 * t149;
t156 = t129 * t132;
t157 = t116 * t125;
t158 = qJD(3) * t2;
t160 = qJD(3) * t1;
t161 = t160 * t3;
t163 = qJD(3) * t14 * pkin(12);
t164 = -t161 - t163;
t176 = t158 * t3;
t178 = qJD(3) * t6 * pkin(12);
t179 = -t176 + t178;
t184 = 0.2e1 * pkin(12) * t14 * t179 + 0.2e1 * pkin(12) * t164 * t6 - 0.2e1 * qJD(3) * t17 * t24 - 0.2e1 * qJD(3) * t25 * t8 + 0.2e1 * qJD(3) * t26 * t7 + 0.2e1 * t1 * t158 * t20;
t187 = 0.1e1 / t37;
t191 = 0.4e1 * t8 * t164 + 0.4e1 * t26 * t179;
t195 = -0.2e1 * t29 * t184 + 0.2e1 * t9 * t191;
t200 = t33 ^ 2;
t201 = 0.1e1 / t200;
t204 = t161 + t163 + (-t187 * t195 * t26 + 0.2e1 * t164 * t29 - 0.2e1 * t179 * t37 + 0.2e1 * t8 * t184) * t40 - 0.2e1 * t39 * t201 * t191;
t217 = t176 - t178 + (t187 * t195 * t8 + 0.2e1 * t164 * t37 + 0.2e1 * t29 * t179 + 0.2e1 * t184 * t26) * t40 - 0.2e1 * t46 * t201 * t191;
t219 = t1 * t204 + t158 * t42 - t160 * t48 + t2 * t217;
t220 = t219 * t52;
t221 = t220 * t76;
t226 = -t1 * t217 - t158 * t48 - t160 * t42 + t2 * t204;
t227 = t226 * t52;
t228 = t227 * t80;
t229 = t227 * t76;
t230 = t220 * t80;
t231 = t229 - t230;
t234 = t221 + t228;
t240 = -0.2e1 * t91 * t231 + 0.2e1 * t88 * t234;
t244 = 0.1e1 / t102;
t248 = -0.4e1 * t91 * t231 + 0.4e1 * t88 * t234;
t252 = -0.2e1 * t94 * t240 + 0.2e1 * t82 * t248;
t257 = t98 ^ 2;
t258 = 0.1e1 / t257;
t261 = -t221 - t228 - (t244 * t252 * t91 - 0.2e1 * t231 * t102 - 0.2e1 * t94 * t234 - 0.2e1 * t240 * t88) * t105 + 0.2e1 * t104 * t258 * t248;
t264 = t107 ^ 2;
t265 = t112 ^ 2;
t266 = 0.1e1 / t265;
t269 = 0.1e1 / (t264 * t266 + 0.1e1);
t270 = t261 / t112 * t269;
t281 = t229 - t230 + (t244 * t252 * t88 + 0.2e1 * t234 * t102 - 0.2e1 * t231 * t94 + 0.2e1 * t91 * t240) * t105 - 0.2e1 * t110 * t258 * t248;
t284 = t281 * t107 * t266 * t269;
t285 = t156 - t157 + t270 - t284;
t287 = pkin(24) * t285 + t154 + t155;
t289 = t73 * t151 - t153 * t287;
t291 = sin(t71);
t292 = t153 * t151;
t293 = t73 * t287;
t296 = t56 ^ 2;
t297 = t50 ^ 2;
t298 = 0.1e1 / t297;
t301 = 0.1e1 / (t296 * t298 + 0.1e1);
t306 = t156 - t157 + t270 - t284 + t219 / t50 * t301 - t226 * t56 * t298 * t301;
t308 = pkin(19) * t306 + t292 + t293;
t310 = -t72 * t289 - t291 * t308;
t311 = t310 ^ 2;
t314 = t291 * t289 - t72 * t308;
t315 = t314 ^ 2;
t316 = t129 * t127;
t317 = t116 * t134;
t320 = t115 * t138 + t142 * t147;
t321 = t320 * pkin(24);
t324 = t115 * t147 - t142 * t138;
t327 = t153 * t324 + t73 * t320;
t329 = -pkin(19) * t327 + t316 - t317 - t321;
t330 = t329 ^ 2;
t338 = t289 ^ 2;
t339 = t292 + t293;
t340 = t339 ^ 2;
t341 = t316 - t317 - t321;
t342 = t341 ^ 2;
t346 = 0.1e1 / pkin(22);
t347 = t107 * t346;
t348 = sin(pkin(6));
t350 = -t112 * t346;
t351 = cos(pkin(6));
t353 = t347 * t348 + t350 * t351;
t357 = t347 * t351 - t350 * t348;
t361 = -(t357 * t348 - t353 * t351) * pkin(22) + pkin(24) + t84 - t85;
t366 = (t353 * t348 + t357 * t351) * pkin(22) + t77 + t81;
t367 = atan2(t361, t366);
t368 = t113 - t367;
t369 = sin(t368);
t372 = -t348 * t140 + t351 * t149;
t374 = cos(t368);
t375 = t351 * t140;
t376 = t348 * t149;
t377 = t156 - t157;
t379 = -pkin(22) * t377 - t375 - t376;
t381 = t369 * t372 - t374 * t379;
t384 = t351 * t138 - t348 * t147;
t388 = -t348 * t138 - t351 * t147;
t390 = t369 * t388 + t374 * t384;
t392 = t261 * t346;
t394 = -t281 * t346;
t396 = t392 * t348 + t394 * t351;
t400 = -t394 * t348 + t392 * t351;
t407 = t361 ^ 2;
t408 = t366 ^ 2;
t409 = 0.1e1 / t408;
t412 = 0.1e1 / (t407 * t409 + 0.1e1);
t422 = t156 - t157 + t270 - t284 - (-(t400 * t348 - t396 * t351) * pkin(22) + t229 - t230) / t366 * t412 + ((t396 * t348 + t400 * t351) * pkin(22) + t221 + t228) * t361 * t409 * t412;
t428 = t369 * t379 + t374 * t372;
t431 = t369 * t384 - t374 * t388;
t438 = pkin(23) * t285 + t154 + t155;
t440 = t1 * t438 + t2 * t151;
t441 = t440 ^ 2;
t442 = t1 * t151;
t443 = t2 * t438;
t444 = -t442 + t443;
t445 = t444 ^ 2;
t446 = t320 * pkin(23);
t447 = t316 - t317 - t446;
t448 = t447 ^ 2;
t456 = t154 + t155;
t463 = -t153 * t320 + t73 * t324;
t466 = t291 * t463 - t72 * t327;
t479 = t61 ^ 2;
t480 = t68 ^ 2;
t481 = 0.1e1 / t480;
t484 = 0.1e1 / (t479 * t481 + 0.1e1);
t498 = t156 - t157 + t270 - t284 - ((t1 * t226 * t52 - t2 * t219 * t52 + t158 * t78 + t160 * t74) * pkin(19) + t176 - t178) / t68 * t484 + ((t1 * t219 * t52 + t2 * t226 * t52 + t158 * t74 - t160 * t78) * pkin(19) - t161 - t163) * t61 * t481 * t484 + qJD(3);
t504 = -t291 * t327 - t72 * t463;
t509 = sin(pkin(3));
t512 = t1 * t320 + t2 * t324;
t516 = -t1 * t324 + t2 * t320;
t518 = t1 * t512 - t2 * t516;
t520 = cos(pkin(3));
t523 = t1 * t516 + t2 * t512;
t525 = t509 * t518 - t520 * t523;
t529 = t509 * t523 + t520 * t518;
t532 = t156 - t157 + t270 - t284 + 0.2e1 * qJD(3);
t543 = t143 + t144;
t548 = -t128 - t135;
t549 = t548 ^ 2;
t550 = -t316 + t317;
t551 = t550 ^ 2;
t552 = t543 ^ 2;
t556 = m(12) * (t311 + t315 + t330) / 0.2e1 + t289 * (-mrSges(11,2) * t306 + mrSges(11,3) * t327) + m(11) * (t338 + t340 + t342) / 0.2e1 + t381 * (-mrSges(10,2) * t422 + mrSges(10,3) * t390) + t428 * (mrSges(10,1) * t422 - mrSges(10,3) * t431) + m(5) * (t441 + t445 + t448) / 0.2e1 + t151 * (-mrSges(4,2) * t285 + mrSges(4,3) * t320) + t456 * (mrSges(4,1) * t285 - mrSges(4,3) * t324) + t310 * (-mrSges(12,2) * t498 + mrSges(12,3) * t466) + t314 * (mrSges(12,1) * t498 - mrSges(12,3) * t504) + t525 * (Ifges(14,1) * t525 + Ifges(14,4) * t529 + Ifges(14,5) * t532) / 0.2e1 + t377 * (Ifges(9,5) * t384 + Ifges(9,6) * t388 + Ifges(9,3) * t377) / 0.2e1 + t543 * (mrSges(3,1) * t377 + mrSges(3,2) * t147) + m(3) * (t549 + t551 + t552) / 0.2e1;
t591 = -t375 - t376;
t600 = t372 ^ 2;
t601 = t591 ^ 2;
t614 = t136 + t137;
t621 = t156 - t157 + t270 - t284 + (2 * qJD(4));
t622 = qJ(4) - qJ(3) + pkin(4);
t623 = sin(t622);
t624 = cos(qJ(4));
t626 = sin(qJ(4));
t628 = t624 * t512 + t626 * t516;
t630 = cos(t622);
t633 = -t626 * t512 + t624 * t516;
t635 = -t623 * t628 + t630 * t633;
t637 = t630 * t628;
t638 = t623 * t633;
t639 = -t637 - t638;
t645 = t516 * pkin(9);
t646 = t529 * pkin(12);
t647 = t316 - t317 - t446 - t645 - t646;
t649 = pkin(3) + qJ(3) - qJ(4);
t650 = sin(t649);
t653 = pkin(12) * t650 - pkin(14) * t630 + pkin(15) * t623;
t656 = cos(t649);
t658 = -pkin(12) * t656 - pkin(14) * t623 - pkin(15) * t630 - pkin(10);
t659 = atan2(t653, t658);
t660 = t659 + pkin(3) + qJ(3) - qJ(4);
t661 = sin(t660);
t663 = cos(t660);
t665 = t661 * t525 + t663 * t529;
t669 = -t663 * t525 + t661 * t529;
t673 = t147 * (Ifges(3,1) * t147 - Ifges(3,4) * t377 + Ifges(3,5) * t138) / 0.2e1 - t377 * (Ifges(3,4) * t147 - Ifges(3,2) * t377 + Ifges(3,6) * t138) / 0.2e1 + t138 * (Ifges(3,5) * t147 - Ifges(3,6) * t377 + Ifges(3,3) * t138) / 0.2e1 + t384 * (Ifges(9,1) * t384 + Ifges(9,4) * t388 + Ifges(9,5) * t377) / 0.2e1 + t388 * (Ifges(9,4) * t384 + Ifges(9,2) * t388 + Ifges(9,6) * t377) / 0.2e1 + t372 * (-mrSges(9,2) * t377 + mrSges(9,3) * t388) + t591 * (mrSges(9,1) * t377 - mrSges(9,3) * t384) - t550 * (-mrSges(9,1) * t388 + mrSges(9,2) * t384) + m(9) * (t600 + t601 + t551) / 0.2e1 + t548 * (-mrSges(3,2) * t138 - mrSges(3,3) * t377) + t550 * (mrSges(3,1) * t138 - mrSges(3,3) * t147) + t132 * (Ifges(2,1) * t132 + Ifges(2,4) * t614 + Ifges(2,5) * t125) / 0.2e1 + t621 * (Ifges(7,5) * t635 + Ifges(7,6) * t639 + Ifges(7,3) * t621) / 0.2e1 + t647 * (-mrSges(15,1) * t669 + mrSges(15,2) * t665);
t675 = t633 * pkin(10);
t676 = t316 - t317 - t446 - t645 - t675;
t681 = t658 ^ 2;
t682 = t653 ^ 2;
t684 = sqrt(t681 + t682);
t686 = -t665 * t684 + t316 - t317 - t446 - t645 - t646;
t692 = t156 - t157 + t270 - t284 + qJD(3);
t694 = pkin(9) * t692 - t442 + t443;
t696 = t624 * t440 + t626 * t694;
t697 = t696 ^ 2;
t698 = t626 * t440;
t699 = t624 * t694;
t700 = -t698 + t699;
t701 = t700 ^ 2;
t702 = t316 - t317 - t446 - t645;
t703 = t702 ^ 2;
t709 = t1 * t440 - t2 * t694;
t713 = t1 * t694 + t2 * t440;
t715 = t509 * t709 - t520 * t713;
t716 = t661 * t715;
t717 = t520 * t709;
t718 = t509 * t713;
t720 = pkin(12) * t532 + t717 + t718;
t721 = t663 * t720;
t722 = t716 + t721;
t723 = t722 ^ 2;
t724 = t663 * t715;
t725 = t661 * t720;
t726 = -t724 + t725;
t727 = t726 ^ 2;
t728 = t647 ^ 2;
t732 = t156 - t157 + t270 - t284 + qJD(3) + qJD(4);
t741 = qJD(4) - qJD(3);
t742 = t741 * t630;
t746 = t741 * t623;
t748 = -pkin(12) * t656 * t741 + pkin(14) * t746 + pkin(15) * t742;
t751 = 0.1e1 / t681;
t754 = 0.1e1 / (t682 * t751 + 0.1e1);
t760 = -pkin(12) * t650 * t741 - pkin(14) * t742 + pkin(15) * t746;
t764 = t156 - t157 + t270 - t284 + qJD(3) - t748 / t658 * t754 + t760 * t653 * t751 * t754 + qJD(4);
t797 = t623 * t696;
t799 = pkin(10) * t732 - t698 + t699;
t800 = t630 * t799;
t801 = -t797 + t800;
t808 = -t623 * t799 - t630 * t696;
t817 = t676 * (-mrSges(7,1) * t639 + mrSges(7,2) * t635) + t686 * (-mrSges(16,1) * t669 - mrSges(16,3) * t665) + m(6) * (t697 + t701 + t703) / 0.2e1 + m(15) * (t723 + t727 + t728) / 0.2e1 + t732 * (Ifges(6,5) * t628 + Ifges(6,6) * t633 + Ifges(6,3) * t732) / 0.2e1 + t669 * (Ifges(15,4) * t665 + Ifges(15,2) * t669 + Ifges(15,6) * t764) / 0.2e1 + t764 * (Ifges(15,5) * t665 + Ifges(15,6) * t669 + Ifges(15,3) * t764) / 0.2e1 + t702 * (-mrSges(6,1) * t633 + mrSges(6,2) * t628) + t665 * (Ifges(16,1) * t665 + Ifges(16,4) * t764 - Ifges(16,5) * t669) / 0.2e1 + t764 * (Ifges(16,4) * t665 + Ifges(16,2) * t764 - Ifges(16,6) * t669) / 0.2e1 - t669 * (Ifges(16,5) * t665 + Ifges(16,6) * t764 - Ifges(16,3) * t669) / 0.2e1 + t801 * (-mrSges(7,2) * t621 + mrSges(7,3) * t639) + t808 * (mrSges(7,1) * t621 - mrSges(7,3) * t635) + t696 * (-mrSges(6,2) * t732 + mrSges(6,3) * t633);
t819 = t764 * t684 + t716 + t721;
t848 = t120 - t124;
t849 = t848 ^ 2;
t850 = V_base(3) ^ 2;
t888 = t819 * (mrSges(16,2) * t669 + mrSges(16,3) * t764) + t341 * (-mrSges(11,1) * t327 + mrSges(11,2) * t463) + t447 * (-mrSges(5,1) * t516 + mrSges(5,2) * t512) + t614 * (Ifges(2,4) * t132 + Ifges(2,2) * t614 + Ifges(2,6) * t125) / 0.2e1 + t125 * (Ifges(2,5) * t132 + Ifges(2,6) * t614 + Ifges(2,3) * t125) / 0.2e1 + V_base(3) * (-mrSges(2,1) * t614 + mrSges(2,2) * t132) + m(2) * (t849 + t552 + t850) / 0.2e1 + t848 * (-mrSges(2,2) * t125 + mrSges(2,3) * t614) + t543 * (mrSges(2,1) * t125 - mrSges(2,3) * t132) + V_base(4) * (Ifges(1,1) * V_base(4) + Ifges(1,4) * V_base(5) + Ifges(1,5) * V_base(6)) / 0.2e1 + V_base(5) * (Ifges(1,4) * V_base(4) + Ifges(1,2) * V_base(5) + Ifges(1,6) * V_base(6)) / 0.2e1 + V_base(6) * (Ifges(1,5) * V_base(4) + Ifges(1,6) * V_base(5) + Ifges(1,3) * V_base(6)) / 0.2e1 + V_base(2) * (mrSges(1,1) * V_base(6) - mrSges(1,3) * V_base(4)) + V_base(1) * (-mrSges(1,2) * V_base(6) + mrSges(1,3) * V_base(5));
t895 = V_base(1) ^ 2;
t896 = V_base(2) ^ 2;
t918 = t151 ^ 2;
t919 = t456 ^ 2;
t957 = V_base(3) * (-mrSges(1,1) * V_base(5) + mrSges(1,2) * V_base(4)) + m(1) * (t895 + t896 + t850) / 0.2e1 + t306 * (Ifges(11,5) * t463 + Ifges(11,6) * t327 + Ifges(11,3) * t306) / 0.2e1 + t702 * (-mrSges(13,1) * t523 + mrSges(13,2) * t518) + t702 * (-mrSges(14,1) * t529 + mrSges(14,2) * t525) - t550 * (-mrSges(4,1) * t320 + mrSges(4,2) * t324) + m(4) * (t918 + t919 + t551) / 0.2e1 + t329 * (-mrSges(12,1) * t466 + mrSges(12,2) * t504) + t339 * (mrSges(11,1) * t306 - mrSges(11,3) * t463) + t440 * (-mrSges(5,2) * t692 + mrSges(5,3) * t516) + t512 * (Ifges(5,1) * t512 + Ifges(5,4) * t516 + Ifges(5,5) * t692) / 0.2e1 + t532 * (Ifges(14,5) * t525 + Ifges(14,6) * t529 + Ifges(14,3) * t532) / 0.2e1 + t518 * (Ifges(13,1) * t518 + Ifges(13,4) * t523 + Ifges(13,5) * t532) / 0.2e1 + t444 * (mrSges(5,1) * t692 - mrSges(5,3) * t512);
t962 = t717 + t718;
t995 = cos(qJ(5));
t997 = sin(qJ(5));
t999 = t997 * t621 + t995 * t635;
t1003 = t995 * t621 - t997 * t635;
t1008 = pkin(13) * t621 - t797 + t800;
t1011 = -pkin(13) * t635 + t316 - t317 - t446 - t645 - t675;
t1013 = t995 * t1008 + t997 * t1011;
t1014 = t1013 ^ 2;
t1017 = -t997 * t1008 + t995 * t1011;
t1018 = t1017 ^ 2;
t1019 = t808 ^ 2;
t1025 = t637 + t638 + qJD(5);
t1050 = t715 * (-mrSges(14,2) * t532 + mrSges(14,3) * t529) + t962 * (mrSges(14,1) * t532 - mrSges(14,3) * t525) + t709 * (-mrSges(13,2) * t532 + mrSges(13,3) * t523) + t431 * (Ifges(10,1) * t431 + Ifges(10,4) * t390 + Ifges(10,5) * t422) / 0.2e1 + t390 * (Ifges(10,4) * t431 + Ifges(10,2) * t390 + Ifges(10,6) * t422) / 0.2e1 + t422 * (Ifges(10,5) * t431 + Ifges(10,6) * t390 + Ifges(10,3) * t422) / 0.2e1 + t529 * (Ifges(14,4) * t525 + Ifges(14,2) * t529 + Ifges(14,6) * t532) / 0.2e1 - t808 * (-mrSges(8,1) * t1003 + mrSges(8,2) * t999) + m(8) * (t1014 + t1018 + t1019) / 0.2e1 + t999 * (Ifges(8,1) * t999 + Ifges(8,4) * t1003 + Ifges(8,5) * t1025) / 0.2e1 + t1003 * (Ifges(8,4) * t999 + Ifges(8,2) * t1003 + Ifges(8,6) * t1025) / 0.2e1 + t1025 * (Ifges(8,5) * t999 + Ifges(8,6) * t1003 + Ifges(8,3) * t1025) / 0.2e1 + t1013 * (-mrSges(8,2) * t1025 + mrSges(8,3) * t1003) + t1017 * (mrSges(8,1) * t1025 - mrSges(8,3) * t999);
t1058 = t724 - t725 + 0.1e1 / t684 * (t653 * t748 + t658 * t760);
t1063 = t819 ^ 2;
t1064 = t686 ^ 2;
t1065 = t1058 ^ 2;
t1073 = t801 ^ 2;
t1074 = t676 ^ 2;
t1116 = t381 ^ 2;
t1117 = t428 ^ 2;
t1119 = pkin(22) * t388 + t316 - t317;
t1120 = t1119 ^ 2;
t1132 = t1058 * (-mrSges(16,1) * t764 + mrSges(16,2) * t665) + m(16) * (t1063 + t1064 + t1065) / 0.2e1 + t722 * (-mrSges(15,2) * t764 + mrSges(15,3) * t669) + m(7) * (t1073 + t1019 + t1074) / 0.2e1 + t628 * (Ifges(6,1) * t628 + Ifges(6,4) * t633 + Ifges(6,5) * t732) / 0.2e1 + t633 * (Ifges(6,4) * t628 + Ifges(6,2) * t633 + Ifges(6,6) * t732) / 0.2e1 + t726 * (mrSges(15,1) * t764 - mrSges(15,3) * t665) + t665 * (Ifges(15,1) * t665 + Ifges(15,4) * t669 + Ifges(15,5) * t764) / 0.2e1 + t700 * (mrSges(6,1) * t732 - mrSges(6,3) * t628) + t635 * (Ifges(7,1) * t635 + Ifges(7,4) * t639 + Ifges(7,5) * t621) / 0.2e1 + t639 * (Ifges(7,4) * t635 + Ifges(7,2) * t639 + Ifges(7,6) * t621) / 0.2e1 + m(10) * (t1116 + t1117 + t1120) / 0.2e1 + t1119 * (-mrSges(10,1) * t390 + mrSges(10,2) * t431) + t713 * (mrSges(13,1) * t532 - mrSges(13,3) * t518);
t1133 = t715 ^ 2;
t1134 = t962 ^ 2;
t1137 = t709 ^ 2;
t1138 = t713 ^ 2;
t1201 = m(14) * (t1133 + t1134 + t703) + m(13) * (t1137 + t1138 + t703) + t516 * (Ifges(5,4) * t512 + Ifges(5,2) * t516 + Ifges(5,6) * t692) + t692 * (Ifges(5,5) * t512 + Ifges(5,6) * t516 + Ifges(5,3) * t692) + t324 * (Ifges(4,1) * t324 + Ifges(4,4) * t320 + Ifges(4,5) * t285) + t320 * (Ifges(4,4) * t324 + Ifges(4,2) * t320 + Ifges(4,6) * t285) + t285 * (Ifges(4,5) * t324 + Ifges(4,6) * t320 + Ifges(4,3) * t285) + t523 * (Ifges(13,4) * t518 + Ifges(13,2) * t523 + Ifges(13,6) * t532) + t532 * (Ifges(13,5) * t518 + Ifges(13,6) * t523 + Ifges(13,3) * t532) + t504 * (Ifges(12,1) * t504 + Ifges(12,4) * t466 + Ifges(12,5) * t498) + t466 * (Ifges(12,4) * t504 + Ifges(12,2) * t466 + Ifges(12,6) * t498) + t498 * (Ifges(12,5) * t504 + Ifges(12,6) * t466 + Ifges(12,3) * t498) + t463 * (Ifges(11,1) * t463 + Ifges(11,4) * t327 + Ifges(11,5) * t306) + t327 * (Ifges(11,4) * t463 + Ifges(11,2) * t327 + Ifges(11,6) * t306);
t1204 = t556 + t673 + t817 + t888 + t957 + t1050 + t1132 + t1201 / 0.2e1;
T = t1204;
