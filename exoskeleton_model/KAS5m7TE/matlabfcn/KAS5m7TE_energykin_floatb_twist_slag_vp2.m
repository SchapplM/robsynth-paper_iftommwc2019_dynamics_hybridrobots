% Calculate kinetic energy for
% KAS5m7TE
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
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = KAS5m7TE_energykin_floatb_twist_slag_vp2(qJ, qJD, V_base, ...
  pkin, m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),zeros(6,1),zeros(24,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_energykin_floatb_twist_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m7TE_energykin_floatb_twist_slag_vp2: qJD has to be [5x1] (double)');
assert(isreal(V_base) && all(size(V_base) == [6 1]), ...
  'KAS5m7TE_energykin_floatb_twist_slag_vp2: V_base has to be [6x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_energykin_floatb_twist_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7TE_energykin_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7TE_energykin_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7TE_energykin_floatb_twist_slag_vp2: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_floatb_twist_linkframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-04-13 19:06:46
% EndTime: 2020-04-13 19:33:53
% DurationCPUTime: 1163.80s
% Computational Cost: add. (79757697->602), mult. (105284862->906), div. (1465322->26), fcn. (41494302->28), ass. (0->350)
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
t50 = t1 * t42 + t2 * t48;
t51 = 0.1e1 / pkin(19);
t52 = t50 * t51;
t53 = cos(pkin(7));
t54 = t53 * pkin(18);
t55 = t52 * t54;
t58 = -t1 * t48 + t2 * t42;
t59 = t58 * t51;
t60 = sin(pkin(7));
t61 = t60 * pkin(18);
t62 = t59 * t61;
t63 = pkin(17) ^ 2;
t64 = pkin(22) ^ 2;
t65 = t59 * t54;
t66 = t52 * t61;
t67 = t65 - t66;
t68 = t67 ^ 2;
t69 = t55 + t62;
t70 = t69 ^ 2;
t71 = pkin(24) ^ 2;
t72 = -pkin(24) - t65 + t66;
t75 = -0.2e1 * t72 * t67 + 0.2e1 * t69 ^ 2 + t63 - t64 - t68 - t70 + t71;
t79 = 0.4e1 * t69 ^ 2 + 0.4e1 * t72 ^ 2;
t81 = t75 ^ 2;
t83 = sqrt(t63 * t79 - t81);
t85 = -0.2e1 * t75 * t69 + 0.2e1 * t72 * t83;
t86 = 0.1e1 / t79;
t88 = -t85 * t86 - t55 - t62;
t89 = 0.1e1 / pkin(22);
t90 = t88 * t89;
t91 = sin(pkin(6));
t95 = 0.2e1 * t69 * t83 + 0.2e1 * t72 * t75;
t97 = -t95 * t86 - pkin(24) - t65 + t66;
t98 = t97 * t89;
t99 = cos(pkin(6));
t101 = -t90 * t91 - t98 * t99;
t102 = sin(qJ(2));
t103 = sin(qJ(1));
t105 = cos(qJ(1));
t107 = t103 * V_base(4) - t105 * V_base(5);
t109 = cos(qJ(2));
t110 = V_base(6) + qJD(1);
t112 = -t102 * t107 - t109 * t110;
t116 = t90 * t99 - t98 * t91;
t117 = t105 * V_base(4);
t118 = t103 * V_base(5);
t119 = t117 + t118 + qJD(2);
t121 = t101 * t112 + t116 * t119;
t124 = -t101 * t119 + t116 * t112;
t127 = t109 * t107;
t128 = t102 * t110;
t129 = qJD(3) * t2;
t131 = qJD(3) * t1;
t132 = t131 * t3;
t134 = qJD(3) * t14 * pkin(12);
t135 = -t132 - t134;
t147 = t129 * t3;
t149 = qJD(3) * t6 * pkin(12);
t150 = -t147 + t149;
t155 = 0.2e1 * pkin(12) * t135 * t6 + 0.2e1 * pkin(12) * t14 * t150 - 0.2e1 * qJD(3) * t17 * t24 - 0.2e1 * qJD(3) * t25 * t8 + 0.2e1 * qJD(3) * t26 * t7 + 0.2e1 * t1 * t129 * t20;
t158 = 0.1e1 / t37;
t162 = 0.4e1 * t8 * t135 + 0.4e1 * t26 * t150;
t166 = -0.2e1 * t29 * t155 + 0.2e1 * t9 * t162;
t171 = t33 ^ 2;
t172 = 0.1e1 / t171;
t175 = t132 + t134 + (-t158 * t166 * t26 + 0.2e1 * t135 * t29 - 0.2e1 * t150 * t37 + 0.2e1 * t8 * t155) * t40 - 0.2e1 * t39 * t172 * t162;
t188 = t147 - t149 + (t158 * t166 * t8 + 0.2e1 * t135 * t37 + 0.2e1 * t29 * t150 + 0.2e1 * t155 * t26) * t40 - 0.2e1 * t46 * t172 * t162;
t190 = t1 * t175 + t129 * t42 - t131 * t48 + t2 * t188;
t191 = t190 * t51;
t192 = t191 * t54;
t197 = -t1 * t188 - t129 * t48 - t131 * t42 + t2 * t175;
t198 = t197 * t51;
t199 = t198 * t61;
t200 = t198 * t54;
t201 = t191 * t61;
t202 = t200 - t201;
t205 = t192 + t199;
t211 = -0.2e1 * t72 * t202 + 0.2e1 * t69 * t205;
t215 = 0.1e1 / t83;
t219 = -0.4e1 * t72 * t202 + 0.4e1 * t69 * t205;
t223 = -0.2e1 * t75 * t211 + 0.2e1 * t63 * t219;
t228 = t79 ^ 2;
t229 = 0.1e1 / t228;
t232 = -t192 - t199 - (t215 * t223 * t72 - 0.2e1 * t202 * t83 - 0.2e1 * t75 * t205 - 0.2e1 * t211 * t69) * t86 + 0.2e1 * t85 * t229 * t219;
t235 = t88 ^ 2;
t236 = t97 ^ 2;
t237 = 0.1e1 / t236;
t240 = 0.1e1 / (t235 * t237 + 0.1e1);
t241 = -t232 / t97 * t240;
t252 = t200 - t201 + (t215 * t223 * t69 - 0.2e1 * t202 * t75 + 0.2e1 * t205 * t83 + 0.2e1 * t72 * t211) * t86 - 0.2e1 * t95 * t229 * t219;
t255 = t252 * t88 * t237 * t240;
t256 = t127 - t128 + t241 - t255;
t268 = pkin(8) * V_base(5) + V_base(1);
t269 = t103 * t268;
t271 = -pkin(8) * V_base(4) + V_base(2);
t272 = t105 * t271;
t274 = -pkin(16) * t110 + t269 - t272;
t275 = t102 * t274;
t277 = pkin(16) * t107 + V_base(3);
t278 = t109 * t277;
t280 = pkin(21) * t119 - t275 - t278;
t281 = t101 * t280;
t282 = t105 * t268;
t283 = t103 * t271;
t285 = -pkin(21) * t112 + t282 + t283;
t286 = t116 * t285;
t287 = t281 + t286;
t294 = -t101 * t285 + t116 * t280;
t297 = pkin(23) * t256 + t281 + t286;
t299 = t1 * t297 + t2 * t294;
t301 = t1 * t294;
t302 = t2 * t297;
t303 = t127 - t128 + t241 - t255 + qJD(3);
t305 = pkin(9) * t303 - t301 + t302;
t307 = t1 * t299 - t2 * t305;
t308 = t307 ^ 2;
t311 = t1 * t305 + t2 * t299;
t312 = t311 ^ 2;
t313 = t109 * t274;
t314 = t102 * t277;
t315 = t121 * pkin(23);
t318 = -t1 * t124 + t2 * t121;
t319 = t318 * pkin(9);
t320 = t313 - t314 - t315 - t319;
t321 = t320 ^ 2;
t329 = t1 * t50 * t51 + t2 * t51 * t58;
t331 = pkin(19) * t329 - pkin(9) + t4 + t7;
t333 = 0.1e1 / pkin(11);
t339 = t1 * t51 * t58 - t2 * t50 * t51;
t341 = pkin(19) * t339 + t24 - t25;
t344 = -t329 * t331 * t333 - t333 * t339 * t341;
t347 = pkin(24) * t256 + t281 + t286;
t349 = t59 * t294 + t52 * t347;
t355 = t329 * t333 * t341 - t331 * t333 * t339;
t356 = t52 * t294;
t357 = t59 * t347;
t360 = t50 ^ 2;
t361 = t58 ^ 2;
t362 = 0.1e1 / t361;
t365 = 0.1e1 / (t360 * t362 + 0.1e1);
t370 = t127 - t128 + t241 - t255 + t190 / t58 * t365 - t197 * t50 * t362 * t365;
t372 = pkin(19) * t370 - t356 + t357;
t374 = t344 * t349 + t355 * t372;
t375 = t374 ^ 2;
t378 = t344 * t372 - t355 * t349;
t379 = t378 ^ 2;
t380 = t121 * pkin(24);
t383 = t59 * t121 - t52 * t124;
t385 = -pkin(19) * t383 + t313 - t314 - t380;
t386 = t385 ^ 2;
t394 = -t356 + t357;
t397 = t52 * t121 + t59 * t124;
t402 = sin(pkin(3));
t404 = cos(pkin(3));
t406 = t402 * t307 - t404 * t311;
t409 = t1 * t121 + t2 * t124;
t412 = t1 * t409 - t2 * t318;
t416 = t1 * t318 + t2 * t409;
t418 = t402 * t416 + t404 * t412;
t421 = t127 - t128 + t241 - t255 + 0.2e1 * qJD(3);
t425 = t404 * t307;
t426 = t402 * t311;
t427 = t425 + t426;
t430 = t402 * t412 - t404 * t416;
t435 = t299 ^ 2;
t436 = -t301 + t302;
t437 = t436 ^ 2;
t438 = t313 - t314 - t315;
t439 = t438 ^ 2;
t447 = t349 ^ 2;
t448 = t394 ^ 2;
t449 = t313 - t314 - t380;
t450 = t449 ^ 2;
t456 = -t101 * t91 + t116 * t99;
t458 = pkin(22) * t456 + t55 + t62;
t460 = 0.1e1 / pkin(17);
t464 = t101 * t99 + t116 * t91;
t466 = -pkin(22) * t464 + pkin(24) + t65 - t66;
t469 = t456 * t458 * t460 - t460 * t464 * t466;
t472 = -t91 * t280 + t99 * t285;
t478 = -t456 * t460 * t466 - t458 * t460 * t464;
t479 = t99 * t280;
t480 = t91 * t285;
t481 = t127 - t128;
t483 = -pkin(22) * t481 - t479 - t480;
t485 = t469 * t472 + t478 * t483;
t488 = -t91 * t112 + t99 * t119;
t492 = -t99 * t112 - t91 * t119;
t494 = t469 * t492 - t478 * t488;
t496 = t232 * t89;
t498 = -t252 * t89;
t500 = t496 * t91 + t498 * t99;
t504 = t496 * t99 - t498 * t91;
t511 = t466 ^ 2;
t512 = t458 ^ 2;
t513 = 0.1e1 / t512;
t516 = 0.1e1 / (t511 * t513 + 0.1e1);
t526 = t127 - t128 + t241 - t255 - (-(-t500 * t99 + t504 * t91) * pkin(22) + t200 - t201) / t458 * t516 + ((t500 * t91 + t504 * t99) * pkin(22) + t192 + t199) * t466 * t513 * t516;
t532 = t469 * t483 - t478 * t472;
t535 = t469 * t488 + t478 * t492;
t540 = t121 * (Ifges(4,4) * t124 + Ifges(4,2) * t121 + Ifges(4,6) * t256) / 0.2e1 + t256 * (Ifges(4,5) * t124 + Ifges(4,6) * t121 + Ifges(4,3) * t256) / 0.2e1 + t287 * (mrSges(4,1) * t256 - mrSges(4,3) * t124) + m(13) * (t308 + t312 + t321) / 0.2e1 + m(12) * (t375 + t379 + t386) / 0.2e1 + t349 * (-mrSges(11,2) * t370 + mrSges(11,3) * t383) + t394 * (mrSges(11,1) * t370 - mrSges(11,3) * t397) + t406 * (-mrSges(14,2) * t421 + mrSges(14,3) * t418) + t427 * (mrSges(14,1) * t421 - mrSges(14,3) * t430) + m(5) * (t435 + t437 + t439) / 0.2e1 + t294 * (-mrSges(4,2) * t256 + mrSges(4,3) * t121) + m(11) * (t447 + t448 + t450) / 0.2e1 + t485 * (-mrSges(10,2) * t526 + mrSges(10,3) * t494) + t532 * (mrSges(10,1) * t526 - mrSges(10,3) * t535);
t547 = t344 * t383 - t355 * t397;
t560 = t341 ^ 2;
t561 = t331 ^ 2;
t562 = 0.1e1 / t561;
t565 = 0.1e1 / (t560 * t562 + 0.1e1);
t579 = t127 - t128 + t241 - t255 - ((t1 * t197 * t51 - t190 * t2 * t51 + t129 * t59 + t131 * t52) * pkin(19) + t147 - t149) / t331 * t565 + ((t1 * t190 * t51 + t2 * t197 * t51 + t129 * t52 - t131 * t59) * pkin(19) - t132 - t134) * t341 * t562 * t565 + qJD(3);
t613 = t282 + t283;
t618 = -t275 - t278;
t619 = t618 ^ 2;
t620 = -t313 + t314;
t621 = t620 ^ 2;
t622 = t613 ^ 2;
t640 = t472 ^ 2;
t641 = -t479 - t480;
t642 = t641 ^ 2;
t650 = t299 * (-mrSges(5,2) * t303 + mrSges(5,3) * t318) + t374 * (-mrSges(12,2) * t579 + mrSges(12,3) * t547) + t488 * (Ifges(9,1) * t488 + Ifges(9,4) * t492 + Ifges(9,5) * t481) / 0.2e1 + t492 * (Ifges(9,4) * t488 + Ifges(9,2) * t492 + Ifges(9,6) * t481) / 0.2e1 + t481 * (Ifges(9,5) * t488 + Ifges(9,6) * t492 + Ifges(9,3) * t481) / 0.2e1 - t481 * (Ifges(3,4) * t112 - Ifges(3,2) * t481 + Ifges(3,6) * t119) / 0.2e1 + t119 * (Ifges(3,5) * t112 - Ifges(3,6) * t481 + Ifges(3,3) * t119) / 0.2e1 + t613 * (mrSges(3,1) * t481 + mrSges(3,2) * t112) + m(3) * (t619 + t621 + t622) / 0.2e1 + t112 * (Ifges(3,1) * t112 - Ifges(3,4) * t481 + Ifges(3,5) * t119) / 0.2e1 + t618 * (-mrSges(3,2) * t119 - mrSges(3,3) * t481) - t620 * (-mrSges(9,1) * t492 + mrSges(9,2) * t488) + m(9) * (t640 + t642 + t621) / 0.2e1 + t472 * (-mrSges(9,2) * t481 + mrSges(9,3) * t492);
t652 = qJ(4) - qJ(3) + pkin(4);
t653 = sin(t652);
t655 = cos(t652);
t657 = pkin(3) + qJ(3) - qJ(4);
t658 = cos(t657);
t660 = -pkin(12) * t658 - pkin(14) * t653 - pkin(15) * t655 - pkin(10);
t661 = t660 ^ 2;
t664 = sin(t657);
t666 = -pkin(12) * t664 + pkin(14) * t655 - pkin(15) * t653;
t667 = t666 ^ 2;
t669 = sqrt(t661 + t667);
t670 = 0.1e1 / t669;
t671 = t660 * t670;
t673 = -t666 * t670;
t675 = -t671 * t658 + t673 * t664;
t676 = t675 * t406;
t679 = t673 * t658 + t671 * t664;
t681 = pkin(12) * t421 + t425 + t426;
t682 = t679 * t681;
t683 = qJD(4) - qJD(3);
t684 = t683 * t655;
t686 = t683 * t653;
t690 = -pkin(12) * t664 * t683 - pkin(14) * t684 + pkin(15) * t686;
t696 = pkin(12) * t658 * t683 - pkin(14) * t686 - pkin(15) * t684;
t701 = -t676 - t682 + t670 * (t660 * t690 + t666 * t696);
t704 = -t675 * t418 + t679 * t430;
t708 = 0.1e1 / t661;
t711 = 0.1e1 / (t667 * t708 + 0.1e1);
t716 = t127 - t128 + t241 - t255 + qJD(3) + t696 / t660 * t711 - t690 * t666 * t708 * t711 + qJD(4);
t720 = t406 ^ 2;
t721 = t427 ^ 2;
t735 = t344 * t397 + t355 * t383;
t796 = t701 * (-mrSges(16,1) * t716 + mrSges(16,2) * t704) + m(14) * (t720 + t721 + t321) / 0.2e1 + t307 * (-mrSges(13,2) * t421 + mrSges(13,3) * t416) + t311 * (mrSges(13,1) * t421 - mrSges(13,3) * t412) + t735 * (Ifges(12,1) * t735 + Ifges(12,4) * t547 + Ifges(12,5) * t579) / 0.2e1 + t547 * (Ifges(12,4) * t735 + Ifges(12,2) * t547 + Ifges(12,6) * t579) / 0.2e1 + t579 * (Ifges(12,5) * t735 + Ifges(12,6) * t547 + Ifges(12,3) * t579) / 0.2e1 + t397 * (Ifges(11,1) * t397 + Ifges(11,4) * t383 + Ifges(11,5) * t370) / 0.2e1 + t383 * (Ifges(11,4) * t397 + Ifges(11,2) * t383 + Ifges(11,6) * t370) / 0.2e1 + t370 * (Ifges(11,5) * t397 + Ifges(11,6) * t383 + Ifges(11,3) * t370) / 0.2e1 + t535 * (Ifges(10,1) * t535 + Ifges(10,4) * t494 + Ifges(10,5) * t526) / 0.2e1 + t494 * (Ifges(10,4) * t535 + Ifges(10,2) * t494 + Ifges(10,6) * t526) / 0.2e1 + t526 * (Ifges(10,5) * t535 + Ifges(10,6) * t494 + Ifges(10,3) * t526) / 0.2e1 + t430 * (Ifges(14,1) * t430 + Ifges(14,4) * t418 + Ifges(14,5) * t421) / 0.2e1;
t823 = t679 * t406;
t824 = -t675 * t681;
t826 = t716 * t669 + t823 + t824;
t827 = t826 ^ 2;
t828 = t418 * pkin(12);
t830 = -t704 * t669 + t313 - t314 - t315 - t319 - t828;
t831 = t830 ^ 2;
t832 = t701 ^ 2;
t836 = t823 + t824;
t839 = t679 * t418 + t675 * t430;
t844 = t676 + t682;
t849 = cos(qJ(4));
t851 = sin(qJ(4));
t853 = t851 * t318 + t849 * t409;
t857 = t849 * t318 - t851 * t409;
t859 = t127 - t128 + t241 - t255 + qJD(3) + qJD(4);
t864 = t851 * t299;
t865 = t849 * t305;
t866 = -t864 + t865;
t873 = t849 * t299 + t851 * t305;
t874 = t653 * t873;
t876 = pkin(10) * t859 - t864 + t865;
t877 = t655 * t876;
t878 = -t874 + t877;
t879 = t878 ^ 2;
t882 = -t653 * t876 - t655 * t873;
t883 = t882 ^ 2;
t884 = t857 * pkin(10);
t885 = t313 - t314 - t315 - t319 - t884;
t886 = t885 ^ 2;
t890 = t836 ^ 2;
t891 = t844 ^ 2;
t892 = t313 - t314 - t315 - t319 - t828;
t893 = t892 ^ 2;
t897 = t873 ^ 2;
t898 = t866 ^ 2;
t902 = cos(qJ(5));
t904 = t127 - t128 + t241 - t255 + 0.2e1 * qJD(4);
t906 = pkin(13) * t904 - t874 + t877;
t908 = sin(qJ(5));
t911 = -t653 * t853 + t655 * t857;
t913 = -pkin(13) * t911 + t313 - t314 - t315 - t319 - t884;
t915 = t902 * t906 + t908 * t913;
t918 = t902 * t904 - t908 * t911;
t920 = t655 * t853;
t921 = t653 * t857;
t922 = t920 + t921 + qJD(5);
t926 = t418 * (Ifges(14,4) * t430 + Ifges(14,2) * t418 + Ifges(14,6) * t421) / 0.2e1 + t378 * (mrSges(12,1) * t579 - mrSges(12,3) * t735) + t436 * (mrSges(5,1) * t303 - mrSges(5,3) * t409) + t303 * (Ifges(5,5) * t409 + Ifges(5,6) * t318 + Ifges(5,3) * t303) / 0.2e1 + t124 * (Ifges(4,1) * t124 + Ifges(4,4) * t121 + Ifges(4,5) * t256) / 0.2e1 + m(16) * (t827 + t831 + t832) / 0.2e1 + t836 * (-mrSges(15,2) * t716 + mrSges(15,3) * t839) + t844 * (mrSges(15,1) * t716 - mrSges(15,3) * t704) + t853 * (Ifges(6,1) * t853 + Ifges(6,4) * t857 + Ifges(6,5) * t859) / 0.2e1 + t866 * (mrSges(6,1) * t859 - mrSges(6,3) * t853) + m(7) * (t879 + t883 + t886) / 0.2e1 + m(15) * (t890 + t891 + t893) / 0.2e1 + m(6) * (t897 + t898 + t321) / 0.2e1 + t915 * (-mrSges(8,2) * t922 + mrSges(8,3) * t918);
t931 = t902 * t913 - t908 * t906;
t934 = t902 * t911 + t908 * t904;
t960 = t117 + t118;
t978 = t269 - t272;
t979 = t978 ^ 2;
t980 = V_base(3) ^ 2;
t1008 = t931 * (mrSges(8,1) * t922 - mrSges(8,3) * t934) + t918 * (Ifges(8,4) * t934 + Ifges(8,2) * t918 + Ifges(8,6) * t922) / 0.2e1 + t922 * (Ifges(8,5) * t934 + Ifges(8,6) * t918 + Ifges(8,3) * t922) / 0.2e1 + t641 * (mrSges(9,1) * t481 - mrSges(9,3) * t488) + t620 * (mrSges(3,1) * t119 - mrSges(3,3) * t112) + t107 * (Ifges(2,1) * t107 + Ifges(2,4) * t960 + Ifges(2,5) * t110) / 0.2e1 + t960 * (Ifges(2,4) * t107 + Ifges(2,2) * t960 + Ifges(2,6) * t110) / 0.2e1 + t110 * (Ifges(2,5) * t107 + Ifges(2,6) * t960 + Ifges(2,3) * t110) / 0.2e1 + m(2) * (t979 + t622 + t980) / 0.2e1 + V_base(3) * (-mrSges(2,1) * t960 + mrSges(2,2) * t107) + t978 * (-mrSges(2,2) * t110 + mrSges(2,3) * t960) + t613 * (mrSges(2,1) * t110 - mrSges(2,3) * t107) + V_base(4) * (Ifges(1,1) * V_base(4) + Ifges(1,4) * V_base(5) + Ifges(1,5) * V_base(6)) / 0.2e1 + V_base(5) * (Ifges(1,4) * V_base(4) + Ifges(1,2) * V_base(5) + Ifges(1,6) * V_base(6)) / 0.2e1;
t1027 = V_base(1) ^ 2;
t1028 = V_base(2) ^ 2;
t1042 = t915 ^ 2;
t1043 = t931 ^ 2;
t1047 = -t920 - t921;
t1076 = V_base(6) * (Ifges(1,5) * V_base(4) + Ifges(1,6) * V_base(5) + Ifges(1,3) * V_base(6)) / 0.2e1 + V_base(2) * (mrSges(1,1) * V_base(6) - mrSges(1,3) * V_base(4)) + V_base(1) * (-mrSges(1,2) * V_base(6) + mrSges(1,3) * V_base(5)) + V_base(3) * (-mrSges(1,1) * V_base(5) + mrSges(1,2) * V_base(4)) + m(1) * (t1027 + t1028 + t980) / 0.2e1 + t934 * (Ifges(8,1) * t934 + Ifges(8,4) * t918 + Ifges(8,5) * t922) / 0.2e1 - t882 * (-mrSges(8,1) * t918 + mrSges(8,2) * t934) + m(8) * (t1042 + t1043 + t883) / 0.2e1 + t878 * (-mrSges(7,2) * t904 + mrSges(7,3) * t1047) + t882 * (mrSges(7,1) * t904 - mrSges(7,3) * t911) + t873 * (-mrSges(6,2) * t859 + mrSges(6,3) * t857) + t826 * (mrSges(16,2) * t839 + mrSges(16,3) * t716) + t911 * (Ifges(7,1) * t911 + Ifges(7,4) * t1047 + Ifges(7,5) * t904) / 0.2e1 + t1047 * (Ifges(7,4) * t911 + Ifges(7,2) * t1047 + Ifges(7,6) * t904) / 0.2e1;
t1154 = t857 * (Ifges(6,4) * t853 + Ifges(6,2) * t857 + Ifges(6,6) * t859) / 0.2e1 - t839 * (Ifges(16,5) * t704 + Ifges(16,6) * t716 - Ifges(16,3) * t839) / 0.2e1 + t704 * (Ifges(15,1) * t704 + Ifges(15,4) * t839 + Ifges(15,5) * t716) / 0.2e1 + t839 * (Ifges(15,4) * t704 + Ifges(15,2) * t839 + Ifges(15,6) * t716) / 0.2e1 + t716 * (Ifges(15,5) * t704 + Ifges(15,6) * t839 + Ifges(15,3) * t716) / 0.2e1 + t904 * (Ifges(7,5) * t911 + Ifges(7,6) * t1047 + Ifges(7,3) * t904) / 0.2e1 + t830 * (-mrSges(16,1) * t839 - mrSges(16,3) * t704) + t320 * (-mrSges(6,1) * t857 + mrSges(6,2) * t853) + t892 * (-mrSges(15,1) * t839 + mrSges(15,2) * t704) + t885 * (-mrSges(7,1) * t1047 + mrSges(7,2) * t911) + t859 * (Ifges(6,5) * t853 + Ifges(6,6) * t857 + Ifges(6,3) * t859) / 0.2e1 + t704 * (Ifges(16,1) * t704 + Ifges(16,4) * t716 - Ifges(16,5) * t839) / 0.2e1 + t716 * (Ifges(16,4) * t704 + Ifges(16,2) * t716 - Ifges(16,6) * t839) / 0.2e1 + t421 * (Ifges(14,5) * t430 + Ifges(14,6) * t418 + Ifges(14,3) * t421) / 0.2e1;
t1183 = t294 ^ 2;
t1184 = t287 ^ 2;
t1197 = pkin(22) * t492 + t313 - t314;
t1202 = t485 ^ 2;
t1203 = t532 ^ 2;
t1204 = t1197 ^ 2;
t1226 = t412 * (Ifges(13,1) * t412 + Ifges(13,4) * t416 + Ifges(13,5) * t421) / 0.2e1 + t416 * (Ifges(13,4) * t412 + Ifges(13,2) * t416 + Ifges(13,6) * t421) / 0.2e1 + t409 * (Ifges(5,1) * t409 + Ifges(5,4) * t318 + Ifges(5,5) * t303) / 0.2e1 + t318 * (Ifges(5,4) * t409 + Ifges(5,2) * t318 + Ifges(5,6) * t303) / 0.2e1 - t620 * (-mrSges(4,1) * t121 + mrSges(4,2) * t124) + m(4) * (t1183 + t1184 + t621) / 0.2e1 + t320 * (-mrSges(14,1) * t418 + mrSges(14,2) * t430) + t320 * (-mrSges(13,1) * t416 + mrSges(13,2) * t412) + t1197 * (-mrSges(10,1) * t494 + mrSges(10,2) * t535) + m(10) * (t1202 + t1203 + t1204) / 0.2e1 + t385 * (-mrSges(12,1) * t547 + mrSges(12,2) * t735) + t449 * (-mrSges(11,1) * t383 + mrSges(11,2) * t397) + t438 * (-mrSges(5,1) * t318 + mrSges(5,2) * t409) + t421 * (Ifges(13,5) * t412 + Ifges(13,6) * t416 + Ifges(13,3) * t421) / 0.2e1;
t1229 = t540 + t650 + t796 + t926 + t1008 + t1076 + t1154 + t1226;
T = t1229;
