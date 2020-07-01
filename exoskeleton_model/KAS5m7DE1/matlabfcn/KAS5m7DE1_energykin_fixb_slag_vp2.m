% Calculate kinetic energy for
% KAS5m7DE1
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% qJD [5x1]
%   Generalized joint velocities
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

function T = KAS5m7DE1_energykin_fixb_slag_vp2(qJ, qJD, ...
  pkin, m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),zeros(24,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_energykin_fixb_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m7DE1_energykin_fixb_slag_vp2: qJD has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_energykin_fixb_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE1_energykin_fixb_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE1_energykin_fixb_slag_vp2: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7DE1_energykin_fixb_slag_vp2: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_fixb_linkframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-14 20:36:29
% EndTime: 2020-05-14 20:55:35
% DurationCPUTime: 1146.05s
% Computational Cost: add. (78688955->527), mult. (103904693->832), div. (1437062->24), fcn. (40937380->41), ass. (0->331)
t1 = cos(qJ(2));
t2 = t1 * qJD(1);
t3 = sin(qJ(3));
t4 = cos(qJ(3));
t5 = -pkin(23) + pkin(24);
t6 = t4 * t5;
t7 = pkin(3) + qJ(3);
t8 = cos(t7);
t9 = t8 * pkin(12);
t10 = -pkin(9) + t6 + t9;
t11 = pkin(11) ^ 2;
t12 = pkin(19) ^ 2;
t13 = t8 ^ 2;
t14 = pkin(12) ^ 2;
t16 = sin(t7);
t17 = t16 ^ 2;
t19 = -pkin(9) + t6;
t20 = t19 ^ 2;
t21 = t3 ^ 2;
t22 = t5 ^ 2;
t26 = t3 * t5;
t27 = t16 * pkin(12);
t28 = -t26 + t27;
t31 = 0.2e1 * t10 * t8 * pkin(12) + 0.2e1 * t28 * t16 * pkin(12) - t13 * t14 - t17 * t14 + t21 * t22 + t11 - t12 + t20;
t35 = 0.4e1 * t10 ^ 2 + 0.4e1 * t28 ^ 2;
t37 = t31 ^ 2;
t39 = sqrt(t11 * t35 - t37);
t41 = 0.2e1 * t10 * t31 - 0.2e1 * t28 * t39;
t42 = 0.1e1 / t35;
t44 = t41 * t42 + pkin(9) - t6 - t9;
t48 = 0.2e1 * t10 * t39 + 0.2e1 * t31 * t28;
t50 = t48 * t42 + t26 - t27;
t52 = t3 * t44 + t4 * t50;
t53 = 0.1e1 / pkin(19);
t54 = t52 * t53;
t55 = cos(pkin(7));
t56 = t55 * pkin(18);
t57 = t54 * t56;
t60 = -t3 * t50 + t4 * t44;
t61 = t60 * t53;
t62 = sin(pkin(7));
t63 = t62 * pkin(18);
t64 = t61 * t63;
t65 = pkin(17) ^ 2;
t66 = pkin(22) ^ 2;
t67 = t61 * t56;
t68 = t54 * t63;
t69 = t67 - t68;
t70 = t69 ^ 2;
t71 = t57 + t64;
t72 = t71 ^ 2;
t73 = pkin(24) ^ 2;
t74 = -pkin(24) - t67 + t68;
t77 = -0.2e1 * t74 * t69 + 0.2e1 * t71 ^ 2 + t65 - t66 - t70 - t72 + t73;
t81 = 0.4e1 * t71 ^ 2 + 0.4e1 * t74 ^ 2;
t83 = t77 ^ 2;
t85 = sqrt(t65 * t81 - t83);
t87 = -0.2e1 * t77 * t71 + 0.2e1 * t74 * t85;
t88 = 0.1e1 / t81;
t90 = -t87 * t88 - t57 - t64;
t93 = 0.2e1 * t71 * t85 + 0.2e1 * t74 * t77;
t95 = t93 * t88 + pkin(24) + t67 - t68;
t96 = atan2(t90, t95);
t97 = t96 + pkin(6);
t98 = sin(t97);
t99 = t98 * t1;
t101 = cos(t97);
t103 = -t99 * qJD(1) - t101 * qJD(2);
t105 = t101 * t1;
t108 = -t105 * qJD(1) + t98 * qJD(2);
t113 = sin(pkin(6));
t114 = t113 * t1;
t116 = cos(pkin(6));
t118 = t114 * qJD(1) + t116 * qJD(2);
t120 = t116 * t1;
t123 = t120 * qJD(1) - t113 * qJD(2);
t128 = sin(qJ(2));
t129 = t128 * qJD(1);
t140 = qJ(4) - qJ(3) + pkin(4);
t141 = sin(t140);
t142 = cos(qJ(4));
t145 = t129 * pkin(16) + qJD(2) * pkin(21);
t147 = qJD(1) * pkin(21);
t149 = -t105 * t147 + t98 * t145;
t151 = t101 * t145;
t152 = t99 * t147;
t153 = qJD(3) * t4;
t155 = qJD(3) * t3;
t156 = t155 * t5;
t158 = qJD(3) * t16 * pkin(12);
t159 = -t156 - t158;
t171 = t153 * t5;
t173 = qJD(3) * t8 * pkin(12);
t174 = -t171 + t173;
t179 = 0.2e1 * t159 * t8 * pkin(12) + 0.2e1 * t174 * t16 * pkin(12) - 0.2e1 * t10 * qJD(3) * t27 - 0.2e1 * t19 * qJD(3) * t26 + 0.2e1 * t28 * qJD(3) * t9 + 0.2e1 * t3 * t22 * t153;
t182 = 0.1e1 / t39;
t186 = 0.4e1 * t10 * t159 + 0.4e1 * t28 * t174;
t190 = 0.2e1 * t11 * t186 - 0.2e1 * t31 * t179;
t195 = t35 ^ 2;
t196 = 0.1e1 / t195;
t199 = t156 + t158 + (-t28 * t182 * t190 + 0.2e1 * t10 * t179 + 0.2e1 * t159 * t31 - 0.2e1 * t174 * t39) * t42 - 0.2e1 * t41 * t196 * t186;
t212 = t171 - t173 + (t10 * t182 * t190 + 0.2e1 * t159 * t39 + 0.2e1 * t31 * t174 + 0.2e1 * t179 * t28) * t42 - 0.2e1 * t48 * t196 * t186;
t214 = t153 * t44 - t155 * t50 + t3 * t199 + t4 * t212;
t215 = t214 * t53;
t216 = t215 * t56;
t221 = -t153 * t50 - t155 * t44 + t4 * t199 - t3 * t212;
t222 = t221 * t53;
t223 = t222 * t63;
t224 = t222 * t56;
t225 = t215 * t63;
t226 = t224 - t225;
t229 = t216 + t223;
t235 = -0.2e1 * t74 * t226 + 0.2e1 * t71 * t229;
t239 = 0.1e1 / t85;
t243 = -0.4e1 * t74 * t226 + 0.4e1 * t71 * t229;
t247 = -0.2e1 * t77 * t235 + 0.2e1 * t65 * t243;
t252 = t81 ^ 2;
t253 = 0.1e1 / t252;
t256 = -t216 - t223 - (t74 * t239 * t247 - 0.2e1 * t226 * t85 - 0.2e1 * t77 * t229 - 0.2e1 * t235 * t71) * t88 + 0.2e1 * t87 * t253 * t243;
t259 = t90 ^ 2;
t260 = t95 ^ 2;
t261 = 0.1e1 / t260;
t264 = 0.1e1 / (t259 * t261 + 0.1e1);
t265 = t256 / t95 * t264;
t276 = t224 - t225 + (t71 * t239 * t247 - 0.2e1 * t226 * t77 + 0.2e1 * t229 * t85 + 0.2e1 * t74 * t235) * t88 - 0.2e1 * t93 * t253 * t243;
t279 = t276 * t90 * t261 * t264;
t280 = -t129 + t265 - t279;
t282 = t280 * pkin(23) + t151 + t152;
t284 = t4 * t149 + t3 * t282;
t286 = sin(qJ(4));
t287 = t3 * t149;
t288 = t4 * t282;
t289 = -t129 + t265 - t279 + qJD(3);
t291 = t289 * pkin(9) - t287 + t288;
t293 = t142 * t284 + t286 * t291;
t294 = t141 * t293;
t295 = cos(t140);
t296 = t286 * t284;
t297 = t142 * t291;
t298 = -t129 + t265 - t279 + qJD(3) + qJD(4);
t300 = t298 * pkin(10) - t296 + t297;
t301 = t295 * t300;
t302 = -t294 + t301;
t305 = t4 * t103 + t3 * t108;
t309 = -t3 * t103 + t4 * t108;
t311 = t142 * t305 + t286 * t309;
t312 = t295 * t311;
t315 = t142 * t309 - t286 * t305;
t316 = t141 * t315;
t317 = -t312 - t316;
t320 = -t129 + t265 - t279 + 0.2e1 * qJD(4);
t325 = pkin(3) + qJ(3) - qJ(4);
t326 = sin(t325);
t329 = t326 * pkin(12) - t295 * pkin(14) + t141 * pkin(15);
t332 = cos(t325);
t334 = -t332 * pkin(12) - t141 * pkin(14) - t295 * pkin(15) - pkin(10);
t335 = atan2(t329, t334);
t336 = t335 + pkin(3) + qJ(3) - qJ(4);
t337 = cos(t336);
t338 = sin(pkin(3));
t341 = t3 * t284 - t4 * t291;
t343 = cos(pkin(3));
t346 = t4 * t284 + t3 * t291;
t348 = t338 * t341 - t343 * t346;
t349 = t337 * t348;
t350 = sin(t336);
t351 = t343 * t341;
t352 = t338 * t346;
t354 = -t129 + t265 - t279 + 0.2e1 * qJD(3);
t356 = t354 * pkin(12) + t351 + t352;
t357 = t350 * t356;
t358 = t334 ^ 2;
t359 = t329 ^ 2;
t361 = sqrt(t358 + t359);
t363 = qJD(4) - qJD(3);
t364 = t363 * t295;
t366 = t363 * t141;
t370 = -t363 * t326 * pkin(12) - t364 * pkin(14) + t366 * pkin(15);
t376 = t363 * t332 * pkin(12) - t366 * pkin(14) - t364 * pkin(15);
t381 = t349 - t357 + 0.1e1 / t361 * (-t329 * t376 + t334 * t370);
t384 = t3 * t305 - t4 * t309;
t388 = t3 * t309 + t4 * t305;
t390 = t338 * t384 - t343 * t388;
t394 = t338 * t388 + t343 * t384;
t396 = t337 * t394 + t350 * t390;
t400 = 0.1e1 / t358;
t403 = 0.1e1 / (t359 * t400 + 0.1e1);
t408 = -t129 + t265 - t279 + qJD(3) + t376 / t334 * t403 + t370 * t329 * t400 * t403 + qJD(4);
t418 = (t3 * t60 * t53 - t4 * t52 * t53) * pkin(19) + t26 - t27;
t425 = (t3 * t52 * t53 + t4 * t60 * t53) * pkin(19) - pkin(9) + t6 + t9;
t426 = atan2(t418, t425);
t427 = atan2(-t52, t60);
t428 = -t426 + qJ(3) + t427;
t429 = cos(t428);
t430 = cos(t427);
t432 = sin(t427);
t434 = t280 * pkin(24) + t151 + t152;
t436 = t430 * t149 - t432 * t434;
t438 = sin(t428);
t439 = t432 * t149;
t440 = t430 * t434;
t443 = t52 ^ 2;
t444 = t60 ^ 2;
t445 = 0.1e1 / t444;
t448 = 0.1e1 / (t443 * t445 + 0.1e1);
t453 = -t129 + t265 - t279 + t214 / t60 * t448 - t221 * t52 * t445 * t448;
t455 = t453 * pkin(19) + t439 + t440;
t457 = -t429 * t436 - t438 * t455;
t460 = t430 * t103 - t432 * t108;
t464 = t432 * t103 + t430 * t108;
t466 = -t429 * t464 + t438 * t460;
t479 = t418 ^ 2;
t480 = t425 ^ 2;
t481 = 0.1e1 / t480;
t484 = 0.1e1 / (t479 * t481 + 0.1e1);
t498 = -t129 + t265 - t279 - ((-t4 * t214 * t53 + t3 * t221 * t53 + t153 * t61 + t155 * t54) * pkin(19) + t171 - t173) / t425 * t484 + ((t3 * t214 * t53 + t4 * t221 * t53 + t153 * t54 - t155 * t61) * pkin(19) - t156 - t158) * t418 * t481 * t484 + qJD(3);
t502 = t350 * t348;
t503 = t337 * t356;
t505 = t408 * t361 + t502 + t503;
t506 = t505 ^ 2;
t507 = t2 * pkin(16);
t508 = t108 * pkin(23);
t509 = t309 * pkin(9);
t510 = t394 * pkin(12);
t512 = -t396 * t361 - t507 - t508 - t509 - t510;
t513 = t512 ^ 2;
t514 = t381 ^ 2;
t518 = t502 + t503;
t521 = -t337 * t390 + t350 * t394;
t526 = t108 * pkin(24);
t527 = -t507 - t526;
t532 = -t507 - t508;
t538 = -t464 * pkin(19) - t507 - t526;
t541 = -t429 * t460 - t438 * t464;
t546 = -t2 * pkin(16) * (-t108 * mrSges(4,1) + t103 * mrSges(4,2)) - t2 * pkin(16) * (-t123 * mrSges(9,1) + t118 * mrSges(9,2)) + t129 * pkin(16) * (-qJD(2) * mrSges(3,2) + t129 * mrSges(3,3)) + t2 * pkin(16) * (qJD(2) * mrSges(3,1) + t2 * mrSges(3,3)) + t302 * (-t320 * mrSges(7,2) + t317 * mrSges(7,3)) + t381 * (-t408 * mrSges(16,1) + t396 * mrSges(16,2)) + t457 * (-t498 * mrSges(12,2) + t466 * mrSges(12,3)) + m(16) * (t506 + t513 + t514) / 0.2e1 + t518 * (-t408 * mrSges(15,2) + t521 * mrSges(15,3)) + t527 * (-t464 * mrSges(11,1) + t460 * mrSges(11,2)) + t532 * (-t309 * mrSges(5,1) + t305 * mrSges(5,2)) + t538 * (-t466 * mrSges(12,1) + t541 * mrSges(12,2));
t547 = 0.1e1 / pkin(22);
t548 = t90 * t547;
t550 = -t95 * t547;
t552 = t548 * t113 + t550 * t116;
t556 = -t550 * t113 + t548 * t116;
t560 = -(t556 * t113 - t552 * t116) * pkin(22) + pkin(24) + t67 - t68;
t565 = (t552 * t113 + t556 * t116) * pkin(22) + t57 + t64;
t566 = atan2(t560, t565);
t567 = t96 - t566;
t568 = sin(t567);
t570 = cos(t567);
t572 = t568 * t118 - t570 * t123;
t576 = t570 * t118 + t568 * t123;
t578 = t256 * t547;
t580 = -t276 * t547;
t582 = t578 * t113 + t580 * t116;
t586 = -t580 * t113 + t578 * t116;
t593 = t560 ^ 2;
t594 = t565 ^ 2;
t595 = 0.1e1 / t594;
t598 = 0.1e1 / (t593 * t595 + 0.1e1);
t608 = -t129 + t265 - t279 - (-(t586 * t113 - t582 * t116) * pkin(22) + t224 - t225) / t565 * t598 + ((t582 * t113 + t586 * t116) * pkin(22) + t216 + t223) * t560 * t595 * t598;
t671 = t351 + t352;
t676 = t572 * (Ifges(10,1) * t572 + Ifges(10,4) * t576 + Ifges(10,5) * t608) / 0.2e1 + t576 * (Ifges(10,4) * t572 + Ifges(10,2) * t576 + Ifges(10,6) * t608) / 0.2e1 + t541 * (Ifges(12,1) * t541 + Ifges(12,4) * t466 + Ifges(12,5) * t498) / 0.2e1 + t466 * (Ifges(12,4) * t541 + Ifges(12,2) * t466 + Ifges(12,6) * t498) / 0.2e1 + t384 * (Ifges(13,1) * t384 + Ifges(13,4) * t388 + Ifges(13,5) * t354) / 0.2e1 + t388 * (Ifges(13,4) * t384 + Ifges(13,2) * t388 + Ifges(13,6) * t354) / 0.2e1 + t354 * (Ifges(13,5) * t384 + Ifges(13,6) * t388 + Ifges(13,3) * t354) / 0.2e1 + t123 * (-Ifges(9,6) * t128 * qJD(1) + Ifges(9,4) * t118 + Ifges(9,2) * t123) / 0.2e1 + t118 * (-Ifges(9,5) * t128 * qJD(1) + Ifges(9,1) * t118 + Ifges(9,4) * t123) / 0.2e1 + t346 * (t354 * mrSges(13,1) - t384 * mrSges(13,3)) + t348 * (-t354 * mrSges(14,2) + t394 * mrSges(14,3)) + t671 * (t354 * mrSges(14,1) - t390 * mrSges(14,3));
t678 = t348 ^ 2;
t679 = t671 ^ 2;
t680 = -t507 - t508 - t509;
t681 = t680 ^ 2;
t687 = -t113 * t145 + t120 * t147;
t689 = t116 * t145;
t690 = t114 * t147;
t692 = t129 * pkin(22) - t689 - t690;
t694 = t568 * t692 + t570 * t687;
t699 = t284 ^ 2;
t700 = -t287 + t288;
t701 = t700 ^ 2;
t702 = t532 ^ 2;
t710 = t151 + t152;
t760 = m(14) * (t678 + t679 + t681) / 0.2e1 + t694 * (t608 * mrSges(10,1) - t572 * mrSges(10,3)) + m(5) * (t699 + t701 + t702) / 0.2e1 + t149 * (-t280 * mrSges(4,2) + t108 * mrSges(4,3)) + t710 * (t280 * mrSges(4,1) - t103 * mrSges(4,3)) + t521 * (Ifges(15,4) * t396 + Ifges(15,2) * t521 + Ifges(15,6) * t408) / 0.2e1 + t408 * (Ifges(15,5) * t396 + Ifges(15,6) * t521 + Ifges(15,3) * t408) / 0.2e1 + t311 * (Ifges(6,1) * t311 + Ifges(6,4) * t315 + Ifges(6,5) * t298) / 0.2e1 + t284 * (-t289 * mrSges(5,2) + t309 * mrSges(5,3)) - t129 * (-Ifges(9,3) * t128 * qJD(1) + Ifges(9,5) * t118 + Ifges(9,6) * t123) / 0.2e1 - t2 * (-Ifges(3,1) * t1 * qJD(1) + Ifges(3,4) * t128 * qJD(1) + Ifges(3,5) * qJD(2)) / 0.2e1 + t129 * (-Ifges(3,4) * t1 * qJD(1) + Ifges(3,2) * t128 * qJD(1) + Ifges(3,6) * qJD(2)) / 0.2e1;
t763 = -t429 * t455 + t438 * t436;
t768 = t341 ^ 2;
t769 = t346 ^ 2;
t781 = t315 * pkin(10);
t782 = -t507 - t508 - t509 - t781;
t785 = -t141 * t311 + t295 * t315;
t790 = -t507 - t508 - t509 - t510;
t795 = t457 ^ 2;
t796 = t763 ^ 2;
t797 = t538 ^ 2;
t801 = t436 ^ 2;
t802 = t439 + t440;
t803 = t802 ^ 2;
t804 = t527 ^ 2;
t810 = t568 * t687 - t570 * t692;
t837 = t763 * (t498 * mrSges(12,1) - t541 * mrSges(12,3)) + m(13) * (t768 + t769 + t681) / 0.2e1 + t680 * (-t315 * mrSges(6,1) + t311 * mrSges(6,2)) + t512 * (-t521 * mrSges(16,1) - t396 * mrSges(16,3)) + t782 * (-t317 * mrSges(7,1) + t785 * mrSges(7,2)) + t790 * (-t521 * mrSges(15,1) + t396 * mrSges(15,2)) + m(12) * (t795 + t796 + t797) / 0.2e1 + m(11) * (t801 + t803 + t804) / 0.2e1 + t810 * (-t608 * mrSges(10,2) + t576 * mrSges(10,3)) + t505 * (t521 * mrSges(16,2) + t408 * mrSges(16,3)) + t317 * (Ifges(7,4) * t785 + Ifges(7,2) * t317 + Ifges(7,6) * t320) / 0.2e1 + t408 * (Ifges(16,4) * t396 + Ifges(16,2) * t408 - Ifges(16,6) * t521) / 0.2e1 - t521 * (Ifges(16,5) * t396 + Ifges(16,6) * t408 - Ifges(16,3) * t521) / 0.2e1;
t910 = t396 * (Ifges(15,1) * t396 + Ifges(15,4) * t521 + Ifges(15,5) * t408) / 0.2e1 + t320 * (Ifges(7,5) * t785 + Ifges(7,6) * t317 + Ifges(7,3) * t320) / 0.2e1 + t396 * (Ifges(16,1) * t396 + Ifges(16,4) * t408 - Ifges(16,5) * t521) / 0.2e1 + t390 * (Ifges(14,1) * t390 + Ifges(14,4) * t394 + Ifges(14,5) * t354) / 0.2e1 + t394 * (Ifges(14,4) * t390 + Ifges(14,2) * t394 + Ifges(14,6) * t354) / 0.2e1 + t354 * (Ifges(14,5) * t390 + Ifges(14,6) * t394 + Ifges(14,3) * t354) / 0.2e1 + t103 * (Ifges(4,1) * t103 + Ifges(4,4) * t108 + Ifges(4,5) * t280) / 0.2e1 + t108 * (Ifges(4,4) * t103 + Ifges(4,2) * t108 + Ifges(4,6) * t280) / 0.2e1 + t280 * (Ifges(4,5) * t103 + Ifges(4,6) * t108 + Ifges(4,3) * t280) / 0.2e1 + t498 * (Ifges(12,5) * t541 + Ifges(12,6) * t466 + Ifges(12,3) * t498) / 0.2e1 + t305 * (Ifges(5,1) * t305 + Ifges(5,4) * t309 + Ifges(5,5) * t289) / 0.2e1 + t802 * (t453 * mrSges(11,1) - t460 * mrSges(11,3));
t911 = qJD(1) ^ 2;
t934 = t312 + t316 + qJD(5);
t935 = cos(qJ(5));
t937 = sin(qJ(5));
t939 = t937 * t320 + t935 * t785;
t943 = t935 * t320 - t937 * t785;
t967 = t149 ^ 2;
t968 = t710 ^ 2;
t969 = t1 ^ 2;
t971 = pkin(16) ^ 2;
t972 = t969 * t911 * t971;
t992 = t911 * Ifges(2,3) / 0.2e1 + qJD(2) * (-Ifges(3,5) * t1 * qJD(1) + Ifges(3,6) * t128 * qJD(1) + Ifges(3,3) * qJD(2)) / 0.2e1 + t315 * (Ifges(6,4) * t311 + Ifges(6,2) * t315 + Ifges(6,6) * t298) / 0.2e1 + t298 * (Ifges(6,5) * t311 + Ifges(6,6) * t315 + Ifges(6,3) * t298) / 0.2e1 + t934 * (Ifges(8,5) * t939 + Ifges(8,6) * t943 + Ifges(8,3) * t934) / 0.2e1 + t939 * (Ifges(8,1) * t939 + Ifges(8,4) * t943 + Ifges(8,5) * t934) / 0.2e1 + t943 * (Ifges(8,4) * t939 + Ifges(8,2) * t943 + Ifges(8,6) * t934) / 0.2e1 + t785 * (Ifges(7,1) * t785 + Ifges(7,4) * t317 + Ifges(7,5) * t320) / 0.2e1 + m(4) * (t967 + t968 + t972) / 0.2e1 + t680 * (-t388 * mrSges(13,1) + t384 * mrSges(13,2)) + t309 * (Ifges(5,4) * t305 + Ifges(5,2) * t309 + Ifges(5,6) * t289) / 0.2e1 + t289 * (Ifges(5,5) * t305 + Ifges(5,6) * t309 + Ifges(5,3) * t289) / 0.2e1;
t994 = t128 ^ 2;
t1000 = -t689 - t690;
t1005 = t687 ^ 2;
t1006 = t1000 ^ 2;
t1014 = t293 ^ 2;
t1015 = -t296 + t297;
t1016 = t1015 ^ 2;
t1032 = t518 ^ 2;
t1033 = -t349 + t357;
t1034 = t1033 ^ 2;
t1035 = t790 ^ 2;
t1040 = t320 * pkin(13) - t294 + t301;
t1043 = -t785 * pkin(13) - t507 - t508 - t509 - t781;
t1045 = -t937 * t1040 + t935 * t1043;
t1052 = t935 * t1040 + t937 * t1043;
t1059 = t141 * t300 + t295 * t293;
t1064 = m(3) * (t994 * t911 * t971 + t972) / 0.2e1 + t1000 * (-t129 * mrSges(9,1) - t118 * mrSges(9,3)) + m(9) * (t1005 + t1006 + t972) / 0.2e1 + t687 * (t129 * mrSges(9,2) + t123 * mrSges(9,3)) + m(6) * (t1014 + t1016 + t681) / 0.2e1 + t341 * (-t354 * mrSges(13,2) + t388 * mrSges(13,3)) + t700 * (t289 * mrSges(5,1) - t305 * mrSges(5,3)) + t436 * (-t453 * mrSges(11,2) + t464 * mrSges(11,3)) + m(15) * (t1032 + t1034 + t1035) / 0.2e1 + t1045 * (t934 * mrSges(8,1) - t939 * mrSges(8,3)) + t1052 * (-t934 * mrSges(8,2) + t943 * mrSges(8,3)) + t1059 * (-t943 * mrSges(8,1) + t939 * mrSges(8,2));
t1065 = t1052 ^ 2;
t1066 = t1045 ^ 2;
t1067 = t1059 ^ 2;
t1075 = t302 ^ 2;
t1076 = t782 ^ 2;
t1116 = t810 ^ 2;
t1117 = t694 ^ 2;
t1119 = t123 * pkin(22) - t507;
t1120 = t1119 ^ 2;
t1132 = m(8) * (t1065 + t1066 + t1067) / 0.2e1 + t1015 * (t298 * mrSges(6,1) - t311 * mrSges(6,3)) + m(7) * (t1075 + t1067 + t1076) / 0.2e1 + t293 * (-t298 * mrSges(6,2) + t315 * mrSges(6,3)) - t1059 * (t320 * mrSges(7,1) - t785 * mrSges(7,3)) + t1033 * (t408 * mrSges(15,1) - t396 * mrSges(15,3)) + t460 * (Ifges(11,1) * t460 + Ifges(11,4) * t464 + Ifges(11,5) * t453) / 0.2e1 + t464 * (Ifges(11,4) * t460 + Ifges(11,2) * t464 + Ifges(11,6) * t453) / 0.2e1 + t453 * (Ifges(11,5) * t460 + Ifges(11,6) * t464 + Ifges(11,3) * t453) / 0.2e1 + t608 * (Ifges(10,5) * t572 + Ifges(10,6) * t576 + Ifges(10,3) * t608) / 0.2e1 + m(10) * (t1116 + t1117 + t1120) / 0.2e1 + t680 * (-t394 * mrSges(14,1) + t390 * mrSges(14,2)) + t1119 * (-t576 * mrSges(10,1) + t572 * mrSges(10,2));
t1135 = t546 + t676 + t760 + t837 + t910 + t992 + t1064 + t1132;
T = t1135;
