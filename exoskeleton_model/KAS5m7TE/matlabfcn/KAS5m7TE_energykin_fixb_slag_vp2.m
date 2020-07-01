% Calculate kinetic energy for
% KAS5m7TE
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
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = KAS5m7TE_energykin_fixb_slag_vp2(qJ, qJD, ...
  pkin, m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),zeros(24,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_energykin_fixb_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m7TE_energykin_fixb_slag_vp2: qJD has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_energykin_fixb_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7TE_energykin_fixb_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7TE_energykin_fixb_slag_vp2: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7TE_energykin_fixb_slag_vp2: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_fixb_linkframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-04-13 19:36:33
% EndTime: 2020-04-13 19:58:44
% DurationCPUTime: 1210.70s
% Computational Cost: add. (79745171->528), mult. (105272309->856), div. (1465322->26), fcn. (41483508->26), ass. (0->326)
t1 = sin(qJ(2));
t2 = t1 * qJD(1);
t8 = cos(qJ(2));
t9 = t8 * qJD(1);
t10 = sin(pkin(6));
t11 = t10 * t8;
t13 = cos(pkin(6));
t15 = qJD(1) * t11 + qJD(2) * t13;
t17 = t13 * t8;
t20 = qJD(1) * t17 - qJD(2) * t10;
t30 = cos(qJ(3));
t31 = -pkin(23) + pkin(24);
t32 = t30 * t31;
t33 = pkin(3) + qJ(3);
t34 = cos(t33);
t35 = t34 * pkin(12);
t36 = -pkin(9) + t32 + t35;
t37 = pkin(11) ^ 2;
t38 = pkin(19) ^ 2;
t39 = t34 ^ 2;
t40 = pkin(12) ^ 2;
t42 = sin(t33);
t43 = t42 ^ 2;
t45 = -pkin(9) + t32;
t46 = t45 ^ 2;
t47 = sin(qJ(3));
t48 = t47 ^ 2;
t49 = t31 ^ 2;
t53 = t47 * t31;
t54 = t42 * pkin(12);
t55 = -t53 + t54;
t58 = 0.2e1 * pkin(12) * t34 * t36 + 0.2e1 * pkin(12) * t42 * t55 - t39 * t40 - t43 * t40 + t48 * t49 + t37 - t38 + t46;
t62 = 0.4e1 * t36 ^ 2 + 0.4e1 * t55 ^ 2;
t64 = t58 ^ 2;
t66 = sqrt(t37 * t62 - t64);
t68 = 0.2e1 * t36 * t58 - 0.2e1 * t55 * t66;
t69 = 0.1e1 / t62;
t71 = t68 * t69 + pkin(9) - t32 - t35;
t75 = 0.2e1 * t36 * t66 + 0.2e1 * t58 * t55;
t77 = t75 * t69 + t53 - t54;
t79 = t30 * t71 - t47 * t77;
t80 = 0.1e1 / pkin(19);
t81 = t79 * t80;
t82 = cos(pkin(7));
t83 = t82 * pkin(18);
t84 = t81 * t83;
t87 = t30 * t77 + t47 * t71;
t88 = t87 * t80;
t89 = sin(pkin(7));
t90 = t89 * pkin(18);
t91 = t88 * t90;
t92 = -pkin(24) - t84 + t91;
t93 = pkin(17) ^ 2;
t94 = pkin(22) ^ 2;
t95 = t84 - t91;
t96 = t95 ^ 2;
t97 = t88 * t83;
t98 = t81 * t90;
t99 = t97 + t98;
t100 = t99 ^ 2;
t101 = pkin(24) ^ 2;
t104 = -0.2e1 * t92 * t95 + 0.2e1 * t99 ^ 2 - t100 + t101 + t93 - t94 - t96;
t108 = 0.4e1 * t92 ^ 2 + 0.4e1 * t99 ^ 2;
t110 = t104 ^ 2;
t112 = sqrt(t93 * t108 - t110);
t114 = 0.2e1 * t92 * t104 + 0.2e1 * t99 * t112;
t115 = 0.1e1 / t108;
t117 = -t114 * t115 - pkin(24) - t84 + t91;
t118 = 0.1e1 / pkin(22);
t119 = t117 * t118;
t123 = -0.2e1 * t104 * t99 + 0.2e1 * t92 * t112;
t125 = -t123 * t115 - t97 - t98;
t126 = t125 * t118;
t128 = -t119 * t10 + t126 * t13;
t129 = t128 * t8;
t133 = t126 * t10 + t119 * t13;
t135 = -qJD(1) * t129 + qJD(2) * t133;
t140 = qJD(1) * t133 * t8 + qJD(2) * t128;
t145 = qJ(4) - qJ(3) + pkin(4);
t146 = sin(t145);
t148 = cos(t145);
t150 = pkin(3) + qJ(3) - qJ(4);
t151 = cos(t150);
t153 = -pkin(12) * t151 - pkin(14) * t146 - pkin(15) * t148 - pkin(10);
t154 = t153 ^ 2;
t157 = sin(t150);
t159 = -pkin(12) * t157 + pkin(14) * t148 - pkin(15) * t146;
t160 = t159 ^ 2;
t162 = sqrt(t154 + t160);
t163 = 0.1e1 / t162;
t164 = t153 * t163;
t166 = -t159 * t163;
t168 = -t164 * t151 + t166 * t157;
t169 = sin(pkin(3));
t172 = pkin(16) * t2 + qJD(2) * pkin(21);
t175 = qJD(1) * pkin(21);
t177 = t133 * t175 * t8 + t128 * t172;
t179 = -t133 * t172;
t180 = t129 * t175;
t181 = qJD(3) * t30;
t183 = qJD(3) * t47;
t184 = t183 * t31;
t186 = qJD(3) * t42 * pkin(12);
t187 = -t184 - t186;
t199 = t181 * t31;
t201 = qJD(3) * t34 * pkin(12);
t202 = -t199 + t201;
t207 = 0.2e1 * pkin(12) * t187 * t34 + 0.2e1 * pkin(12) * t202 * t42 + 0.2e1 * qJD(3) * t35 * t55 - 0.2e1 * qJD(3) * t36 * t54 - 0.2e1 * qJD(3) * t45 * t53 + 0.2e1 * t181 * t47 * t49;
t210 = 0.1e1 / t66;
t214 = 0.4e1 * t36 * t187 + 0.4e1 * t55 * t202;
t218 = -0.2e1 * t58 * t207 + 0.2e1 * t37 * t214;
t223 = t62 ^ 2;
t224 = 0.1e1 / t223;
t227 = t184 + t186 + (-t210 * t218 * t55 + 0.2e1 * t187 * t58 - 0.2e1 * t202 * t66 + 0.2e1 * t36 * t207) * t69 - 0.2e1 * t68 * t224 * t214;
t240 = t199 - t201 + (t210 * t218 * t36 + 0.2e1 * t187 * t66 + 0.2e1 * t58 * t202 + 0.2e1 * t207 * t55) * t69 - 0.2e1 * t75 * t224 * t214;
t242 = t181 * t71 - t183 * t77 + t47 * t227 + t30 * t240;
t243 = t242 * t80;
t244 = t243 * t83;
t249 = -t181 * t77 - t183 * t71 + t30 * t227 - t47 * t240;
t250 = t249 * t80;
t251 = t250 * t90;
t252 = t250 * t83;
t253 = t243 * t90;
t254 = t252 - t253;
t257 = t244 + t251;
t263 = -0.2e1 * t92 * t254 + 0.2e1 * t99 * t257;
t267 = 0.1e1 / t112;
t271 = -0.4e1 * t92 * t254 + 0.4e1 * t99 * t257;
t275 = -0.2e1 * t104 * t263 + 0.2e1 * t93 * t271;
t280 = t108 ^ 2;
t281 = 0.1e1 / t280;
t284 = -t244 - t251 - (t267 * t275 * t92 - 0.2e1 * t104 * t257 - 0.2e1 * t254 * t112 - 0.2e1 * t263 * t99) * t115 + 0.2e1 * t123 * t281 * t271;
t287 = t125 ^ 2;
t288 = t117 ^ 2;
t289 = 0.1e1 / t288;
t292 = 0.1e1 / (t287 * t289 + 0.1e1);
t293 = -t284 / t117 * t292;
t304 = t252 - t253 + (t267 * t275 * t99 - 0.2e1 * t254 * t104 + 0.2e1 * t257 * t112 + 0.2e1 * t92 * t263) * t115 - 0.2e1 * t114 * t281 * t271;
t307 = t304 * t125 * t289 * t292;
t308 = -t2 + t293 - t307;
t310 = pkin(23) * t308 + t179 + t180;
t312 = t30 * t177 + t47 * t310;
t314 = t47 * t177;
t315 = t30 * t310;
t316 = -t2 + t293 - t307 + qJD(3);
t318 = pkin(9) * t316 - t314 + t315;
t320 = -t30 * t318 + t47 * t312;
t322 = cos(pkin(3));
t325 = t30 * t312 + t47 * t318;
t327 = t169 * t320 - t322 * t325;
t328 = t168 * t327;
t331 = t166 * t151 + t164 * t157;
t332 = t322 * t320;
t333 = t169 * t325;
t335 = -t2 + t293 - t307 + 0.2e1 * qJD(3);
t337 = pkin(12) * t335 + t332 + t333;
t338 = t331 * t337;
t339 = t328 + t338;
t342 = t30 * t135 + t47 * t140;
t346 = -t47 * t135 + t30 * t140;
t348 = -t30 * t346 + t47 * t342;
t352 = t30 * t342 + t47 * t346;
t354 = t169 * t348 - t322 * t352;
t358 = t169 * t352 + t322 * t348;
t360 = -t168 * t358 + t331 * t354;
t362 = qJD(4) - qJD(3);
t363 = t362 * t148;
t367 = t362 * t146;
t369 = -pkin(12) * t151 * t362 + t367 * pkin(14) + t363 * pkin(15);
t372 = 0.1e1 / t154;
t375 = 0.1e1 / (t160 * t372 + 0.1e1);
t381 = -pkin(12) * t157 * t362 - t363 * pkin(14) + t367 * pkin(15);
t385 = -t2 + t293 - t307 + qJD(3) - t369 / t153 * t375 - t381 * t159 * t372 * t375 + qJD(4);
t389 = -t2 + t293 - t307 + qJD(3) + qJD(4);
t390 = cos(qJ(4));
t392 = sin(qJ(4));
t394 = t390 * t342 + t392 * t346;
t398 = -t392 * t342 + t390 * t346;
t404 = sin(qJ(5));
t407 = -t146 * t394 + t148 * t398;
t409 = cos(qJ(5));
t411 = -t2 + t293 - t307 + 0.2e1 * qJD(4);
t413 = -t404 * t407 + t409 * t411;
t416 = t404 * t411 + t409 * t407;
t419 = t148 * t394;
t420 = t146 * t398;
t421 = t419 + t420 + qJD(5);
t432 = -t314 + t315;
t437 = t331 * t327;
t438 = -t168 * t337;
t440 = t385 * t162 + t437 + t438;
t441 = t440 ^ 2;
t442 = t9 * pkin(16);
t443 = t140 * pkin(23);
t444 = t346 * pkin(9);
t445 = t358 * pkin(12);
t447 = -t360 * t162 - t442 - t443 - t444 - t445;
t448 = t447 ^ 2;
t454 = -t328 - t338 + t163 * (t153 * t381 - t159 * t369);
t455 = t454 ^ 2;
t460 = t20 * pkin(22) - t442;
t463 = t133 * t10 + t128 * t13;
t465 = t463 * pkin(22) + t97 + t98;
t467 = 0.1e1 / pkin(17);
t471 = t128 * t10 - t133 * t13;
t473 = -t471 * pkin(22) + pkin(24) + t84 - t91;
t476 = t463 * t465 * t467 - t467 * t471 * t473;
t482 = -t463 * t467 * t473 - t465 * t467 * t471;
t484 = t476 * t15 + t482 * t20;
t488 = -t482 * t15 + t476 * t20;
t492 = t398 * pkin(10);
t493 = -t442 - t443 - t444 - t492;
t495 = -t419 - t420;
t499 = t2 * pkin(16) * (-qJD(2) * mrSges(3,2) + t2 * mrSges(3,3)) - t9 * pkin(16) * (-t20 * mrSges(9,1) + t15 * mrSges(9,2)) + t9 * pkin(16) * (qJD(2) * mrSges(3,1) + t9 * mrSges(3,3)) - t9 * pkin(16) * (-t140 * mrSges(4,1) + t135 * mrSges(4,2)) + t339 * (t385 * mrSges(15,1) - t360 * mrSges(15,3)) + t389 * (Ifges(6,5) * t394 + Ifges(6,6) * t398 + Ifges(6,3) * t389) / 0.2e1 + t413 * (Ifges(8,4) * t416 + Ifges(8,2) * t413 + Ifges(8,6) * t421) / 0.2e1 + t421 * (Ifges(8,5) * t416 + Ifges(8,6) * t413 + Ifges(8,3) * t421) / 0.2e1 + t432 * (t316 * mrSges(5,1) - t342 * mrSges(5,3)) + m(16) * (t441 + t448 + t455) / 0.2e1 + t460 * (-t488 * mrSges(10,1) + t484 * mrSges(10,2)) + t493 * (-t495 * mrSges(7,1) + t407 * mrSges(7,2));
t500 = t177 ^ 2;
t501 = t179 + t180;
t502 = t501 ^ 2;
t503 = t8 ^ 2;
t504 = qJD(1) ^ 2;
t506 = pkin(16) ^ 2;
t507 = t503 * t504 * t506;
t511 = -t442 - t443 - t444;
t518 = -t10 * t172 + t17 * t175;
t520 = t13 * t172;
t521 = t11 * t175;
t523 = t2 * pkin(22) - t520 - t521;
t525 = t476 * t518 + t482 * t523;
t526 = t525 ^ 2;
t529 = t476 * t523 - t482 * t518;
t530 = t529 ^ 2;
t531 = t460 ^ 2;
t535 = t140 * pkin(24);
t538 = -t88 * t135 + t81 * t140;
t540 = -t538 * pkin(19) - t442 - t535;
t545 = t30 * t79 * t80 + t47 * t80 * t87;
t547 = t545 * pkin(19) - pkin(9) + t32 + t35;
t549 = 0.1e1 / pkin(11);
t555 = -t30 * t80 * t87 + t47 * t79 * t80;
t557 = t555 * pkin(19) + t53 - t54;
t560 = -t545 * t547 * t549 - t549 * t555 * t557;
t563 = t81 * t135 + t88 * t140;
t569 = t545 * t549 * t557 - t547 * t549 * t555;
t571 = t569 * t538 + t560 * t563;
t575 = t560 * t538 - t569 * t563;
t588 = t390 * t312 + t392 * t318;
t589 = t146 * t588;
t590 = t392 * t312;
t591 = t390 * t318;
t593 = t389 * pkin(10) - t590 + t591;
t594 = t148 * t593;
t595 = -t589 + t594;
t596 = t595 ^ 2;
t599 = -t146 * t593 - t148 * t588;
t600 = t599 ^ 2;
t601 = t493 ^ 2;
t605 = t437 + t438;
t606 = t605 ^ 2;
t607 = t339 ^ 2;
t608 = -t442 - t443 - t444 - t445;
t609 = t608 ^ 2;
t613 = t588 ^ 2;
t614 = -t590 + t591;
t615 = t614 ^ 2;
t616 = t511 ^ 2;
t621 = t411 * pkin(13) - t589 + t594;
t624 = -t407 * pkin(13) - t442 - t443 - t444 - t492;
t626 = t404 * t624 + t409 * t621;
t633 = t284 * t118;
t635 = -t304 * t118;
t637 = t633 * t10 + t635 * t13;
t641 = -t635 * t10 + t633 * t13;
t648 = t473 ^ 2;
t649 = t465 ^ 2;
t650 = 0.1e1 / t649;
t653 = 0.1e1 / (t648 * t650 + 0.1e1);
t663 = -t2 + t293 - t307 - (-(t641 * t10 - t637 * t13) * pkin(22) + t252 - t253) / t465 * t653 + ((t637 * t10 + t641 * t13) * pkin(22) + t244 + t251) * t473 * t650 * t653;
t680 = m(4) * (t500 + t502 + t507) / 0.2e1 + t511 * (-t358 * mrSges(14,1) + t354 * mrSges(14,2)) + m(10) * (t526 + t530 + t531) / 0.2e1 + t540 * (-t575 * mrSges(12,1) + t571 * mrSges(12,2)) - t2 * (-Ifges(9,3) * qJD(1) * t1 + Ifges(9,5) * t15 + Ifges(9,6) * t20) / 0.2e1 + m(7) * (t596 + t600 + t601) / 0.2e1 + m(15) * (t606 + t607 + t609) / 0.2e1 + m(6) * (t613 + t615 + t616) / 0.2e1 + t626 * (-t421 * mrSges(8,2) + t413 * mrSges(8,3)) + t484 * (Ifges(10,1) * t484 + Ifges(10,4) * t488 + Ifges(10,5) * t663) / 0.2e1 + t488 * (Ifges(10,4) * t484 + Ifges(10,2) * t488 + Ifges(10,6) * t663) / 0.2e1 + t663 * (Ifges(10,5) * t484 + Ifges(10,6) * t488 + Ifges(10,3) * t663) / 0.2e1;
t728 = t308 * pkin(24) + t179 + t180;
t730 = t81 * t177 + t88 * t728;
t732 = t88 * t177;
t733 = t81 * t728;
t736 = t87 ^ 2;
t737 = t79 ^ 2;
t738 = 0.1e1 / t737;
t741 = 0.1e1 / (t736 * t738 + 0.1e1);
t746 = -t2 + t293 - t307 + t242 / t79 * t741 - t249 * t87 * t738 * t741;
t748 = t746 * pkin(19) - t732 + t733;
t750 = t560 * t730 + t569 * t748;
t763 = t557 ^ 2;
t764 = t547 ^ 2;
t765 = 0.1e1 / t764;
t768 = 0.1e1 / (t763 * t765 + 0.1e1);
t782 = -t2 + t293 - t307 - ((-t242 * t30 * t80 + t249 * t47 * t80 + t181 * t81 + t183 * t88) * pkin(19) + t199 - t201) / t547 * t768 + ((t242 * t47 * t80 + t249 * t30 * t80 + t181 * t88 - t183 * t81) * pkin(19) - t184 - t186) * t557 * t765 * t768 + qJD(3);
t786 = t1 ^ 2;
t792 = t518 ^ 2;
t793 = -t520 - t521;
t794 = t793 ^ 2;
t802 = t308 * (Ifges(4,5) * t135 + Ifges(4,6) * t140 + Ifges(4,3) * t308) / 0.2e1 + t15 * (-Ifges(9,5) * qJD(1) * t1 + Ifges(9,1) * t15 + Ifges(9,4) * t20) / 0.2e1 + t20 * (-Ifges(9,6) * qJD(1) * t1 + Ifges(9,4) * t15 + Ifges(9,2) * t20) / 0.2e1 + qJD(2) * (-Ifges(3,5) * qJD(1) * t8 + Ifges(3,6) * qJD(1) * t1 + Ifges(3,3) * qJD(2)) / 0.2e1 + t511 * (-t352 * mrSges(13,1) + t348 * mrSges(13,2)) + t525 * (-t663 * mrSges(10,2) + t488 * mrSges(10,3)) + t529 * (t663 * mrSges(10,1) - t484 * mrSges(10,3)) + t312 * (-t316 * mrSges(5,2) + t346 * mrSges(5,3)) + t750 * (-t782 * mrSges(12,2) + t575 * mrSges(12,3)) + m(3) * (t504 * t506 * t786 + t507) / 0.2e1 + m(9) * (t792 + t794 + t507) / 0.2e1 + t518 * (t2 * mrSges(9,2) + t20 * mrSges(9,3));
t807 = t327 ^ 2;
t808 = t332 + t333;
t809 = t808 ^ 2;
t824 = -t168 * t354 - t331 * t358;
t870 = t793 * (-t2 * mrSges(9,1) - t15 * mrSges(9,3)) + m(14) * (t807 + t809 + t616) / 0.2e1 + t320 * (-t335 * mrSges(13,2) + t352 * mrSges(13,3)) + t325 * (t335 * mrSges(13,1) - t348 * mrSges(13,3)) + t440 * (-t824 * mrSges(16,2) + t385 * mrSges(16,3)) + t447 * (t824 * mrSges(16,1) - t360 * mrSges(16,3)) + t614 * (t389 * mrSges(6,1) - t394 * mrSges(6,3)) + t454 * (-t385 * mrSges(16,1) + t360 * mrSges(16,2)) + t360 * (Ifges(16,1) * t360 + Ifges(16,4) * t385 + Ifges(16,5) * t824) / 0.2e1 + t385 * (Ifges(16,4) * t360 + Ifges(16,2) * t385 + Ifges(16,6) * t824) / 0.2e1 + t335 * (Ifges(14,5) * t354 + Ifges(14,6) * t358 + Ifges(14,3) * t335) / 0.2e1 + t348 * (Ifges(13,1) * t348 + Ifges(13,4) * t352 + Ifges(13,5) * t335) / 0.2e1 + t352 * (Ifges(13,4) * t348 + Ifges(13,2) * t352 + Ifges(13,6) * t335) / 0.2e1;
t927 = -t442 - t535;
t932 = -t442 - t443;
t943 = t342 * (Ifges(5,1) * t342 + Ifges(5,4) * t346 + Ifges(5,5) * t316) / 0.2e1 + t346 * (Ifges(5,4) * t342 + Ifges(5,2) * t346 + Ifges(5,6) * t316) / 0.2e1 + t335 * (Ifges(13,5) * t348 + Ifges(13,6) * t352 + Ifges(13,3) * t335) / 0.2e1 + t571 * (Ifges(12,1) * t571 + Ifges(12,4) * t575 + Ifges(12,5) * t782) / 0.2e1 + t575 * (Ifges(12,4) * t571 + Ifges(12,2) * t575 + Ifges(12,6) * t782) / 0.2e1 + t782 * (Ifges(12,5) * t571 + Ifges(12,6) * t575 + Ifges(12,3) * t782) / 0.2e1 + t563 * (Ifges(11,1) * t563 + Ifges(11,4) * t538 + Ifges(11,5) * t746) / 0.2e1 + t538 * (Ifges(11,4) * t563 + Ifges(11,2) * t538 + Ifges(11,6) * t746) / 0.2e1 + t746 * (Ifges(11,5) * t563 + Ifges(11,6) * t538 + Ifges(11,3) * t746) / 0.2e1 + t927 * (-t538 * mrSges(11,1) + t563 * mrSges(11,2)) + t932 * (-t346 * mrSges(5,1) + t342 * mrSges(5,2)) + t416 * (Ifges(8,1) * t416 + Ifges(8,4) * t413 + Ifges(8,5) * t421) / 0.2e1;
t948 = t626 ^ 2;
t951 = -t404 * t621 + t409 * t624;
t952 = t951 ^ 2;
t1006 = -t599 * (-t413 * mrSges(8,1) + t416 * mrSges(8,2)) + m(8) * (t948 + t952 + t600) / 0.2e1 + t595 * (-t411 * mrSges(7,2) + t495 * mrSges(7,3)) + t599 * (t411 * mrSges(7,1) - t407 * mrSges(7,3)) + t588 * (-t389 * mrSges(6,2) + t398 * mrSges(6,3)) + t407 * (Ifges(7,1) * t407 + Ifges(7,4) * t495 + Ifges(7,5) * t411) / 0.2e1 + t495 * (Ifges(7,4) * t407 + Ifges(7,2) * t495 + Ifges(7,6) * t411) / 0.2e1 + t398 * (Ifges(6,4) * t394 + Ifges(6,2) * t398 + Ifges(6,6) * t389) / 0.2e1 + t824 * (Ifges(16,5) * t360 + Ifges(16,6) * t385 + Ifges(16,3) * t824) / 0.2e1 + t605 * (-t385 * mrSges(15,2) - t824 * mrSges(15,3)) + t394 * (Ifges(6,1) * t394 + Ifges(6,4) * t398 + Ifges(6,5) * t389) / 0.2e1 + t951 * (t421 * mrSges(8,1) - t416 * mrSges(8,3));
t1040 = t750 ^ 2;
t1043 = t560 * t748 - t569 * t730;
t1044 = t1043 ^ 2;
t1045 = t540 ^ 2;
t1053 = -t732 + t733;
t1062 = t312 ^ 2;
t1063 = t432 ^ 2;
t1064 = t932 ^ 2;
t1072 = t360 * (Ifges(15,1) * t360 - Ifges(15,4) * t824 + Ifges(15,5) * t385) / 0.2e1 - t824 * (Ifges(15,4) * t360 - Ifges(15,2) * t824 + Ifges(15,6) * t385) / 0.2e1 + t385 * (Ifges(15,5) * t360 - Ifges(15,6) * t824 + Ifges(15,3) * t385) / 0.2e1 + t411 * (Ifges(7,5) * t407 + Ifges(7,6) * t495 + Ifges(7,3) * t411) / 0.2e1 + t511 * (-t398 * mrSges(6,1) + t394 * mrSges(6,2)) + t608 * (t824 * mrSges(15,1) + t360 * mrSges(15,2)) + m(12) * (t1040 + t1044 + t1045) / 0.2e1 + t730 * (-t746 * mrSges(11,2) + t538 * mrSges(11,3)) + t1053 * (t746 * mrSges(11,1) - t563 * mrSges(11,3)) + t808 * (t335 * mrSges(14,1) - t354 * mrSges(14,3)) + m(5) * (t1062 + t1063 + t1064) / 0.2e1 + t177 * (-t308 * mrSges(4,2) + t140 * mrSges(4,3));
t1073 = t730 ^ 2;
t1074 = t1053 ^ 2;
t1075 = t927 ^ 2;
t1135 = t320 ^ 2;
t1136 = t325 ^ 2;
t1144 = m(11) * (t1073 + t1074 + t1075) / 0.2e1 + t2 * (-Ifges(3,4) * qJD(1) * t8 + Ifges(3,2) * qJD(1) * t1 + Ifges(3,6) * qJD(2)) / 0.2e1 - t9 * (-Ifges(3,1) * qJD(1) * t8 + Ifges(3,4) * qJD(1) * t1 + Ifges(3,5) * qJD(2)) / 0.2e1 + t354 * (Ifges(14,1) * t354 + Ifges(14,4) * t358 + Ifges(14,5) * t335) / 0.2e1 + t358 * (Ifges(14,4) * t354 + Ifges(14,2) * t358 + Ifges(14,6) * t335) / 0.2e1 + t316 * (Ifges(5,5) * t342 + Ifges(5,6) * t346 + Ifges(5,3) * t316) / 0.2e1 + t504 * Ifges(2,3) / 0.2e1 + t1043 * (t782 * mrSges(12,1) - t571 * mrSges(12,3)) + t135 * (Ifges(4,1) * t135 + Ifges(4,4) * t140 + Ifges(4,5) * t308) / 0.2e1 + t140 * (Ifges(4,4) * t135 + Ifges(4,2) * t140 + Ifges(4,6) * t308) / 0.2e1 + t501 * (t308 * mrSges(4,1) - t135 * mrSges(4,3)) + m(13) * (t1135 + t1136 + t616) / 0.2e1 + t327 * (-t335 * mrSges(14,2) + t358 * mrSges(14,3));
t1147 = t499 + t680 + t802 + t870 + t943 + t1006 + t1072 + t1144;
T = t1147;
