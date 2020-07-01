% Calculate kinetic energy for
% KAS5m7DE2
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
% Datum: 2020-06-03 15:49
% Revision: caa0dbda1e8a16d11faaa29ba3bbef6afcd619f7 (2020-05-25)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = KAS5m7DE2_energykin_fixb_slag_vp2(qJ, qJD, ...
  pkin, m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(5,1),zeros(24,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE2_energykin_fixb_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [5 1]), ...
  'KAS5m7DE2_energykin_fixb_slag_vp2: qJD has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE2_energykin_fixb_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE2_energykin_fixb_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE2_energykin_fixb_slag_vp2: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7DE2_energykin_fixb_slag_vp2: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_fixb_linkframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-25 12:45:15
% EndTime: 2020-05-25 13:07:01
% DurationCPUTime: 1230.86s
% Computational Cost: add. (78688955->527), mult. (103904693->821), div. (1437062->24), fcn. (40937380->41), ass. (0->331)
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
t91 = 0.2e1 * t69 * t83 + 0.2e1 * t72 * t75;
t93 = t91 * t86 + pkin(24) + t65 - t66;
t94 = atan2(t88, t93);
t95 = 0.1e1 / pkin(22);
t96 = t88 * t95;
t97 = sin(pkin(6));
t99 = -t93 * t95;
t100 = cos(pkin(6));
t102 = t99 * t100 + t96 * t97;
t106 = t96 * t100 - t99 * t97;
t110 = -(-t102 * t100 + t106 * t97) * pkin(22) + pkin(24) + t65 - t66;
t115 = (t106 * t100 + t102 * t97) * pkin(22) + t55 + t62;
t116 = atan2(t110, t115);
t117 = t94 - t116;
t118 = cos(t117);
t119 = cos(qJ(2));
t120 = t97 * t119;
t123 = t120 * qJD(1) + t100 * qJD(2);
t125 = sin(t117);
t126 = t100 * t119;
t129 = t126 * qJD(1) - t97 * qJD(2);
t131 = t118 * t123 + t125 * t129;
t134 = -t118 * t129 + t125 * t123;
t137 = sin(qJ(2));
t138 = t137 * qJD(1);
t139 = qJD(3) * t2;
t141 = qJD(3) * t1;
t142 = t141 * t3;
t144 = qJD(3) * t14 * pkin(12);
t145 = -t142 - t144;
t157 = t139 * t3;
t159 = qJD(3) * t6 * pkin(12);
t160 = -t157 + t159;
t165 = 0.2e1 * t160 * t14 * pkin(12) + 0.2e1 * t145 * t6 * pkin(12) - 0.2e1 * t17 * qJD(3) * t24 - 0.2e1 * t8 * qJD(3) * t25 + 0.2e1 * t26 * qJD(3) * t7 + 0.2e1 * t1 * t20 * t139;
t168 = 0.1e1 / t37;
t172 = 0.4e1 * t8 * t145 + 0.4e1 * t26 * t160;
t176 = -0.2e1 * t29 * t165 + 0.2e1 * t9 * t172;
t181 = t33 ^ 2;
t182 = 0.1e1 / t181;
t185 = t142 + t144 + (-t26 * t168 * t176 + 0.2e1 * t145 * t29 - 0.2e1 * t160 * t37 + 0.2e1 * t8 * t165) * t40 - 0.2e1 * t39 * t182 * t172;
t198 = t157 - t159 + (t8 * t168 * t176 + 0.2e1 * t145 * t37 + 0.2e1 * t29 * t160 + 0.2e1 * t165 * t26) * t40 - 0.2e1 * t46 * t182 * t172;
t200 = t1 * t185 + t139 * t42 - t141 * t48 + t2 * t198;
t201 = t200 * t51;
t202 = t201 * t54;
t207 = -t1 * t198 - t139 * t48 - t141 * t42 + t2 * t185;
t208 = t207 * t51;
t209 = t208 * t61;
t210 = t208 * t54;
t211 = t201 * t61;
t212 = t210 - t211;
t215 = t202 + t209;
t221 = -0.2e1 * t72 * t212 + 0.2e1 * t69 * t215;
t225 = 0.1e1 / t83;
t229 = -0.4e1 * t72 * t212 + 0.4e1 * t69 * t215;
t233 = -0.2e1 * t75 * t221 + 0.2e1 * t63 * t229;
t238 = t79 ^ 2;
t239 = 0.1e1 / t238;
t242 = -t202 - t209 - (t72 * t225 * t233 - 0.2e1 * t212 * t83 - 0.2e1 * t75 * t215 - 0.2e1 * t221 * t69) * t86 + 0.2e1 * t85 * t239 * t229;
t245 = t88 ^ 2;
t246 = t93 ^ 2;
t247 = 0.1e1 / t246;
t250 = 0.1e1 / (t245 * t247 + 0.1e1);
t251 = t242 / t93 * t250;
t262 = t210 - t211 + (t69 * t225 * t233 - 0.2e1 * t212 * t75 + 0.2e1 * t215 * t83 + 0.2e1 * t72 * t221) * t86 - 0.2e1 * t91 * t239 * t229;
t265 = t262 * t88 * t247 * t250;
t266 = t242 * t95;
t268 = -t262 * t95;
t270 = t268 * t100 + t266 * t97;
t274 = t266 * t100 - t268 * t97;
t281 = t110 ^ 2;
t282 = t115 ^ 2;
t283 = 0.1e1 / t282;
t286 = 0.1e1 / (t281 * t283 + 0.1e1);
t296 = -t138 + t251 - t265 - (-(-t270 * t100 + t274 * t97) * pkin(22) + t210 - t211) / t115 * t286 + ((t274 * t100 + t270 * t97) * pkin(22) + t202 + t209) * t110 * t283 * t286;
t305 = qJ(4) - qJ(3) + pkin(4);
t306 = sin(t305);
t307 = cos(qJ(4));
t308 = t94 + pkin(6);
t309 = sin(t308);
t310 = t309 * t119;
t312 = cos(t308);
t314 = -t310 * qJD(1) - t312 * qJD(2);
t316 = t312 * t119;
t319 = -t316 * qJD(1) + t309 * qJD(2);
t321 = t1 * t319 + t2 * t314;
t323 = sin(qJ(4));
t326 = -t1 * t314 + t2 * t319;
t328 = t307 * t321 + t323 * t326;
t330 = cos(t305);
t333 = t307 * t326 - t323 * t321;
t335 = -t306 * t328 + t330 * t333;
t337 = t330 * t328;
t338 = t306 * t333;
t339 = -t337 - t338;
t342 = -t138 + t251 - t265 + (2 * qJD(4));
t356 = -t138 + t251 - t265 + qJD(3) + qJD(4);
t363 = pkin(3) + qJ(3) - qJ(4);
t364 = sin(t363);
t367 = t364 * pkin(12) - t330 * pkin(14) + t306 * pkin(15);
t370 = cos(t363);
t372 = -t370 * pkin(12) - t306 * pkin(14) - t330 * pkin(15) - pkin(10);
t373 = atan2(t367, t372);
t374 = t373 + pkin(3) + qJ(3) - qJ(4);
t375 = cos(t374);
t376 = sin(pkin(3));
t379 = t1 * t321 - t2 * t326;
t381 = cos(pkin(3));
t384 = t1 * t326 + t2 * t321;
t386 = t376 * t379 - t381 * t384;
t388 = sin(t374);
t391 = t376 * t384 + t381 * t379;
t393 = -t375 * t386 + t388 * t391;
t396 = t375 * t391 + t388 * t386;
t399 = qJD(4) - qJD(3);
t400 = t399 * t330;
t404 = t399 * t306;
t406 = -t399 * t370 * pkin(12) + t404 * pkin(14) + t400 * pkin(15);
t409 = t367 ^ 2;
t410 = t372 ^ 2;
t411 = 0.1e1 / t410;
t414 = 0.1e1 / (t409 * t411 + 0.1e1);
t420 = -t399 * t364 * pkin(12) - t400 * pkin(14) + t404 * pkin(15);
t424 = -t138 + t251 - t265 + qJD(3) - t406 / t372 * t414 + t420 * t367 * t411 * t414 + qJD(4);
t450 = -t138 + t251 - t265 + qJD(3);
t454 = t131 * (Ifges(10,4) * t134 + Ifges(10,2) * t131 + Ifges(10,6) * t296) + t296 * (Ifges(10,5) * t134 + Ifges(10,6) * t131 + Ifges(10,3) * t296) + t335 * (Ifges(7,1) * t335 + Ifges(7,4) * t339 + Ifges(7,5) * t342) + t339 * (Ifges(7,4) * t335 + Ifges(7,2) * t339 + Ifges(7,6) * t342) + t342 * (Ifges(7,5) * t335 + Ifges(7,6) * t339 + Ifges(7,3) * t342) + t356 * (Ifges(6,5) * t328 + Ifges(6,6) * t333 + Ifges(6,3) * t356) + t393 * (Ifges(15,4) * t396 + Ifges(15,2) * t393 + Ifges(15,6) * t424) + t424 * (Ifges(15,5) * t396 + Ifges(15,6) * t393 + Ifges(15,3) * t424) + t396 * (Ifges(16,1) * t396 + Ifges(16,4) * t424 - Ifges(16,5) * t393) + t424 * (Ifges(16,4) * t396 + Ifges(16,2) * t424 - Ifges(16,6) * t393) - t393 * (Ifges(16,5) * t396 + Ifges(16,6) * t424 - Ifges(16,3) * t393) + t326 * (Ifges(5,4) * t321 + Ifges(5,2) * t326 + Ifges(5,6) * t450);
t463 = -t138 + t251 - t265;
t476 = t138 * pkin(16) + qJD(2) * pkin(21);
t478 = qJD(1) * pkin(21);
t480 = t309 * t476 - t316 * t478;
t482 = t312 * t476;
t483 = t310 * t478;
t485 = t463 * pkin(23) + t482 + t483;
t487 = t1 * t485 + t2 * t480;
t489 = t1 * t480;
t490 = t2 * t485;
t492 = t450 * pkin(9) - t489 + t490;
t494 = t1 * t487 - t2 * t492;
t498 = t1 * t492 + t2 * t487;
t500 = t376 * t494 - t381 * t498;
t501 = t388 * t500;
t502 = t381 * t494;
t503 = t376 * t498;
t505 = -t138 + t251 - t265 + 0.2e1 * qJD(3);
t507 = t505 * pkin(12) + t502 + t503;
t508 = t375 * t507;
t510 = sqrt(t410 + t409);
t512 = t424 * t510 + t501 + t508;
t513 = t512 ^ 2;
t514 = t119 * qJD(1);
t515 = t514 * pkin(16);
t516 = t319 * pkin(23);
t517 = t326 * pkin(9);
t518 = t391 * pkin(12);
t520 = -t396 * t510 - t515 - t516 - t517 - t518;
t521 = t520 ^ 2;
t522 = t375 * t500;
t523 = t388 * t507;
t530 = t522 - t523 + 0.1e1 / t510 * (t367 * t406 + t372 * t420);
t531 = t530 ^ 2;
t535 = t501 + t508;
t542 = t307 * t487 + t323 * t492;
t543 = t306 * t542;
t544 = t323 * t487;
t545 = t307 * t492;
t547 = t356 * pkin(10) - t544 + t545;
t548 = t330 * t547;
t549 = -t543 + t548;
t550 = t549 ^ 2;
t553 = -t306 * t547 - t330 * t542;
t554 = t553 ^ 2;
t555 = t333 * pkin(10);
t556 = -t515 - t516 - t517 - t555;
t557 = t556 ^ 2;
t561 = -t522 + t523;
t566 = -t544 + t545;
t571 = -t515 - t516 - t517 - t518;
t586 = (t1 * t58 * t51 - t2 * t50 * t51) * pkin(19) + t24 - t25;
t593 = (t1 * t50 * t51 + t2 * t58 * t51) * pkin(19) - pkin(9) + t4 + t7;
t594 = atan2(t586, t593);
t595 = atan2(-t50, t58);
t596 = -t594 + qJ(3) + t595;
t597 = cos(t596);
t598 = cos(t595);
t600 = sin(t595);
t602 = t463 * pkin(24) + t482 + t483;
t604 = t598 * t480 - t600 * t602;
t606 = sin(t596);
t607 = t600 * t480;
t608 = t598 * t602;
t611 = t50 ^ 2;
t612 = t58 ^ 2;
t613 = 0.1e1 / t612;
t616 = 0.1e1 / (t611 * t613 + 0.1e1);
t621 = -t138 + t251 - t265 + t200 / t58 * t616 - t207 * t50 * t613 * t616;
t623 = t621 * pkin(19) + t607 + t608;
t625 = -t597 * t604 - t606 * t623;
t626 = t625 ^ 2;
t629 = -t597 * t623 + t606 * t604;
t630 = t629 ^ 2;
t631 = t319 * pkin(24);
t634 = t600 * t314 + t598 * t319;
t636 = -t634 * pkin(19) - t515 - t631;
t637 = t636 ^ 2;
t645 = t450 * (Ifges(5,5) * t321 + Ifges(5,6) * t326 + Ifges(5,3) * t450) / 0.2e1 + t314 * (Ifges(4,1) * t314 + Ifges(4,4) * t319 + Ifges(4,5) * t463) / 0.2e1 + t319 * (Ifges(4,4) * t314 + Ifges(4,2) * t319 + Ifges(4,6) * t463) / 0.2e1 + m(16) * (t513 + t521 + t531) / 0.2e1 + t535 * (-t424 * mrSges(15,2) + t393 * mrSges(15,3)) + m(7) * (t550 + t554 + t557) / 0.2e1 + t561 * (t424 * mrSges(15,1) - t396 * mrSges(15,3)) + t566 * (t356 * mrSges(6,1) - t328 * mrSges(6,3)) + t571 * (-t393 * mrSges(15,1) + t396 * mrSges(15,2)) + t556 * (-t339 * mrSges(7,1) + t335 * mrSges(7,2)) + m(12) * (t626 + t630 + t637) / 0.2e1 + t604 * (-t621 * mrSges(11,2) + t634 * mrSges(11,3));
t647 = t604 ^ 2;
t648 = t607 + t608;
t649 = t648 ^ 2;
t650 = -t515 - t631;
t651 = t650 ^ 2;
t657 = t126 * t478 - t97 * t476;
t659 = t100 * t476;
t660 = t120 * t478;
t662 = t138 * pkin(22) - t659 - t660;
t664 = -t118 * t662 + t125 * t657;
t669 = qJD(1) ^ 2;
t672 = t657 ^ 2;
t673 = -t659 - t660;
t674 = t673 ^ 2;
t675 = t119 ^ 2;
t677 = pkin(16) ^ 2;
t678 = t675 * t669 * t677;
t682 = cos(qJ(5));
t684 = sin(qJ(5));
t686 = t682 * t335 + t684 * t342;
t690 = -t684 * t335 + t682 * t342;
t698 = t542 ^ 2;
t699 = t566 ^ 2;
t700 = -t515 - t516 - t517;
t701 = t700 ^ 2;
t705 = t535 ^ 2;
t706 = t561 ^ 2;
t707 = t571 ^ 2;
t727 = m(11) * (t647 + t649 + t651) / 0.2e1 + t664 * (-t296 * mrSges(10,2) + t131 * mrSges(10,3)) + t669 * Ifges(2,3) / 0.2e1 + m(9) * (t672 + t674 + t678) / 0.2e1 - t553 * (-t690 * mrSges(8,1) + t686 * mrSges(8,2)) + t520 * (-t393 * mrSges(16,1) - t396 * mrSges(16,3)) + m(6) * (t698 + t699 + t701) / 0.2e1 + m(15) * (t705 + t706 + t707) / 0.2e1 + t700 * (-t333 * mrSges(6,1) + t328 * mrSges(6,2)) + t549 * (-t342 * mrSges(7,2) + t339 * mrSges(7,3)) + t553 * (t342 * mrSges(7,1) - t335 * mrSges(7,3)) + t542 * (-t356 * mrSges(6,2) + t333 * mrSges(6,3));
t734 = t598 * t314 - t600 * t319;
t739 = -t515 - t516;
t744 = t664 ^ 2;
t747 = t118 * t657 + t125 * t662;
t748 = t747 ^ 2;
t750 = t129 * pkin(22) - t515;
t751 = t750 ^ 2;
t777 = t480 ^ 2;
t778 = t482 + t483;
t779 = t778 ^ 2;
t785 = -t597 * t734 - t606 * t634;
t789 = -t597 * t634 + t606 * t734;
t801 = -t489 + t490;
t806 = t512 * (t393 * mrSges(16,2) + t424 * mrSges(16,3)) + t650 * (-t634 * mrSges(11,1) + t734 * mrSges(11,2)) + t739 * (-t326 * mrSges(5,1) + t321 * mrSges(5,2)) + m(10) * (t744 + t748 + t751) / 0.2e1 + t750 * (-t131 * mrSges(10,1) + t134 * mrSges(10,2)) + t621 * (Ifges(11,5) * t734 + Ifges(11,6) * t634 + Ifges(11,3) * t621) / 0.2e1 + t321 * (Ifges(5,1) * t321 + Ifges(5,4) * t326 + Ifges(5,5) * t450) / 0.2e1 + t505 * (Ifges(14,5) * t386 + Ifges(14,6) * t391 + Ifges(14,3) * t505) / 0.2e1 + m(4) * (t777 + t779 + t678) / 0.2e1 + t636 * (-t789 * mrSges(12,1) + t785 * mrSges(12,2)) + t648 * (t621 * mrSges(11,1) - t734 * mrSges(11,3)) + t487 * (-t450 * mrSges(5,2) + t326 * mrSges(5,3)) + t801 * (t450 * mrSges(5,1) - t321 * mrSges(5,3));
t813 = t502 + t503;
t876 = t500 * (-t505 * mrSges(14,2) + t391 * mrSges(14,3)) + t813 * (t505 * mrSges(14,1) - t386 * mrSges(14,3)) + t391 * (Ifges(14,4) * t386 + Ifges(14,2) * t391 + Ifges(14,6) * t505) / 0.2e1 + t386 * (Ifges(14,1) * t386 + Ifges(14,4) * t391 + Ifges(14,5) * t505) / 0.2e1 + qJD(2) * (-Ifges(3,5) * t119 * qJD(1) + Ifges(3,6) * t137 * qJD(1) + Ifges(3,3) * qJD(2)) / 0.2e1 + t123 * (-Ifges(9,5) * t137 * qJD(1) + Ifges(9,1) * t123 + Ifges(9,4) * t129) / 0.2e1 + t129 * (-Ifges(9,6) * t137 * qJD(1) + Ifges(9,4) * t123 + Ifges(9,2) * t129) / 0.2e1 + t747 * (t296 * mrSges(10,1) - t134 * mrSges(10,3)) - t514 * pkin(16) * (-t319 * mrSges(4,1) + t314 * mrSges(4,2)) - t514 * pkin(16) * (-t129 * mrSges(9,1) + t123 * mrSges(9,2)) + t138 * pkin(16) * (-qJD(2) * mrSges(3,2) + t138 * mrSges(3,3)) + t514 * pkin(16) * (qJD(2) * mrSges(3,1) + t514 * mrSges(3,3));
t878 = t342 * pkin(13) - t543 + t548;
t881 = -t335 * pkin(13) - t515 - t516 - t517 - t555;
t883 = t682 * t878 + t684 * t881;
t884 = t883 ^ 2;
t887 = t682 * t881 - t684 * t878;
t888 = t887 ^ 2;
t893 = t337 + t338 + qJD(5);
t917 = t586 ^ 2;
t918 = t593 ^ 2;
t919 = 0.1e1 / t918;
t922 = 0.1e1 / (t917 * t919 + 0.1e1);
t936 = -t138 + t251 - t265 - ((t1 * t207 * t51 - t2 * t200 * t51 + t139 * t59 + t141 * t52) * pkin(19) + t157 - t159) / t593 * t922 + ((t1 * t200 * t51 + t2 * t207 * t51 + t139 * t52 - t141 * t59) * pkin(19) - t142 - t144) * t586 * t919 * t922 + qJD(3);
t940 = t137 ^ 2;
t978 = m(8) * (t884 + t888 + t554) / 0.2e1 + t883 * (-t893 * mrSges(8,2) + t690 * mrSges(8,3)) + t887 * (t893 * mrSges(8,1) - t686 * mrSges(8,3)) + t530 * (-t424 * mrSges(16,1) + t396 * mrSges(16,2)) + t629 * (t936 * mrSges(12,1) - t785 * mrSges(12,3)) + m(3) * (t940 * t669 * t677 + t678) / 0.2e1 + t657 * (t138 * mrSges(9,2) + t129 * mrSges(9,3)) + t673 * (-t138 * mrSges(9,1) - t123 * mrSges(9,3)) + t463 * (Ifges(4,5) * t314 + Ifges(4,6) * t319 + Ifges(4,3) * t463) / 0.2e1 + t384 * (Ifges(13,4) * t379 + Ifges(13,2) * t384 + Ifges(13,6) * t505) / 0.2e1 + t505 * (Ifges(13,5) * t379 + Ifges(13,6) * t384 + Ifges(13,3) * t505) / 0.2e1 + t785 * (Ifges(12,1) * t785 + Ifges(12,4) * t789 + Ifges(12,5) * t936) / 0.2e1;
t1008 = t500 ^ 2;
t1009 = t813 ^ 2;
t1013 = t494 ^ 2;
t1014 = t498 ^ 2;
t1044 = t789 * (Ifges(12,4) * t785 + Ifges(12,2) * t789 + Ifges(12,6) * t936) / 0.2e1 + t936 * (Ifges(12,5) * t785 + Ifges(12,6) * t789 + Ifges(12,3) * t936) / 0.2e1 + t734 * (Ifges(11,1) * t734 + Ifges(11,4) * t634 + Ifges(11,5) * t621) / 0.2e1 + t634 * (Ifges(11,4) * t734 + Ifges(11,2) * t634 + Ifges(11,6) * t621) / 0.2e1 + t498 * (t505 * mrSges(13,1) - t379 * mrSges(13,3)) + m(14) * (t1008 + t1009 + t701) / 0.2e1 + m(13) * (t1013 + t1014 + t701) / 0.2e1 + t700 * (-t384 * mrSges(13,1) + t379 * mrSges(13,2)) + t700 * (-t391 * mrSges(14,1) + t386 * mrSges(14,2)) + t893 * (Ifges(8,5) * t686 + Ifges(8,6) * t690 + Ifges(8,3) * t893) / 0.2e1 + t328 * (Ifges(6,1) * t328 + Ifges(6,4) * t333 + Ifges(6,5) * t356) / 0.2e1 + t333 * (Ifges(6,4) * t328 + Ifges(6,2) * t333 + Ifges(6,6) * t356) / 0.2e1;
t1090 = t487 ^ 2;
t1091 = t801 ^ 2;
t1092 = t739 ^ 2;
t1120 = t396 * (Ifges(15,1) * t396 + Ifges(15,4) * t393 + Ifges(15,5) * t424) / 0.2e1 + t379 * (Ifges(13,1) * t379 + Ifges(13,4) * t384 + Ifges(13,5) * t505) / 0.2e1 + t134 * (Ifges(10,1) * t134 + Ifges(10,4) * t131 + Ifges(10,5) * t296) / 0.2e1 - t138 * (-Ifges(9,3) * t137 * qJD(1) + Ifges(9,5) * t123 + Ifges(9,6) * t129) / 0.2e1 - t514 * (-Ifges(3,1) * t119 * qJD(1) + Ifges(3,4) * t137 * qJD(1) + Ifges(3,5) * qJD(2)) / 0.2e1 + t138 * (-Ifges(3,4) * t119 * qJD(1) + Ifges(3,2) * t137 * qJD(1) + Ifges(3,6) * qJD(2)) / 0.2e1 + t494 * (-t505 * mrSges(13,2) + t384 * mrSges(13,3)) + m(5) * (t1090 + t1091 + t1092) / 0.2e1 + t480 * (-t463 * mrSges(4,2) + t319 * mrSges(4,3)) + t778 * (t463 * mrSges(4,1) - t314 * mrSges(4,3)) + t625 * (-t936 * mrSges(12,2) + t789 * mrSges(12,3)) + t686 * (Ifges(8,1) * t686 + Ifges(8,4) * t690 + Ifges(8,5) * t893) / 0.2e1 + t690 * (Ifges(8,4) * t686 + Ifges(8,2) * t690 + Ifges(8,6) * t893) / 0.2e1;
t1123 = t454 / 0.2e1 + t645 + t727 + t806 + t876 + t978 + t1044 + t1120;
T = t1123;
