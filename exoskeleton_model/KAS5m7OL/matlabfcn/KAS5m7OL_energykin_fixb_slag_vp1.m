% Calculate kinetic energy for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% qJD [13x1]
%   Generalized joint velocities
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% m [16x1]
%   mass of all robot links (including the base)
% rSges [16x3]
%   center of mass of all robot links (in body frames)
%   rows: links of the robot (starting with base)
%   columns: x-, y-, z-coordinates
% Icges [16x6]
%   inertia of all robot links about their respective center of mass, in body frames
%   rows: links of the robot (starting with base)
%   columns: xx, yy, zz, xy, xz, yz (see inertiavector2matrix.m)
% 
% Output:
% T [1x1]
%   kinetic energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = KAS5m7OL_energykin_fixb_slag_vp1(qJ, qJD, ...
  pkin, m, rSges, Icges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),zeros(19,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energykin_fixb_slag_vp1: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m7OL_energykin_fixb_slag_vp1: qJD has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energykin_fixb_slag_vp1: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_energykin_fixb_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7OL_energykin_fixb_slag_vp1: rSges has to be [16x3] (double)');
assert(isreal(Icges) && all(size(Icges) == [16 6]), ...
  'KAS5m7OL_energykin_fixb_slag_vp1: Icges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_fixb_worldframe_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:21
% EndTime: 2020-06-30 17:46:22
% DurationCPUTime: 1.06s
% Computational Cost: add. (8100->1253), mult. (8743->1666), div. (0->0), fcn. (8948->28), ass. (0->507)
t1 = sin(qJ(2));
t2 = qJD(3) * t1;
t3 = qJD(9) * t1;
t5 = -qJD(10) * t1 + qJD(1) - t2 - t3;
t6 = cos(qJ(2));
t7 = qJ(3) + qJ(9) + qJ(10);
t8 = cos(t7);
t9 = t6 * t8;
t10 = sin(qJ(1));
t11 = t10 * t1;
t13 = cos(qJ(1));
t14 = sin(t7);
t16 = t11 * t8 - t13 * t14;
t20 = -t11 * t14 - t13 * t8;
t24 = Icges(12,5) * t10 * t6 + Icges(12,1) * t16 + Icges(12,4) * t20;
t26 = t6 * t14;
t31 = Icges(12,6) * t10 * t6 + Icges(12,4) * t16 + Icges(12,2) * t20;
t37 = Icges(12,3) * t10 * t6 + Icges(12,5) * t16 + Icges(12,6) * t20;
t40 = qJD(2) * t13;
t42 = qJD(3) * t10 * t6;
t44 = qJD(9) * t10 * t6;
t47 = qJD(10) * t10 * t6 + t40 + t42 + t44;
t49 = t13 * t1;
t52 = -t10 * t14 - t49 * t8;
t56 = -t10 * t8 + t49 * t14;
t60 = -Icges(12,5) * t13 * t6 + Icges(12,1) * t52 + Icges(12,4) * t56;
t66 = -Icges(12,6) * t13 * t6 + Icges(12,4) * t52 + Icges(12,2) * t56;
t72 = -Icges(12,3) * t13 * t6 + Icges(12,5) * t52 + Icges(12,6) * t56;
t75 = qJD(2) * t10;
t77 = qJD(3) * t13 * t6;
t79 = qJD(9) * t13 * t6;
t82 = -qJD(10) * t13 * t6 + t75 - t77 - t79;
t86 = Icges(12,4) * t6;
t89 = Icges(12,1) * t6 * t8 - Icges(12,5) * t1 - t86 * t14;
t95 = -Icges(12,2) * t6 * t14 - Icges(12,6) * t1 + t86 * t8;
t102 = Icges(12,5) * t6 * t8 - Icges(12,6) * t6 * t14 - Icges(12,3) * t1;
t110 = t10 * t6;
t126 = t40 + t42 + t44;
t127 = qJ(3) + qJ(9);
t128 = cos(t127);
t130 = sin(t127);
t132 = -t11 * t128 + t13 * t130;
t136 = t11 * t130 + t13 * t128;
t140 = Icges(11,5) * t10 * t6 + Icges(11,1) * t132 + Icges(11,4) * t136;
t146 = Icges(11,6) * t10 * t6 + Icges(11,4) * t132 + Icges(11,2) * t136;
t152 = Icges(11,3) * t10 * t6 + Icges(11,5) * t132 + Icges(11,6) * t136;
t158 = t10 * t130 + t49 * t128;
t162 = t10 * t128 - t49 * t130;
t166 = -Icges(11,5) * t13 * t6 + Icges(11,1) * t158 + Icges(11,4) * t162;
t172 = -Icges(11,6) * t13 * t6 + Icges(11,4) * t158 + Icges(11,2) * t162;
t178 = -Icges(11,3) * t13 * t6 + Icges(11,5) * t158 + Icges(11,6) * t162;
t181 = t75 - t77 - t79;
t185 = Icges(11,4) * t6;
t188 = -Icges(11,1) * t6 * t128 - Icges(11,5) * t1 + t185 * t130;
t194 = Icges(11,2) * t6 * t130 - Icges(11,6) * t1 - t185 * t128;
t201 = -Icges(11,5) * t6 * t128 + Icges(11,6) * t6 * t130 - Icges(11,3) * t1;
t204 = qJD(1) - t2 - t3;
t210 = t13 * t6;
t227 = qJD(4) * t10 * t6;
t229 = qJD(11) * t10 * t6;
t230 = t40 + t42 + t227 + t229;
t231 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t232 = cos(t231);
t234 = sin(t231);
t236 = t11 * t232 - t13 * t234;
t240 = -t11 * t234 - t13 * t232;
t244 = Icges(14,5) * t10 * t6 + Icges(14,1) * t236 + Icges(14,4) * t240;
t250 = Icges(14,6) * t10 * t6 + Icges(14,4) * t236 + Icges(14,2) * t240;
t256 = Icges(14,3) * t10 * t6 + Icges(14,5) * t236 + Icges(14,6) * t240;
t262 = -t10 * t234 - t49 * t232;
t266 = -t10 * t232 + t49 * t234;
t270 = -Icges(14,5) * t13 * t6 + Icges(14,1) * t262 + Icges(14,4) * t266;
t276 = -Icges(14,6) * t13 * t6 + Icges(14,4) * t262 + Icges(14,2) * t266;
t282 = -Icges(14,3) * t13 * t6 + Icges(14,5) * t262 + Icges(14,6) * t266;
t286 = qJD(4) * t13 * t6;
t288 = qJD(11) * t13 * t6;
t289 = t75 - t77 - t286 - t288;
t293 = Icges(14,4) * t6;
t296 = Icges(14,1) * t6 * t232 - Icges(14,5) * t1 - t293 * t234;
t302 = -Icges(14,2) * t6 * t234 - Icges(14,6) * t1 + t293 * t232;
t309 = Icges(14,5) * t6 * t232 - Icges(14,6) * t6 * t234 - Icges(14,3) * t1;
t312 = qJD(4) * t1;
t313 = qJD(11) * t1;
t314 = qJD(1) - t2 - t312 - t313;
t335 = t6 * t232;
t337 = t6 * t234;
t354 = qJ(3) + qJ(4) + qJ(11);
t355 = sin(t354);
t357 = cos(t354);
t359 = -t11 * t355 - t13 * t357;
t363 = -t11 * t357 + t13 * t355;
t367 = Icges(13,5) * t10 * t6 + Icges(13,1) * t359 + Icges(13,4) * t363;
t373 = Icges(13,6) * t10 * t6 + Icges(13,4) * t359 + Icges(13,2) * t363;
t379 = Icges(13,3) * t10 * t6 + Icges(13,5) * t359 + Icges(13,6) * t363;
t385 = -t10 * t357 + t49 * t355;
t389 = t10 * t355 + t49 * t357;
t393 = -Icges(13,5) * t13 * t6 + Icges(13,1) * t385 + Icges(13,4) * t389;
t399 = -Icges(13,6) * t13 * t6 + Icges(13,4) * t385 + Icges(13,2) * t389;
t405 = -Icges(13,3) * t13 * t6 + Icges(13,5) * t385 + Icges(13,6) * t389;
t411 = Icges(13,4) * t6;
t414 = -Icges(13,1) * t6 * t355 - Icges(13,5) * t1 - t411 * t357;
t420 = -Icges(13,2) * t6 * t357 - Icges(13,6) * t1 - t411 * t355;
t427 = -Icges(13,5) * t6 * t355 - Icges(13,6) * t6 * t357 - Icges(13,3) * t1;
t450 = t6 * t355;
t452 = t6 * t357;
t487 = -qJD(12) * t1 + qJD(1) - t2 - t312 - t313;
t488 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t489 = sin(t488);
t490 = t6 * t489;
t492 = cos(t488);
t494 = t11 * t489 + t13 * t492;
t498 = t11 * t492 - t13 * t489;
t502 = Icges(15,5) * t10 * t6 + Icges(15,1) * t494 + Icges(15,4) * t498;
t504 = t6 * t492;
t509 = Icges(15,6) * t10 * t6 + Icges(15,4) * t494 + Icges(15,2) * t498;
t515 = Icges(15,3) * t10 * t6 + Icges(15,5) * t494 + Icges(15,6) * t498;
t520 = qJD(12) * t10 * t6 + t227 + t229 + t40 + t42;
t524 = t10 * t492 - t49 * t489;
t528 = -t10 * t489 - t49 * t492;
t532 = -Icges(15,5) * t13 * t6 + Icges(15,1) * t524 + Icges(15,4) * t528;
t538 = -Icges(15,6) * t13 * t6 + Icges(15,4) * t524 + Icges(15,2) * t528;
t544 = -Icges(15,3) * t13 * t6 + Icges(15,5) * t524 + Icges(15,6) * t528;
t549 = -qJD(12) * t13 * t6 - t286 - t288 + t75 - t77;
t553 = Icges(15,4) * t6;
t556 = Icges(15,1) * t6 * t489 - Icges(15,5) * t1 + t553 * t492;
t562 = Icges(15,2) * t6 * t492 - Icges(15,6) * t1 + t553 * t489;
t569 = Icges(15,5) * t6 * t489 + Icges(15,6) * t6 * t492 - Icges(15,3) * t1;
t592 = qJD(1) * t10;
t593 = t592 * pkin(11);
t594 = t1 * pkin(16);
t595 = t75 * t594;
t596 = qJD(1) * t13;
t597 = t6 * pkin(16);
t598 = t596 * t597;
t599 = t75 - t77;
t600 = t599 * t6;
t601 = cos(qJ(3));
t602 = t601 * pkin(18);
t603 = t600 * t602;
t604 = qJD(1) - t2;
t605 = t49 * t602;
t606 = sin(qJ(3));
t607 = t10 * t606;
t608 = t607 * pkin(18);
t609 = t605 + t608;
t610 = t604 * t609;
t611 = t75 - t77 - t286;
t612 = qJ(3) + qJ(4);
t613 = cos(t612);
t614 = t6 * t613;
t616 = sin(t612);
t617 = t6 * t616;
t620 = -t614 * rSges(5,1) + t617 * rSges(5,2) - t1 * rSges(5,3);
t622 = qJD(1) - t2 - t312;
t625 = t10 * t616 + t49 * t613;
t629 = t10 * t613 - t49 * t616;
t632 = t625 * rSges(5,1) + t629 * rSges(5,2) - t210 * rSges(5,3);
t635 = (t611 * t620 - t622 * t632 - t593 - t595 + t598 - t603 - t610) ^ 2;
t636 = t596 * pkin(11);
t637 = t40 * t594;
t638 = t592 * t597;
t639 = t40 + t42;
t640 = t639 * t6;
t641 = t640 * t602;
t642 = t11 * t602;
t643 = t13 * t606;
t644 = t643 * pkin(18);
t645 = -t642 + t644;
t646 = t604 * t645;
t647 = t40 + t42 + t227;
t651 = -t11 * t613 + t13 * t616;
t655 = t11 * t616 + t13 * t613;
t658 = t651 * rSges(5,1) + t655 * rSges(5,2) + t110 * rSges(5,3);
t661 = (-t647 * t620 + t622 * t658 + t636 + t637 + t638 + t641 + t646) ^ 2;
t662 = t13 ^ 2;
t664 = qJD(2) * t662 * t597;
t665 = t10 ^ 2;
t667 = qJD(2) * t665 * t597;
t668 = t639 * t609;
t669 = t599 * t645;
t673 = (-t611 * t658 + t647 * t632 - t664 - t667 + t668 - t669) ^ 2;
t676 = t5 * ((-t1 * t37 + t9 * t24 - t26 * t31) * t47 + (-t1 * t72 - t26 * t66 + t9 * t60) * t82 + (-t1 * t102 - t26 * t95 + t9 * t89) * t5) + t47 * ((t110 * t37 + t16 * t24 + t20 * t31) * t47 + (t110 * t72 + t16 * t60 + t20 * t66) * t82 + (t110 * t102 + t16 * t89 + t20 * t95) * t5) + t126 * ((t110 * t152 + t132 * t140 + t136 * t146) * t126 + (t110 * t178 + t132 * t166 + t136 * t172) * t181 + (t110 * t201 + t132 * t188 + t136 * t194) * t204) + t181 * ((t158 * t140 + t162 * t146 - t210 * t152) * t126 + (t158 * t166 + t162 * t172 - t210 * t178) * t181 + (t158 * t188 + t162 * t194 - t210 * t201) * t204) + t230 * ((t110 * t256 + t236 * t244 + t240 * t250) * t230 + (t110 * t282 + t236 * t270 + t240 * t276) * t289 + (t110 * t309 + t236 * t296 + t240 * t302) * t314) + t289 * ((-t210 * t256 + t262 * t244 + t266 * t250) * t230 + (-t210 * t282 + t262 * t270 + t266 * t276) * t289 + (-t210 * t309 + t262 * t296 + t266 * t302) * t314) + t314 * ((-t1 * t256 + t335 * t244 - t337 * t250) * t230 + (-t1 * t282 + t335 * t270 - t337 * t276) * t289 + (-t1 * t309 + t335 * t296 - t337 * t302) * t314) + t230 * ((t110 * t379 + t359 * t367 + t363 * t373) * t230 + (t110 * t405 + t359 * t393 + t363 * t399) * t289 + (t110 * t427 + t359 * t414 + t363 * t420) * t314) + t289 * ((-t210 * t379 + t385 * t367 + t389 * t373) * t230 + (-t210 * t405 + t385 * t393 + t389 * t399) * t289 + (-t210 * t427 + t385 * t414 + t389 * t420) * t314) + t314 * ((-t1 * t379 - t450 * t367 - t452 * t373) * t230 + (-t1 * t405 - t450 * t393 - t452 * t399) * t289 + (-t1 * t427 - t450 * t414 - t452 * t420) * t314) + t82 * ((-t210 * t37 + t52 * t24 + t56 * t31) * t47 + (-t210 * t72 + t52 * t60 + t56 * t66) * t82 + (-t210 * t102 + t52 * t89 + t56 * t95) * t5) + t487 * ((-t1 * t515 + t490 * t502 + t504 * t509) * t520 + (-t1 * t544 + t490 * t532 + t504 * t538) * t549 + (-t1 * t569 + t490 * t556 + t504 * t562) * t487) + t520 * ((t110 * t515 + t494 * t502 + t498 * t509) * t520 + (t110 * t544 + t494 * t532 + t498 * t538) * t549 + (t110 * t569 + t494 * t556 + t498 * t562) * t487) + m(5) * (t635 + t661 + t673);
t677 = t6 * t601;
t679 = t6 * t606;
t682 = -t677 * rSges(4,1) + t679 * rSges(4,2) - t1 * rSges(4,3);
t685 = t49 * t601 + t607;
t689 = t10 * t601 - t49 * t606;
t692 = t685 * rSges(4,1) + t689 * rSges(4,2) - t210 * rSges(4,3);
t695 = (t599 * t682 - t604 * t692 - t593 - t595 + t598) ^ 2;
t698 = -t11 * t601 + t643;
t702 = t11 * t606 + t13 * t601;
t705 = t698 * rSges(4,1) + t702 * rSges(4,2) + t110 * rSges(4,3);
t708 = (t604 * t705 - t639 * t682 + t636 + t637 + t638) ^ 2;
t712 = (-t599 * t705 + t639 * t692 - t664 - t667) ^ 2;
t715 = sin(pkin(3));
t716 = t6 * t715;
t718 = cos(pkin(3));
t719 = t6 * t718;
t722 = t716 * rSges(9,1) + t719 * rSges(9,2) - t1 * rSges(9,3);
t725 = t10 * t718;
t726 = -t49 * t715 + t725;
t730 = -t10 * t715 - t49 * t718;
t733 = t726 * rSges(9,1) + t730 * rSges(9,2) - t210 * rSges(9,3);
t736 = (-qJD(1) * t733 + t75 * t722 - t593 - t595 + t598) ^ 2;
t739 = t13 * t718;
t740 = t11 * t715 + t739;
t744 = t11 * t718 - t13 * t715;
t747 = t740 * rSges(9,1) + t744 * rSges(9,2) + t110 * rSges(9,3);
t750 = (qJD(1) * t747 - t40 * t722 + t636 + t637 + t638) ^ 2;
t754 = (t40 * t733 - t75 * t747 - t664 - t667) ^ 2;
t759 = -t6 * rSges(3,1) + t1 * rSges(3,2);
t764 = t49 * rSges(3,1) + t210 * rSges(3,2) + t10 * rSges(3,3);
t767 = (-qJD(1) * t764 + t75 * t759 - t593) ^ 2;
t772 = -t11 * rSges(3,1) - t110 * rSges(3,2) + t13 * rSges(3,3);
t775 = (qJD(1) * t772 - t40 * t759 + t636) ^ 2;
t779 = (t40 * t764 - t75 * t772) ^ 2;
t782 = qJD(1) ^ 2;
t786 = (-t13 * rSges(2,1) + t10 * rSges(2,2)) ^ 2;
t791 = (t10 * rSges(2,1) + t13 * rSges(2,2)) ^ 2;
t799 = Icges(16,4) * t10 * t6 + Icges(16,1) * t494 - Icges(16,5) * t498;
t805 = Icges(16,2) * t10 * t6 + Icges(16,4) * t494 - Icges(16,6) * t498;
t811 = Icges(16,6) * t10 * t6 + Icges(16,5) * t494 - Icges(16,3) * t498;
t819 = -Icges(16,4) * t13 * t6 + Icges(16,1) * t524 - Icges(16,5) * t528;
t825 = -Icges(16,2) * t13 * t6 + Icges(16,4) * t524 - Icges(16,6) * t528;
t831 = -Icges(16,6) * t13 * t6 + Icges(16,5) * t524 - Icges(16,3) * t528;
t838 = Icges(16,5) * t6;
t840 = Icges(16,1) * t6 * t489 - Icges(16,4) * t1 - t838 * t492;
t847 = Icges(16,4) * t6 * t489 - Icges(16,6) * t6 * t492 - Icges(16,2) * t1;
t853 = -Icges(16,3) * t6 * t492 - Icges(16,6) * t1 + t838 * t489;
t877 = pkin(6) * t613;
t878 = t877 + t602;
t879 = t6 * t878;
t880 = t677 * pkin(18) - t879;
t881 = t611 * t880;
t882 = t49 * t878;
t883 = pkin(6) * t616;
t884 = t606 * pkin(18);
t885 = t883 + t884;
t886 = t10 * t885;
t887 = -t605 - t608 + t882 + t886;
t888 = t622 * t887;
t890 = -pkin(9) * t232 + t602 + t877;
t892 = -t6 * t890 + t879;
t893 = t289 * t892;
t896 = -pkin(9) * t234 + t883 + t884;
t898 = t10 * t896 + t49 * t890 - t882 - t886;
t899 = t314 * t898;
t903 = t490 * rSges(15,1) + t504 * rSges(15,2) - t1 * rSges(15,3);
t908 = t524 * rSges(15,1) + t528 * rSges(15,2) - t210 * rSges(15,3);
t910 = -t487 * t908 + t549 * t903 - t593 - t595 + t598 - t603 - t610 + t881 - t888 + t893 - t899;
t911 = t910 ^ 2;
t912 = t647 * t880;
t913 = t11 * t878;
t914 = t13 * t885;
t915 = t642 - t644 - t913 + t914;
t916 = t622 * t915;
t917 = t230 * t892;
t920 = -t11 * t890 + t13 * t896 + t913 - t914;
t921 = t314 * t920;
t926 = t494 * rSges(15,1) + t498 * rSges(15,2) + t110 * rSges(15,3);
t928 = t487 * t926 - t520 * t903 + t636 + t637 + t638 + t641 + t646 - t912 + t916 - t917 + t921;
t929 = t928 ^ 2;
t930 = t647 * t887;
t931 = t611 * t915;
t932 = t230 * t898;
t933 = t289 * t920;
t937 = (t520 * t908 - t549 * t926 - t664 - t667 + t668 - t669 + t930 - t931 + t932 - t933) ^ 2;
t941 = t492 * qJ(13);
t949 = t490 * rSges(16,1) - t1 * rSges(16,2) - t504 * rSges(16,3);
t954 = t524 * rSges(16,1) - t210 * rSges(16,2) - t528 * rSges(16,3);
t956 = t487 * t528 * qJ(13) - t549 * t6 * t941 - qJD(13) * t498 - t487 * t954 + t549 * t949 - t593 - t595 + t598 - t603 - t610 + t881 - t888 + t893 - t899;
t957 = t956 ^ 2;
t967 = t494 * rSges(16,1) + t110 * rSges(16,2) - t498 * rSges(16,3);
t969 = -t487 * t498 * qJ(13) + t520 * t6 * t941 - qJD(13) * t528 + t487 * t967 - t520 * t949 + t636 + t637 + t638 + t641 + t646 - t912 + t916 - t917 + t921;
t970 = t969 ^ 2;
t979 = t549 * t498 * qJ(13) - t520 * t528 * qJ(13) - qJD(13) * t6 * t492 + t520 * t954 - t549 * t967 - t664 - t667 + t668 - t669 + t930 - t931 + t932 - t933;
t980 = t979 ^ 2;
t1017 = t601 * pkin(19);
t1018 = t600 * t1017;
t1019 = t49 * t1017;
t1020 = t607 * pkin(19);
t1021 = t1019 + t1020;
t1022 = t604 * t1021;
t1025 = pkin(14) * t128 + t1017;
t1027 = t677 * pkin(19) - t6 * t1025;
t1032 = pkin(14) * t130 + t606 * pkin(19);
t1034 = t10 * t1032 + t49 * t1025 - t1019 - t1020;
t1039 = t9 * rSges(12,1) - t26 * rSges(12,2) - t1 * rSges(12,3);
t1044 = t52 * rSges(12,1) + t56 * rSges(12,2) - t210 * rSges(12,3);
t1047 = (t181 * t1027 - t204 * t1034 + t82 * t1039 - t5 * t1044 - t1018 - t1022 - t593 - t595 + t598) ^ 2;
t1048 = t640 * t1017;
t1049 = t11 * t1017;
t1050 = t643 * pkin(19);
t1051 = -t1049 + t1050;
t1052 = t604 * t1051;
t1056 = -t11 * t1025 + t13 * t1032 + t1049 - t1050;
t1062 = t16 * rSges(12,1) + t20 * rSges(12,2) + t110 * rSges(12,3);
t1065 = (-t126 * t1027 - t47 * t1039 + t204 * t1056 + t5 * t1062 + t1048 + t1052 + t636 + t637 + t638) ^ 2;
t1066 = t639 * t1021;
t1067 = t599 * t1051;
t1073 = (t126 * t1034 + t47 * t1044 - t181 * t1056 - t82 * t1062 + t1066 - t1067 - t664 - t667) ^ 2;
t1076 = t716 * pkin(17);
t1078 = t715 * pkin(17);
t1081 = -t725 * pkin(17) + t49 * t1078;
t1085 = -qJD(8) * t13 * t6 + t75;
t1086 = pkin(3) + qJ(8);
t1087 = sin(t1086);
t1088 = t6 * t1087;
t1090 = cos(t1086);
t1091 = t6 * t1090;
t1094 = t1088 * rSges(10,1) + t1091 * rSges(10,2) - t1 * rSges(10,3);
t1097 = -qJD(8) * t1 + qJD(1);
t1100 = t10 * t1090 - t49 * t1087;
t1104 = -t10 * t1087 - t49 * t1090;
t1107 = t1100 * rSges(10,1) + t1104 * rSges(10,2) - t210 * rSges(10,3);
t1110 = (-qJD(1) * t1081 - t75 * t1076 + t1085 * t1094 - t1097 * t1107 - t593 - t595 + t598) ^ 2;
t1114 = -t739 * pkin(17) - t11 * t1078;
t1118 = qJD(8) * t10 * t6 + t40;
t1122 = t11 * t1087 + t13 * t1090;
t1126 = -t13 * t1087 + t11 * t1090;
t1129 = t1122 * rSges(10,1) + t1126 * rSges(10,2) + t110 * rSges(10,3);
t1132 = (qJD(1) * t1114 + t40 * t1076 - t1118 * t1094 + t1097 * t1129 + t636 + t637 + t638) ^ 2;
t1138 = (t40 * t1081 - t1085 * t1129 + t1118 * t1107 - t75 * t1114 - t664 - t667) ^ 2;
t1141 = t6 * t128;
t1143 = t6 * t130;
t1146 = -t1141 * rSges(11,1) + t1143 * rSges(11,2) - t1 * rSges(11,3);
t1151 = t158 * rSges(11,1) + t162 * rSges(11,2) - t210 * rSges(11,3);
t1154 = (t181 * t1146 - t204 * t1151 - t1018 - t1022 - t593 - t595 + t598) ^ 2;
t1159 = t132 * rSges(11,1) + t136 * rSges(11,2) + t110 * rSges(11,3);
t1162 = (-t126 * t1146 + t204 * t1159 + t1048 + t1052 + t636 + t637 + t638) ^ 2;
t1166 = (t126 * t1151 - t181 * t1159 + t1066 - t1067 - t664 - t667) ^ 2;
t1170 = qJD(5) * t13 * t6;
t1171 = t75 - t77 - t286 - t1170;
t1172 = qJ(3) + qJ(4) + qJ(5);
t1173 = cos(t1172);
t1175 = pkin(7) * t1173 + t602 + t877;
t1177 = -t6 * t1175 + t879;
t1178 = t1171 * t1177;
t1179 = qJD(5) * t1;
t1180 = qJD(1) - t2 - t312 - t1179;
t1182 = sin(t1172);
t1184 = pkin(7) * t1182 + t883 + t884;
t1186 = t10 * t1184 + t49 * t1175 - t882 - t886;
t1187 = t1180 * t1186;
t1189 = qJD(6) * t13 * t6;
t1190 = t75 - t77 - t286 - t1170 - t1189;
t1192 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t1193 = cos(t1192);
t1194 = t1193 * pkin(10);
t1196 = qJD(6) * t1;
t1197 = qJD(1) - t2 - t312 - t1179 - t1196;
t1199 = sin(t1192);
t1201 = -t10 * t1199 - t49 * t1193;
t1205 = -qJD(7) * t1201 - t1170 - t1189 - t286 + t75 - t77;
t1206 = t6 * t1199;
t1207 = cos(qJ(7));
t1209 = sin(qJ(7));
t1211 = -t1 * t1209 + t1206 * t1207;
t1215 = -t1 * t1207 - t1206 * t1209;
t1217 = t6 * t1193;
t1219 = t1211 * rSges(8,1) + t1215 * rSges(8,2) - t1217 * rSges(8,3);
t1223 = -qJD(7) * t6 * t1193 + qJD(1) - t1179 - t1196 - t2 - t312;
t1226 = t10 * t1193 - t49 * t1199;
t1229 = t1226 * t1207 - t210 * t1209;
t1233 = -t210 * t1207 - t1226 * t1209;
t1236 = t1229 * rSges(8,1) + t1233 * rSges(8,2) - t1201 * rSges(8,3);
t1238 = t1197 * t1201 * pkin(10) - t1190 * t6 * t1194 + t1205 * t1219 - t1223 * t1236 + t1178 - t1187 - t593 - t595 + t598 - t603 - t610 + t881 - t888;
t1239 = t1238 ^ 2;
t1241 = qJD(5) * t10 * t6;
t1242 = t40 + t42 + t227 + t1241;
t1243 = t1242 * t1177;
t1246 = -t11 * t1175 + t13 * t1184 + t913 - t914;
t1247 = t1180 * t1246;
t1249 = qJD(6) * t10 * t6;
t1250 = t40 + t42 + t227 + t1241 + t1249;
t1255 = t11 * t1193 - t13 * t1199;
t1259 = -qJD(7) * t1255 + t1241 + t1249 + t227 + t40 + t42;
t1263 = t11 * t1199 + t13 * t1193;
t1266 = t110 * t1209 + t1263 * t1207;
t1270 = t110 * t1207 - t1263 * t1209;
t1273 = t1266 * rSges(8,1) + t1270 * rSges(8,2) - t1255 * rSges(8,3);
t1275 = -t1197 * t1255 * pkin(10) + t1250 * t6 * t1194 - t1259 * t1219 + t1223 * t1273 - t1243 + t1247 + t636 + t637 + t638 + t641 + t646 - t912 + t916;
t1276 = t1275 ^ 2;
t1277 = t1242 * t1186;
t1278 = t1171 * t1246;
t1285 = t1190 * t1255 * pkin(10) - t1250 * t1201 * pkin(10) - t1205 * t1273 + t1259 * t1236 + t1277 - t1278 - t664 - t667 + t668 - t669 + t930 - t931;
t1286 = t1285 ^ 2;
t1292 = t1206 * rSges(7,1) + t1217 * rSges(7,2) - t1 * rSges(7,3);
t1297 = t1226 * rSges(7,1) + t1201 * rSges(7,2) - t210 * rSges(7,3);
t1299 = t1190 * t1292 - t1197 * t1297 + t1178 - t1187 - t593 - t595 + t598 - t603 - t610 + t881 - t888;
t1300 = t1299 ^ 2;
t1305 = t1263 * rSges(7,1) + t1255 * rSges(7,2) + t110 * rSges(7,3);
t1307 = t1197 * t1305 - t1250 * t1292 - t1243 + t1247 + t636 + t637 + t638 + t641 + t646 - t912 + t916;
t1308 = t1307 ^ 2;
t1312 = (-t1190 * t1305 + t1250 * t1297 + t1277 - t1278 - t664 - t667 + t668 - t669 + t930 - t931) ^ 2;
t1315 = m(4) * (t695 + t708 + t712) + m(9) * (t736 + t750 + t754) + m(3) * (t767 + t775 + t779) + m(2) * (t782 * t786 + t782 * t791) + t520 * ((t110 * t805 + t494 * t799 - t498 * t811) * t520 + (t110 * t825 + t494 * t819 - t498 * t831) * t549 + (t110 * t847 + t494 * t840 - t498 * t853) * t487) + t549 * ((-t210 * t515 + t524 * t502 + t528 * t509) * t520 + (-t210 * t544 + t524 * t532 + t528 * t538) * t549 + (-t210 * t569 + t524 * t556 + t528 * t562) * t487) + m(15) * (t911 + t929 + t937) + m(16) * (t957 + t970 + t980) + t549 * ((-t210 * t805 + t524 * t799 - t528 * t811) * t520 + (-t210 * t825 + t524 * t819 - t528 * t831) * t549 + (-t210 * t847 + t524 * t840 - t528 * t853) * t487) + t487 * ((-t1 * t805 + t490 * t799 - t504 * t811) * t520 + (-t1 * t825 + t490 * t819 - t504 * t831) * t549 + (-t1 * t847 + t490 * t840 - t504 * t853) * t487) + m(12) * (t1047 + t1065 + t1073) + m(10) * (t1110 + t1132 + t1138) + m(11) * (t1154 + t1162 + t1166) + m(8) * (t1239 + t1276 + t1286) + m(7) * (t1300 + t1308 + t1312);
t1317 = t6 * t1173;
t1319 = t6 * t1182;
t1322 = -t1317 * rSges(6,1) + t1319 * rSges(6,2) - t1 * rSges(6,3);
t1326 = t10 * t1182 + t49 * t1173;
t1330 = t10 * t1173 - t49 * t1182;
t1333 = t1326 * rSges(6,1) + t1330 * rSges(6,2) - t210 * rSges(6,3);
t1336 = (t1171 * t1322 - t1180 * t1333 - t593 - t595 + t598 - t603 - t610 + t881 - t888) ^ 2;
t1340 = -t11 * t1173 + t13 * t1182;
t1344 = t11 * t1182 + t13 * t1173;
t1347 = t1340 * rSges(6,1) + t1344 * rSges(6,2) + t110 * rSges(6,3);
t1350 = (t1180 * t1347 - t1242 * t1322 + t636 + t637 + t638 + t641 + t646 - t912 + t916) ^ 2;
t1354 = (-t1171 * t1347 + t1242 * t1333 - t664 - t667 + t668 - t669 + t930 - t931) ^ 2;
t1360 = Icges(3,4) * t10;
t1362 = Icges(3,5) * t13;
t1363 = -Icges(3,1) * t10 * t1 - t1360 * t6 + t1362;
t1368 = Icges(3,6) * t13;
t1369 = -Icges(3,2) * t10 * t6 - t1360 * t1 + t1368;
t1376 = Icges(3,4) * t13;
t1378 = Icges(3,5) * t10;
t1379 = Icges(3,1) * t13 * t1 + t1376 * t6 + t1378;
t1384 = Icges(3,6) * t10;
t1385 = Icges(3,2) * t13 * t6 + t1376 * t1 + t1384;
t1392 = -Icges(3,1) * t6 + Icges(3,4) * t1;
t1396 = -Icges(3,4) * t6 + Icges(3,2) * t1;
t1405 = t335 * rSges(14,1) - t337 * rSges(14,2) - t1 * rSges(14,3);
t1410 = t262 * rSges(14,1) + t266 * rSges(14,2) - t210 * rSges(14,3);
t1413 = (t289 * t1405 - t314 * t1410 - t593 - t595 + t598 - t603 - t610 + t881 - t888) ^ 2;
t1418 = t236 * rSges(14,1) + t240 * rSges(14,2) + t110 * rSges(14,3);
t1421 = (-t230 * t1405 + t314 * t1418 + t636 + t637 + t638 + t641 + t646 - t912 + t916) ^ 2;
t1425 = (t230 * t1410 - t289 * t1418 - t664 - t667 + t668 - t669 + t930 - t931) ^ 2;
t1431 = -t450 * rSges(13,1) - t452 * rSges(13,2) - t1 * rSges(13,3);
t1436 = t385 * rSges(13,1) + t389 * rSges(13,2) - t210 * rSges(13,3);
t1439 = (t289 * t1431 - t314 * t1436 - t593 - t595 + t598 - t603 - t610 + t881 - t888) ^ 2;
t1444 = t359 * rSges(13,1) + t363 * rSges(13,2) + t110 * rSges(13,3);
t1447 = (-t230 * t1431 + t314 * t1444 + t636 + t637 + t638 + t641 + t646 - t912 + t916) ^ 2;
t1451 = (t230 * t1436 - t289 * t1444 - t664 - t667 + t668 - t669 + t930 - t931) ^ 2;
t1458 = Icges(5,5) * t10 * t6 + Icges(5,1) * t651 + Icges(5,4) * t655;
t1464 = Icges(5,6) * t10 * t6 + Icges(5,4) * t651 + Icges(5,2) * t655;
t1470 = Icges(5,3) * t10 * t6 + Icges(5,5) * t651 + Icges(5,6) * t655;
t1478 = -Icges(5,5) * t13 * t6 + Icges(5,1) * t625 + Icges(5,4) * t629;
t1484 = -Icges(5,6) * t13 * t6 + Icges(5,4) * t625 + Icges(5,2) * t629;
t1490 = -Icges(5,3) * t13 * t6 + Icges(5,5) * t625 + Icges(5,6) * t629;
t1496 = Icges(5,4) * t6;
t1499 = -Icges(5,1) * t6 * t613 - Icges(5,5) * t1 + t1496 * t616;
t1505 = Icges(5,2) * t6 * t616 - Icges(5,6) * t1 - t1496 * t613;
t1512 = -Icges(5,5) * t6 * t613 + Icges(5,6) * t6 * t616 - Icges(5,3) * t1;
t1522 = Icges(4,5) * t10 * t6 + Icges(4,1) * t698 + Icges(4,4) * t702;
t1528 = Icges(4,6) * t10 * t6 + Icges(4,4) * t698 + Icges(4,2) * t702;
t1534 = Icges(4,3) * t10 * t6 + Icges(4,5) * t698 + Icges(4,6) * t702;
t1542 = -Icges(4,5) * t13 * t6 + Icges(4,1) * t685 + Icges(4,4) * t689;
t1548 = -Icges(4,6) * t13 * t6 + Icges(4,4) * t685 + Icges(4,2) * t689;
t1554 = -Icges(4,3) * t13 * t6 + Icges(4,5) * t685 + Icges(4,6) * t689;
t1560 = Icges(4,4) * t6;
t1563 = -Icges(4,1) * t6 * t601 - Icges(4,5) * t1 + t1560 * t606;
t1569 = Icges(4,2) * t6 * t606 - Icges(4,6) * t1 - t1560 * t601;
t1576 = -Icges(4,5) * t6 * t601 + Icges(4,6) * t6 * t606 - Icges(4,3) * t1;
t1620 = Icges(9,5) * t10 * t6 + Icges(9,1) * t740 + Icges(9,4) * t744;
t1626 = Icges(9,6) * t10 * t6 + Icges(9,4) * t740 + Icges(9,2) * t744;
t1632 = Icges(9,3) * t10 * t6 + Icges(9,5) * t740 + Icges(9,6) * t744;
t1641 = -Icges(9,5) * t13 * t6 + Icges(9,1) * t726 + Icges(9,4) * t730;
t1647 = -Icges(9,6) * t13 * t6 + Icges(9,4) * t726 + Icges(9,2) * t730;
t1653 = -Icges(9,3) * t13 * t6 + Icges(9,5) * t726 + Icges(9,6) * t730;
t1660 = Icges(9,4) * t6;
t1663 = Icges(9,1) * t6 * t715 - Icges(9,5) * t1 + t1660 * t718;
t1669 = Icges(9,2) * t6 * t718 - Icges(9,6) * t1 + t1660 * t715;
t1676 = Icges(9,5) * t6 * t715 + Icges(9,6) * t6 * t718 - Icges(9,3) * t1;
t1685 = Icges(8,1) * t1266 + Icges(8,4) * t1270 - Icges(8,5) * t1255;
t1690 = Icges(8,4) * t1266 + Icges(8,2) * t1270 - Icges(8,6) * t1255;
t1695 = Icges(8,5) * t1266 + Icges(8,6) * t1270 - Icges(8,3) * t1255;
t1702 = Icges(8,1) * t1229 + Icges(8,4) * t1233 - Icges(8,5) * t1201;
t1707 = Icges(8,4) * t1229 + Icges(8,2) * t1233 - Icges(8,6) * t1201;
t1712 = Icges(8,5) * t1229 + Icges(8,6) * t1233 - Icges(8,3) * t1201;
t1720 = -Icges(8,5) * t6 * t1193 + Icges(8,1) * t1211 + Icges(8,4) * t1215;
t1726 = -Icges(8,6) * t6 * t1193 + Icges(8,4) * t1211 + Icges(8,2) * t1215;
t1732 = -Icges(8,3) * t6 * t1193 + Icges(8,5) * t1211 + Icges(8,6) * t1215;
t1759 = Icges(10,5) * t10 * t6 + Icges(10,1) * t1122 + Icges(10,4) * t1126;
t1765 = Icges(10,6) * t10 * t6 + Icges(10,4) * t1122 + Icges(10,2) * t1126;
t1771 = Icges(10,3) * t10 * t6 + Icges(10,5) * t1122 + Icges(10,6) * t1126;
t1779 = -Icges(10,5) * t13 * t6 + Icges(10,1) * t1100 + Icges(10,4) * t1104;
t1785 = -Icges(10,6) * t13 * t6 + Icges(10,4) * t1100 + Icges(10,2) * t1104;
t1791 = -Icges(10,3) * t13 * t6 + Icges(10,5) * t1100 + Icges(10,6) * t1104;
t1797 = Icges(10,4) * t6;
t1800 = Icges(10,1) * t6 * t1087 - Icges(10,5) * t1 + t1797 * t1090;
t1806 = Icges(10,2) * t6 * t1090 - Icges(10,6) * t1 + t1797 * t1087;
t1813 = Icges(10,5) * t6 * t1087 + Icges(10,6) * t6 * t1090 - Icges(10,3) * t1;
t1823 = Icges(7,5) * t10 * t6 + Icges(7,1) * t1263 + Icges(7,4) * t1255;
t1829 = Icges(7,6) * t10 * t6 + Icges(7,4) * t1263 + Icges(7,2) * t1255;
t1835 = Icges(7,3) * t10 * t6 + Icges(7,5) * t1263 + Icges(7,6) * t1255;
t1843 = -Icges(7,5) * t13 * t6 + Icges(7,1) * t1226 + Icges(7,4) * t1201;
t1849 = -Icges(7,6) * t13 * t6 + Icges(7,4) * t1226 + Icges(7,2) * t1201;
t1855 = -Icges(7,3) * t13 * t6 + Icges(7,5) * t1226 + Icges(7,6) * t1201;
t1861 = Icges(7,4) * t6;
t1864 = Icges(7,1) * t6 * t1199 - Icges(7,5) * t1 + t1861 * t1193;
t1870 = Icges(7,2) * t6 * t1193 - Icges(7,6) * t1 + t1861 * t1199;
t1877 = Icges(7,5) * t6 * t1199 + Icges(7,6) * t6 * t1193 - Icges(7,3) * t1;
t1883 = m(6) * (t1336 + t1350 + t1354) + t782 * Icges(2,3) + qJD(1) * ((t1 * t1369 - t6 * t1363) * qJD(2) * t13 + (t1 * t1385 - t6 * t1379) * qJD(2) * t10 + (t1 * t1396 - t6 * t1392) * qJD(1)) + m(14) * (t1413 + t1421 + t1425) + m(13) * (t1439 + t1447 + t1451) + t622 * ((-t1 * t1470 - t614 * t1458 + t617 * t1464) * t647 + (-t1 * t1490 - t614 * t1478 + t617 * t1484) * t611 + (-t1 * t1512 - t614 * t1499 + t617 * t1505) * t622) + t639 * ((t110 * t1534 + t698 * t1522 + t702 * t1528) * t639 + (t110 * t1554 + t698 * t1542 + t702 * t1548) * t599 + (t110 * t1576 + t698 * t1563 + t702 * t1569) * t604) + t599 * ((t685 * t1522 + t689 * t1528 - t210 * t1534) * t639 + (t685 * t1542 + t689 * t1548 - t210 * t1554) * t599 + (t685 * t1563 + t689 * t1569 - t210 * t1576) * t604) + t604 * ((-t1 * t1534 - t677 * t1522 + t679 * t1528) * t639 + (-t1 * t1554 - t677 * t1542 + t679 * t1548) * t599 + (-t1 * t1576 - t677 * t1563 + t679 * t1569) * t604) + qJD(1) * ((-t1 * t1632 + t716 * t1620 + t719 * t1626) * qJD(2) * t13 + (-t1 * t1653 + t716 * t1641 + t719 * t1647) * qJD(2) * t10 + (-t1 * t1676 + t716 * t1663 + t719 * t1669) * qJD(1)) + t1205 * ((-t1201 * t1695 + t1229 * t1685 + t1233 * t1690) * t1259 + (-t1201 * t1712 + t1229 * t1702 + t1233 * t1707) * t1205 + (-t1201 * t1732 + t1229 * t1720 + t1233 * t1726) * t1223) + t1259 * ((-t1255 * t1695 + t1266 * t1685 + t1270 * t1690) * t1259 + (-t1255 * t1712 + t1266 * t1702 + t1270 * t1707) * t1205 + (-t1255 * t1732 + t1266 * t1720 + t1270 * t1726) * t1223) + t1097 * ((-t1 * t1771 + t1088 * t1759 + t1091 * t1765) * t1118 + (-t1 * t1791 + t1088 * t1779 + t1091 * t1785) * t1085 + (-t1 * t1813 + t1088 * t1800 + t1091 * t1806) * t1097) + t1190 * ((t1201 * t1829 + t1226 * t1823 - t210 * t1835) * t1250 + (t1201 * t1849 + t1226 * t1843 - t210 * t1855) * t1190 + (t1201 * t1870 + t1226 * t1864 - t210 * t1877) * t1197);
t1990 = Icges(6,5) * t10 * t6 + Icges(6,1) * t1340 + Icges(6,4) * t1344;
t1996 = Icges(6,6) * t10 * t6 + Icges(6,4) * t1340 + Icges(6,2) * t1344;
t2002 = Icges(6,3) * t10 * t6 + Icges(6,5) * t1340 + Icges(6,6) * t1344;
t2010 = -Icges(6,5) * t13 * t6 + Icges(6,1) * t1326 + Icges(6,4) * t1330;
t2016 = -Icges(6,6) * t13 * t6 + Icges(6,4) * t1326 + Icges(6,2) * t1330;
t2022 = -Icges(6,3) * t13 * t6 + Icges(6,5) * t1326 + Icges(6,6) * t1330;
t2028 = Icges(6,4) * t6;
t2031 = -Icges(6,1) * t6 * t1173 - Icges(6,5) * t1 + t2028 * t1182;
t2037 = Icges(6,2) * t6 * t1182 - Icges(6,6) * t1 - t2028 * t1173;
t2044 = -Icges(6,5) * t6 * t1173 + Icges(6,6) * t6 * t1182 - Icges(6,3) * t1;
t2161 = Icges(3,3) * t13 - t1378 * t1 - t1384 * t6;
t2171 = Icges(3,3) * t10 + t1362 * t1 + t1368 * t6;
t2180 = -Icges(3,5) * t6 + Icges(3,6) * t1;
t2205 = t1250 * ((t110 * t1835 + t1255 * t1829 + t1263 * t1823) * t1250 + (t110 * t1855 + t1255 * t1849 + t1263 * t1843) * t1190 + (t110 * t1877 + t1255 * t1870 + t1263 * t1864) * t1197) + t1197 * ((-t1 * t1835 + t1206 * t1823 + t1217 * t1829) * t1250 + (-t1 * t1855 + t1206 * t1843 + t1217 * t1849) * t1190 + (-t1 * t1877 + t1206 * t1864 + t1217 * t1870) * t1197) + t1118 * ((t110 * t1771 + t1122 * t1759 + t1126 * t1765) * t1118 + (t110 * t1791 + t1122 * t1779 + t1126 * t1785) * t1085 + (t110 * t1813 + t1122 * t1800 + t1126 * t1806) * t1097) + t1085 * ((t1100 * t1759 + t1104 * t1765 - t210 * t1771) * t1118 + (t1100 * t1779 + t1104 * t1785 - t210 * t1791) * t1085 + (t1100 * t1800 + t1104 * t1806 - t210 * t1813) * t1097) + t1223 * ((t1211 * t1685 + t1215 * t1690 - t1217 * t1695) * t1259 + (t1211 * t1702 + t1215 * t1707 - t1217 * t1712) * t1205 + (t1211 * t1720 + t1215 * t1726 - t1217 * t1732) * t1223) + t204 * ((-t1 * t152 - t1141 * t140 + t1143 * t146) * t126 + (-t1 * t178 - t1141 * t166 + t1143 * t172) * t181 + (-t1 * t201 - t1141 * t188 + t1143 * t194) * t204) + t1180 * ((-t1 * t2002 - t1317 * t1990 + t1319 * t1996) * t1242 + (-t1 * t2022 - t1317 * t2010 + t1319 * t2016) * t1171 + (-t1 * t2044 - t1317 * t2031 + t1319 * t2037) * t1180) + t1242 * ((t110 * t2002 + t1340 * t1990 + t1344 * t1996) * t1242 + (t110 * t2022 + t1340 * t2010 + t1344 * t2016) * t1171 + (t110 * t2044 + t1340 * t2031 + t1344 * t2037) * t1180) + t1171 * ((t1326 * t1990 + t1330 * t1996 - t210 * t2002) * t1242 + (t1326 * t2010 + t1330 * t2016 - t210 * t2022) * t1171 + (t1326 * t2031 + t1330 * t2037 - t210 * t2044) * t1180) + t647 * ((t110 * t1470 + t651 * t1458 + t655 * t1464) * t647 + (t110 * t1490 + t651 * t1478 + t655 * t1484) * t611 + (t110 * t1512 + t651 * t1499 + t655 * t1505) * t622) + t611 * ((t625 * t1458 + t629 * t1464 - t210 * t1470) * t647 + (t625 * t1478 + t629 * t1484 - t210 * t1490) * t611 + (t625 * t1499 + t629 * t1505 - t210 * t1512) * t622) + t40 * ((t110 * t1632 + t740 * t1620 + t744 * t1626) * qJD(2) * t13 + (t110 * t1653 + t740 * t1641 + t744 * t1647) * qJD(2) * t10 + (t110 * t1676 + t740 * t1663 + t744 * t1669) * qJD(1)) + t75 * ((t726 * t1620 + t730 * t1626 - t210 * t1632) * qJD(2) * t13 + (t726 * t1641 + t730 * t1647 - t210 * t1653) * qJD(2) * t10 + (t726 * t1663 + t730 * t1669 - t210 * t1676) * qJD(1)) + t75 * ((t10 * t2161 + t49 * t1363 + t210 * t1369) * qJD(2) * t13 + (t10 * t2171 + t49 * t1379 + t210 * t1385) * qJD(2) * t10 + (t10 * t2180 + t49 * t1392 + t210 * t1396) * qJD(1)) + t40 * ((-t11 * t1363 - t110 * t1369 + t13 * t2161) * qJD(2) * t13 + (-t11 * t1379 - t110 * t1385 + t13 * t2171) * qJD(2) * t10 + (-t11 * t1392 - t110 * t1396 + t13 * t2180) * qJD(1));
t2207 = t676 / 0.2e1 + t1315 / 0.2e1 + t1883 / 0.2e1 + t2205 / 0.2e1;
T = t2207;
