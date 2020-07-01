% Calculate kinetic energy for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% qJD [13x1]
%   Generalized joint velocities
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
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
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = KAS5m7OL_energykin_floatb_twist_slag_vp2(qJ, qJD, V_base, ...
  pkin, m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),zeros(6,1),zeros(19,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energykin_floatb_twist_slag_vp2: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m7OL_energykin_floatb_twist_slag_vp2: qJD has to be [13x1] (double)');
assert(isreal(V_base) && all(size(V_base) == [6 1]), ...
  'KAS5m7OL_energykin_floatb_twist_slag_vp2: V_base has to be [6x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energykin_floatb_twist_slag_vp2: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_energykin_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7OL_energykin_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7OL_energykin_floatb_twist_slag_vp2: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_floatb_twist_linkframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:20
% EndTime: 2020-06-30 17:46:20
% DurationCPUTime: 0.70s
% Computational Cost: add. (16677->439), mult. (21750->593), div. (0->0), fcn. (18276->28), ass. (0->208)
t1 = sin(qJ(1));
t3 = cos(qJ(1));
t5 = t1 * V_base(4) - t3 * V_base(5);
t7 = t3 * V_base(4);
t8 = t1 * V_base(5);
t9 = t7 + t8;
t11 = V_base(6) + qJD(1);
t29 = pkin(5) * V_base(5) + V_base(1);
t30 = t1 * t29;
t32 = -pkin(5) * V_base(4) + V_base(2);
t33 = t3 * t32;
t34 = t30 - t33;
t39 = t3 * t29;
t40 = t1 * t32;
t41 = t39 + t40;
t76 = V_base(1) ^ 2;
t77 = V_base(2) ^ 2;
t78 = V_base(3) ^ 2;
t82 = cos(qJ(4));
t83 = cos(qJ(3));
t84 = sin(qJ(2));
t86 = cos(qJ(2));
t88 = -t11 * t86 - t5 * t84;
t90 = sin(qJ(3));
t91 = t7 + t8 + qJD(2);
t93 = t83 * t88 + t90 * t91;
t95 = sin(qJ(4));
t98 = t83 * t91 - t88 * t90;
t100 = t82 * t93 + t95 * t98;
t104 = t82 * t98 - t93 * t95;
t106 = t86 * t5;
t107 = t84 * t11;
t108 = t106 - t107 + qJD(3) + qJD(4);
t119 = t5 * (Ifges(2,1) * t5 + Ifges(2,4) * t9 + Ifges(2,5) * t11) / 0.2e1 + t9 * (Ifges(2,4) * t5 + Ifges(2,2) * t9 + Ifges(2,6) * t11) / 0.2e1 + t11 * (Ifges(2,5) * t5 + Ifges(2,6) * t9 + Ifges(2,3) * t11) / 0.2e1 + t34 * (-mrSges(2,2) * t11 + mrSges(2,3) * t9) + t41 * (mrSges(2,1) * t11 - mrSges(2,3) * t5) + V_base(4) * (Ifges(1,1) * V_base(4) + Ifges(1,4) * V_base(5) + Ifges(1,5) * V_base(6)) / 0.2e1 + V_base(5) * (Ifges(1,4) * V_base(4) + Ifges(1,2) * V_base(5) + Ifges(1,6) * V_base(6)) / 0.2e1 + V_base(6) * (Ifges(1,5) * V_base(4) + Ifges(1,6) * V_base(5) + Ifges(1,3) * V_base(6)) / 0.2e1 + V_base(2) * (mrSges(1,1) * V_base(6) - mrSges(1,3) * V_base(4)) + V_base(1) * (-mrSges(1,2) * V_base(6) + mrSges(1,3) * V_base(5)) + V_base(3) * (-mrSges(1,1) * V_base(5) + mrSges(1,2) * V_base(4)) + m(1) * (t76 + t77 + t78) / 0.2e1 + t100 * (Ifges(5,1) * t100 + Ifges(5,4) * t104 + Ifges(5,5) * t108) / 0.2e1 + t104 * (Ifges(5,4) * t100 + Ifges(5,2) * t104 + Ifges(5,6) * t108) / 0.2e1;
t128 = t106 - t107 + qJD(3);
t146 = -pkin(11) * t11 + t30 - t33;
t147 = t86 * t146;
t149 = pkin(11) * t5 + V_base(3);
t150 = t84 * t149;
t151 = t147 - t150;
t156 = t84 * t146;
t157 = t86 * t149;
t159 = pkin(16) * t91 - t156 - t157;
t162 = -pkin(16) * t88 + t39 + t40;
t164 = t159 * t83 + t162 * t90;
t165 = t164 ^ 2;
t166 = t90 * t159;
t167 = t83 * t162;
t168 = -t166 + t167;
t169 = t168 ^ 2;
t170 = t151 ^ 2;
t183 = -t106 + t107;
t187 = -t156 - t157;
t188 = t187 ^ 2;
t189 = t41 ^ 2;
t193 = cos(pkin(3));
t194 = t193 * t159;
t195 = sin(pkin(3));
t196 = t195 * t162;
t197 = -t194 - t196;
t200 = t193 * t91 - t195 * t88;
t208 = -t193 * t88 - t195 * t91;
t214 = -t159 * t195 + t162 * t193;
t215 = t214 ^ 2;
t216 = t197 ^ 2;
t226 = t108 * (Ifges(5,5) * t100 + Ifges(5,6) * t104 + Ifges(5,3) * t108) / 0.2e1 + t93 * (Ifges(4,1) * t93 + Ifges(4,4) * t98 + Ifges(4,5) * t128) / 0.2e1 + t98 * (Ifges(4,4) * t93 + Ifges(4,2) * t98 + Ifges(4,6) * t128) / 0.2e1 + t128 * (Ifges(4,5) * t93 + Ifges(4,6) * t98 + Ifges(4,3) * t128) / 0.2e1 + t151 * (-mrSges(4,1) * t98 + mrSges(4,2) * t93) + m(4) * (t165 + t169 + t170) / 0.2e1 + t164 * (-mrSges(4,2) * t128 + mrSges(4,3) * t98) + t168 * (mrSges(4,1) * t128 - mrSges(4,3) * t93) + t41 * (-mrSges(3,1) * t183 + mrSges(3,2) * t88) + m(3) * (t188 + t170 + t189) / 0.2e1 + t197 * (-mrSges(9,1) * t183 - mrSges(9,3) * t200) + t151 * (-mrSges(9,1) * t208 + mrSges(9,2) * t200) + m(9) * (t215 + t216 + t170) / 0.2e1 + t88 * (Ifges(3,1) * t88 + Ifges(3,4) * t183 + Ifges(3,5) * t91) / 0.2e1;
t270 = t34 ^ 2;
t278 = t106 - t107 + qJD(3) + qJD(4) + qJD(5) + qJD(6);
t279 = sin(qJ(6));
t280 = cos(qJ(5));
t282 = sin(qJ(5));
t284 = t100 * t280 + t104 * t282;
t286 = cos(qJ(6));
t289 = -t100 * t282 + t104 * t280;
t291 = -t279 * t284 + t286 * t289;
t293 = t286 * t284;
t294 = t279 * t289;
t295 = -t293 - t294;
t309 = pkin(18) * t128 - t166 + t167;
t311 = t164 * t82 + t309 * t95;
t313 = t95 * t164;
t314 = t82 * t309;
t316 = pkin(6) * t108 - t313 + t314;
t318 = t280 * t311 + t282 * t316;
t319 = t279 * t318;
t320 = t282 * t311;
t321 = t280 * t316;
t322 = t106 - t107 + qJD(3) + qJD(4) + qJD(5);
t324 = pkin(7) * t322 - t320 + t321;
t325 = t286 * t324;
t326 = -t319 + t325;
t333 = -t279 * t324 - t286 * t318;
t338 = t183 * (Ifges(3,4) * t88 + Ifges(3,2) * t183 + Ifges(3,6) * t91) / 0.2e1 + t91 * (Ifges(3,5) * t88 + Ifges(3,6) * t183 + Ifges(3,3) * t91) / 0.2e1 + t200 * (Ifges(9,1) * t200 + Ifges(9,4) * t208 - Ifges(9,5) * t183) / 0.2e1 + t208 * (Ifges(9,4) * t200 + Ifges(9,2) * t208 - Ifges(9,6) * t183) / 0.2e1 - t183 * (Ifges(9,5) * t200 + Ifges(9,6) * t208 - Ifges(9,3) * t183) / 0.2e1 + t187 * (-mrSges(3,2) * t91 + mrSges(3,3) * t183) - t151 * (mrSges(3,1) * t91 - mrSges(3,3) * t88) + t214 * (mrSges(9,2) * t183 + mrSges(9,3) * t208) + m(2) * (t270 + t189 + t78) / 0.2e1 + V_base(3) * (-mrSges(2,1) * t9 + mrSges(2,2) * t5) + t278 * (Ifges(7,5) * t291 + Ifges(7,6) * t295 + Ifges(7,3) * t278) / 0.2e1 + t291 * (Ifges(7,1) * t291 + Ifges(7,4) * t295 + Ifges(7,5) * t278) / 0.2e1 + t326 * (-mrSges(7,2) * t278 + mrSges(7,3) * t295) + t333 * (mrSges(7,1) * t278 - mrSges(7,3) * t291);
t339 = cos(qJ(7));
t341 = pkin(10) * t278 - t319 + t325;
t343 = sin(qJ(7));
t344 = t98 * pkin(18);
t345 = t104 * pkin(6);
t346 = t289 * pkin(7);
t348 = -pkin(10) * t291 + t147 - t150 - t344 - t345 - t346;
t350 = t339 * t341 + t343 * t348;
t351 = t350 ^ 2;
t354 = t339 * t348 - t341 * t343;
t355 = t354 ^ 2;
t356 = t333 ^ 2;
t362 = t278 * t343 + t291 * t339;
t366 = t278 * t339 - t291 * t343;
t372 = t293 + t294 + qJD(7);
t415 = -t320 + t321;
t420 = t318 ^ 2;
t421 = t415 ^ 2;
t422 = t147 - t150 - t344 - t345;
t423 = t422 ^ 2;
t435 = m(8) * (t351 + t355 + t356) / 0.2e1 - t333 * (-mrSges(8,1) * t366 + mrSges(8,2) * t362) + t362 * (Ifges(8,1) * t362 + Ifges(8,4) * t366 + Ifges(8,5) * t372) / 0.2e1 + t366 * (Ifges(8,4) * t362 + Ifges(8,2) * t366 + Ifges(8,6) * t372) / 0.2e1 + t354 * (mrSges(8,1) * t372 - mrSges(8,3) * t362) + t350 * (-mrSges(8,2) * t372 + mrSges(8,3) * t366) + t372 * (Ifges(8,5) * t362 + Ifges(8,6) * t366 + Ifges(8,3) * t372) / 0.2e1 + t284 * (Ifges(6,1) * t284 + Ifges(6,4) * t289 + Ifges(6,5) * t322) / 0.2e1 + t289 * (Ifges(6,4) * t284 + Ifges(6,2) * t289 + Ifges(6,6) * t322) / 0.2e1 + t322 * (Ifges(6,5) * t284 + Ifges(6,6) * t289 + Ifges(6,3) * t322) / 0.2e1 + t415 * (mrSges(6,1) * t322 - mrSges(6,3) * t284) + m(6) * (t420 + t421 + t423) / 0.2e1 + t422 * (-mrSges(6,1) * t289 + mrSges(6,2) * t284) + t318 * (-mrSges(6,2) * t322 + mrSges(6,3) * t289);
t438 = t147 - t150 - t344 - t345 - t346;
t447 = -t313 + t314;
t452 = t147 - t150 - t344;
t457 = t311 ^ 2;
t458 = t447 ^ 2;
t459 = t452 ^ 2;
t463 = cos(qJ(10));
t464 = cos(qJ(9));
t466 = sin(qJ(9));
t468 = t464 * t93 + t466 * t98;
t470 = sin(qJ(10));
t473 = t464 * t98 - t466 * t93;
t475 = -t463 * t468 - t470 * t473;
t479 = -t463 * t473 + t468 * t470;
t481 = t106 - t107 + qJD(3) + qJD(9) + qJD(10);
t500 = pkin(19) * t128 - t166 + t167;
t502 = t164 * t464 + t466 * t500;
t504 = t466 * t164;
t505 = t464 * t500;
t506 = t106 - t107 + qJD(3) + qJD(9);
t508 = pkin(14) * t506 - t504 + t505;
t510 = -t463 * t502 - t470 * t508;
t515 = sin(qJ(8));
t517 = cos(qJ(8));
t519 = pkin(17) * t183 - t194 - t196;
t521 = -t214 * t515 + t517 * t519;
t524 = t200 * t517 + t208 * t515;
t526 = t106 - t107 + qJD(8);
t532 = -t200 * t515 + t208 * t517;
t553 = t214 * t517 + t515 * t519;
t558 = t438 * (-mrSges(7,1) * t295 + mrSges(7,2) * t291) + t311 * (-mrSges(5,2) * t108 + mrSges(5,3) * t104) + t447 * (mrSges(5,1) * t108 - mrSges(5,3) * t100) + t452 * (-mrSges(5,1) * t104 + mrSges(5,2) * t100) + m(5) * (t457 + t458 + t459) / 0.2e1 + t475 * (Ifges(12,1) * t475 + Ifges(12,4) * t479 + Ifges(12,5) * t481) / 0.2e1 + t479 * (Ifges(12,4) * t475 + Ifges(12,2) * t479 + Ifges(12,6) * t481) / 0.2e1 + t481 * (Ifges(12,5) * t475 + Ifges(12,6) * t479 + Ifges(12,3) * t481) / 0.2e1 + t510 * (-mrSges(12,2) * t481 + mrSges(12,3) * t479) + t521 * (mrSges(10,1) * t526 - mrSges(10,3) * t524) + t532 * (Ifges(10,4) * t524 + Ifges(10,2) * t532 + Ifges(10,6) * t526) / 0.2e1 + t526 * (Ifges(10,5) * t524 + Ifges(10,6) * t532 + Ifges(10,3) * t526) / 0.2e1 + t524 * (Ifges(10,1) * t524 + Ifges(10,4) * t532 + Ifges(10,5) * t526) / 0.2e1 + t553 * (-mrSges(10,2) * t526 + mrSges(10,3) * t532);
t560 = pkin(17) * t208 + t147 - t150;
t565 = t553 ^ 2;
t566 = t521 ^ 2;
t567 = t560 ^ 2;
t571 = t326 ^ 2;
t572 = t438 ^ 2;
t582 = t106 - t107 + qJD(3) + qJD(4) + qJD(11);
t583 = sin(pkin(1));
t584 = sin(qJ(11));
t586 = cos(qJ(11));
t588 = t100 * t584 - t104 * t586;
t590 = cos(pkin(1));
t593 = t100 * t586 + t104 * t584;
t595 = t583 * t588 - t590 * t593;
t599 = t583 * t593 + t588 * t590;
t607 = t311 * t584 - t316 * t586;
t614 = t311 * t586 + t316 * t584;
t625 = t590 * t607;
t626 = t583 * t614;
t627 = t625 + t626;
t644 = t607 ^ 2;
t645 = t614 ^ 2;
t657 = t560 * (-mrSges(10,1) * t532 + mrSges(10,2) * t524) + m(10) * (t565 + t566 + t567) / 0.2e1 + m(7) * (t571 + t356 + t572) / 0.2e1 + t295 * (Ifges(7,4) * t291 + Ifges(7,2) * t295 + Ifges(7,6) * t278) / 0.2e1 + t582 * (Ifges(14,5) * t595 + Ifges(14,6) * t599 + Ifges(14,3) * t582) / 0.2e1 + t607 * (-mrSges(13,2) * t582 + mrSges(13,3) * t593) + t614 * (mrSges(13,1) * t582 - mrSges(13,3) * t588) + t588 * (Ifges(13,1) * t588 + Ifges(13,4) * t593 + Ifges(13,5) * t582) / 0.2e1 + t627 * (mrSges(14,1) * t582 - mrSges(14,3) * t595) + t593 * (Ifges(13,4) * t588 + Ifges(13,2) * t593 + Ifges(13,6) * t582) / 0.2e1 + t582 * (Ifges(13,5) * t588 + Ifges(13,6) * t593 + Ifges(13,3) * t582) / 0.2e1 + m(13) * (t644 + t645 + t423) / 0.2e1 + t422 * (-mrSges(14,1) * t599 + mrSges(14,2) * t595) + t422 * (-mrSges(13,1) * t593 + mrSges(13,2) * t588);
t661 = t583 * t607 - t590 * t614;
t662 = t661 ^ 2;
t663 = t627 ^ 2;
t675 = -t504 + t505;
t698 = t98 * pkin(19);
t699 = t147 - t150 - t698;
t704 = t502 ^ 2;
t705 = t675 ^ 2;
t706 = t699 ^ 2;
t710 = t510 ^ 2;
t713 = -t463 * t508 + t470 * t502;
t714 = t713 ^ 2;
t716 = -pkin(14) * t473 + t147 - t150 - t698;
t717 = t716 ^ 2;
t729 = sin(qJ(12));
t731 = cos(qJ(12));
t733 = t595 * t729 - t599 * t731;
t737 = t595 * t731 + t599 * t729;
t739 = t106 - t107 + qJD(3) + qJD(4) + qJD(11) + qJD(12);
t750 = m(14) * (t662 + t663 + t423) / 0.2e1 + t661 * (-mrSges(14,2) * t582 + mrSges(14,3) * t599) + t502 * (-mrSges(11,2) * t506 + mrSges(11,3) * t473) + t675 * (mrSges(11,1) * t506 - mrSges(11,3) * t468) + t468 * (Ifges(11,1) * t468 + Ifges(11,4) * t473 + Ifges(11,5) * t506) / 0.2e1 + t473 * (Ifges(11,4) * t468 + Ifges(11,2) * t473 + Ifges(11,6) * t506) / 0.2e1 + t506 * (Ifges(11,5) * t468 + Ifges(11,6) * t473 + Ifges(11,3) * t506) / 0.2e1 + t699 * (-mrSges(11,1) * t473 + mrSges(11,2) * t468) + m(11) * (t704 + t705 + t706) / 0.2e1 + m(12) * (t710 + t714 + t717) / 0.2e1 + t716 * (-mrSges(12,1) * t479 + mrSges(12,2) * t475) + t713 * (mrSges(12,1) * t481 - mrSges(12,3) * t475) + t733 * (Ifges(15,1) * t733 + Ifges(15,4) * t737 + Ifges(15,5) * t739) / 0.2e1 + t737 * (Ifges(15,4) * t733 + Ifges(15,2) * t737 + Ifges(15,6) * t739) / 0.2e1;
t757 = t731 * t661;
t759 = pkin(9) * t582 + t625 + t626;
t760 = t729 * t759;
t761 = t757 + t760;
t784 = t729 * t661;
t785 = t731 * t759;
t786 = t784 - t785;
t791 = t786 ^ 2;
t792 = t761 ^ 2;
t793 = t599 * pkin(9);
t794 = t147 - t150 - t344 - t345 - t793;
t795 = t794 ^ 2;
t799 = -t757 - t760 + qJD(13);
t805 = -qJ(13) * t733 + t147 - t150 - t344 - t345 - t793;
t811 = qJ(13) * t739 + t784 - t785;
t816 = t811 ^ 2;
t817 = t805 ^ 2;
t818 = t799 ^ 2;
t838 = t739 * (Ifges(15,5) * t733 + Ifges(15,6) * t737 + Ifges(15,3) * t739) / 0.2e1 + t761 * (mrSges(15,1) * t739 - mrSges(15,3) * t733) + t733 * (Ifges(16,1) * t733 + Ifges(16,4) * t739 - Ifges(16,5) * t737) / 0.2e1 + t739 * (Ifges(16,4) * t733 + Ifges(16,2) * t739 - Ifges(16,6) * t737) / 0.2e1 - t737 * (Ifges(16,5) * t733 + Ifges(16,6) * t739 - Ifges(16,3) * t737) / 0.2e1 + t786 * (-mrSges(15,2) * t739 + mrSges(15,3) * t737) + m(15) * (t791 + t792 + t795) / 0.2e1 + t799 * (-mrSges(16,1) * t739 + mrSges(16,2) * t733) + t805 * (-mrSges(16,1) * t737 - mrSges(16,3) * t733) + t811 * (mrSges(16,2) * t737 + mrSges(16,3) * t739) + m(16) * (t816 + t817 + t818) / 0.2e1 + t595 * (Ifges(14,1) * t595 + Ifges(14,4) * t599 + Ifges(14,5) * t582) / 0.2e1 + t599 * (Ifges(14,4) * t595 + Ifges(14,2) * t599 + Ifges(14,6) * t582) / 0.2e1 + t794 * (-mrSges(15,1) * t737 + mrSges(15,2) * t733);
t841 = t119 + t226 + t338 + t435 + t558 + t657 + t750 + t838;
T = t841;
