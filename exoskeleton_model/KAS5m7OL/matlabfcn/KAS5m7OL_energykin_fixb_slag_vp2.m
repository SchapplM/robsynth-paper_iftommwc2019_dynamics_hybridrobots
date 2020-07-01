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

function T = KAS5m7OL_energykin_fixb_slag_vp2(qJ, qJD, ...
  pkin, m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),zeros(19,1),zeros(16,1),zeros(16,3),zeros(16,6)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energykin_fixb_slag_vp2: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m7OL_energykin_fixb_slag_vp2: qJD has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energykin_fixb_slag_vp2: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_energykin_fixb_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7OL_energykin_fixb_slag_vp2: mrSges has to be [16x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [16 6]), ...
  'KAS5m7OL_energykin_fixb_slag_vp2: Ifges has to be [16x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_fixb_linkframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:20
% EndTime: 2020-06-30 17:46:21
% DurationCPUTime: 0.39s
% Computational Cost: add. (4151->364), mult. (9197->520), div. (0->0), fcn. (7482->26), ass. (0->185)
t1 = cos(pkin(1));
t2 = sin(qJ(11));
t3 = cos(qJ(4));
t4 = cos(qJ(3));
t5 = cos(qJ(2));
t6 = t4 * t5;
t8 = sin(qJ(3));
t10 = -qJD(1) * t6 + qJD(2) * t8;
t12 = sin(qJ(4));
t13 = t8 * t5;
t16 = qJD(1) * t13 + qJD(2) * t4;
t18 = t3 * t10 + t12 * t16;
t20 = cos(qJ(11));
t23 = -t12 * t10 + t3 * t16;
t25 = t2 * t18 - t20 * t23;
t27 = sin(pkin(1));
t30 = t20 * t18 + t2 * t23;
t32 = t1 * t25 + t27 * t30;
t35 = -t1 * t30 + t27 * t25;
t38 = sin(qJ(2));
t39 = t38 * qJD(1);
t40 = -t39 + qJD(3) + qJD(4) + qJD(11);
t44 = sin(qJ(5));
t46 = cos(qJ(5));
t48 = -t44 * t18 + t46 * t23;
t51 = t46 * t18 + t44 * t23;
t54 = -t39 + qJD(3) + qJD(4) + qJD(5);
t65 = -t39 + qJD(3) + qJD(4);
t81 = -t39 + qJD(3);
t102 = sin(pkin(3));
t103 = t102 * t5;
t105 = cos(pkin(3));
t107 = qJD(1) * t103 + qJD(2) * t105;
t109 = t105 * t5;
t112 = qJD(1) * t109 - qJD(2) * t102;
t123 = t32 * (Ifges(14,4) * t35 + Ifges(14,2) * t32 + Ifges(14,6) * t40) + t48 * (Ifges(6,4) * t51 + Ifges(6,2) * t48 + Ifges(6,6) * t54) + t54 * (Ifges(6,5) * t51 + Ifges(6,6) * t48 + Ifges(6,3) * t54) + t18 * (Ifges(5,1) * t18 + Ifges(5,4) * t23 + Ifges(5,5) * t65) + t23 * (Ifges(5,4) * t18 + Ifges(5,2) * t23 + Ifges(5,6) * t65) + t65 * (Ifges(5,5) * t18 + Ifges(5,6) * t23 + Ifges(5,3) * t65) + t10 * (Ifges(4,1) * t10 + Ifges(4,4) * t16 + Ifges(4,5) * t81) + t16 * (Ifges(4,4) * t10 + Ifges(4,2) * t16 + Ifges(4,6) * t81) + t81 * (Ifges(4,5) * t10 + Ifges(4,6) * t16 + Ifges(4,3) * t81) + qJD(2) * (-Ifges(3,5) * qJD(1) * t5 + Ifges(3,6) * qJD(1) * t38 + Ifges(3,3) * qJD(2)) + t107 * (-Ifges(9,5) * qJD(1) * t38 + Ifges(9,1) * t107 + Ifges(9,4) * t112) + t40 * (Ifges(14,5) * t35 + Ifges(14,6) * t32 + Ifges(14,3) * t40);
t139 = cos(qJ(9));
t141 = sin(qJ(9));
t143 = t139 * t10 + t141 * t16;
t147 = -t141 * t10 + t139 * t16;
t149 = -t39 + qJD(3) + qJD(9);
t158 = qJD(1) ^ 2;
t160 = sin(qJ(12));
t162 = cos(qJ(12));
t164 = t160 * t35 - t162 * t32;
t168 = t160 * t32 + t162 * t35;
t170 = -t39 + qJD(3) + qJD(4) + qJD(11) + qJD(12);
t186 = pkin(11) * t39 + qJD(2) * pkin(16);
t188 = qJD(1) * pkin(16);
t190 = t13 * t188 + t4 * t186;
t192 = t8 * t186;
t193 = t6 * t188;
t195 = pkin(18) * t81 - t192 + t193;
t197 = t12 * t195 + t3 * t190;
t199 = t12 * t190;
t200 = t3 * t195;
t202 = pkin(6) * t65 - t199 + t200;
t204 = t2 * t197 - t20 * t202;
t208 = t20 * t197 + t2 * t202;
t210 = -t1 * t208 + t27 * t204;
t211 = t160 * t210;
t212 = t1 * t204;
t213 = t27 * t208;
t215 = pkin(9) * t40 + t212 + t213;
t216 = t162 * t215;
t217 = t211 - t216;
t218 = t217 ^ 2;
t219 = t162 * t210;
t220 = t160 * t215;
t221 = t219 + t220;
t222 = t221 ^ 2;
t223 = t5 * qJD(1);
t224 = t223 * pkin(11);
t225 = t16 * pkin(18);
t226 = t23 * pkin(6);
t227 = t32 * pkin(9);
t228 = -t224 - t225 - t226 - t227;
t229 = t228 ^ 2;
t232 = cos(qJ(6));
t233 = t232 * t51;
t234 = sin(qJ(6));
t235 = t234 * t48;
t236 = -t233 - t235;
t239 = t232 * t48 - t234 * t51;
t242 = -t39 + qJD(3) + qJD(4) + qJD(5) + qJD(6);
t251 = t25 * (Ifges(13,1) * t25 + Ifges(13,4) * t30 + Ifges(13,5) * t40) + t30 * (Ifges(13,4) * t25 + Ifges(13,2) * t30 + Ifges(13,6) * t40) + t40 * (Ifges(13,5) * t25 + Ifges(13,6) * t30 + Ifges(13,3) * t40) + t143 * (Ifges(11,1) * t143 + Ifges(11,4) * t147 + Ifges(11,5) * t149) + t147 * (Ifges(11,4) * t143 + Ifges(11,2) * t147 + Ifges(11,6) * t149) + t158 * Ifges(2,3) + t164 * (Ifges(15,1) * t164 + Ifges(15,4) * t168 + Ifges(15,5) * t170) + t168 * (Ifges(15,4) * t164 + Ifges(15,2) * t168 + Ifges(15,6) * t170) + t170 * (Ifges(15,5) * t164 + Ifges(15,6) * t168 + Ifges(15,3) * t170) + m(15) * (t218 + t222 + t229) + t236 * (Ifges(7,4) * t239 + Ifges(7,2) * t236 + Ifges(7,6) * t242) + t242 * (Ifges(7,5) * t239 + Ifges(7,6) * t236 + Ifges(7,3) * t242);
t259 = cos(qJ(7));
t261 = sin(qJ(7));
t263 = t259 * t239 + t261 * t242;
t267 = -t261 * t239 + t259 * t242;
t269 = t233 + t235 + qJD(7);
t293 = qJ(13) * t170 + t211 - t216;
t294 = t293 ^ 2;
t296 = -qJ(13) * t164 - t224 - t225 - t226 - t227;
t297 = t296 ^ 2;
t298 = -t219 - t220 + qJD(13);
t299 = t298 ^ 2;
t317 = cos(qJ(10));
t319 = sin(qJ(10));
t321 = -t317 * t143 - t319 * t147;
t325 = t319 * t143 - t317 * t147;
t327 = -t39 + qJD(3) + qJD(9) + qJD(10);
t344 = t239 * (Ifges(7,1) * t239 + Ifges(7,4) * t236 + Ifges(7,5) * t242) / 0.2e1 + t263 * (Ifges(8,1) * t263 + Ifges(8,4) * t267 + Ifges(8,5) * t269) / 0.2e1 + t267 * (Ifges(8,4) * t263 + Ifges(8,2) * t267 + Ifges(8,6) * t269) / 0.2e1 + t269 * (Ifges(8,5) * t263 + Ifges(8,6) * t267 + Ifges(8,3) * t269) / 0.2e1 + t51 * (Ifges(6,1) * t51 + Ifges(6,4) * t48 + Ifges(6,5) * t54) / 0.2e1 + m(16) * (t294 + t297 + t299) / 0.2e1 + t204 * (-mrSges(13,2) * t40 + mrSges(13,3) * t30) + t208 * (mrSges(13,1) * t40 - mrSges(13,3) * t25) + t149 * (Ifges(11,5) * t143 + Ifges(11,6) * t147 + Ifges(11,3) * t149) / 0.2e1 + t321 * (Ifges(12,1) * t321 + Ifges(12,4) * t325 + Ifges(12,5) * t327) / 0.2e1 + t325 * (Ifges(12,4) * t321 + Ifges(12,2) * t325 + Ifges(12,6) * t327) / 0.2e1 + t327 * (Ifges(12,5) * t321 + Ifges(12,6) * t325 + Ifges(12,3) * t327) / 0.2e1;
t345 = sin(qJ(8));
t347 = cos(qJ(8));
t349 = -t345 * t107 + t347 * t112;
t352 = t347 * t107 + t345 * t112;
t355 = -t39 + qJD(8);
t372 = t44 * t197;
t373 = t46 * t202;
t374 = -t372 + t373;
t381 = t46 * t197 + t44 * t202;
t382 = t381 ^ 2;
t383 = t374 ^ 2;
t384 = -t224 - t225 - t226;
t385 = t384 ^ 2;
t397 = t48 * pkin(7);
t398 = -t224 - t225 - t226 - t397;
t431 = t349 * (Ifges(10,4) * t352 + Ifges(10,2) * t349 + Ifges(10,6) * t355) / 0.2e1 + t355 * (Ifges(10,5) * t352 + Ifges(10,6) * t349 + Ifges(10,3) * t355) / 0.2e1 + t352 * (Ifges(10,1) * t352 + Ifges(10,4) * t349 + Ifges(10,5) * t355) / 0.2e1 + t374 * (mrSges(6,1) * t54 - mrSges(6,3) * t51) + m(6) * (t382 + t383 + t385) / 0.2e1 + t384 * (-mrSges(6,1) * t48 + mrSges(6,2) * t51) + t381 * (-mrSges(6,2) * t54 + mrSges(6,3) * t48) + t398 * (-mrSges(7,1) * t236 + mrSges(7,2) * t239) + t164 * (Ifges(16,1) * t164 + Ifges(16,4) * t170 - Ifges(16,5) * t168) / 0.2e1 + t170 * (Ifges(16,4) * t164 + Ifges(16,2) * t170 - Ifges(16,6) * t168) / 0.2e1 - t168 * (Ifges(16,5) * t164 + Ifges(16,6) * t170 - Ifges(16,3) * t168) / 0.2e1 + t35 * (Ifges(14,1) * t35 + Ifges(14,4) * t32 + Ifges(14,5) * t40) / 0.2e1 + t228 * (-mrSges(15,1) * t168 + mrSges(15,2) * t164);
t434 = -t199 + t200;
t439 = -t224 - t225;
t444 = t197 ^ 2;
t445 = t434 ^ 2;
t446 = t439 ^ 2;
t454 = t234 * t381;
t456 = pkin(7) * t54 - t372 + t373;
t457 = t232 * t456;
t459 = pkin(10) * t242 - t454 + t457;
t462 = -pkin(10) * t239 - t224 - t225 - t226 - t397;
t464 = t259 * t459 + t261 * t462;
t471 = pkin(19) * t81 - t192 + t193;
t473 = t139 * t190 + t141 * t471;
t475 = t141 * t190;
t476 = t139 * t471;
t478 = pkin(14) * t149 - t475 + t476;
t480 = -t317 * t473 - t319 * t478;
t481 = t480 ^ 2;
t484 = -t317 * t478 + t319 * t473;
t485 = t484 ^ 2;
t486 = t16 * pkin(19);
t488 = -pkin(14) * t147 - t224 - t486;
t489 = t488 ^ 2;
t505 = t105 * t186;
t506 = t103 * t188;
t507 = -t505 - t506;
t512 = t464 ^ 2;
t515 = t259 * t462 - t261 * t459;
t516 = t515 ^ 2;
t519 = t232 * t381 + t234 * t456;
t520 = t519 ^ 2;
t528 = t434 * (mrSges(5,1) * t65 - mrSges(5,3) * t18) + t439 * (-mrSges(5,1) * t23 + mrSges(5,2) * t18) + m(5) * (t444 + t445 + t446) / 0.2e1 + t197 * (-mrSges(5,2) * t65 + mrSges(5,3) * t23) + t464 * (-mrSges(8,2) * t269 + mrSges(8,3) * t267) + m(12) * (t481 + t485 + t489) / 0.2e1 + t488 * (-mrSges(12,1) * t325 + mrSges(12,2) * t321) + t484 * (mrSges(12,1) * t327 - mrSges(12,3) * t321) + t480 * (-mrSges(12,2) * t327 + mrSges(12,3) * t325) + t507 * (-mrSges(9,1) * t39 - mrSges(9,3) * t107) + m(8) * (t512 + t516 + t520) / 0.2e1 + t519 * (-mrSges(8,1) * t267 + mrSges(8,2) * t263);
t548 = t473 ^ 2;
t549 = -t475 + t476;
t550 = t549 ^ 2;
t551 = -t224 - t486;
t552 = t551 ^ 2;
t558 = -t102 * t186 + t109 * t188;
t561 = pkin(17) * t39 - t505 - t506;
t563 = -t345 * t558 + t347 * t561;
t570 = t345 * t561 + t347 * t558;
t576 = pkin(17) * t112 - t224;
t581 = t570 ^ 2;
t582 = t563 ^ 2;
t583 = t576 ^ 2;
t587 = -t454 + t457;
t588 = t587 ^ 2;
t589 = t398 ^ 2;
t601 = t515 * (mrSges(8,1) * t269 - mrSges(8,3) * t263) - t519 * (mrSges(7,1) * t242 - mrSges(7,3) * t239) + t112 * (-Ifges(9,6) * qJD(1) * t38 + Ifges(9,4) * t107 + Ifges(9,2) * t112) / 0.2e1 + t217 * (-mrSges(15,2) * t170 + mrSges(15,3) * t168) + m(11) * (t548 + t550 + t552) / 0.2e1 + t563 * (mrSges(10,1) * t355 - mrSges(10,3) * t352) + t570 * (-mrSges(10,2) * t355 + mrSges(10,3) * t349) + t576 * (-mrSges(10,1) * t349 + mrSges(10,2) * t352) + m(10) * (t581 + t582 + t583) / 0.2e1 + m(7) * (t588 + t520 + t589) / 0.2e1 + t587 * (-mrSges(7,2) * t242 + mrSges(7,3) * t236) + t549 * (mrSges(11,1) * t149 - mrSges(11,3) * t143);
t603 = t212 + t213;
t608 = t204 ^ 2;
t609 = t208 ^ 2;
t621 = t210 ^ 2;
t622 = t603 ^ 2;
t638 = t38 ^ 2;
t640 = pkin(11) ^ 2;
t642 = t5 ^ 2;
t644 = t642 * t158 * t640;
t648 = t190 ^ 2;
t649 = -t192 + t193;
t650 = t649 ^ 2;
t662 = t603 * (mrSges(14,1) * t40 - mrSges(14,3) * t35) + m(13) * (t608 + t609 + t385) / 0.2e1 + t384 * (-mrSges(14,1) * t32 + mrSges(14,2) * t35) + t384 * (-mrSges(13,1) * t30 + mrSges(13,2) * t25) + m(14) * (t621 + t622 + t385) / 0.2e1 + t210 * (-mrSges(14,2) * t40 + mrSges(14,3) * t32) + t473 * (-mrSges(11,2) * t149 + mrSges(11,3) * t147) + t221 * (mrSges(15,1) * t170 - mrSges(15,3) * t164) + m(3) * (t158 * t638 * t640 + t644) / 0.2e1 + m(4) * (t648 + t650 + t644) / 0.2e1 + t190 * (-mrSges(4,2) * t81 + mrSges(4,3) * t16) + t649 * (mrSges(4,1) * t81 - mrSges(4,3) * t10);
t663 = t558 ^ 2;
t664 = t507 ^ 2;
t731 = m(9) * (t663 + t664 + t644) / 0.2e1 + t558 * (mrSges(9,2) * t39 + mrSges(9,3) * t112) + t298 * (-mrSges(16,1) * t170 + mrSges(16,2) * t164) + t296 * (-mrSges(16,1) * t168 - mrSges(16,3) * t164) + t293 * (mrSges(16,2) * t168 + mrSges(16,3) * t170) + t551 * (-mrSges(11,1) * t147 + mrSges(11,2) * t143) - t223 * (-Ifges(3,1) * qJD(1) * t5 + Ifges(3,4) * qJD(1) * t38 + Ifges(3,5) * qJD(2)) / 0.2e1 + t39 * (-Ifges(3,4) * qJD(1) * t5 + Ifges(3,2) * qJD(1) * t38 + Ifges(3,6) * qJD(2)) / 0.2e1 - t39 * (-Ifges(9,3) * qJD(1) * t38 + Ifges(9,5) * t107 + Ifges(9,6) * t112) / 0.2e1 - t223 * pkin(11) * (-mrSges(9,1) * t112 + mrSges(9,2) * t107) + t39 * pkin(11) * (-qJD(2) * mrSges(3,2) + mrSges(3,3) * t39) + t223 * pkin(11) * (qJD(2) * mrSges(3,1) + mrSges(3,3) * t223) - t223 * pkin(11) * (-t16 * mrSges(4,1) + t10 * mrSges(4,2));
t734 = t123 / 0.2e1 + t251 / 0.2e1 + t344 + t431 + t528 + t601 + t662 + t731;
T = t734;
