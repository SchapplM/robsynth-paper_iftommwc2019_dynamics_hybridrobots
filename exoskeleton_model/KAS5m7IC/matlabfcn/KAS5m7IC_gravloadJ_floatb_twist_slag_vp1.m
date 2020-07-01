% Calculate Gravitation load on the joints for
% KAS5m7IC
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [20x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% m [16x1]
%   mass of all robot links (including the base)
% rSges [16x3]
%   center of mass of all robot links (in body frames)
%   rows: links of the robot (starting with base)
%   columns: x-, y-, z-coordinates
% 
% Output:
% taug [5x1]
%   joint torques required to compensate gravitation load

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:50
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function taug = KAS5m7IC_gravloadJ_floatb_twist_slag_vp1(qJ, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(20,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp1: qJ has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp1: pkin has to be [20x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7IC_gravloadJ_floatb_twist_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From gravload_joint_floatb_twist_par1_ic_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 18:49:43
% EndTime: 2020-06-30 18:49:43
% DurationCPUTime: 0.57s
% Computational Cost: add. (3559->994), mult. (2759->994), div. (12->7), fcn. (2742->65), ass. (0->362)
unknown=NaN(5,1);
t1 = cos(qJ(1));
t3 = sin(qJ(1));
t13 = t3 * pkin(12);
t14 = sin(qJ(2));
t15 = t1 * t14;
t17 = cos(qJ(2));
t18 = t1 * t17;
t23 = t1 * pkin(12);
t24 = t3 * t14;
t26 = t3 * t17;
t33 = t18 * pkin(17);
t34 = cos(qJ(3));
t36 = sin(qJ(3));
t37 = t3 * t36;
t38 = -t15 * t34 - t37;
t41 = t3 * t34;
t42 = t15 * t36 - t41;
t47 = t26 * pkin(17);
t49 = t1 * t36;
t50 = -t24 * t34 + t49;
t53 = t1 * t34;
t54 = t24 * t36 + t53;
t61 = t34 * pkin(19);
t64 = qJ(3) + qJ(4);
t65 = cos(t64);
t67 = sin(t64);
t69 = -t15 * t65 - t3 * t67;
t73 = t15 * t67 - t3 * t65;
t82 = t1 * t67 - t24 * t65;
t86 = t1 * t65 + t24 * t67;
t93 = pkin(7) * t65;
t94 = t93 + t61;
t95 = t15 * t94;
t96 = pkin(7) * t67;
t97 = t36 * pkin(19);
t98 = t96 + t97;
t99 = t3 * t98;
t100 = qJ(3) + qJ(4) + qJ(5);
t101 = cos(t100);
t103 = sin(t100);
t105 = -t15 * t101 - t3 * t103;
t109 = -t3 * t101 + t15 * t103;
t114 = t24 * t94;
t115 = t1 * t98;
t118 = t1 * t103 - t24 * t101;
t122 = t1 * t101 + t24 * t103;
t129 = pkin(8) * t101;
t130 = t129 + t93 + t61;
t131 = t15 * t130;
t132 = pkin(8) * t103;
t133 = t132 + t96 + t97;
t134 = t3 * t133;
t135 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t136 = sin(t135);
t138 = cos(t135);
t140 = t15 * t136 - t3 * t138;
t144 = t3 * t136 + t15 * t138;
t149 = t24 * t130;
t150 = t1 * t133;
t153 = t1 * t138 + t24 * t136;
t157 = -t1 * t136 + t24 * t138;
t165 = cos(qJ(7));
t167 = sin(qJ(7));
t168 = t18 * t167;
t172 = t18 * t165;
t181 = t153 * t165 + t26 * t167;
t185 = -t153 * t167 + t26 * t165;
t192 = sin(pkin(4));
t194 = cos(pkin(4));
t195 = t3 * t194;
t206 = t1 * t194;
t218 = t192 * pkin(18);
t221 = pkin(4) + qJ(8);
t222 = sin(t221);
t224 = cos(t221);
t226 = t15 * t222 - t3 * t224;
t230 = t15 * t224 + t3 * t222;
t239 = t1 * t224 + t24 * t222;
t243 = -t1 * t222 + t24 * t224;
t250 = t34 * pkin(20);
t253 = qJ(3) + qJ(9);
t254 = cos(t253);
t256 = sin(t253);
t258 = -t15 * t254 - t3 * t256;
t262 = t15 * t256 - t3 * t254;
t271 = t1 * t256 - t24 * t254;
t275 = t1 * t254 + t24 * t256;
t283 = pkin(15) * t254 + t250;
t285 = pkin(15) * t256;
t286 = t36 * pkin(20);
t287 = t285 + t286;
t289 = qJ(3) + qJ(9) + qJ(10);
t290 = cos(t289);
t292 = sin(t289);
t294 = t15 * t290 + t3 * t292;
t298 = -t15 * t292 + t3 * t290;
t307 = -t1 * t292 + t24 * t290;
t311 = -t1 * t290 - t24 * t292;
t318 = qJ(3) + qJ(4) + qJ(11);
t319 = sin(t318);
t321 = cos(t318);
t323 = -t15 * t319 + t3 * t321;
t327 = -t15 * t321 - t3 * t319;
t334 = -t1 * t321 - t24 * t319;
t338 = t1 * t319 - t24 * t321;
t345 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t346 = cos(t345);
t348 = sin(t345);
t350 = t15 * t346 + t3 * t348;
t354 = -t15 * t348 + t3 * t346;
t361 = -t1 * t348 + t24 * t346;
t365 = -t1 * t346 - t24 * t348;
t372 = pkin(10) * t346;
t373 = t93 - t372 + t61;
t374 = t15 * t373;
t375 = pkin(10) * t348;
t376 = t96 - t375 + t97;
t377 = t3 * t376;
t378 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t379 = sin(t378);
t381 = cos(t378);
t383 = t15 * t379 - t3 * t381;
t387 = t15 * t381 + t3 * t379;
t392 = t24 * t373;
t393 = t1 * t376;
t396 = t1 * t381 + t24 * t379;
t400 = -t1 * t379 + t24 * t381;
t421 = -m(2) * (g(1) * (t1 * rSges(2,1) - t3 * rSges(2,2)) + g(2) * (t3 * rSges(2,1) + t1 * rSges(2,2))) - m(3) * (g(1) * (-t15 * rSges(3,1) - t18 * rSges(3,2) - t3 * rSges(3,3) - t13) + g(2) * (-t24 * rSges(3,1) - t26 * rSges(3,2) + t1 * rSges(3,3) + t23)) - m(4) * (g(1) * (t38 * rSges(4,1) + t42 * rSges(4,2) + t18 * rSges(4,3) - t13 + t33) + g(2) * (t50 * rSges(4,1) + t54 * rSges(4,2) + t26 * rSges(4,3) + t23 + t47)) - m(5) * (g(1) * (t69 * rSges(5,1) + t73 * rSges(5,2) + t18 * rSges(5,3) - t37 * pkin(19) - t15 * t61 - t13 + t33) + g(2) * (t82 * rSges(5,1) + t86 * rSges(5,2) + t26 * rSges(5,3) + t49 * pkin(19) - t24 * t61 + t23 + t47)) - m(6) * (g(1) * (t105 * rSges(6,1) + t109 * rSges(6,2) + t18 * rSges(6,3) - t13 + t33 - t95 - t99) + g(2) * (t118 * rSges(6,1) + t122 * rSges(6,2) + t26 * rSges(6,3) - t114 + t115 + t23 + t47)) - m(7) * (g(1) * (t140 * rSges(7,1) + t144 * rSges(7,2) + t18 * rSges(7,3) - t13 - t131 - t134 + t33) + g(2) * (t153 * rSges(7,1) + t157 * rSges(7,2) + t26 * rSges(7,3) - t149 + t150 + t23 + t47)) - m(8) * (g(1) * (-t144 * pkin(11) - t131 + t33 - t134 - t13 + (t140 * t165 + t168) * rSges(8,1) + (-t140 * t167 + t172) * rSges(8,2) - t144 * rSges(8,3)) + g(2) * (t181 * rSges(8,1) + t185 * rSges(8,2) - t157 * rSges(8,3) - t157 * pkin(11) - t149 + t150 + t23 + t47)) - m(9) * (g(1) * (t33 - t13 + (t15 * t192 - t195) * rSges(9,1) + (t15 * t194 + t3 * t192) * rSges(9,2) + t18 * rSges(9,3)) + g(2) * (t23 + t47 + (t24 * t192 + t206) * rSges(9,1) + (-t1 * t192 + t24 * t194) * rSges(9,2) + t26 * rSges(9,3))) - m(10) * (g(1) * (t226 * rSges(10,1) + t230 * rSges(10,2) + t18 * rSges(10,3) + t195 * pkin(18) - t15 * t218 - t13 + t33) + g(2) * (t239 * rSges(10,1) + t243 * rSges(10,2) + t26 * rSges(10,3) - t206 * pkin(18) - t24 * t218 + t23 + t47)) - m(11) * (g(1) * (t258 * rSges(11,1) + t262 * rSges(11,2) + t18 * rSges(11,3) - t37 * pkin(20) - t15 * t250 - t13 + t33) + g(2) * (t271 * rSges(11,1) + t275 * rSges(11,2) + t26 * rSges(11,3) + t49 * pkin(20) - t24 * t250 + t23 + t47)) - m(12) * (g(1) * (t294 * rSges(12,1) + t298 * rSges(12,2) + t18 * rSges(12,3) - t15 * t283 - t3 * t287 - t13 + t33) + g(2) * (t307 * rSges(12,1) + t311 * rSges(12,2) + t26 * rSges(12,3) + t1 * t287 - t24 * t283 + t23 + t47)) - m(13) * (g(1) * (t323 * rSges(13,1) + t327 * rSges(13,2) + t18 * rSges(13,3) - t13 + t33 - t95 - t99) + g(2) * (t334 * rSges(13,1) + t338 * rSges(13,2) + t26 * rSges(13,3) - t114 + t115 + t23 + t47)) - m(14) * (g(1) * (t350 * rSges(14,1) + t354 * rSges(14,2) + t18 * rSges(14,3) - t13 + t33 - t95 - t99) + g(2) * (t361 * rSges(14,1) + t365 * rSges(14,2) + t26 * rSges(14,3) - t114 + t115 + t23 + t47)) - m(15) * (g(1) * (t383 * rSges(15,1) + t387 * rSges(15,2) + t18 * rSges(15,3) - t13 + t33 - t374 - t377) + g(2) * (t396 * rSges(15,1) + t400 * rSges(15,2) + t26 * rSges(15,3) + t23 - t392 + t393 + t47)) - m(16) * (g(1) * (t383 * rSges(16,1) + t18 * rSges(16,2) - t387 * rSges(16,3) - t387 * qJ(13) - t13 + t33 - t374 - t377) + g(2) * (t396 * rSges(16,1) + t26 * rSges(16,2) - t400 * rSges(16,3) - t400 * qJ(13) + t23 - t392 + t393 + t47));
t436 = t24 * pkin(17);
t437 = t34 * rSges(4,1);
t439 = t36 * rSges(4,2);
t444 = t15 * pkin(17);
t450 = t17 * pkin(17);
t451 = t14 * t34;
t461 = t65 * rSges(5,1);
t463 = t67 * rSges(5,2);
t484 = t26 * t94;
t485 = t101 * rSges(6,1);
t487 = t103 * rSges(6,2);
t492 = t18 * t94;
t498 = t14 * t94;
t508 = t26 * t130;
t509 = t136 * rSges(7,1);
t511 = t138 * rSges(7,2);
t516 = t18 * t130;
t522 = t14 * t130;
t523 = t14 * t136;
t525 = t14 * t138;
t532 = t138 * pkin(11);
t534 = t136 * t165;
t539 = t136 * t167;
t544 = t138 * rSges(8,3);
t574 = t192 * rSges(9,1);
t576 = t194 * rSges(9,2);
t586 = t14 * t192;
t596 = t222 * rSges(10,1);
t598 = t224 * rSges(10,2);
t620 = t254 * rSges(11,1);
t622 = t256 * rSges(11,2);
t644 = t290 * rSges(12,1);
t646 = t292 * rSges(12,2);
t667 = t319 * rSges(13,1);
t669 = t321 * rSges(13,2);
t688 = t346 * rSges(14,1);
t690 = t348 * rSges(14,2);
t709 = t26 * t373;
t710 = t379 * rSges(15,1);
t712 = t381 * rSges(15,2);
t717 = t18 * t373;
t723 = t14 * t373;
t724 = t14 * t379;
t726 = t14 * t381;
t733 = t381 * qJ(13);
t735 = t379 * rSges(16,1);
t738 = t381 * rSges(16,3);
t756 = -m(3) * (g(1) * (-t26 * rSges(3,1) + t24 * rSges(3,2)) + g(2) * (t18 * rSges(3,1) - t15 * rSges(3,2)) + g(3) * (t14 * rSges(3,1) + t17 * rSges(3,2))) - m(4) * (g(1) * (-t24 * rSges(4,3) - t26 * t437 + t26 * t439 - t436) + g(2) * (t15 * rSges(4,3) + t18 * t437 - t18 * t439 + t444) + g(3) * (-t14 * t36 * rSges(4,2) + t451 * rSges(4,1) - t17 * rSges(4,3) - t450)) - m(5) * (g(1) * (-t24 * rSges(5,3) - t26 * t461 + t26 * t463 - t26 * t61 - t436) + g(2) * (t15 * rSges(5,3) + t18 * t461 - t18 * t463 + t18 * t61 + t444) + g(3) * (t14 * t65 * rSges(5,1) - t14 * t67 * rSges(5,2) - t17 * rSges(5,3) + t451 * pkin(19) - t450)) - m(6) * (g(1) * (-t24 * rSges(6,3) - t26 * t485 + t26 * t487 - t436 - t484) + g(2) * (t15 * rSges(6,3) + t18 * t485 - t18 * t487 + t444 + t492) + g(3) * (t14 * t101 * rSges(6,1) - t14 * t103 * rSges(6,2) - t17 * rSges(6,3) - t450 + t498)) - m(7) * (g(1) * (-t24 * rSges(7,3) + t26 * t509 + t26 * t511 - t436 - t508) + g(2) * (t15 * rSges(7,3) - t18 * t509 - t18 * t511 + t444 + t516) + g(3) * (-t523 * rSges(7,1) - t525 * rSges(7,2) - t17 * rSges(7,3) - t450 + t522)) - m(8) * (g(1) * (-t26 * t532 - t508 - t436 + (-t24 * t167 + t26 * t534) * rSges(8,1) + (-t24 * t165 - t26 * t539) * rSges(8,2) - t26 * t544) + g(2) * (t18 * t532 + t516 + t444 + (t15 * t167 - t18 * t534) * rSges(8,1) + (t15 * t165 + t18 * t539) * rSges(8,2) + t18 * t544) + g(3) * (t525 * pkin(11) + t522 - t450 + (-t523 * t165 - t17 * t167) * rSges(8,1) + (-t17 * t165 + t523 * t167) * rSges(8,2) + t525 * rSges(8,3))) - m(9) * (g(1) * (-t24 * rSges(9,3) + t26 * t574 + t26 * t576 - t436) + g(2) * (t15 * rSges(9,3) - t18 * t574 - t18 * t576 + t444) + g(3) * (-t14 * t194 * rSges(9,2) - t586 * rSges(9,1) - t17 * rSges(9,3) - t450)) - m(10) * (g(1) * (-t24 * rSges(10,3) - t26 * t218 + t26 * t596 + t26 * t598 - t436) + g(2) * (t15 * rSges(10,3) + t18 * t218 - t18 * t596 - t18 * t598 + t444) + g(3) * (-t14 * t222 * rSges(10,1) - t14 * t224 * rSges(10,2) - t17 * rSges(10,3) + t586 * pkin(18) - t450)) - m(11) * (g(1) * (-t24 * rSges(11,3) - t26 * t250 - t26 * t620 + t26 * t622 - t436) + g(2) * (t15 * rSges(11,3) + t18 * t250 + t18 * t620 - t18 * t622 + t444) + g(3) * (t14 * t254 * rSges(11,1) - t14 * t256 * rSges(11,2) - t17 * rSges(11,3) + t451 * pkin(20) - t450)) - m(12) * (g(1) * (-t24 * rSges(12,3) - t26 * t283 + t26 * t644 - t26 * t646 - t436) + g(2) * (t15 * rSges(12,3) + t18 * t283 - t18 * t644 + t18 * t646 + t444) + g(3) * (-t14 * t290 * rSges(12,1) + t14 * t292 * rSges(12,2) - t17 * rSges(12,3) + t14 * t283 - t450)) - m(13) * (g(1) * (-t24 * rSges(13,3) - t26 * t667 - t26 * t669 - t436 - t484) + g(2) * (t15 * rSges(13,3) + t18 * t667 + t18 * t669 + t444 + t492) + g(3) * (t14 * t319 * rSges(13,1) + t14 * t321 * rSges(13,2) - t17 * rSges(13,3) - t450 + t498)) - m(14) * (g(1) * (-t24 * rSges(14,3) + t26 * t688 - t26 * t690 - t436 - t484) + g(2) * (t15 * rSges(14,3) - t18 * t688 + t18 * t690 + t444 + t492) + g(3) * (-t14 * t346 * rSges(14,1) + t14 * t348 * rSges(14,2) - t17 * rSges(14,3) - t450 + t498)) - m(15) * (g(1) * (-t24 * rSges(15,3) + t26 * t710 + t26 * t712 - t436 - t709) + g(2) * (t15 * rSges(15,3) - t18 * t710 - t18 * t712 + t444 + t717) + g(3) * (-t724 * rSges(15,1) - t726 * rSges(15,2) - t17 * rSges(15,3) - t450 + t723)) - m(16) * (g(1) * (-t24 * rSges(16,2) - t26 * t733 + t26 * t735 - t26 * t738 - t436 - t709) + g(2) * (t15 * rSges(16,2) + t18 * t733 - t18 * t735 + t18 * t738 + t444 + t717) + g(3) * (-t724 * rSges(16,1) - t17 * rSges(16,2) + t726 * rSges(16,3) + t726 * qJ(13) - t450 + t723));
t757 = 0.2e1 * qJ(9);
t759 = sin(qJ(4) + qJ(11) + pkin(1) - t757 - qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3));
t761 = qJ(4) + qJ(11) + pkin(1);
t762 = sin(t761);
t764 = sin(qJ(4));
t766 = -t764 * pkin(7) + t762 * pkin(10);
t767 = -t757 - qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3);
t768 = cos(t767);
t770 = cos(qJ(4));
t772 = cos(t761);
t773 = t772 * pkin(10);
t774 = -t770 * pkin(7) + t773;
t775 = sin(t767);
t778 = sin(qJ(4) + qJ(11) + pkin(1) - qJ(10) + pkin(5) - pkin(4) - qJ(8) + qJ(3));
t780 = qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3);
t781 = cos(t780);
t783 = sin(t780);
t787 = 0.1e1 / pkin(15);
t789 = sin(-qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3) - qJ(9));
t790 = t789 * pkin(14);
t792 = sin(qJ(10) - pkin(5) + pkin(4) + qJ(8) - qJ(3) - qJ(9));
t793 = t792 * pkin(14);
t795 = sin(pkin(4) + qJ(8) - qJ(3) - qJ(10));
t797 = sin(pkin(4) + qJ(8) - qJ(3) + qJ(10));
t811 = t17 * t36;
t821 = t86 * rSges(5,1);
t822 = -t82 * rSges(5,2);
t827 = -t73 * rSges(5,1);
t828 = t69 * rSges(5,2);
t833 = t17 * t67 * rSges(5,1);
t835 = t17 * t65 * rSges(5,2);
t840 = -t24 * t98;
t841 = t1 * t94;
t842 = t122 * rSges(6,1);
t843 = -t118 * rSges(6,2);
t846 = -t15 * t98;
t847 = t3 * t94;
t848 = -t109 * rSges(6,1);
t849 = t105 * rSges(6,2);
t852 = -t17 * t98;
t854 = t17 * t103 * rSges(6,1);
t856 = t17 * t101 * rSges(6,2);
t861 = -t24 * t133;
t862 = t1 * t130;
t863 = t157 * rSges(7,1);
t864 = -t153 * rSges(7,2);
t867 = -t15 * t133;
t868 = t3 * t130;
t869 = -t144 * rSges(7,1);
t870 = t140 * rSges(7,2);
t873 = -t17 * t133;
t874 = t17 * t138;
t875 = t874 * rSges(7,1);
t876 = t17 * t136;
t877 = t876 * rSges(7,2);
t882 = -t153 * pkin(11);
t884 = t157 * t165 * rSges(8,1);
t886 = t157 * t167 * rSges(8,2);
t887 = t153 * rSges(8,3);
t890 = t140 * pkin(11);
t892 = -t144 * t165 * rSges(8,1);
t894 = -t144 * t167 * rSges(8,2);
t895 = -t140 * rSges(8,3);
t898 = t876 * pkin(11);
t900 = t874 * t165 * rSges(8,1);
t902 = t874 * t167 * rSges(8,2);
t903 = t876 * rSges(8,3);
t910 = t275 * rSges(11,1);
t911 = -t271 * rSges(11,2);
t916 = -t262 * rSges(11,1);
t917 = t258 * rSges(11,2);
t922 = t17 * t256 * rSges(11,1);
t924 = t17 * t254 * rSges(11,2);
t931 = t311 * rSges(12,1);
t932 = -t307 * rSges(12,2);
t937 = -t298 * rSges(12,1);
t938 = t294 * rSges(12,2);
t943 = t17 * t292 * rSges(12,1);
t945 = t17 * t290 * rSges(12,2);
t950 = t338 * rSges(13,1);
t951 = -t334 * rSges(13,2);
t954 = -t327 * rSges(13,1);
t955 = t323 * rSges(13,2);
t959 = t17 * t321 * rSges(13,1);
t961 = t17 * t319 * rSges(13,2);
t966 = t365 * rSges(14,1);
t967 = -t361 * rSges(14,2);
t970 = -t354 * rSges(14,1);
t971 = t350 * rSges(14,2);
t975 = t17 * t348 * rSges(14,1);
t977 = t17 * t346 * rSges(14,2);
t982 = -t24 * t376;
t983 = t1 * t373;
t984 = t400 * rSges(15,1);
t985 = -t396 * rSges(15,2);
t988 = -t15 * t376;
t989 = t3 * t373;
t990 = -t387 * rSges(15,1);
t991 = t383 * rSges(15,2);
t994 = -t17 * t376;
t995 = t17 * t381;
t996 = t995 * rSges(15,1);
t997 = t17 * t379;
t998 = t997 * rSges(15,2);
t1003 = -t396 * qJ(13);
t1004 = t400 * rSges(16,1);
t1005 = t396 * rSges(16,3);
t1008 = t383 * qJ(13);
t1009 = -t387 * rSges(16,1);
t1010 = -t383 * rSges(16,3);
t1013 = t997 * qJ(13);
t1014 = t995 * rSges(16,1);
t1015 = t997 * rSges(16,3);
t1020 = -m(4) * (g(1) * (t54 * rSges(4,1) - t50 * rSges(4,2)) + g(2) * (-t42 * rSges(4,1) + t38 * rSges(4,2)) + g(3) * (t17 * t34 * rSges(4,2) + t811 * rSges(4,1))) - m(5) * (g(1) * (t53 * pkin(19) + t24 * t97 + t821 + t822) + g(2) * (t41 * pkin(19) - t15 * t97 + t827 + t828) + g(3) * (t811 * pkin(19) + t833 + t835)) - m(6) * (g(1) * (-t840 + t841 + t842 + t843) + g(2) * (t846 + t847 + t848 + t849) + g(3) * (-t852 + t854 + t856)) - m(7) * (g(1) * (-t861 + t862 + t863 + t864) + g(2) * (t867 + t868 + t869 + t870) + g(3) * (-t873 + t875 - t877)) - m(8) * (g(1) * (-t882 - t861 + t862 + t884 - t886 + t887) + g(2) * (-t890 + t867 + t868 + t892 - t894 + t895) + g(3) * (t898 - t873 + t900 - t902 + t903)) - m(11) * (g(1) * (t53 * pkin(20) + t24 * t286 + t910 + t911) + g(2) * (t41 * pkin(20) - t15 * t286 + t916 + t917) + g(3) * (t811 * pkin(20) + t922 + t924)) - m(12) * (g(1) * (t1 * t283 + t24 * t287 + t931 + t932) + g(2) * (-t15 * t287 + t3 * t283 + t937 + t938) + g(3) * (t17 * t287 - t943 - t945)) - m(13) * (g(1) * (-t840 + t841 + t950 + t951) + g(2) * (t846 + t847 + t954 + t955) + g(3) * (-t852 - t959 + t961)) - m(14) * (g(1) * (-t840 + t841 + t966 + t967) + g(2) * (t846 + t847 + t970 + t971) + g(3) * (-t852 - t975 - t977)) - m(15) * (g(1) * (-t982 + t983 + t984 + t985) + g(2) * (t988 + t989 + t990 + t991) + g(3) * (-t994 + t996 - t998)) - m(16) * (g(1) * (-t1003 - t982 + t983 + t1004 + t1005) + g(2) * (-t1008 + t988 + t989 + t1009 + t1010) + g(3) * (t1013 - t994 + t1014 + t1015));
t1031 = t24 * t96;
t1033 = t1 * pkin(7) * t65;
t1036 = t15 * t96;
t1038 = t3 * pkin(7) * t65;
t1042 = t17 * pkin(7) * t67;
t1047 = -t132 - t96;
t1048 = t24 * t1047;
t1049 = t129 + t93;
t1050 = t1 * t1049;
t1053 = t15 * t1047;
t1054 = t3 * t1049;
t1057 = t17 * t1047;
t1086 = -t96 + t375;
t1087 = t24 * t1086;
t1088 = t93 - t372;
t1089 = t1 * t1088;
t1092 = t15 * t1086;
t1093 = t3 * t1088;
t1096 = t17 * t1086;
t1116 = m(7) * (g(1) * (t863 + t864) + g(2) * (t869 + t870) + g(3) * (t875 - t877));
t1124 = m(8) * (g(1) * (-t882 + t884 - t886 + t887) + g(2) * (-t890 + t892 - t894 + t895) + g(3) * (t898 + t900 - t902 + t903));
t1126 = t757 + pkin(5) + qJ(10);
t1127 = cos(t1126);
t1132 = -qJ(10) + pkin(5);
t1133 = cos(t1132);
t1138 = sin(t1132);
t1143 = sin(t1126);
t1149 = cos(pkin(5) + qJ(4) + qJ(11) + pkin(1) - qJ(10));
t1152 = cos(-t757 - pkin(5) + qJ(4) + qJ(11) + pkin(1) - qJ(10));
t1181 = qJ(9) + qJ(10);
t1182 = sin(t1181);
t1187 = cos(t1181);
t1193 = sin(qJ(4) + qJ(11) + pkin(1) - qJ(9) - qJ(10));
t1196 = sin(qJ(10));
t1197 = 0.1e1 / t1196;
t1229 = sin(-qJ(9) + qJ(4) + qJ(11) + pkin(1));
t1233 = sin(qJ(9));
t1238 = cos(qJ(9));
t1283 = t24 * t375;
t1285 = t1 * pkin(10) * t346;
t1288 = t15 * t375;
t1290 = t3 * pkin(10) * t346;
t1294 = t17 * pkin(10) * t348;
t1307 = qJ(11) + pkin(1);
t1308 = sin(t1307);
t1309 = qJ(11) + pkin(1) + qJ(12);
t1310 = sin(t1309);
t1313 = cos(t1307);
t1314 = cos(t1309);
t1317 = -qJ(5) - qJ(6) + pkin(3);
t1318 = cos(t1317);
t1322 = sin(t1317);
t1325 = 0.1e1 / qJ(13);
t1343 = -m(15) * (g(1) * (t984 + t985) + g(2) * (t990 + t991) + g(3) * (t996 - t998)) - m(16) * (g(1) * (-t1003 + t1004 + t1005) + g(2) * (-t1008 + t1009 + t1010) + g(3) * (t1013 + t1014 + t1015));
t1359 = -g(3) * t17 * t381 - g(1) * t400 + g(2) * t387;
t1361 = pkin(14) * (pkin(10) * t759 + pkin(10) * t778 + t766 * t768 + t766 * t781 + t774 * t775 - t783 * t774) * t787 / (t790 - t793 + pkin(20) * (t795 - t797)) * t1020 - m(5) * (g(1) * (t821 + t822) + g(2) * (t827 + t828) + g(3) * (t833 + t835)) - m(6) * (g(1) * (t1031 + t1033 + t842 + t843) + g(2) * (-t1036 + t1038 + t848 + t849) + g(3) * (t1042 + t854 + t856)) - m(7) * (g(1) * (-t1048 + t1050 + t863 + t864) + g(2) * (t1053 + t1054 + t869 + t870) + g(3) * (-t1057 + t875 - t877)) - m(8) * (g(1) * (-t882 - t1048 + t1050 + t884 - t886 + t887) + g(2) * (-t890 + t1053 + t1054 + t892 - t894 + t895) + g(3) * (t898 - t1057 + t900 - t902 + t903)) - m(13) * (g(1) * (t1031 + t1033 + t950 + t951) + g(2) * (-t1036 + t1038 + t954 + t955) + g(3) * (t1042 - t959 + t961)) - m(14) * (g(1) * (t1031 + t1033 + t966 + t967) + g(2) * (-t1036 + t1038 + t970 + t971) + g(3) * (t1042 - t975 - t977)) - m(15) * (g(1) * (-t1087 + t1089 + t984 + t985) + g(2) * (t1092 + t1093 + t990 + t991) + g(3) * (-t1096 + t996 - t998)) - m(16) * (g(1) * (-t1003 - t1087 + t1089 + t1004 + t1005) + g(2) * (-t1008 + t1092 + t1093 + t1009 + t1010) + g(3) * (t1013 - t1096 + t1014 + t1015)) + t1116 + t1124 - pkin(20) * pkin(14) * (-t1127 * t770 * pkin(7) + t1133 * t770 * pkin(7) - t1138 * t764 * pkin(7) - t1143 * t764 * pkin(7) + t1127 * t772 * pkin(10) - t1133 * t772 * pkin(10) + t1138 * t762 * pkin(10) + t1143 * t762 * pkin(10) - pkin(10) * t1149 + pkin(10) * t1152) / pkin(13) * t787 / (-t795 * pkin(20) + t797 * pkin(20) - t790 + t793) * m(10) * (g(1) * (t243 * rSges(10,1) - t239 * rSges(10,2)) + g(2) * (-t230 * rSges(10,1) + t226 * rSges(10,2)) + g(3) * (t17 * t224 * rSges(10,1) - t17 * t222 * rSges(10,2))) + (t1182 * t770 * pkin(7) - t1187 * t764 * pkin(7) - t1182 * t772 * pkin(10) + t1187 * t762 * pkin(10) + pkin(10) * t1193) * t1197 * t787 * (-m(11) * (g(1) * (t910 + t911) + g(2) * (t916 + t917) + g(3) * (t922 + t924)) - m(12) * (g(1) * (t1 * pkin(15) * t254 + t24 * t285 + t931 + t932) + g(2) * (t3 * pkin(15) * t254 - t15 * t285 + t937 + t938) + g(3) * (t17 * pkin(15) * t256 - t943 - t945))) - (-t1193 * pkin(9) * pkin(10) + t1229 * pkin(10) * pkin(15) - t773 * (-t1182 * pkin(9) + t1233 * pkin(15)) + pkin(10) * (-t1187 * pkin(9) + t1238 * pkin(15)) * t762 + pkin(7) * (t764 * pkin(9) * t1187 - pkin(9) * t770 * t1182 + pkin(15) * (t1233 * t770 - t1238 * t764))) / pkin(9) * t787 * t1197 * m(12) * (g(1) * (t931 + t932) + g(2) * (t937 + t938) + g(3) * (-t943 - t945)) - m(13) * (g(1) * (t950 + t951) + g(2) * (t954 + t955) + g(3) * (-t959 + t961)) - m(14) * (g(1) * (t966 + t967) + g(2) * (t970 + t971) + g(3) * (-t975 - t977)) - m(15) * (g(1) * (-t1283 - t1285 + t984 + t985) + g(2) * (t1288 - t1290 + t990 + t991) + g(3) * (-t1294 + t996 - t998)) - m(16) * (g(1) * (-t1003 - t1283 - t1285 + t1004 + t1005) + g(2) * (-t1008 + t1288 - t1290 + t1009 + t1010) + g(3) * (t1013 - t1294 + t1014 + t1015)) + (t1308 * t1310 * pkin(10) + t1313 * t1314 * pkin(10) + t1310 * pkin(16) * t1322 - pkin(16) * t1318 * t1314 - qJ(13)) * t1325 * t1343 - (-t1308 * t1314 * pkin(10) + t1313 * t1310 * pkin(10) - t1310 * t1318 * pkin(16) - t1314 * t1322 * pkin(16)) * m(16) * t1359;
t1370 = t24 * t132;
t1372 = t1 * pkin(8) * t101;
t1375 = t15 * t132;
t1377 = t3 * pkin(8) * t101;
t1381 = t17 * pkin(8) * t103;
t1394 = qJ(11) + pkin(1) + qJ(12) - qJ(5) - qJ(6) + pkin(3);
t1395 = cos(t1394);
t1398 = qJ(11) + pkin(1) + qJ(12) - qJ(5);
t1399 = cos(t1398);
t1404 = sin(t1394);
t1407 = sin(t1398);
unknown(1,1) = t421;
unknown(2,1) = t756;
unknown(3,1) = t1361;
unknown(4,1) = -m(6) * (g(1) * (t842 + t843) + g(2) * (t848 + t849) + g(3) * (t854 + t856)) - m(7) * (g(1) * (t1370 + t1372 + t863 + t864) + g(2) * (-t1375 + t1377 + t869 + t870) + g(3) * (t1381 + t875 - t877)) - m(8) * (g(1) * (-t882 + t1370 + t1372 + t884 - t886 + t887) + g(2) * (-t890 - t1375 + t1377 + t892 - t894 + t895) + g(3) * (t898 + t1381 + t900 - t902 + t903)) - t1116 - t1124 + (pkin(8) * t1399 + 0.2e1 * pkin(16) * t1395) * t1325 * t1343 - (pkin(8) * t1407 + 0.2e1 * pkin(16) * t1404) * m(16) * t1359;
unknown(5,1) = -m(8) * (g(1) * (t185 * rSges(8,1) - t181 * rSges(8,2)) + g(2) * ((t140 * t167 - t172) * rSges(8,1) + (t140 * t165 + t168) * rSges(8,2)) + g(3) * ((-t14 * t165 - t876 * t167) * rSges(8,1) + (t14 * t167 - t876 * t165) * rSges(8,2)));
taug = unknown(:);
