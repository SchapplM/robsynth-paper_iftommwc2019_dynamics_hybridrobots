% Calculate Gravitation load on the joints for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% m [16x1]
%   mass of all robot links (including the base)
% rSges [16x3]
%   center of mass of all robot links (in body frames)
%   rows: links of the robot (starting with base)
%   columns: x-, y-, z-coordinates
% 
% Output:
% taug [13x1]
%   joint torques required to compensate gravitation load

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function taug = KAS5m7OL_gravloadJ_floatb_twist_slag_vp1(qJ, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(19,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp1: qJ has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp1: pkin has to be [19x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7OL_gravloadJ_floatb_twist_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From gravload_joint_floatb_twist_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:31
% EndTime: 2020-06-30 17:46:33
% DurationCPUTime: 0.54s
% Computational Cost: add. (3065->866), mult. (2507->890), div. (0->0), fcn. (2505->28), ass. (0->308)
unknown=NaN(13,1);
t1 = cos(qJ(1));
t3 = sin(qJ(1));
t13 = t3 * pkin(11);
t14 = sin(qJ(2));
t15 = t1 * t14;
t17 = cos(qJ(2));
t18 = t1 * t17;
t23 = t1 * pkin(11);
t24 = t3 * t14;
t26 = t3 * t17;
t33 = t18 * pkin(16);
t34 = cos(qJ(3));
t36 = sin(qJ(3));
t37 = t3 * t36;
t38 = -t15 * t34 - t37;
t41 = t3 * t34;
t42 = t15 * t36 - t41;
t47 = t26 * pkin(16);
t49 = t1 * t36;
t50 = -t24 * t34 + t49;
t53 = t1 * t34;
t54 = t24 * t36 + t53;
t61 = t34 * pkin(18);
t64 = qJ(3) + qJ(4);
t65 = cos(t64);
t67 = sin(t64);
t69 = -t15 * t65 - t3 * t67;
t73 = t15 * t67 - t3 * t65;
t82 = t1 * t67 - t24 * t65;
t86 = t1 * t65 + t24 * t67;
t93 = pkin(6) * t65;
t94 = t93 + t61;
t95 = t15 * t94;
t96 = pkin(6) * t67;
t97 = t36 * pkin(18);
t98 = t96 + t97;
t99 = t3 * t98;
t100 = qJ(3) + qJ(4) + qJ(5);
t101 = cos(t100);
t103 = sin(t100);
t105 = -t101 * t15 - t103 * t3;
t109 = -t101 * t3 + t103 * t15;
t114 = t24 * t94;
t115 = t1 * t98;
t118 = t1 * t103 - t101 * t24;
t122 = t1 * t101 + t103 * t24;
t129 = pkin(7) * t101;
t130 = t129 + t93 + t61;
t131 = t15 * t130;
t132 = pkin(7) * t103;
t133 = t132 + t96 + t97;
t134 = t3 * t133;
t135 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t136 = sin(t135);
t138 = cos(t135);
t140 = t136 * t15 - t138 * t3;
t144 = t136 * t3 + t138 * t15;
t149 = t24 * t130;
t150 = t1 * t133;
t153 = t1 * t138 + t136 * t24;
t157 = -t1 * t136 + t138 * t24;
t165 = cos(qJ(7));
t167 = sin(qJ(7));
t168 = t18 * t167;
t172 = t18 * t165;
t181 = t153 * t165 + t167 * t26;
t185 = -t153 * t167 + t165 * t26;
t192 = sin(pkin(3));
t194 = cos(pkin(3));
t195 = t3 * t194;
t206 = t1 * t194;
t218 = t192 * pkin(17);
t221 = pkin(3) + qJ(8);
t222 = sin(t221);
t224 = cos(t221);
t226 = t15 * t222 - t224 * t3;
t230 = t15 * t224 + t222 * t3;
t239 = t1 * t224 + t222 * t24;
t243 = -t1 * t222 + t224 * t24;
t250 = t34 * pkin(19);
t253 = qJ(3) + qJ(9);
t254 = cos(t253);
t256 = sin(t253);
t258 = -t15 * t254 - t256 * t3;
t262 = t15 * t256 - t254 * t3;
t271 = t1 * t256 - t24 * t254;
t275 = t1 * t254 + t24 * t256;
t283 = pkin(14) * t254 + t250;
t285 = pkin(14) * t256;
t286 = t36 * pkin(19);
t287 = t285 + t286;
t289 = qJ(3) + qJ(9) + qJ(10);
t290 = cos(t289);
t292 = sin(t289);
t294 = t15 * t290 + t292 * t3;
t298 = -t15 * t292 + t290 * t3;
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
t372 = pkin(9) * t346;
t373 = t93 - t372 + t61;
t374 = t15 * t373;
t375 = pkin(9) * t348;
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
t421 = -m(2) * (g(1) * (rSges(2,1) * t1 - rSges(2,2) * t3) + g(2) * (rSges(2,1) * t3 + rSges(2,2) * t1)) - m(3) * (g(1) * (-rSges(3,1) * t15 - rSges(3,2) * t18 - rSges(3,3) * t3 - t13) + g(2) * (-rSges(3,1) * t24 - rSges(3,2) * t26 + rSges(3,3) * t1 + t23)) - m(4) * (g(1) * (rSges(4,1) * t38 + rSges(4,2) * t42 + rSges(4,3) * t18 - t13 + t33) + g(2) * (rSges(4,1) * t50 + rSges(4,2) * t54 + rSges(4,3) * t26 + t23 + t47)) - m(5) * (g(1) * (rSges(5,1) * t69 + rSges(5,2) * t73 + rSges(5,3) * t18 - pkin(18) * t37 - t15 * t61 - t13 + t33) + g(2) * (rSges(5,1) * t82 + rSges(5,2) * t86 + rSges(5,3) * t26 + pkin(18) * t49 - t24 * t61 + t23 + t47)) - m(6) * (g(1) * (rSges(6,1) * t105 + rSges(6,2) * t109 + rSges(6,3) * t18 - t13 + t33 - t95 - t99) + g(2) * (rSges(6,1) * t118 + rSges(6,2) * t122 + rSges(6,3) * t26 - t114 + t115 + t23 + t47)) - m(7) * (g(1) * (rSges(7,1) * t140 + rSges(7,2) * t144 + rSges(7,3) * t18 - t13 - t131 - t134 + t33) + g(2) * (rSges(7,1) * t153 + rSges(7,2) * t157 + rSges(7,3) * t26 - t149 + t150 + t23 + t47)) - m(8) * (g(1) * (-t144 * pkin(10) - t131 + t33 - t134 - t13 + (t140 * t165 + t168) * rSges(8,1) + (-t140 * t167 + t172) * rSges(8,2) - t144 * rSges(8,3)) + g(2) * (rSges(8,1) * t181 + rSges(8,2) * t185 - rSges(8,3) * t157 - pkin(10) * t157 - t149 + t150 + t23 + t47)) - m(9) * (g(1) * (t33 - t13 + (t15 * t192 - t195) * rSges(9,1) + (t15 * t194 + t192 * t3) * rSges(9,2) + t18 * rSges(9,3)) + g(2) * (t23 + t47 + (t192 * t24 + t206) * rSges(9,1) + (-t1 * t192 + t194 * t24) * rSges(9,2) + t26 * rSges(9,3))) - m(10) * (g(1) * (rSges(10,1) * t226 + rSges(10,2) * t230 + rSges(10,3) * t18 + pkin(17) * t195 - t15 * t218 - t13 + t33) + g(2) * (rSges(10,1) * t239 + rSges(10,2) * t243 + rSges(10,3) * t26 - pkin(17) * t206 - t218 * t24 + t23 + t47)) - m(11) * (g(1) * (rSges(11,1) * t258 + rSges(11,2) * t262 + rSges(11,3) * t18 - pkin(19) * t37 - t15 * t250 - t13 + t33) + g(2) * (rSges(11,1) * t271 + rSges(11,2) * t275 + rSges(11,3) * t26 + pkin(19) * t49 - t24 * t250 + t23 + t47)) - m(12) * (g(1) * (rSges(12,1) * t294 + rSges(12,2) * t298 + rSges(12,3) * t18 - t15 * t283 - t287 * t3 - t13 + t33) + g(2) * (rSges(12,1) * t307 + rSges(12,2) * t311 + rSges(12,3) * t26 + t1 * t287 - t24 * t283 + t23 + t47)) - m(13) * (g(1) * (rSges(13,1) * t323 + rSges(13,2) * t327 + rSges(13,3) * t18 - t13 + t33 - t95 - t99) + g(2) * (rSges(13,1) * t334 + rSges(13,2) * t338 + rSges(13,3) * t26 - t114 + t115 + t23 + t47)) - m(14) * (g(1) * (rSges(14,1) * t350 + rSges(14,2) * t354 + rSges(14,3) * t18 - t13 + t33 - t95 - t99) + g(2) * (rSges(14,1) * t361 + rSges(14,2) * t365 + rSges(14,3) * t26 - t114 + t115 + t23 + t47)) - m(15) * (g(1) * (rSges(15,1) * t383 + rSges(15,2) * t387 + rSges(15,3) * t18 - t13 + t33 - t374 - t377) + g(2) * (rSges(15,1) * t396 + rSges(15,2) * t400 + rSges(15,3) * t26 + t23 - t392 + t393 + t47)) - m(16) * (g(1) * (rSges(16,1) * t383 + rSges(16,2) * t18 - rSges(16,3) * t387 - qJ(13) * t387 - t13 + t33 - t374 - t377) + g(2) * (rSges(16,1) * t396 + rSges(16,2) * t26 - rSges(16,3) * t400 - qJ(13) * t400 + t23 - t392 + t393 + t47));
t436 = t24 * pkin(16);
t437 = t34 * rSges(4,1);
t439 = t36 * rSges(4,2);
t444 = t15 * pkin(16);
t450 = t17 * pkin(16);
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
t532 = t138 * pkin(10);
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
t756 = -m(3) * (g(1) * (-rSges(3,1) * t26 + rSges(3,2) * t24) + g(2) * (rSges(3,1) * t18 - rSges(3,2) * t15) + g(3) * (rSges(3,1) * t14 + rSges(3,2) * t17)) - m(4) * (g(1) * (-rSges(4,3) * t24 - t26 * t437 + t26 * t439 - t436) + g(2) * (rSges(4,3) * t15 + t18 * t437 - t18 * t439 + t444) + g(3) * (-rSges(4,2) * t14 * t36 + rSges(4,1) * t451 - rSges(4,3) * t17 - t450)) - m(5) * (g(1) * (-rSges(5,3) * t24 - t26 * t461 + t26 * t463 - t26 * t61 - t436) + g(2) * (rSges(5,3) * t15 + t18 * t461 - t18 * t463 + t18 * t61 + t444) + g(3) * (rSges(5,1) * t14 * t65 - rSges(5,2) * t14 * t67 - rSges(5,3) * t17 + pkin(18) * t451 - t450)) - m(6) * (g(1) * (-rSges(6,3) * t24 - t26 * t485 + t26 * t487 - t436 - t484) + g(2) * (rSges(6,3) * t15 + t18 * t485 - t18 * t487 + t444 + t492) + g(3) * (rSges(6,1) * t101 * t14 - rSges(6,2) * t103 * t14 - rSges(6,3) * t17 - t450 + t498)) - m(7) * (g(1) * (-rSges(7,3) * t24 + t26 * t509 + t26 * t511 - t436 - t508) + g(2) * (rSges(7,3) * t15 - t18 * t509 - t18 * t511 + t444 + t516) + g(3) * (-rSges(7,1) * t523 - rSges(7,2) * t525 - rSges(7,3) * t17 - t450 + t522)) - m(8) * (g(1) * (-t26 * t532 - t508 - t436 + (-t167 * t24 + t26 * t534) * rSges(8,1) + (-t165 * t24 - t26 * t539) * rSges(8,2) - t26 * t544) + g(2) * (t18 * t532 + t516 + t444 + (t15 * t167 - t18 * t534) * rSges(8,1) + (t15 * t165 + t18 * t539) * rSges(8,2) + t18 * t544) + g(3) * (t525 * pkin(10) + t522 - t450 + (-t165 * t523 - t167 * t17) * rSges(8,1) + (-t165 * t17 + t167 * t523) * rSges(8,2) + t525 * rSges(8,3))) - m(9) * (g(1) * (-rSges(9,3) * t24 + t26 * t574 + t26 * t576 - t436) + g(2) * (rSges(9,3) * t15 - t18 * t574 - t18 * t576 + t444) + g(3) * (-rSges(9,2) * t14 * t194 - rSges(9,1) * t586 - rSges(9,3) * t17 - t450)) - m(10) * (g(1) * (-rSges(10,3) * t24 - t218 * t26 + t26 * t596 + t26 * t598 - t436) + g(2) * (rSges(10,3) * t15 + t18 * t218 - t18 * t596 - t18 * t598 + t444) + g(3) * (-rSges(10,1) * t14 * t222 - rSges(10,2) * t14 * t224 - rSges(10,3) * t17 + pkin(17) * t586 - t450)) - m(11) * (g(1) * (-rSges(11,3) * t24 - t250 * t26 - t26 * t620 + t26 * t622 - t436) + g(2) * (rSges(11,3) * t15 + t18 * t250 + t18 * t620 - t18 * t622 + t444) + g(3) * (rSges(11,1) * t14 * t254 - rSges(11,2) * t14 * t256 - rSges(11,3) * t17 + pkin(19) * t451 - t450)) - m(12) * (g(1) * (-rSges(12,3) * t24 - t26 * t283 + t26 * t644 - t26 * t646 - t436) + g(2) * (rSges(12,3) * t15 + t18 * t283 - t18 * t644 + t18 * t646 + t444) + g(3) * (-rSges(12,1) * t14 * t290 + rSges(12,2) * t14 * t292 - rSges(12,3) * t17 + t14 * t283 - t450)) - m(13) * (g(1) * (-rSges(13,3) * t24 - t26 * t667 - t26 * t669 - t436 - t484) + g(2) * (rSges(13,3) * t15 + t18 * t667 + t18 * t669 + t444 + t492) + g(3) * (rSges(13,1) * t14 * t319 + rSges(13,2) * t14 * t321 - rSges(13,3) * t17 - t450 + t498)) - m(14) * (g(1) * (-rSges(14,3) * t24 + t26 * t688 - t26 * t690 - t436 - t484) + g(2) * (rSges(14,3) * t15 - t18 * t688 + t18 * t690 + t444 + t492) + g(3) * (-rSges(14,1) * t14 * t346 + rSges(14,2) * t14 * t348 - rSges(14,3) * t17 - t450 + t498)) - m(15) * (g(1) * (-rSges(15,3) * t24 + t26 * t710 + t26 * t712 - t436 - t709) + g(2) * (rSges(15,3) * t15 - t18 * t710 - t18 * t712 + t444 + t717) + g(3) * (-rSges(15,1) * t724 - rSges(15,2) * t726 - rSges(15,3) * t17 - t450 + t723)) - m(16) * (g(1) * (-rSges(16,2) * t24 - t26 * t733 + t26 * t735 - t26 * t738 - t436 - t709) + g(2) * (rSges(16,2) * t15 + t18 * t733 - t18 * t735 + t18 * t738 + t444 + t717) + g(3) * (-rSges(16,1) * t724 - rSges(16,2) * t17 + rSges(16,3) * t726 + qJ(13) * t726 - t450 + t723));
t765 = t17 * t36;
t775 = t86 * rSges(5,1);
t776 = -t82 * rSges(5,2);
t781 = -t73 * rSges(5,1);
t782 = t69 * rSges(5,2);
t787 = t17 * t67 * rSges(5,1);
t789 = t17 * t65 * rSges(5,2);
t794 = -t24 * t98;
t795 = t1 * t94;
t796 = t122 * rSges(6,1);
t797 = -t118 * rSges(6,2);
t800 = -t15 * t98;
t801 = t3 * t94;
t802 = -t109 * rSges(6,1);
t803 = t105 * rSges(6,2);
t806 = -t17 * t98;
t808 = t17 * t103 * rSges(6,1);
t810 = t17 * t101 * rSges(6,2);
t815 = -t24 * t133;
t816 = t1 * t130;
t817 = t157 * rSges(7,1);
t818 = -t153 * rSges(7,2);
t821 = -t15 * t133;
t822 = t3 * t130;
t823 = -t144 * rSges(7,1);
t824 = t140 * rSges(7,2);
t827 = -t17 * t133;
t828 = t17 * t138;
t829 = t828 * rSges(7,1);
t830 = t17 * t136;
t831 = t830 * rSges(7,2);
t836 = -t153 * pkin(10);
t838 = t157 * t165 * rSges(8,1);
t840 = t157 * t167 * rSges(8,2);
t841 = t153 * rSges(8,3);
t844 = t140 * pkin(10);
t846 = -t144 * t165 * rSges(8,1);
t848 = -t144 * t167 * rSges(8,2);
t849 = -t140 * rSges(8,3);
t852 = t830 * pkin(10);
t854 = t828 * t165 * rSges(8,1);
t856 = t828 * t167 * rSges(8,2);
t857 = t830 * rSges(8,3);
t864 = t275 * rSges(11,1);
t865 = -t271 * rSges(11,2);
t870 = -t262 * rSges(11,1);
t871 = t258 * rSges(11,2);
t876 = t17 * t256 * rSges(11,1);
t878 = t17 * t254 * rSges(11,2);
t885 = t311 * rSges(12,1);
t886 = -t307 * rSges(12,2);
t891 = -t298 * rSges(12,1);
t892 = t294 * rSges(12,2);
t897 = t17 * t292 * rSges(12,1);
t899 = t17 * t290 * rSges(12,2);
t904 = t338 * rSges(13,1);
t905 = -t334 * rSges(13,2);
t908 = -t327 * rSges(13,1);
t909 = t323 * rSges(13,2);
t913 = t17 * t321 * rSges(13,1);
t915 = t17 * t319 * rSges(13,2);
t920 = t365 * rSges(14,1);
t921 = -t361 * rSges(14,2);
t924 = -t354 * rSges(14,1);
t925 = t350 * rSges(14,2);
t929 = t17 * t348 * rSges(14,1);
t931 = t17 * t346 * rSges(14,2);
t936 = -t24 * t376;
t937 = t1 * t373;
t938 = t400 * rSges(15,1);
t939 = -t396 * rSges(15,2);
t942 = -t15 * t376;
t943 = t3 * t373;
t944 = -t387 * rSges(15,1);
t945 = t383 * rSges(15,2);
t948 = -t17 * t376;
t949 = t17 * t381;
t950 = t949 * rSges(15,1);
t951 = t17 * t379;
t952 = t951 * rSges(15,2);
t957 = -t396 * qJ(13);
t958 = t400 * rSges(16,1);
t959 = t396 * rSges(16,3);
t962 = t383 * qJ(13);
t963 = -t387 * rSges(16,1);
t964 = -t383 * rSges(16,3);
t967 = t951 * qJ(13);
t968 = t949 * rSges(16,1);
t969 = t951 * rSges(16,3);
t974 = -m(4) * (g(1) * (rSges(4,1) * t54 - rSges(4,2) * t50) + g(2) * (-rSges(4,1) * t42 + rSges(4,2) * t38) + g(3) * (rSges(4,2) * t17 * t34 + rSges(4,1) * t765)) - m(5) * (g(1) * (pkin(18) * t53 + t24 * t97 + t775 + t776) + g(2) * (pkin(18) * t41 - t15 * t97 + t781 + t782) + g(3) * (pkin(18) * t765 + t787 + t789)) - m(6) * (g(1) * (-t794 + t795 + t796 + t797) + g(2) * (t800 + t801 + t802 + t803) + g(3) * (-t806 + t808 + t810)) - m(7) * (g(1) * (-t815 + t816 + t817 + t818) + g(2) * (t821 + t822 + t823 + t824) + g(3) * (-t827 + t829 - t831)) - m(8) * (g(1) * (-t836 - t815 + t816 + t838 - t840 + t841) + g(2) * (-t844 + t821 + t822 + t846 - t848 + t849) + g(3) * (t852 - t827 + t854 - t856 + t857)) - m(11) * (g(1) * (pkin(19) * t53 + t24 * t286 + t864 + t865) + g(2) * (pkin(19) * t41 - t15 * t286 + t870 + t871) + g(3) * (pkin(19) * t765 + t876 + t878)) - m(12) * (g(1) * (t1 * t283 + t24 * t287 + t885 + t886) + g(2) * (-t15 * t287 + t283 * t3 + t891 + t892) + g(3) * (t17 * t287 - t897 - t899)) - m(13) * (g(1) * (-t794 + t795 + t904 + t905) + g(2) * (t800 + t801 + t908 + t909) + g(3) * (-t806 - t913 + t915)) - m(14) * (g(1) * (-t794 + t795 + t920 + t921) + g(2) * (t800 + t801 + t924 + t925) + g(3) * (-t806 - t929 - t931)) - m(15) * (g(1) * (-t936 + t937 + t938 + t939) + g(2) * (t942 + t943 + t944 + t945) + g(3) * (-t948 + t950 - t952)) - m(16) * (g(1) * (-t957 - t936 + t937 + t958 + t959) + g(2) * (-t962 + t942 + t943 + t963 + t964) + g(3) * (t967 - t948 + t968 + t969));
t983 = t24 * t96;
t985 = t1 * pkin(6) * t65;
t988 = t15 * t96;
t990 = t3 * pkin(6) * t65;
t994 = t17 * pkin(6) * t67;
t999 = -t132 - t96;
t1000 = t24 * t999;
t1001 = t129 + t93;
t1002 = t1 * t1001;
t1005 = t15 * t999;
t1006 = t3 * t1001;
t1009 = t17 * t999;
t1038 = -t96 + t375;
t1039 = t24 * t1038;
t1040 = t93 - t372;
t1041 = t1 * t1040;
t1044 = t15 * t1038;
t1045 = t3 * t1040;
t1048 = t17 * t1038;
t1070 = t24 * t132;
t1072 = t1 * pkin(7) * t101;
t1075 = t15 * t132;
t1077 = t3 * pkin(7) * t101;
t1081 = t17 * pkin(7) * t103;
t1201 = t24 * t375;
t1203 = t1 * pkin(9) * t346;
t1206 = t15 * t375;
t1208 = t3 * pkin(9) * t346;
t1212 = t17 * pkin(9) * t348;
unknown(1) = t421;
unknown(2) = t756;
unknown(3) = t974;
unknown(4) = -m(5) * (g(1) * (t775 + t776) + g(2) * (t781 + t782) + g(3) * (t787 + t789)) - m(6) * (g(1) * (t983 + t985 + t796 + t797) + g(2) * (-t988 + t990 + t802 + t803) + g(3) * (t994 + t808 + t810)) - m(7) * (g(1) * (-t1000 + t1002 + t817 + t818) + g(2) * (t1005 + t1006 + t823 + t824) + g(3) * (-t1009 + t829 - t831)) - m(8) * (g(1) * (-t836 - t1000 + t1002 + t838 - t840 + t841) + g(2) * (-t844 + t1005 + t1006 + t846 - t848 + t849) + g(3) * (t852 - t1009 + t854 - t856 + t857)) - m(13) * (g(1) * (t983 + t985 + t904 + t905) + g(2) * (-t988 + t990 + t908 + t909) + g(3) * (t994 - t913 + t915)) - m(14) * (g(1) * (t983 + t985 + t920 + t921) + g(2) * (-t988 + t990 + t924 + t925) + g(3) * (t994 - t929 - t931)) - m(15) * (g(1) * (-t1039 + t1041 + t938 + t939) + g(2) * (t1044 + t1045 + t944 + t945) + g(3) * (-t1048 + t950 - t952)) - m(16) * (g(1) * (-t957 - t1039 + t1041 + t958 + t959) + g(2) * (-t962 + t1044 + t1045 + t963 + t964) + g(3) * (t967 - t1048 + t968 + t969));
unknown(5) = -m(6) * (g(1) * (t796 + t797) + g(2) * (t802 + t803) + g(3) * (t808 + t810)) - m(7) * (g(1) * (t1070 + t1072 + t817 + t818) + g(2) * (-t1075 + t1077 + t823 + t824) + g(3) * (t1081 + t829 - t831)) - m(8) * (g(1) * (-t836 + t1070 + t1072 + t838 - t840 + t841) + g(2) * (-t844 - t1075 + t1077 + t846 - t848 + t849) + g(3) * (t852 + t1081 + t854 - t856 + t857));
unknown(6) = -m(7) * (g(1) * (t817 + t818) + g(2) * (t823 + t824) + g(3) * (t829 - t831)) - m(8) * (g(1) * (-t836 + t838 - t840 + t841) + g(2) * (-t844 + t846 - t848 + t849) + g(3) * (t852 + t854 - t856 + t857));
unknown(7) = -m(8) * (g(1) * (rSges(8,1) * t185 - rSges(8,2) * t181) + g(2) * ((t140 * t167 - t172) * rSges(8,1) + (t140 * t165 + t168) * rSges(8,2)) + g(3) * ((-t14 * t165 - t167 * t830) * rSges(8,1) + (t14 * t167 - t165 * t830) * rSges(8,2)));
unknown(8) = -m(10) * (g(1) * (rSges(10,1) * t243 - rSges(10,2) * t239) + g(2) * (-rSges(10,1) * t230 + rSges(10,2) * t226) + g(3) * (rSges(10,1) * t17 * t224 - rSges(10,2) * t17 * t222));
unknown(9) = -m(11) * (g(1) * (t864 + t865) + g(2) * (t870 + t871) + g(3) * (t876 + t878)) - m(12) * (g(1) * (pkin(14) * t1 * t254 + t24 * t285 + t885 + t886) + g(2) * (pkin(14) * t254 * t3 - t15 * t285 + t891 + t892) + g(3) * (pkin(14) * t17 * t256 - t897 - t899));
unknown(10) = -m(12) * (g(1) * (t885 + t886) + g(2) * (t891 + t892) + g(3) * (-t897 - t899));
unknown(11) = -m(13) * (g(1) * (t904 + t905) + g(2) * (t908 + t909) + g(3) * (-t913 + t915)) - m(14) * (g(1) * (t920 + t921) + g(2) * (t924 + t925) + g(3) * (-t929 - t931)) - m(15) * (g(1) * (-t1201 - t1203 + t938 + t939) + g(2) * (t1206 - t1208 + t944 + t945) + g(3) * (-t1212 + t950 - t952)) - m(16) * (g(1) * (-t957 - t1201 - t1203 + t958 + t959) + g(2) * (-t962 + t1206 - t1208 + t963 + t964) + g(3) * (t967 - t1212 + t968 + t969));
unknown(12) = -m(15) * (g(1) * (t938 + t939) + g(2) * (t944 + t945) + g(3) * (t950 - t952)) - m(16) * (g(1) * (-t957 + t958 + t959) + g(2) * (-t962 + t963 + t964) + g(3) * (t967 + t968 + t969));
unknown(13) = -m(16) * (-g(3) * t17 * t381 - g(1) * t400 + g(2) * t387);
taug = unknown(:);
