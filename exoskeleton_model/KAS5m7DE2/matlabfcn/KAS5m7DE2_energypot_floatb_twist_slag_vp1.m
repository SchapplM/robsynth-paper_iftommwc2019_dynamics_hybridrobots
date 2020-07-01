% Calculate potential energy for
% KAS5m7DE2
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% r_base [3x1]
%   Base position in world frame
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% m [16x1]
%   mass of all robot links (including the base)
% rSges [16x3]
%   center of mass of all robot links (in body frames)
%   rows: links of the robot (starting with base)
%   columns: x-, y-, z-coordinates
% 
% Output:
% U [1x1]
%   Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-03 15:49
% Revision: caa0dbda1e8a16d11faaa29ba3bbef6afcd619f7 (2020-05-25)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7DE2_energypot_floatb_twist_slag_vp1(qJ, r_base, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp1: qJ has to be [5x1] (double)');
assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp1: r_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp1: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_floatb_twist_worldframe_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-25 12:21:24
% EndTime: 2020-05-25 12:22:17
% DurationCPUTime: 46.67s
% Computational Cost: add. (3160241->481), mult. (4078218->487), div. (65930->4), fcn. (1635230->49), ass. (0->172)
t9 = sin(qJ(1));
t11 = cos(qJ(1));
t23 = t11 * pkin(16);
t24 = sin(qJ(2));
t25 = t9 * t24;
t27 = cos(qJ(2));
t28 = t9 * t27;
t33 = t9 * pkin(16);
t34 = t11 * t24;
t36 = t11 * t27;
t47 = t28 * pkin(21);
t48 = sin(qJ(3));
t49 = cos(qJ(3));
t50 = -pkin(23) + pkin(24);
t51 = t49 * t50;
t52 = pkin(3) + qJ(3);
t53 = cos(t52);
t54 = t53 * pkin(12);
t55 = -pkin(9) + t51 + t54;
t56 = pkin(11) ^ 2;
t57 = pkin(19) ^ 2;
t58 = t53 ^ 2;
t59 = pkin(12) ^ 2;
t61 = sin(t52);
t62 = t61 ^ 2;
t65 = (-pkin(9) + t51) ^ 2;
t66 = t48 ^ 2;
t67 = t50 ^ 2;
t71 = t48 * t50;
t72 = t61 * pkin(12);
t73 = -t71 + t72;
t76 = 0.2e1 * t55 * t53 * pkin(12) + 0.2e1 * t73 * t61 * pkin(12) - t58 * t59 - t62 * t59 + t66 * t67 + t56 - t57 + t65;
t80 = 0.4e1 * t55 ^ 2 + 0.4e1 * t73 ^ 2;
t82 = t76 ^ 2;
t84 = sqrt(t56 * t80 - t82);
t87 = 0.1e1 / t80;
t89 = pkin(9) - t51 - t54 + (0.2e1 * t55 * t76 - 0.2e1 * t73 * t84) * t87;
t95 = t71 - t72 + (0.2e1 * t55 * t84 + 0.2e1 * t76 * t73) * t87;
t97 = t48 * t89 + t49 * t95;
t98 = 0.1e1 / pkin(19);
t99 = t97 * t98;
t100 = cos(pkin(7));
t101 = t100 * pkin(18);
t102 = t99 * t101;
t105 = -t48 * t95 + t49 * t89;
t106 = t105 * t98;
t107 = sin(pkin(7));
t108 = t107 * pkin(18);
t109 = t106 * t108;
t110 = pkin(17) ^ 2;
t111 = pkin(22) ^ 2;
t112 = t106 * t101;
t113 = t99 * t108;
t114 = t112 - t113;
t115 = t114 ^ 2;
t116 = t102 + t109;
t117 = t116 ^ 2;
t118 = pkin(24) ^ 2;
t119 = -pkin(24) - t112 + t113;
t122 = -0.2e1 * t119 * t114 + 0.2e1 * t116 ^ 2 + t110 - t111 - t115 - t117 + t118;
t126 = 0.4e1 * t116 ^ 2 + 0.4e1 * t119 ^ 2;
t128 = t122 ^ 2;
t130 = sqrt(t110 * t126 - t128);
t133 = 0.1e1 / t126;
t135 = -t102 - t109 - (-0.2e1 * t122 * t116 + 0.2e1 * t119 * t130) * t133;
t140 = t112 - t113 + (0.2e1 * t116 * t130 + 0.2e1 * t119 * t122) * t133 + pkin(24);
t141 = atan2(t135, t140);
t142 = t141 + pkin(6);
t143 = sin(t142);
t145 = cos(t142);
t146 = t11 * t145;
t156 = t36 * pkin(21);
t158 = t9 * t145;
t168 = t24 * pkin(21);
t169 = t27 * t143;
t178 = t143 * pkin(23);
t181 = t141 + pkin(6) + qJ(3);
t182 = sin(t181);
t184 = cos(t181);
t218 = pkin(9) * t182;
t219 = t218 + t178;
t220 = t25 * t219;
t221 = pkin(9) * t184;
t222 = t145 * pkin(23);
t223 = -t221 - t222;
t224 = t11 * t223;
t225 = t141 + pkin(6) + qJ(3) + qJ(4);
t226 = sin(t225);
t228 = cos(t225);
t239 = t34 * t219;
t240 = t9 * t223;
t252 = t27 * t219;
t263 = pkin(10) * t226 + t178 + t218;
t264 = t25 * t263;
t266 = -pkin(10) * t228 - t221 - t222;
t267 = t11 * t266;
t269 = t141 + pkin(6) + 0.2e1 * qJ(4) + pkin(4);
t270 = cos(t269);
t272 = sin(t269);
t274 = t11 * t272 - t25 * t270;
t278 = t11 * t270 + t25 * t272;
t283 = t34 * t263;
t284 = t9 * t266;
t287 = t34 * t270 + t9 * t272;
t291 = t9 * t270 - t34 * t272;
t296 = t27 * t263;
t297 = t27 * t270;
t299 = t27 * t272;
t307 = cos(qJ(5));
t309 = sin(qJ(5));
t346 = sin(pkin(6));
t348 = cos(pkin(6));
t349 = t11 * t348;
t360 = t9 * t348;
t370 = t27 * t346;
t379 = t346 * pkin(22);
t382 = 0.1e1 / pkin(22);
t383 = t135 * t382;
t385 = -t140 * t382;
t387 = t383 * t346 + t385 * t348;
t391 = -t385 * t346 + t383 * t348;
t401 = atan2(-(t391 * t346 - t387 * t348) * pkin(22) + pkin(24) + t112 - t113, (t387 * t346 + t391 * t348) * pkin(22) + t102 + t109);
t402 = pkin(6) + t141 - t401;
t403 = cos(t402);
t405 = sin(t402);
t439 = t143 * pkin(24);
t442 = atan2(-t97, t105);
t443 = t141 + pkin(6) - t442;
t444 = sin(t443);
t446 = cos(t443);
t481 = pkin(19) * t444 + t439;
t485 = -pkin(19) * t446 - t145 * pkin(24);
t501 = atan2((t48 * t105 * t98 - t49 * t97 * t98) * pkin(19) + t71 - t72, (t49 * t105 * t98 + t48 * t97 * t98) * pkin(19) - pkin(9) + t51 + t54);
t502 = t141 + pkin(6) - t501 + qJ(3);
t503 = sin(t502);
t505 = cos(t502);
t539 = 0.2e1 * qJ(3);
t540 = t141 + pkin(6) + t539;
t541 = cos(t540);
t543 = sin(t540);
t574 = t141 + pkin(6) + t539 + pkin(3);
t575 = sin(t574);
t577 = cos(t574);
t609 = -pkin(12) * t575 + t178 + t218;
t610 = t25 * t609;
t612 = pkin(12) * t577 - t221 - t222;
t613 = t11 * t612;
t614 = qJ(4) - qJ(3) + pkin(4);
t615 = sin(t614);
t617 = pkin(3) + qJ(3) - qJ(4);
t618 = sin(t617);
t620 = cos(t614);
t622 = t618 * pkin(12) - t620 * pkin(14) + t615 * pkin(15);
t625 = cos(t617);
t627 = -t625 * pkin(12) - t615 * pkin(14) - t620 * pkin(15) - pkin(10);
t628 = atan2(t622, t627);
t629 = t141 + pkin(6) + qJ(3) - t628 + qJ(4);
t630 = cos(t629);
t632 = sin(t629);
t634 = -t11 * t632 + t25 * t630;
t638 = -t11 * t630 - t25 * t632;
t643 = t34 * t609;
t644 = t9 * t612;
t647 = -t34 * t630 - t9 * t632;
t651 = t34 * t632 - t9 * t630;
t656 = t27 * t609;
t657 = t27 * t630;
t659 = t27 * t632;
t666 = t627 ^ 2;
t667 = t622 ^ 2;
t669 = sqrt(t666 + t667);
t690 = -m(1) * (g(1) * (r_base(1) + rSges(1,1)) + g(2) * (r_base(2) + rSges(1,2)) + g(3) * (r_base(3) + rSges(1,3))) - m(2) * (g(1) * (t9 * rSges(2,1) + t11 * rSges(2,2) + r_base(1)) + g(2) * (-t11 * rSges(2,1) + t9 * rSges(2,2) + r_base(2)) + g(3) * (pkin(8) + r_base(3) + rSges(2,3))) - m(3) * (g(1) * (-t25 * rSges(3,1) - t28 * rSges(3,2) + t11 * rSges(3,3) + t23 + r_base(1)) + g(2) * (t34 * rSges(3,1) + t36 * rSges(3,2) + t9 * rSges(3,3) + t33 + r_base(2)) + g(3) * (-t27 * rSges(3,1) + t24 * rSges(3,2) + pkin(8) + r_base(3))) - m(4) * (g(1) * (t47 + t23 + r_base(1) + (-t25 * t143 - t146) * rSges(4,1) + (t11 * t143 - t25 * t145) * rSges(4,2) + t28 * rSges(4,3)) + g(2) * (-t156 + t33 + r_base(2) + (t34 * t143 - t158) * rSges(4,1) + (t9 * t143 + t34 * t145) * rSges(4,2) - t36 * rSges(4,3)) + g(3) * (-t27 * t145 * rSges(4,2) - t169 * rSges(4,1) - t24 * rSges(4,3) + pkin(8) - t168 + r_base(3))) - m(5) * (g(1) * (-t25 * t178 + t47 - t146 * pkin(23) + t23 + r_base(1) + (-t11 * t184 - t25 * t182) * rSges(5,1) + (t11 * t182 - t25 * t184) * rSges(5,2) + t28 * rSges(5,3)) + g(2) * (t34 * t178 - t156 - t158 * pkin(23) + t33 + r_base(2) + (t34 * t182 - t9 * t184) * rSges(5,1) + (t9 * t182 + t34 * t184) * rSges(5,2) - t36 * rSges(5,3)) + g(3) * (-t27 * t182 * rSges(5,1) - t27 * t184 * rSges(5,2) - t24 * rSges(5,3) - t169 * pkin(23) + pkin(8) - t168 + r_base(3))) - m(6) * (g(1) * (-t220 + t47 + t224 + t23 + r_base(1) + (-t11 * t228 - t25 * t226) * rSges(6,1) + (t11 * t226 - t25 * t228) * rSges(6,2) + t28 * rSges(6,3)) + g(2) * (t239 - t156 + t240 + t33 + r_base(2) + (t34 * t226 - t9 * t228) * rSges(6,1) + (t9 * t226 + t34 * t228) * rSges(6,2) - t36 * rSges(6,3)) + g(3) * (-t27 * t226 * rSges(6,1) - t27 * t228 * rSges(6,2) - t24 * rSges(6,3) + pkin(8) - t168 - t252 + r_base(3))) - m(7) * (g(1) * (t274 * rSges(7,1) + t278 * rSges(7,2) + t28 * rSges(7,3) + t23 - t264 + t267 + t47 + r_base(1)) + g(2) * (t287 * rSges(7,1) + t291 * rSges(7,2) - t36 * rSges(7,3) - t156 + t283 + t284 + t33 + r_base(2)) + g(3) * (-t297 * rSges(7,1) + t299 * rSges(7,2) - t24 * rSges(7,3) + pkin(8) - t168 - t296 + r_base(3))) - m(8) * (g(1) * (-t278 * pkin(13) - t264 + t47 + t267 + t23 + r_base(1) + (t274 * t307 + t28 * t309) * rSges(8,1) + (-t274 * t309 + t28 * t307) * rSges(8,2) - t278 * rSges(8,3)) + g(2) * (-t291 * pkin(13) + t283 - t156 + t284 + t33 + r_base(2) + (t287 * t307 - t36 * t309) * rSges(8,1) + (-t287 * t309 - t36 * t307) * rSges(8,2) - t291 * rSges(8,3)) + g(3) * (-t299 * pkin(13) - t296 - t168 + pkin(8) + r_base(3) + (-t24 * t309 - t297 * t307) * rSges(8,1) + (-t24 * t307 + t297 * t309) * rSges(8,2) - t299 * rSges(8,3))) - m(9) * (g(1) * (t47 + t23 + r_base(1) + (t25 * t346 + t349) * rSges(9,1) + (-t11 * t346 + t25 * t348) * rSges(9,2) + t28 * rSges(9,3)) + g(2) * (-t156 + t33 + r_base(2) + (-t34 * t346 + t360) * rSges(9,1) + (-t34 * t348 - t9 * t346) * rSges(9,2) - t36 * rSges(9,3)) + g(3) * (t27 * t348 * rSges(9,2) + t370 * rSges(9,1) - t24 * rSges(9,3) + pkin(8) - t168 + r_base(3))) - m(10) * (g(1) * (-t25 * t379 + t47 - t349 * pkin(22) + t23 + r_base(1) + (t11 * t405 - t25 * t403) * rSges(10,1) + (t11 * t403 + t25 * t405) * rSges(10,2) + t28 * rSges(10,3)) + g(2) * (t34 * t379 - t156 - t360 * pkin(22) + t33 + r_base(2) + (t34 * t403 + t9 * t405) * rSges(10,1) + (-t34 * t405 + t9 * t403) * rSges(10,2) - t36 * rSges(10,3)) + g(3) * (-t27 * t403 * rSges(10,1) + t27 * t405 * rSges(10,2) - t24 * rSges(10,3) - t370 * pkin(22) + pkin(8) - t168 + r_base(3))) - m(11) * (g(1) * (-t25 * t439 + t47 - t146 * pkin(24) + t23 + r_base(1) + (-t11 * t446 - t25 * t444) * rSges(11,1) + (t11 * t444 - t25 * t446) * rSges(11,2) + t28 * rSges(11,3)) + g(2) * (t34 * t439 - t156 - t158 * pkin(24) + t33 + r_base(2) + (t34 * t444 - t9 * t446) * rSges(11,1) + (t34 * t446 + t9 * t444) * rSges(11,2) - t36 * rSges(11,3)) + g(3) * (-t27 * t444 * rSges(11,1) - t27 * t446 * rSges(11,2) - t24 * rSges(11,3) - t169 * pkin(24) + pkin(8) - t168 + r_base(3))) - m(12) * (g(1) * (-t25 * t481 + t47 + t11 * t485 + t23 + r_base(1) + (t11 * t505 + t25 * t503) * rSges(12,1) + (-t11 * t503 + t25 * t505) * rSges(12,2) + t28 * rSges(12,3)) + g(2) * (t34 * t481 - t156 + t9 * t485 + t33 + r_base(2) + (-t34 * t503 + t9 * t505) * rSges(12,1) + (-t34 * t505 - t9 * t503) * rSges(12,2) - t36 * rSges(12,3)) + g(3) * (t27 * t503 * rSges(12,1) + t27 * t505 * rSges(12,2) - t24 * rSges(12,3) - t27 * t481 + pkin(8) - t168 + r_base(3))) - m(13) * (g(1) * (-t220 + t47 + t224 + t23 + r_base(1) + (-t11 * t543 + t25 * t541) * rSges(13,1) + (-t11 * t541 - t25 * t543) * rSges(13,2) + t28 * rSges(13,3)) + g(2) * (t239 - t156 + t240 + t33 + r_base(2) + (-t34 * t541 - t9 * t543) * rSges(13,1) + (t34 * t543 - t9 * t541) * rSges(13,2) - t36 * rSges(13,3)) + g(3) * (t27 * t541 * rSges(13,1) - t27 * t543 * rSges(13,2) - t24 * rSges(13,3) + pkin(8) - t168 - t252 + r_base(3))) - m(14) * (g(1) * (-t220 + t47 + t224 + t23 + r_base(1) + (t11 * t577 + t25 * t575) * rSges(14,1) + (-t11 * t575 + t25 * t577) * rSges(14,2) + t28 * rSges(14,3)) + g(2) * (t239 - t156 + t240 + t33 + r_base(2) + (-t34 * t575 + t9 * t577) * rSges(14,1) + (-t34 * t577 - t9 * t575) * rSges(14,2) - t36 * rSges(14,3)) + g(3) * (t27 * t575 * rSges(14,1) + t27 * t577 * rSges(14,2) - t24 * rSges(14,3) + pkin(8) - t168 - t252 + r_base(3))) - m(15) * (g(1) * (t634 * rSges(15,1) + t638 * rSges(15,2) + t28 * rSges(15,3) + t23 + t47 - t610 + t613 + r_base(1)) + g(2) * (t647 * rSges(15,1) + t651 * rSges(15,2) - t36 * rSges(15,3) - t156 + t33 + t643 + t644 + r_base(2)) + g(3) * (t657 * rSges(15,1) - t659 * rSges(15,2) - t24 * rSges(15,3) + pkin(8) - t168 - t656 + r_base(3))) - m(16) * (g(1) * (t634 * rSges(16,1) + t28 * rSges(16,2) - t638 * rSges(16,3) - t638 * t669 + t23 + t47 - t610 + t613 + r_base(1)) + g(2) * (t647 * rSges(16,1) - t36 * rSges(16,2) - t651 * rSges(16,3) - t651 * t669 - t156 + t33 + t643 + t644 + r_base(2)) + g(3) * (t657 * rSges(16,1) - t24 * rSges(16,2) + t659 * rSges(16,3) + t659 * t669 + pkin(8) - t168 - t656 + r_base(3)));
U = t690;
