% Calculate potential energy for
% KAS5m7DE2
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
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

function U = KAS5m7DE2_energypot_fixb_slag_vp1(qJ, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp1: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp1: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_worldframe_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-25 12:22:21
% EndTime: 2020-05-25 12:23:14
% DurationCPUTime: 49.57s
% Computational Cost: add. (3160193->433), mult. (4078218->487), div. (65930->4), fcn. (1635230->49), ass. (0->172)
t6 = sin(qJ(1));
t8 = cos(qJ(1));
t20 = t8 * pkin(16);
t21 = sin(qJ(2));
t22 = t6 * t21;
t24 = cos(qJ(2));
t25 = t6 * t24;
t30 = t6 * pkin(16);
t31 = t8 * t21;
t33 = t8 * t24;
t44 = t25 * pkin(21);
t45 = sin(qJ(3));
t46 = cos(qJ(3));
t47 = -pkin(23) + pkin(24);
t48 = t46 * t47;
t49 = pkin(3) + qJ(3);
t50 = cos(t49);
t51 = t50 * pkin(12);
t52 = -pkin(9) + t48 + t51;
t53 = pkin(11) ^ 2;
t54 = pkin(19) ^ 2;
t55 = t50 ^ 2;
t56 = pkin(12) ^ 2;
t58 = sin(t49);
t59 = t58 ^ 2;
t62 = (-pkin(9) + t48) ^ 2;
t63 = t45 ^ 2;
t64 = t47 ^ 2;
t68 = t45 * t47;
t69 = t58 * pkin(12);
t70 = -t68 + t69;
t73 = 0.2e1 * t52 * t50 * pkin(12) + 0.2e1 * t70 * t58 * pkin(12) - t55 * t56 - t59 * t56 + t63 * t64 + t53 - t54 + t62;
t77 = 0.4e1 * t52 ^ 2 + 0.4e1 * t70 ^ 2;
t79 = t73 ^ 2;
t81 = sqrt(t53 * t77 - t79);
t84 = 0.1e1 / t77;
t86 = pkin(9) - t48 - t51 + (0.2e1 * t52 * t73 - 0.2e1 * t70 * t81) * t84;
t92 = t68 - t69 + (0.2e1 * t52 * t81 + 0.2e1 * t73 * t70) * t84;
t94 = t45 * t86 + t46 * t92;
t95 = 0.1e1 / pkin(19);
t96 = t94 * t95;
t97 = cos(pkin(7));
t98 = t97 * pkin(18);
t99 = t96 * t98;
t102 = -t45 * t92 + t46 * t86;
t103 = t102 * t95;
t104 = sin(pkin(7));
t105 = t104 * pkin(18);
t106 = t103 * t105;
t107 = pkin(17) ^ 2;
t108 = pkin(22) ^ 2;
t109 = t103 * t98;
t110 = t96 * t105;
t111 = t109 - t110;
t112 = t111 ^ 2;
t113 = t99 + t106;
t114 = t113 ^ 2;
t115 = pkin(24) ^ 2;
t116 = -pkin(24) - t109 + t110;
t119 = -0.2e1 * t116 * t111 + 0.2e1 * t113 ^ 2 + t107 - t108 - t112 - t114 + t115;
t123 = 0.4e1 * t113 ^ 2 + 0.4e1 * t116 ^ 2;
t125 = t119 ^ 2;
t127 = sqrt(t107 * t123 - t125);
t130 = 0.1e1 / t123;
t132 = -t99 - t106 - (-0.2e1 * t119 * t113 + 0.2e1 * t116 * t127) * t130;
t137 = t109 - t110 + (0.2e1 * t113 * t127 + 0.2e1 * t116 * t119) * t130 + pkin(24);
t138 = atan2(t132, t137);
t139 = t138 + pkin(6);
t140 = sin(t139);
t142 = cos(t139);
t143 = t8 * t142;
t153 = t33 * pkin(21);
t155 = t6 * t142;
t165 = t21 * pkin(21);
t166 = t24 * t140;
t175 = t140 * pkin(23);
t178 = t138 + pkin(6) + qJ(3);
t179 = sin(t178);
t181 = cos(t178);
t215 = pkin(9) * t179;
t216 = t215 + t175;
t217 = t22 * t216;
t218 = pkin(9) * t181;
t219 = t142 * pkin(23);
t220 = -t218 - t219;
t221 = t8 * t220;
t222 = t138 + pkin(6) + qJ(3) + qJ(4);
t223 = sin(t222);
t225 = cos(t222);
t236 = t31 * t216;
t237 = t6 * t220;
t249 = t24 * t216;
t260 = pkin(10) * t223 + t175 + t215;
t261 = t22 * t260;
t263 = -pkin(10) * t225 - t218 - t219;
t264 = t8 * t263;
t266 = t138 + pkin(6) + 0.2e1 * qJ(4) + pkin(4);
t267 = cos(t266);
t269 = sin(t266);
t271 = -t22 * t267 + t8 * t269;
t275 = t22 * t269 + t8 * t267;
t280 = t31 * t260;
t281 = t6 * t263;
t284 = t31 * t267 + t6 * t269;
t288 = t6 * t267 - t31 * t269;
t293 = t24 * t260;
t294 = t24 * t267;
t296 = t24 * t269;
t304 = cos(qJ(5));
t306 = sin(qJ(5));
t343 = sin(pkin(6));
t345 = cos(pkin(6));
t346 = t8 * t345;
t357 = t6 * t345;
t367 = t24 * t343;
t376 = t343 * pkin(22);
t379 = 0.1e1 / pkin(22);
t380 = t132 * t379;
t382 = -t137 * t379;
t384 = t380 * t343 + t382 * t345;
t388 = -t382 * t343 + t380 * t345;
t398 = atan2(-(t388 * t343 - t384 * t345) * pkin(22) + pkin(24) + t109 - t110, (t384 * t343 + t388 * t345) * pkin(22) + t99 + t106);
t399 = pkin(6) + t138 - t398;
t400 = cos(t399);
t402 = sin(t399);
t436 = t140 * pkin(24);
t439 = atan2(-t94, t102);
t440 = t138 + pkin(6) - t439;
t441 = sin(t440);
t443 = cos(t440);
t478 = pkin(19) * t441 + t436;
t482 = -pkin(19) * t443 - t142 * pkin(24);
t498 = atan2((t45 * t102 * t95 - t46 * t94 * t95) * pkin(19) + t68 - t69, (t46 * t102 * t95 + t45 * t94 * t95) * pkin(19) - pkin(9) + t48 + t51);
t499 = t138 + pkin(6) - t498 + qJ(3);
t500 = sin(t499);
t502 = cos(t499);
t536 = 0.2e1 * qJ(3);
t537 = t138 + pkin(6) + t536;
t538 = cos(t537);
t540 = sin(t537);
t571 = t138 + pkin(6) + t536 + pkin(3);
t572 = sin(t571);
t574 = cos(t571);
t606 = -pkin(12) * t572 + t175 + t215;
t607 = t22 * t606;
t609 = pkin(12) * t574 - t218 - t219;
t610 = t8 * t609;
t611 = qJ(4) - qJ(3) + pkin(4);
t612 = sin(t611);
t614 = pkin(3) + qJ(3) - qJ(4);
t615 = sin(t614);
t617 = cos(t611);
t619 = t615 * pkin(12) - t617 * pkin(14) + t612 * pkin(15);
t622 = cos(t614);
t624 = -t622 * pkin(12) - t612 * pkin(14) - t617 * pkin(15) - pkin(10);
t625 = atan2(t619, t624);
t626 = t138 + pkin(6) + qJ(3) - t625 + qJ(4);
t627 = cos(t626);
t629 = sin(t626);
t631 = t22 * t627 - t8 * t629;
t635 = -t22 * t629 - t8 * t627;
t640 = t31 * t606;
t641 = t6 * t609;
t644 = -t31 * t627 - t6 * t629;
t648 = t31 * t629 - t6 * t627;
t653 = t24 * t606;
t654 = t24 * t627;
t656 = t24 * t629;
t663 = t624 ^ 2;
t664 = t619 ^ 2;
t666 = sqrt(t663 + t664);
t687 = -m(1) * (rSges(1,1) * g(1) + rSges(1,2) * g(2) + g(3) * rSges(1,3)) - m(2) * (g(1) * (t6 * rSges(2,1) + t8 * rSges(2,2)) + g(2) * (-t8 * rSges(2,1) + t6 * rSges(2,2)) + g(3) * (pkin(8) + rSges(2,3))) - m(3) * (g(1) * (-t22 * rSges(3,1) - t25 * rSges(3,2) + t8 * rSges(3,3) + t20) + g(2) * (t31 * rSges(3,1) + t33 * rSges(3,2) + t6 * rSges(3,3) + t30) + g(3) * (-t24 * rSges(3,1) + t21 * rSges(3,2) + pkin(8))) - m(4) * (g(1) * (t44 + t20 + (-t22 * t140 - t143) * rSges(4,1) + (t8 * t140 - t22 * t142) * rSges(4,2) + t25 * rSges(4,3)) + g(2) * (-t153 + t30 + (t31 * t140 - t155) * rSges(4,1) + (t6 * t140 + t31 * t142) * rSges(4,2) - t33 * rSges(4,3)) + g(3) * (-t24 * t142 * rSges(4,2) - t166 * rSges(4,1) - t21 * rSges(4,3) + pkin(8) - t165)) - m(5) * (g(1) * (-t22 * t175 + t44 - t143 * pkin(23) + t20 + (-t22 * t179 - t8 * t181) * rSges(5,1) + (t8 * t179 - t22 * t181) * rSges(5,2) + t25 * rSges(5,3)) + g(2) * (t31 * t175 - t153 - t155 * pkin(23) + t30 + (t31 * t179 - t6 * t181) * rSges(5,1) + (t6 * t179 + t31 * t181) * rSges(5,2) - t33 * rSges(5,3)) + g(3) * (-t24 * t179 * rSges(5,1) - t24 * t181 * rSges(5,2) - t21 * rSges(5,3) - t166 * pkin(23) + pkin(8) - t165)) - m(6) * (g(1) * (-t217 + t44 + t221 + t20 + (-t22 * t223 - t8 * t225) * rSges(6,1) + (-t22 * t225 + t8 * t223) * rSges(6,2) + t25 * rSges(6,3)) + g(2) * (t236 - t153 + t237 + t30 + (t31 * t223 - t6 * t225) * rSges(6,1) + (t6 * t223 + t31 * t225) * rSges(6,2) - t33 * rSges(6,3)) + g(3) * (-t24 * t223 * rSges(6,1) - t24 * t225 * rSges(6,2) - t21 * rSges(6,3) + pkin(8) - t165 - t249)) - m(7) * (g(1) * (t271 * rSges(7,1) + t275 * rSges(7,2) + t25 * rSges(7,3) + t20 - t261 + t264 + t44) + g(2) * (t284 * rSges(7,1) + t288 * rSges(7,2) - t33 * rSges(7,3) - t153 + t280 + t281 + t30) + g(3) * (-t294 * rSges(7,1) + t296 * rSges(7,2) - t21 * rSges(7,3) + pkin(8) - t165 - t293)) - m(8) * (g(1) * (-t275 * pkin(13) - t261 + t44 + t264 + t20 + (t25 * t306 + t271 * t304) * rSges(8,1) + (t25 * t304 - t271 * t306) * rSges(8,2) - t275 * rSges(8,3)) + g(2) * (-t288 * pkin(13) + t280 - t153 + t281 + t30 + (t284 * t304 - t33 * t306) * rSges(8,1) + (-t284 * t306 - t33 * t304) * rSges(8,2) - t288 * rSges(8,3)) + g(3) * (-t296 * pkin(13) - t293 - t165 + pkin(8) + (-t21 * t306 - t294 * t304) * rSges(8,1) + (-t21 * t304 + t294 * t306) * rSges(8,2) - t296 * rSges(8,3))) - m(9) * (g(1) * (t44 + t20 + (t22 * t343 + t346) * rSges(9,1) + (t22 * t345 - t8 * t343) * rSges(9,2) + t25 * rSges(9,3)) + g(2) * (-t153 + t30 + (-t31 * t343 + t357) * rSges(9,1) + (-t31 * t345 - t6 * t343) * rSges(9,2) - t33 * rSges(9,3)) + g(3) * (t24 * t345 * rSges(9,2) + t367 * rSges(9,1) - t21 * rSges(9,3) + pkin(8) - t165)) - m(10) * (g(1) * (-t22 * t376 + t44 - t346 * pkin(22) + t20 + (-t22 * t400 + t8 * t402) * rSges(10,1) + (t22 * t402 + t8 * t400) * rSges(10,2) + t25 * rSges(10,3)) + g(2) * (t31 * t376 - t153 - t357 * pkin(22) + t30 + (t31 * t400 + t6 * t402) * rSges(10,1) + (-t31 * t402 + t6 * t400) * rSges(10,2) - t33 * rSges(10,3)) + g(3) * (-t24 * t400 * rSges(10,1) + t24 * t402 * rSges(10,2) - t21 * rSges(10,3) - t367 * pkin(22) + pkin(8) - t165)) - m(11) * (g(1) * (-t22 * t436 + t44 - t143 * pkin(24) + t20 + (-t22 * t441 - t8 * t443) * rSges(11,1) + (-t22 * t443 + t8 * t441) * rSges(11,2) + t25 * rSges(11,3)) + g(2) * (t31 * t436 - t153 - t155 * pkin(24) + t30 + (t31 * t441 - t6 * t443) * rSges(11,1) + (t31 * t443 + t6 * t441) * rSges(11,2) - t33 * rSges(11,3)) + g(3) * (-t24 * t441 * rSges(11,1) - t24 * t443 * rSges(11,2) - t21 * rSges(11,3) - t166 * pkin(24) + pkin(8) - t165)) - m(12) * (g(1) * (-t22 * t478 + t44 + t8 * t482 + t20 + (t22 * t500 + t8 * t502) * rSges(12,1) + (t22 * t502 - t8 * t500) * rSges(12,2) + t25 * rSges(12,3)) + g(2) * (t31 * t478 - t153 + t6 * t482 + t30 + (-t31 * t500 + t6 * t502) * rSges(12,1) + (-t31 * t502 - t6 * t500) * rSges(12,2) - t33 * rSges(12,3)) + g(3) * (t24 * t500 * rSges(12,1) + t24 * t502 * rSges(12,2) - t21 * rSges(12,3) - t24 * t478 + pkin(8) - t165)) - m(13) * (g(1) * (-t217 + t44 + t221 + t20 + (t22 * t538 - t8 * t540) * rSges(13,1) + (-t22 * t540 - t8 * t538) * rSges(13,2) + t25 * rSges(13,3)) + g(2) * (t236 - t153 + t237 + t30 + (-t31 * t538 - t6 * t540) * rSges(13,1) + (t31 * t540 - t6 * t538) * rSges(13,2) - t33 * rSges(13,3)) + g(3) * (t24 * t538 * rSges(13,1) - t24 * t540 * rSges(13,2) - t21 * rSges(13,3) + pkin(8) - t165 - t249)) - m(14) * (g(1) * (-t217 + t44 + t221 + t20 + (t22 * t572 + t8 * t574) * rSges(14,1) + (t22 * t574 - t8 * t572) * rSges(14,2) + t25 * rSges(14,3)) + g(2) * (t236 - t153 + t237 + t30 + (-t31 * t572 + t6 * t574) * rSges(14,1) + (-t31 * t574 - t6 * t572) * rSges(14,2) - t33 * rSges(14,3)) + g(3) * (t24 * t572 * rSges(14,1) + t24 * t574 * rSges(14,2) - t21 * rSges(14,3) + pkin(8) - t165 - t249)) - m(15) * (g(1) * (t631 * rSges(15,1) + t635 * rSges(15,2) + t25 * rSges(15,3) + t20 + t44 - t607 + t610) + g(2) * (t644 * rSges(15,1) + t648 * rSges(15,2) - t33 * rSges(15,3) - t153 + t30 + t640 + t641) + g(3) * (t654 * rSges(15,1) - t656 * rSges(15,2) - t21 * rSges(15,3) + pkin(8) - t165 - t653)) - m(16) * (g(1) * (t631 * rSges(16,1) + t25 * rSges(16,2) - t635 * rSges(16,3) - t635 * t666 + t20 + t44 - t607 + t610) + g(2) * (t644 * rSges(16,1) - t33 * rSges(16,2) - t648 * rSges(16,3) - t648 * t666 - t153 + t30 + t640 + t641) + g(3) * (t654 * rSges(16,1) - t21 * rSges(16,2) + t656 * rSges(16,3) + t656 * t666 + pkin(8) - t165 - t653));
U = t687;
