% Calculate potential energy for
% KAS5m7DE1
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
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7DE1_energypot_floatb_twist_slag_vp1(qJ, r_base, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp1: qJ has to be [5x1] (double)');
assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp1: r_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp1: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_floatb_twist_worldframe_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-14 20:17:16
% EndTime: 2020-05-14 20:20:23
% DurationCPUTime: 186.48s
% Computational Cost: add. (13265304->483), mult. (17122075->476), div. (276600->4), fcn. (6865757->43), ass. (0->181)
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
t147 = -t11 * t145 - t25 * t143;
t151 = t11 * t143 - t25 * t145;
t156 = t36 * pkin(21);
t159 = t34 * t143 - t9 * t145;
t163 = t9 * t143 + t34 * t145;
t168 = t24 * pkin(21);
t169 = t27 * t143;
t171 = t27 * t145;
t178 = t147 * pkin(23);
t181 = t147 * t49 + t151 * t48;
t185 = -t147 * t48 + t151 * t49;
t190 = t159 * pkin(23);
t193 = t159 * t49 + t163 * t48;
t197 = -t159 * t48 + t163 * t49;
t202 = t169 * pkin(23);
t205 = -t169 * t49 - t171 * t48;
t209 = t169 * t48 - t171 * t49;
t216 = t181 * pkin(9);
t217 = cos(qJ(4));
t219 = sin(qJ(4));
t221 = t181 * t217 + t185 * t219;
t225 = -t181 * t219 + t185 * t217;
t230 = t193 * pkin(9);
t233 = t193 * t217 + t197 * t219;
t237 = -t193 * t219 + t197 * t217;
t242 = t205 * pkin(9);
t245 = t205 * t217 + t209 * t219;
t249 = -t205 * t219 + t209 * t217;
t256 = t221 * pkin(10);
t257 = qJ(4) - qJ(3) + pkin(4);
t258 = sin(t257);
t260 = cos(t257);
t262 = -t221 * t258 + t225 * t260;
t266 = -t221 * t260 - t225 * t258;
t271 = t233 * pkin(10);
t274 = -t233 * t258 + t237 * t260;
t278 = -t233 * t260 - t237 * t258;
t283 = t245 * pkin(10);
t286 = -t245 * t258 + t249 * t260;
t290 = -t245 * t260 - t249 * t258;
t298 = cos(qJ(5));
t300 = sin(qJ(5));
t337 = sin(pkin(6));
t339 = cos(pkin(6));
t341 = t11 * t339 + t25 * t337;
t345 = -t11 * t337 + t25 * t339;
t352 = -t34 * t337 + t9 * t339;
t356 = -t9 * t337 - t34 * t339;
t361 = t27 * t337;
t363 = t27 * t339;
t371 = 0.1e1 / pkin(22);
t372 = t135 * t371;
t374 = -t140 * t371;
t376 = t372 * t337 + t374 * t339;
t380 = -t374 * t337 + t372 * t339;
t390 = atan2(-(t380 * t337 - t376 * t339) * pkin(22) + pkin(24) + t112 - t113, (t376 * t337 + t380 * t339) * pkin(22) + t102 + t109);
t391 = t141 - t390;
t392 = sin(t391);
t394 = cos(t391);
t431 = t147 * pkin(24);
t432 = atan2(-t97, t105);
t433 = cos(t432);
t435 = sin(t432);
t437 = t147 * t433 - t151 * t435;
t441 = t147 * t435 + t151 * t433;
t446 = t159 * pkin(24);
t449 = t159 * t433 - t163 * t435;
t453 = t159 * t435 + t163 * t433;
t458 = t169 * pkin(24);
t461 = -t169 * t433 + t171 * t435;
t465 = -t169 * t435 - t171 * t433;
t487 = atan2((t48 * t105 * t98 - t49 * t97 * t98) * pkin(19) + t71 - t72, (t49 * t105 * t98 + t48 * t97 * t98) * pkin(19) - pkin(9) + t51 + t54);
t488 = -t487 + qJ(3) + t432;
t489 = cos(t488);
t491 = sin(t488);
t530 = t181 * t48 - t185 * t49;
t534 = t181 * t49 + t185 * t48;
t541 = t193 * t48 - t197 * t49;
t545 = t193 * t49 + t197 * t48;
t552 = t205 * t48 - t209 * t49;
t556 = t205 * t49 + t209 * t48;
t563 = sin(pkin(3));
t565 = cos(pkin(3));
t567 = t530 * t563 - t534 * t565;
t571 = t530 * t565 + t534 * t563;
t578 = t541 * t563 - t545 * t565;
t582 = t541 * t565 + t545 * t563;
t589 = t552 * t563 - t556 * t565;
t593 = t552 * t565 + t556 * t563;
t600 = t567 * pkin(12);
t602 = pkin(3) + qJ(3) - qJ(4);
t603 = sin(t602);
t606 = t603 * pkin(12) - t260 * pkin(14) + t258 * pkin(15);
t609 = cos(t602);
t611 = -t609 * pkin(12) - t258 * pkin(14) - t260 * pkin(15) - pkin(10);
t612 = atan2(t606, t611);
t613 = t612 + pkin(3) + qJ(3) - qJ(4);
t614 = sin(t613);
t616 = cos(t613);
t618 = t567 * t614 + t571 * t616;
t622 = -t567 * t616 + t571 * t614;
t627 = t578 * pkin(12);
t630 = t578 * t614 + t582 * t616;
t634 = -t578 * t616 + t582 * t614;
t639 = t589 * pkin(12);
t642 = t589 * t614 + t593 * t616;
t646 = -t589 * t616 + t593 * t614;
t653 = t611 ^ 2;
t654 = t606 ^ 2;
t656 = sqrt(t653 + t654);
t677 = -m(1) * (g(1) * (r_base(1) + rSges(1,1)) + g(2) * (r_base(2) + rSges(1,2)) + g(3) * (r_base(3) + rSges(1,3))) - m(2) * (g(1) * (t9 * rSges(2,1) + t11 * rSges(2,2) + r_base(1)) + g(2) * (-t11 * rSges(2,1) + t9 * rSges(2,2) + r_base(2)) + g(3) * (pkin(8) + r_base(3) + rSges(2,3))) - m(3) * (g(1) * (-t25 * rSges(3,1) - t28 * rSges(3,2) + t11 * rSges(3,3) + t23 + r_base(1)) + g(2) * (t34 * rSges(3,1) + t36 * rSges(3,2) + t9 * rSges(3,3) + t33 + r_base(2)) + g(3) * (-t27 * rSges(3,1) + t24 * rSges(3,2) + pkin(8) + r_base(3))) - m(4) * (g(1) * (t147 * rSges(4,1) + t151 * rSges(4,2) + t28 * rSges(4,3) + t23 + t47 + r_base(1)) + g(2) * (t159 * rSges(4,1) + t163 * rSges(4,2) - t36 * rSges(4,3) - t156 + t33 + r_base(2)) + g(3) * (-t169 * rSges(4,1) - t171 * rSges(4,2) - t24 * rSges(4,3) + pkin(8) - t168 + r_base(3))) - m(5) * (g(1) * (t181 * rSges(5,1) + t185 * rSges(5,2) + t28 * rSges(5,3) + t178 + t23 + t47 + r_base(1)) + g(2) * (t193 * rSges(5,1) + t197 * rSges(5,2) - t36 * rSges(5,3) - t156 + t190 + t33 + r_base(2)) + g(3) * (t205 * rSges(5,1) + t209 * rSges(5,2) - t24 * rSges(5,3) + pkin(8) - t168 - t202 + r_base(3))) - m(6) * (g(1) * (t221 * rSges(6,1) + t225 * rSges(6,2) + t28 * rSges(6,3) + t178 + t216 + t23 + t47 + r_base(1)) + g(2) * (t233 * rSges(6,1) + t237 * rSges(6,2) - t36 * rSges(6,3) - t156 + t190 + t230 + t33 + r_base(2)) + g(3) * (t245 * rSges(6,1) + t249 * rSges(6,2) - t24 * rSges(6,3) + pkin(8) - t168 - t202 + t242 + r_base(3))) - m(7) * (g(1) * (t262 * rSges(7,1) + t266 * rSges(7,2) + t28 * rSges(7,3) + t178 + t216 + t23 + t256 + t47 + r_base(1)) + g(2) * (t274 * rSges(7,1) + t278 * rSges(7,2) - t36 * rSges(7,3) - t156 + t190 + t230 + t271 + t33 + r_base(2)) + g(3) * (t286 * rSges(7,1) + t290 * rSges(7,2) - t24 * rSges(7,3) + pkin(8) - t168 - t202 + t242 + t283 + r_base(3))) - m(8) * (g(1) * (-t266 * pkin(13) + t256 + t216 + t178 + t47 + t23 + r_base(1) + (t262 * t298 + t28 * t300) * rSges(8,1) + (-t262 * t300 + t28 * t298) * rSges(8,2) - t266 * rSges(8,3)) + g(2) * (-t278 * pkin(13) + t271 + t230 + t190 - t156 + t33 + r_base(2) + (t274 * t298 - t36 * t300) * rSges(8,1) + (-t274 * t300 - t36 * t298) * rSges(8,2) - t278 * rSges(8,3)) + g(3) * (-t290 * pkin(13) + t283 + t242 - t202 - t168 + pkin(8) + r_base(3) + (-t24 * t300 + t286 * t298) * rSges(8,1) + (-t24 * t298 - t286 * t300) * rSges(8,2) - t290 * rSges(8,3))) - m(9) * (g(1) * (t341 * rSges(9,1) + t345 * rSges(9,2) + t28 * rSges(9,3) + t23 + t47 + r_base(1)) + g(2) * (t352 * rSges(9,1) + t356 * rSges(9,2) - t36 * rSges(9,3) - t156 + t33 + r_base(2)) + g(3) * (t361 * rSges(9,1) + t363 * rSges(9,2) - t24 * rSges(9,3) + pkin(8) - t168 + r_base(3))) - m(10) * (g(1) * (-t341 * pkin(22) + t47 + t23 + r_base(1) + (t341 * t392 - t345 * t394) * rSges(10,1) + (t341 * t394 + t345 * t392) * rSges(10,2) + t28 * rSges(10,3)) + g(2) * (-t352 * pkin(22) - t156 + t33 + r_base(2) + (t352 * t392 - t356 * t394) * rSges(10,1) + (t352 * t394 + t356 * t392) * rSges(10,2) - t36 * rSges(10,3)) + g(3) * (-t361 * pkin(22) - t168 + pkin(8) + r_base(3) + (t361 * t392 - t363 * t394) * rSges(10,1) + (t361 * t394 + t363 * t392) * rSges(10,2) - t24 * rSges(10,3))) - m(11) * (g(1) * (t437 * rSges(11,1) + t441 * rSges(11,2) + t28 * rSges(11,3) + t23 + t431 + t47 + r_base(1)) + g(2) * (t449 * rSges(11,1) + t453 * rSges(11,2) - t36 * rSges(11,3) - t156 + t33 + t446 + r_base(2)) + g(3) * (t461 * rSges(11,1) + t465 * rSges(11,2) - t24 * rSges(11,3) + pkin(8) - t168 - t458 + r_base(3))) - m(12) * (g(1) * (t437 * pkin(19) + t431 + t47 + t23 + r_base(1) + (-t437 * t489 - t441 * t491) * rSges(12,1) + (t437 * t491 - t441 * t489) * rSges(12,2) + t28 * rSges(12,3)) + g(2) * (t449 * pkin(19) + t446 - t156 + t33 + r_base(2) + (-t449 * t489 - t453 * t491) * rSges(12,1) + (t449 * t491 - t453 * t489) * rSges(12,2) - t36 * rSges(12,3)) + g(3) * (t461 * pkin(19) - t458 - t168 + pkin(8) + r_base(3) + (-t461 * t489 - t465 * t491) * rSges(12,1) + (t461 * t491 - t465 * t489) * rSges(12,2) - t24 * rSges(12,3))) - m(13) * (g(1) * (t530 * rSges(13,1) + t534 * rSges(13,2) + t28 * rSges(13,3) + t178 + t216 + t23 + t47 + r_base(1)) + g(2) * (t541 * rSges(13,1) + t545 * rSges(13,2) - t36 * rSges(13,3) - t156 + t190 + t230 + t33 + r_base(2)) + g(3) * (t552 * rSges(13,1) + t556 * rSges(13,2) - t24 * rSges(13,3) + pkin(8) - t168 - t202 + t242 + r_base(3))) - m(14) * (g(1) * (t567 * rSges(14,1) + t571 * rSges(14,2) + t28 * rSges(14,3) + t178 + t216 + t23 + t47 + r_base(1)) + g(2) * (t578 * rSges(14,1) + t582 * rSges(14,2) - t36 * rSges(14,3) - t156 + t190 + t230 + t33 + r_base(2)) + g(3) * (t589 * rSges(14,1) + t593 * rSges(14,2) - t24 * rSges(14,3) + pkin(8) - t168 - t202 + t242 + r_base(3))) - m(15) * (g(1) * (t618 * rSges(15,1) + t622 * rSges(15,2) + t28 * rSges(15,3) + t178 + t216 + t23 + t47 + t600 + r_base(1)) + g(2) * (t630 * rSges(15,1) + t634 * rSges(15,2) - t36 * rSges(15,3) - t156 + t190 + t230 + t33 + t627 + r_base(2)) + g(3) * (t642 * rSges(15,1) + t646 * rSges(15,2) - t24 * rSges(15,3) + pkin(8) - t168 - t202 + t242 + t639 + r_base(3))) - m(16) * (g(1) * (t618 * rSges(16,1) + t28 * rSges(16,2) - t622 * rSges(16,3) - t622 * t656 + t178 + t216 + t23 + t47 + t600 + r_base(1)) + g(2) * (t630 * rSges(16,1) - t36 * rSges(16,2) - t634 * rSges(16,3) - t634 * t656 - t156 + t190 + t230 + t33 + t627 + r_base(2)) + g(3) * (t642 * rSges(16,1) - t24 * rSges(16,2) - t646 * rSges(16,3) - t646 * t656 + pkin(8) - t168 - t202 + t242 + t639 + r_base(3)));
U = t677;
