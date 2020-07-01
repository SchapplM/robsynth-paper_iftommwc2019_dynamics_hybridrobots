% Calculate potential energy for
% KAS5m7TE
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
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% 
% Output:
% U [1x1]
%   Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7TE_energypot_fixb_slag_vp2(qJ, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_energypot_fixb_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7TE_energypot_fixb_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_energypot_fixb_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7TE_energypot_fixb_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7TE_energypot_fixb_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_worldframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-04-13 19:09:42
% EndTime: 2020-04-13 19:15:34
% DurationCPUTime: 207.87s
% Computational Cost: add. (13660860->413), mult. (17634394->535), div. (287358->7), fcn. (7070285->28), ass. (0->189)
t5 = sin(qJ(1));
t6 = sin(qJ(2));
t7 = t5 * t6;
t8 = cos(qJ(3));
t9 = -pkin(23) + pkin(24);
t10 = t8 * t9;
t11 = pkin(3) + qJ(3);
t12 = cos(t11);
t13 = t12 * pkin(12);
t14 = -pkin(9) + t10 + t13;
t15 = pkin(11) ^ 2;
t16 = pkin(19) ^ 2;
t17 = t12 ^ 2;
t18 = pkin(12) ^ 2;
t20 = sin(t11);
t21 = t20 ^ 2;
t24 = (-pkin(9) + t10) ^ 2;
t25 = sin(qJ(3));
t26 = t25 ^ 2;
t27 = t9 ^ 2;
t31 = t25 * t9;
t32 = t20 * pkin(12);
t33 = -t31 + t32;
t36 = 0.2e1 * t14 * t12 * pkin(12) + 0.2e1 * t33 * t20 * pkin(12) - t17 * t18 - t21 * t18 + t26 * t27 + t15 - t16 + t24;
t40 = 0.4e1 * t14 ^ 2 + 0.4e1 * t33 ^ 2;
t42 = t36 ^ 2;
t44 = sqrt(t15 * t40 - t42);
t47 = 0.1e1 / t40;
t49 = pkin(9) - t10 - t13 + (0.2e1 * t14 * t36 - 0.2e1 * t33 * t44) * t47;
t55 = t31 - t32 + (0.2e1 * t14 * t44 + 0.2e1 * t36 * t33) * t47;
t57 = -t25 * t55 + t8 * t49;
t58 = 0.1e1 / pkin(19);
t59 = t57 * t58;
t60 = cos(pkin(7));
t61 = t60 * pkin(18);
t62 = t59 * t61;
t65 = t25 * t49 + t8 * t55;
t66 = t65 * t58;
t67 = sin(pkin(7));
t68 = t67 * pkin(18);
t69 = t66 * t68;
t70 = -pkin(24) - t62 + t69;
t71 = pkin(17) ^ 2;
t72 = pkin(22) ^ 2;
t73 = t62 - t69;
t74 = t73 ^ 2;
t75 = t66 * t61;
t76 = t59 * t68;
t77 = t75 + t76;
t78 = t77 ^ 2;
t79 = pkin(24) ^ 2;
t82 = -0.2e1 * t70 * t73 + 0.2e1 * t77 ^ 2 + t71 - t72 - t74 - t78 + t79;
t86 = 0.4e1 * t70 ^ 2 + 0.4e1 * t77 ^ 2;
t88 = t82 ^ 2;
t90 = sqrt(t71 * t86 - t88);
t93 = 0.1e1 / t86;
t96 = 0.1e1 / pkin(22);
t97 = (-t62 + t69 - (0.2e1 * t70 * t82 + 0.2e1 * t77 * t90) * t93 - pkin(24)) * t96;
t98 = sin(pkin(6));
t105 = (-t75 - t76 - (0.2e1 * t70 * t90 - 0.2e1 * t82 * t77) * t93) * t96;
t106 = cos(pkin(6));
t108 = t105 * t106 - t97 * t98;
t110 = cos(qJ(1));
t113 = t105 * t98 + t97 * t106;
t115 = -t7 * t108 + t110 * t113;
t119 = t110 * t108 + t7 * t113;
t121 = t115 * t8 + t119 * t25;
t122 = cos(qJ(4));
t126 = -t115 * t25 + t119 * t8;
t127 = sin(qJ(4));
t129 = t121 * t122 + t126 * t127;
t130 = qJ(4) - qJ(3) + pkin(4);
t131 = cos(t130);
t135 = -t121 * t127 + t126 * t122;
t136 = sin(t130);
t138 = -t129 * t131 - t135 * t136;
t140 = t129 * pkin(10);
t141 = t121 * pkin(9);
t142 = t115 * pkin(23);
t143 = cos(qJ(2));
t144 = t5 * t143;
t145 = t144 * pkin(21);
t146 = t110 * pkin(16);
t151 = -t129 * t136 + t135 * t131;
t152 = cos(qJ(5));
t154 = sin(qJ(5));
t172 = t141 + t142 + t145 + t146;
t181 = t121 * t25 - t126 * t8;
t182 = sin(pkin(3));
t186 = t121 * t8 + t126 * t25;
t187 = cos(pkin(3));
t189 = t181 * t182 - t186 * t187;
t192 = pkin(3) + qJ(3) - qJ(4);
t193 = cos(t192);
t195 = -t193 * pkin(12) - t136 * pkin(14) - t131 * pkin(15) - pkin(10);
t196 = t195 ^ 2;
t199 = sin(t192);
t201 = -t199 * pkin(12) + t131 * pkin(14) - t136 * pkin(15);
t202 = t201 ^ 2;
t204 = sqrt(t196 + t202);
t205 = 0.1e1 / t204;
t206 = t195 * t205;
t208 = -t201 * t205;
t210 = -t206 * t193 + t208 * t199;
t214 = t181 * t187 + t186 * t182;
t217 = t208 * t193 + t206 * t199;
t219 = t189 * t210 + t214 * t217;
t221 = t189 * pkin(12);
t226 = t189 * t217 - t214 * t210;
t255 = t115 * t57 * t58 + t119 * t65 * t58;
t257 = t115 * pkin(24);
t264 = t25 * t65 * t58 + t8 * t57 * t58;
t266 = t264 * pkin(19) - pkin(9) + t10 + t13;
t268 = 0.1e1 / pkin(11);
t274 = t25 * t57 * t58 - t8 * t65 * t58;
t276 = t274 * pkin(19) + t31 - t32;
t279 = -t264 * t266 * t268 - t274 * t276 * t268;
t285 = -t115 * t65 * t58 + t119 * t57 * t58;
t290 = t264 * t276 * t268 - t266 * t268 * t274;
t308 = t145 + t146;
t315 = -g(3) * (m(2) * pkin(8) + mrSges(2,3)) - g(3) * mrSges(1,3) - g(1) * (m(8) * (-t138 * pkin(13) + t140 + t141 + t142 + t145 + t146) + (t144 * t154 + t151 * t152) * mrSges(8,1) + (t144 * t152 - t151 * t154) * mrSges(8,2) - t138 * mrSges(8,3)) - g(1) * (m(7) * (t140 + t141 + t142 + t145 + t146) + t151 * mrSges(7,1) + t138 * mrSges(7,2) + t144 * mrSges(7,3)) - g(1) * (m(6) * t172 + t129 * mrSges(6,1) + t135 * mrSges(6,2) + t144 * mrSges(6,3)) - g(1) * (m(16) * (-t219 * t204 + t141 + t142 + t145 + t146 + t221) + t226 * mrSges(16,1) + t144 * mrSges(16,2) - t219 * mrSges(16,3)) - g(1) * (m(15) * (t221 + t141 + t142 + t145 + t146) + t226 * mrSges(15,1) + t219 * mrSges(15,2) + t144 * mrSges(15,3)) - g(1) * (m(14) * t172 + t189 * mrSges(14,1) + t214 * mrSges(14,2) + t144 * mrSges(14,3)) - g(1) * (m(13) * t172 + t181 * mrSges(13,1) + t186 * mrSges(13,2) + t144 * mrSges(13,3)) - g(1) * (m(12) * (t255 * pkin(19) + t145 + t146 + t257) + (t255 * t279 + t285 * t290) * mrSges(12,1) + (-t255 * t290 + t285 * t279) * mrSges(12,2) + t144 * mrSges(12,3)) - g(1) * (m(5) * (t142 + t145 + t146) + t121 * mrSges(5,1) + t126 * mrSges(5,2) + t144 * mrSges(5,3)) - g(1) * (m(4) * t308 + t115 * mrSges(4,1) + t119 * mrSges(4,2) + t144 * mrSges(4,3));
t325 = t110 * t106 + t7 * t98;
t331 = t108 * t106 + t113 * t98;
t333 = t331 * pkin(22) + t75 + t76;
t335 = 0.1e1 / pkin(17);
t339 = -t113 * t106 + t108 * t98;
t341 = -t339 * pkin(22) + pkin(24) + t62 - t69;
t344 = t331 * t333 * t335 - t339 * t341 * t335;
t348 = t7 * t106 - t110 * t98;
t353 = -t331 * t341 * t335 - t333 * t335 * t339;
t382 = t110 * t6;
t385 = t382 * t108 + t5 * t113;
t389 = t5 * t108 - t382 * t113;
t391 = t389 * t25 + t385 * t8;
t395 = -t385 * t25 + t389 * t8;
t397 = t391 * t122 + t395 * t127;
t398 = t397 * pkin(10);
t399 = t391 * pkin(9);
t400 = t385 * pkin(23);
t401 = t110 * t143;
t402 = t401 * pkin(21);
t403 = t5 * pkin(16);
t409 = t395 * t122 - t391 * t127;
t411 = t409 * t131 - t397 * t136;
t415 = -t397 * t131 - t409 * t136;
t420 = t399 + t400 - t402 + t403;
t429 = t391 * t25 - t395 * t8;
t433 = t395 * t25 + t391 * t8;
t435 = t429 * t182 - t433 * t187;
t439 = t433 * t182 + t429 * t187;
t441 = t435 * t210 + t439 * t217;
t443 = t435 * pkin(12);
t448 = -t439 * t210 + t435 * t217;
t475 = t385 * pkin(24);
t482 = t385 * t57 * t58 + t389 * t65 * t58;
t488 = -t385 * t65 * t58 + t389 * t57 * t58;
t493 = -g(1) * (m(11) * (t257 + t145 + t146) + t255 * mrSges(11,1) + t285 * mrSges(11,2) + t144 * mrSges(11,3)) - g(1) * (m(10) * (-t325 * pkin(22) + t145 + t146) + (t325 * t344 + t348 * t353) * mrSges(10,1) + (-t325 * t353 + t348 * t344) * mrSges(10,2) + t144 * mrSges(10,3)) - g(1) * (m(9) * t308 + t325 * mrSges(9,1) + t348 * mrSges(9,2) + t144 * mrSges(9,3)) - g(1) * (m(3) * t110 * pkin(16) - t7 * mrSges(3,1) - t144 * mrSges(3,2) + t110 * mrSges(3,3)) - g(1) * (t5 * mrSges(2,1) + t110 * mrSges(2,2)) - g(1) * mrSges(1,1) - g(2) * (m(7) * (t398 + t399 + t400 - t402 + t403) + t411 * mrSges(7,1) + t415 * mrSges(7,2) - t401 * mrSges(7,3)) - g(2) * (m(6) * t420 + t397 * mrSges(6,1) + t409 * mrSges(6,2) - t401 * mrSges(6,3)) - g(2) * (m(16) * (-t441 * t204 + t399 + t400 - t402 + t403 + t443) + t448 * mrSges(16,1) - t401 * mrSges(16,2) - t441 * mrSges(16,3)) - g(2) * (m(15) * (t443 + t399 + t400 - t402 + t403) + t448 * mrSges(15,1) + t441 * mrSges(15,2) - t401 * mrSges(15,3)) - g(2) * (m(8) * (-t415 * pkin(13) + t398 + t399 + t400 - t402 + t403) + (t411 * t152 - t401 * t154) * mrSges(8,1) + (-t401 * t152 - t411 * t154) * mrSges(8,2) - t415 * mrSges(8,3)) - g(2) * (m(11) * (t475 - t402 + t403) + t482 * mrSges(11,1) + t488 * mrSges(11,2) - t401 * mrSges(11,3));
t497 = t5 * t106 - t382 * t98;
t504 = -t382 * t106 - t5 * t98;
t522 = -t402 + t403;
t559 = t143 * t108;
t561 = -t143 * t113;
t563 = -t561 * t25 - t559 * t8;
t567 = t559 * t25 - t561 * t8;
t569 = t563 * t122 + t567 * t127;
t573 = t567 * t122 - t563 * t127;
t575 = -t569 * t131 - t573 * t136;
t577 = t569 * pkin(10);
t578 = t563 * pkin(9);
t579 = t559 * pkin(23);
t580 = t6 * pkin(21);
t585 = t573 * t131 - t569 * t136;
t604 = t578 - t579 - t580 + pkin(8);
t611 = -g(2) * (m(10) * (-t497 * pkin(22) - t402 + t403) + (t497 * t344 + t504 * t353) * mrSges(10,1) + (t504 * t344 - t497 * t353) * mrSges(10,2) - t401 * mrSges(10,3)) - g(2) * (m(5) * (t400 - t402 + t403) + t391 * mrSges(5,1) + t395 * mrSges(5,2) - t401 * mrSges(5,3)) - g(2) * (m(4) * t522 + t385 * mrSges(4,1) + t389 * mrSges(4,2) - t401 * mrSges(4,3)) - g(2) * (m(14) * t420 + t435 * mrSges(14,1) + t439 * mrSges(14,2) - t401 * mrSges(14,3)) - g(2) * (m(13) * t420 + t429 * mrSges(13,1) + t433 * mrSges(13,2) - t401 * mrSges(13,3)) - g(2) * (m(9) * t522 + t497 * mrSges(9,1) + t504 * mrSges(9,2) - t401 * mrSges(9,3)) - g(2) * (m(3) * t5 * pkin(16) + t382 * mrSges(3,1) + t401 * mrSges(3,2) + t5 * mrSges(3,3)) - g(2) * (-t110 * mrSges(2,1) + t5 * mrSges(2,2)) - g(2) * mrSges(1,2) - g(3) * (m(8) * (-t575 * pkin(13) + pkin(8) + t577 + t578 - t579 - t580) + (t585 * t152 - t6 * t154) * mrSges(8,1) + (-t6 * t152 - t585 * t154) * mrSges(8,2) - t575 * mrSges(8,3)) - g(3) * (m(7) * (t577 + t578 - t579 - t580 + pkin(8)) + t585 * mrSges(7,1) + t575 * mrSges(7,2) - t6 * mrSges(7,3)) - g(3) * (m(6) * t604 + t569 * mrSges(6,1) + t573 * mrSges(6,2) - t6 * mrSges(6,3));
t628 = t563 * t25 - t567 * t8;
t632 = t567 * t25 + t563 * t8;
t634 = t628 * t182 - t632 * t187;
t638 = t632 * t182 + t628 * t187;
t640 = t634 * t210 + t638 * t217;
t642 = t634 * pkin(12);
t647 = -t638 * t210 + t634 * t217;
t662 = -t559 * t59 - t561 * t66;
t664 = t559 * pkin(24);
t670 = t559 * t66 - t561 * t59;
t688 = t143 * t98;
t693 = t143 * t106;
t704 = -t580 + pkin(8);
t741 = -g(2) * (m(12) * (t482 * pkin(19) - t402 + t403 + t475) + (t482 * t279 + t488 * t290) * mrSges(12,1) + (t488 * t279 - t482 * t290) * mrSges(12,2) - t401 * mrSges(12,3)) - g(3) * (m(16) * (-t640 * t204 + pkin(8) + t578 - t579 - t580 + t642) + t647 * mrSges(16,1) - t6 * mrSges(16,2) - t640 * mrSges(16,3)) - g(3) * (m(15) * (t642 + t578 - t579 - t580 + pkin(8)) + t647 * mrSges(15,1) + t640 * mrSges(15,2) - t6 * mrSges(15,3)) - g(3) * (m(12) * (t662 * pkin(19) + pkin(8) - t580 - t664) + (t662 * t279 + t670 * t290) * mrSges(12,1) + (t670 * t279 - t662 * t290) * mrSges(12,2) - t6 * mrSges(12,3)) - g(3) * (m(11) * (-t664 - t580 + pkin(8)) + t662 * mrSges(11,1) + t670 * mrSges(11,2) - t6 * mrSges(11,3)) - g(3) * (m(10) * (-t688 * pkin(22) + pkin(8) - t580) + (t688 * t344 + t693 * t353) * mrSges(10,1) + (t693 * t344 - t688 * t353) * mrSges(10,2) - t6 * mrSges(10,3)) - g(3) * (m(4) * t704 - t559 * mrSges(4,1) - t561 * mrSges(4,2) - t6 * mrSges(4,3)) - g(3) * (m(5) * (-t579 - t580 + pkin(8)) + t563 * mrSges(5,1) + t567 * mrSges(5,2) - t6 * mrSges(5,3)) - g(3) * (m(14) * t604 + t634 * mrSges(14,1) + t638 * mrSges(14,2) - t6 * mrSges(14,3)) - g(3) * (m(13) * t604 + t628 * mrSges(13,1) + t632 * mrSges(13,2) - t6 * mrSges(13,3)) - g(3) * (m(9) * t704 + t688 * mrSges(9,1) + t693 * mrSges(9,2) - t6 * mrSges(9,3)) - g(3) * (m(3) * pkin(8) - t143 * mrSges(3,1) + t6 * mrSges(3,2));
t743 = t315 + t493 + t611 + t741;
U = t743;
