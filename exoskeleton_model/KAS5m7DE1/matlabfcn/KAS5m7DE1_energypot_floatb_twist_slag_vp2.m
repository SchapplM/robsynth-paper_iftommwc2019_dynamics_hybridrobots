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
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% 
% Output:
% U [1x1]
%   Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7DE1_energypot_floatb_twist_slag_vp2(qJ, r_base, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp2: r_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE1_energypot_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_floatb_twist_worldframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-14 20:13:54
% EndTime: 2020-05-14 20:16:58
% DurationCPUTime: 183.96s
% Computational Cost: add. (13265304->452), mult. (17122107->508), div. (276600->4), fcn. (6865757->43), ass. (0->192)
t1 = sin(qJ(1));
t2 = sin(qJ(2));
t3 = t1 * t2;
t4 = sin(pkin(6));
t6 = cos(qJ(1));
t7 = cos(pkin(6));
t9 = t3 * t4 + t6 * t7;
t11 = cos(qJ(2));
t12 = t1 * t11;
t13 = t12 * pkin(21);
t14 = t6 * pkin(16);
t17 = sin(qJ(3));
t18 = cos(qJ(3));
t19 = -pkin(23) + pkin(24);
t20 = t18 * t19;
t21 = pkin(3) + qJ(3);
t22 = cos(t21);
t23 = t22 * pkin(12);
t24 = -pkin(9) + t20 + t23;
t25 = pkin(11) ^ 2;
t26 = pkin(19) ^ 2;
t27 = t22 ^ 2;
t28 = pkin(12) ^ 2;
t30 = sin(t21);
t31 = t30 ^ 2;
t34 = (-pkin(9) + t20) ^ 2;
t35 = t17 ^ 2;
t36 = t19 ^ 2;
t40 = t17 * t19;
t41 = t30 * pkin(12);
t42 = -t40 + t41;
t45 = 0.2e1 * t24 * t22 * pkin(12) + 0.2e1 * t42 * t30 * pkin(12) - t27 * t28 - t31 * t28 + t35 * t36 + t25 - t26 + t34;
t49 = 0.4e1 * t24 ^ 2 + 0.4e1 * t42 ^ 2;
t51 = t45 ^ 2;
t53 = sqrt(t25 * t49 - t51);
t56 = 0.1e1 / t49;
t58 = pkin(9) - t20 - t23 + (0.2e1 * t24 * t45 - 0.2e1 * t42 * t53) * t56;
t64 = t40 - t41 + (0.2e1 * t24 * t53 + 0.2e1 * t45 * t42) * t56;
t66 = t17 * t58 + t18 * t64;
t67 = 0.1e1 / pkin(19);
t68 = t66 * t67;
t69 = cos(pkin(7));
t70 = t69 * pkin(18);
t71 = t68 * t70;
t74 = -t17 * t64 + t18 * t58;
t75 = t74 * t67;
t76 = sin(pkin(7));
t77 = t76 * pkin(18);
t78 = t75 * t77;
t79 = pkin(17) ^ 2;
t80 = pkin(22) ^ 2;
t81 = t75 * t70;
t82 = t68 * t77;
t83 = t81 - t82;
t84 = t83 ^ 2;
t85 = t71 + t78;
t86 = t85 ^ 2;
t87 = pkin(24) ^ 2;
t88 = -pkin(24) - t81 + t82;
t91 = -0.2e1 * t88 * t83 + 0.2e1 * t85 ^ 2 + t79 - t80 - t84 - t86 + t87;
t95 = 0.4e1 * t85 ^ 2 + 0.4e1 * t88 ^ 2;
t97 = t91 ^ 2;
t99 = sqrt(t79 * t95 - t97);
t102 = 0.1e1 / t95;
t104 = -t71 - t78 - (-0.2e1 * t91 * t85 + 0.2e1 * t88 * t99) * t102;
t109 = t81 - t82 + (0.2e1 * t85 * t99 + 0.2e1 * t88 * t91) * t102 + pkin(24);
t110 = atan2(t104, t109);
t111 = 0.1e1 / pkin(22);
t112 = t104 * t111;
t114 = -t109 * t111;
t116 = t112 * t4 + t114 * t7;
t120 = t112 * t7 - t114 * t4;
t130 = atan2(-(-t116 * t7 + t120 * t4) * pkin(22) + pkin(24) + t81 - t82, (t116 * t4 + t120 * t7) * pkin(22) + t71 + t78);
t131 = t110 - t130;
t132 = sin(t131);
t136 = t3 * t7 - t6 * t4;
t137 = cos(t131);
t148 = t6 * t2;
t149 = t110 + pkin(6);
t150 = sin(t149);
t152 = cos(t149);
t154 = -t1 * t152 + t148 * t150;
t155 = t154 * pkin(23);
t156 = t6 * t11;
t157 = t156 * pkin(21);
t158 = t1 * pkin(16);
t164 = t1 * t150 + t148 * t152;
t166 = t154 * t18 + t164 * t17;
t170 = -t154 * t17 + t164 * t18;
t175 = -t157 + t158 + r_base(2);
t182 = t166 * pkin(9);
t183 = t182 + t155 - t157 + t158 + r_base(2);
t187 = t166 * t17 - t170 * t18;
t188 = sin(pkin(3));
t192 = t166 * t18 + t170 * t17;
t193 = cos(pkin(3));
t195 = t187 * t188 - t192 * t193;
t199 = t187 * t193 + t192 * t188;
t210 = atan2(-t66, t74);
t211 = cos(t210);
t213 = sin(t210);
t215 = t154 * t211 - t164 * t213;
t217 = t154 * pkin(24);
t234 = atan2((t17 * t74 * t67 - t18 * t66 * t67) * pkin(19) + t40 - t41, (t17 * t66 * t67 + t18 * t74 * t67) * pkin(19) - pkin(9) + t20 + t23);
t235 = -t234 + qJ(3) + t210;
t236 = cos(t235);
t240 = t154 * t213 + t164 * t211;
t241 = sin(t235);
t252 = t11 * t150;
t254 = t11 * t152;
t256 = -t252 * t211 + t254 * t213;
t258 = t252 * pkin(24);
t259 = t2 * pkin(21);
t265 = -t254 * t211 - t252 * t213;
t283 = t11 * t4;
t288 = t11 * t7;
t301 = t1 * t7 - t148 * t4;
t308 = -t1 * t4 - t148 * t7;
t321 = -t3 * t150 - t6 * t152;
t325 = t6 * t150 - t3 * t152;
t327 = t325 * t17 + t321 * t18;
t328 = t327 * pkin(9);
t329 = t321 * pkin(23);
t330 = t328 + t329 + t13 + t14 + r_base(1);
t335 = -t321 * t17 + t325 * t18;
t337 = t327 * t17 - t335 * t18;
t341 = t335 * t17 + t327 * t18;
t343 = t337 * t188 - t341 * t193;
t347 = t341 * t188 + t337 * t193;
t354 = -t254 * t17 - t252 * t18;
t358 = t252 * t17 - t254 * t18;
t360 = t354 * t17 - t358 * t18;
t364 = t358 * t17 + t354 * t18;
t366 = t360 * t188 - t364 * t193;
t367 = qJ(4) - qJ(3) + pkin(4);
t368 = sin(t367);
t370 = pkin(3) + qJ(3) - qJ(4);
t371 = sin(t370);
t373 = cos(t367);
t375 = t371 * pkin(12) - t373 * pkin(14) + t368 * pkin(15);
t378 = cos(t370);
t380 = -t378 * pkin(12) - t368 * pkin(14) - t373 * pkin(15) - pkin(10);
t381 = atan2(t375, t380);
t382 = t381 + pkin(3) + qJ(3) - qJ(4);
t383 = cos(t382);
t387 = t364 * t188 + t360 * t193;
t388 = sin(t382);
t390 = -t366 * t383 + t387 * t388;
t391 = t380 ^ 2;
t392 = t375 ^ 2;
t394 = sqrt(t391 + t392);
t396 = t366 * pkin(12);
t397 = t354 * pkin(9);
t398 = t252 * pkin(23);
t403 = t366 * t388 + t387 * t383;
t409 = -g(1) * (m(10) * (-t9 * pkin(22) + t13 + t14 + r_base(1)) + (t9 * t132 - t136 * t137) * mrSges(10,1) + (t136 * t132 + t9 * t137) * mrSges(10,2) + t12 * mrSges(10,3)) - g(2) * (m(5) * (t155 - t157 + t158 + r_base(2)) + t166 * mrSges(5,1) + t170 * mrSges(5,2) - t156 * mrSges(5,3)) - g(2) * (m(4) * t175 + t154 * mrSges(4,1) + t164 * mrSges(4,2) - t156 * mrSges(4,3)) - g(2) * (m(14) * t183 + t195 * mrSges(14,1) + t199 * mrSges(14,2) - t156 * mrSges(14,3)) - g(2) * (m(13) * t183 + t187 * mrSges(13,1) + t192 * mrSges(13,2) - t156 * mrSges(13,3)) - g(2) * (m(12) * (t215 * pkin(19) - t157 + t158 + t217 + r_base(2)) + (-t215 * t236 - t240 * t241) * mrSges(12,1) + (t215 * t241 - t240 * t236) * mrSges(12,2) - t156 * mrSges(12,3)) - g(3) * (m(12) * (t256 * pkin(19) + pkin(8) - t258 - t259 + r_base(3)) + (-t256 * t236 - t265 * t241) * mrSges(12,1) + (-t265 * t236 + t256 * t241) * mrSges(12,2) - t2 * mrSges(12,3)) - g(3) * (m(11) * (-t258 - t259 + pkin(8) + r_base(3)) + t256 * mrSges(11,1) + t265 * mrSges(11,2) - t2 * mrSges(11,3)) - g(3) * (m(10) * (-t283 * pkin(22) + pkin(8) - t259 + r_base(3)) + (t283 * t132 - t288 * t137) * mrSges(10,1) + (t288 * t132 + t283 * t137) * mrSges(10,2) - t2 * mrSges(10,3)) - g(2) * (m(10) * (-t301 * pkin(22) - t157 + t158 + r_base(2)) + (t301 * t132 - t308 * t137) * mrSges(10,1) + (t308 * t132 + t301 * t137) * mrSges(10,2) - t156 * mrSges(10,3)) - g(1) * (m(14) * t330 + t343 * mrSges(14,1) + t347 * mrSges(14,2) + t12 * mrSges(14,3)) - g(3) * (m(16) * (-t390 * t394 + pkin(8) - t259 + t396 + t397 - t398 + r_base(3)) + t403 * mrSges(16,1) - t2 * mrSges(16,2) - t390 * mrSges(16,3));
t417 = cos(qJ(4));
t419 = sin(qJ(4));
t421 = t354 * t417 + t358 * t419;
t422 = t421 * pkin(10);
t428 = -t354 * t419 + t358 * t417;
t430 = -t421 * t368 + t428 * t373;
t434 = -t428 * t368 - t421 * t373;
t439 = t397 - t398 - t259 + pkin(8) + r_base(3);
t459 = t13 + t14 + r_base(1);
t485 = -t259 + pkin(8) + r_base(3);
t492 = pkin(8) + r_base(3);
t505 = -g(3) * (m(15) * (t396 + t397 - t398 - t259 + pkin(8) + r_base(3)) + t403 * mrSges(15,1) + t390 * mrSges(15,2) - t2 * mrSges(15,3)) - g(3) * (m(7) * (t422 + t397 - t398 - t259 + pkin(8) + r_base(3)) + t430 * mrSges(7,1) + t434 * mrSges(7,2) - t2 * mrSges(7,3)) - g(3) * (m(6) * t439 + t421 * mrSges(6,1) + t428 * mrSges(6,2) - t2 * mrSges(6,3)) - g(1) * (m(13) * t330 + t337 * mrSges(13,1) + t341 * mrSges(13,2) + t12 * mrSges(13,3)) - g(1) * (m(5) * (t329 + t13 + t14 + r_base(1)) + t327 * mrSges(5,1) + t335 * mrSges(5,2) + t12 * mrSges(5,3)) - g(1) * (m(4) * t459 + t321 * mrSges(4,1) + t325 * mrSges(4,2) + t12 * mrSges(4,3)) - g(3) * (m(14) * t439 + t366 * mrSges(14,1) + t387 * mrSges(14,2) - t2 * mrSges(14,3)) - g(3) * (m(13) * t439 + t360 * mrSges(13,1) + t364 * mrSges(13,2) - t2 * mrSges(13,3)) - g(3) * (m(5) * (-t398 - t259 + pkin(8) + r_base(3)) + t354 * mrSges(5,1) + t358 * mrSges(5,2) - t2 * mrSges(5,3)) - g(3) * (m(4) * t485 - t252 * mrSges(4,1) - t254 * mrSges(4,2) - t2 * mrSges(4,3)) - g(3) * (m(3) * t492 - t11 * mrSges(3,1) + t2 * mrSges(3,2)) - g(1) * (m(3) * (t14 + r_base(1)) - t3 * mrSges(3,1) - t12 * mrSges(3,2) + t6 * mrSges(3,3));
t557 = cos(qJ(5));
t559 = sin(qJ(5));
t572 = t166 * t417 + t170 * t419;
t576 = -t166 * t419 + t170 * t417;
t578 = -t576 * t368 - t572 * t373;
t580 = t572 * pkin(10);
t585 = -t572 * t368 + t576 * t373;
t597 = -g(3) * (m(9) * t485 + t283 * mrSges(9,1) + t288 * mrSges(9,2) - t2 * mrSges(9,3)) - g(2) * (m(9) * t175 + t301 * mrSges(9,1) + t308 * mrSges(9,2) - t156 * mrSges(9,3)) - g(1) * (m(9) * t459 + t9 * mrSges(9,1) + t136 * mrSges(9,2) + t12 * mrSges(9,3)) - g(2) * (m(3) * (t158 + r_base(2)) + t148 * mrSges(3,1) + t156 * mrSges(3,2) + t1 * mrSges(3,3)) - g(1) * (m(2) * r_base(1) + t1 * mrSges(2,1) + t6 * mrSges(2,2)) - g(2) * (m(1) * r_base(2) + mrSges(1,2)) - g(3) * (m(2) * t492 + mrSges(2,3)) - g(3) * (m(1) * r_base(3) + mrSges(1,3)) - g(2) * (m(2) * r_base(2) - t6 * mrSges(2,1) + t1 * mrSges(2,2)) - g(1) * (m(1) * r_base(1) + mrSges(1,1)) - g(3) * (m(8) * (-t434 * pkin(13) + pkin(8) - t259 + t397 - t398 + t422 + r_base(3)) + (-t2 * t559 + t430 * t557) * mrSges(8,1) + (-t2 * t557 - t430 * t559) * mrSges(8,2) - t434 * mrSges(8,3)) - g(2) * (m(8) * (-t578 * pkin(13) + t155 - t157 + t158 + t182 + t580 + r_base(2)) + (-t156 * t559 + t585 * t557) * mrSges(8,1) + (-t156 * t557 - t585 * t559) * mrSges(8,2) - t578 * mrSges(8,3));
t600 = t327 * t417 + t335 * t419;
t604 = -t327 * t419 + t335 * t417;
t606 = -t604 * t368 - t600 * t373;
t608 = t600 * pkin(10);
t613 = -t600 * t368 + t604 * t373;
t640 = -t195 * t383 + t199 * t388;
t642 = t195 * pkin(12);
t647 = t195 * t388 + t199 * t383;
t669 = -t343 * t383 + t347 * t388;
t671 = t343 * pkin(12);
t676 = t343 * t388 + t347 * t383;
t704 = t321 * t211 - t325 * t213;
t706 = t321 * pkin(24);
t712 = t325 * t211 + t321 * t213;
t730 = -g(1) * (m(8) * (-t606 * pkin(13) + t13 + t14 + t328 + t329 + t608 + r_base(1)) + (t12 * t559 + t613 * t557) * mrSges(8,1) + (t12 * t557 - t613 * t559) * mrSges(8,2) - t606 * mrSges(8,3)) - g(2) * (m(11) * (t217 - t157 + t158 + r_base(2)) + t215 * mrSges(11,1) + t240 * mrSges(11,2) - t156 * mrSges(11,3)) - g(2) * (m(6) * t183 + t572 * mrSges(6,1) + t576 * mrSges(6,2) - t156 * mrSges(6,3)) - g(2) * (m(16) * (-t640 * t394 + t155 - t157 + t158 + t182 + t642 + r_base(2)) + t647 * mrSges(16,1) - t156 * mrSges(16,2) - t640 * mrSges(16,3)) - g(2) * (m(15) * (t642 + t182 + t155 - t157 + t158 + r_base(2)) + t647 * mrSges(15,1) + t640 * mrSges(15,2) - t156 * mrSges(15,3)) - g(2) * (m(7) * (t580 + t182 + t155 - t157 + t158 + r_base(2)) + t585 * mrSges(7,1) + t578 * mrSges(7,2) - t156 * mrSges(7,3)) - g(1) * (m(16) * (-t669 * t394 + t13 + t14 + t328 + t329 + t671 + r_base(1)) + t676 * mrSges(16,1) + t12 * mrSges(16,2) - t669 * mrSges(16,3)) - g(1) * (m(15) * (t671 + t328 + t329 + t13 + t14 + r_base(1)) + t676 * mrSges(15,1) + t669 * mrSges(15,2) + t12 * mrSges(15,3)) - g(1) * (m(7) * (t608 + t328 + t329 + t13 + t14 + r_base(1)) + t613 * mrSges(7,1) + t606 * mrSges(7,2) + t12 * mrSges(7,3)) - g(1) * (m(6) * t330 + t600 * mrSges(6,1) + t604 * mrSges(6,2) + t12 * mrSges(6,3)) - g(1) * (m(12) * (t704 * pkin(19) + t13 + t14 + t706 + r_base(1)) + (-t704 * t236 - t712 * t241) * mrSges(12,1) + (-t712 * t236 + t704 * t241) * mrSges(12,2) + t12 * mrSges(12,3)) - g(1) * (m(11) * (t706 + t13 + t14 + r_base(1)) + t704 * mrSges(11,1) + t712 * mrSges(11,2) + t12 * mrSges(11,3));
t732 = t409 + t505 + t597 + t730;
U = t732;
