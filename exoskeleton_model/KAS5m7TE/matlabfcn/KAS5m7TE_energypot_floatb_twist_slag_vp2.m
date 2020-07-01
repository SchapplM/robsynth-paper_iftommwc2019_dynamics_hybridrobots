% Calculate potential energy for
% KAS5m7TE
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
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7TE_energypot_floatb_twist_slag_vp2(qJ, r_base, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp2: r_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_floatb_twist_worldframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-04-13 19:03:37
% EndTime: 2020-04-13 19:09:01
% DurationCPUTime: 197.82s
% Computational Cost: add. (13660908->450), mult. (17634399->538), div. (287358->7), fcn. (7070285->28), ass. (0->190)
t1 = cos(qJ(2));
t2 = cos(qJ(3));
t3 = -pkin(23) + pkin(24);
t4 = t2 * t3;
t5 = pkin(3) + qJ(3);
t6 = cos(t5);
t7 = t6 * pkin(12);
t8 = -pkin(9) + t4 + t7;
t9 = pkin(11) ^ 2;
t10 = pkin(19) ^ 2;
t11 = t6 ^ 2;
t12 = pkin(12) ^ 2;
t14 = sin(t5);
t15 = t14 ^ 2;
t18 = (-pkin(9) + t4) ^ 2;
t19 = sin(qJ(3));
t20 = t19 ^ 2;
t21 = t3 ^ 2;
t25 = t19 * t3;
t26 = t14 * pkin(12);
t27 = -t25 + t26;
t30 = 0.2e1 * t27 * t14 * pkin(12) + 0.2e1 * t8 * t6 * pkin(12) - t11 * t12 - t15 * t12 + t20 * t21 - t10 + t18 + t9;
t34 = 0.4e1 * t27 ^ 2 + 0.4e1 * t8 ^ 2;
t36 = t30 ^ 2;
t38 = sqrt(t9 * t34 - t36);
t41 = 0.1e1 / t34;
t43 = pkin(9) - t4 - t7 + (-0.2e1 * t27 * t38 + 0.2e1 * t8 * t30) * t41;
t49 = t25 - t26 + (0.2e1 * t30 * t27 + 0.2e1 * t8 * t38) * t41;
t51 = -t19 * t49 + t2 * t43;
t52 = 0.1e1 / pkin(19);
t53 = t51 * t52;
t54 = cos(pkin(7));
t55 = t54 * pkin(18);
t56 = t53 * t55;
t59 = t19 * t43 + t2 * t49;
t60 = t59 * t52;
t61 = sin(pkin(7));
t62 = t61 * pkin(18);
t63 = t60 * t62;
t64 = -pkin(24) - t56 + t63;
t65 = pkin(17) ^ 2;
t66 = pkin(22) ^ 2;
t67 = t56 - t63;
t68 = t67 ^ 2;
t69 = t60 * t55;
t70 = t53 * t62;
t71 = t69 + t70;
t72 = t71 ^ 2;
t73 = pkin(24) ^ 2;
t76 = -0.2e1 * t64 * t67 + 0.2e1 * t71 ^ 2 + t65 - t66 - t68 - t72 + t73;
t80 = 0.4e1 * t64 ^ 2 + 0.4e1 * t71 ^ 2;
t82 = t76 ^ 2;
t84 = sqrt(t65 * t80 - t82);
t87 = 0.1e1 / t80;
t90 = 0.1e1 / pkin(22);
t91 = (-t56 + t63 - (0.2e1 * t64 * t76 + 0.2e1 * t71 * t84) * t87 - pkin(24)) * t90;
t92 = sin(pkin(6));
t99 = (-t69 - t70 - (0.2e1 * t64 * t84 - 0.2e1 * t76 * t71) * t87) * t90;
t100 = cos(pkin(6));
t102 = t99 * t100 - t91 * t92;
t103 = t1 * t102;
t107 = -t91 * t100 - t99 * t92;
t108 = t1 * t107;
t110 = -t103 * t2 - t108 * t19;
t111 = cos(qJ(4));
t115 = t103 * t19 - t108 * t2;
t116 = sin(qJ(4));
t118 = t110 * t111 + t115 * t116;
t119 = qJ(4) - qJ(3) + pkin(4);
t120 = cos(t119);
t124 = -t110 * t116 + t115 * t111;
t125 = sin(t119);
t127 = -t118 * t120 - t124 * t125;
t129 = t118 * pkin(10);
t130 = t110 * pkin(9);
t131 = t103 * pkin(23);
t132 = sin(qJ(2));
t133 = t132 * pkin(21);
t138 = -t118 * t125 + t124 * t120;
t139 = cos(qJ(5));
t141 = sin(qJ(5));
t152 = sin(qJ(1));
t153 = t152 * t132;
t155 = cos(qJ(1));
t157 = -t153 * t102 - t155 * t107;
t161 = t155 * t102 - t153 * t107;
t163 = t157 * t2 + t161 * t19;
t167 = -t157 * t19 + t161 * t2;
t169 = t163 * t111 + t167 * t116;
t173 = t167 * t111 - t163 * t116;
t175 = -t169 * t120 - t173 * t125;
t177 = t169 * pkin(10);
t178 = t163 * pkin(9);
t179 = t157 * pkin(23);
t180 = t152 * t1;
t181 = t180 * pkin(21);
t182 = t155 * pkin(16);
t187 = t173 * t120 - t169 * t125;
t199 = t155 * t132;
t202 = t199 * t102 - t152 * t107;
t206 = t152 * t102 + t199 * t107;
t208 = t206 * t19 + t202 * t2;
t212 = -t202 * t19 + t206 * t2;
t214 = t208 * t111 + t212 * t116;
t215 = t214 * pkin(10);
t216 = t208 * pkin(9);
t217 = t202 * pkin(23);
t218 = t155 * t1;
t219 = t218 * pkin(21);
t220 = t152 * pkin(16);
t226 = t212 * t111 - t208 * t116;
t228 = t226 * t120 - t214 * t125;
t232 = -t214 * t120 - t226 * t125;
t237 = t216 + t217 - t219 + t220 + r_base(2);
t246 = -t103 * t53 - t108 * t60;
t248 = t103 * pkin(24);
t255 = t19 * t59 * t52 + t2 * t51 * t52;
t257 = t255 * pkin(19) - pkin(9) + t4 + t7;
t259 = 0.1e1 / pkin(11);
t265 = t19 * t51 * t52 - t2 * t59 * t52;
t267 = t265 * pkin(19) + t25 - t26;
t270 = -t255 * t257 * t259 - t265 * t267 * t259;
t274 = t103 * t60 - t108 * t53;
t279 = t255 * t267 * t259 - t257 * t259 * t265;
t297 = t1 * t92;
t303 = t102 * t100 - t107 * t92;
t305 = t303 * pkin(22) + t69 + t70;
t307 = 0.1e1 / pkin(17);
t311 = t107 * t100 + t102 * t92;
t313 = -t311 * pkin(22) + pkin(24) + t56 - t63;
t316 = t303 * t305 * t307 - t311 * t313 * t307;
t318 = t1 * t100;
t323 = -t303 * t313 * t307 - t305 * t307 * t311;
t341 = t181 + t182 + r_base(1);
t351 = t208 * t19 - t212 * t2;
t352 = sin(pkin(3));
t356 = t212 * t19 + t208 * t2;
t357 = cos(pkin(3));
t359 = t351 * t352 - t356 * t357;
t363 = t351 * t357 + t356 * t352;
t374 = -t133 + pkin(8) + r_base(3);
t381 = -g(3) * (m(8) * (-t127 * pkin(13) + pkin(8) + t129 + t130 - t131 - t133 + r_base(3)) + (-t132 * t141 + t138 * t139) * mrSges(8,1) + (-t132 * t139 - t138 * t141) * mrSges(8,2) - t127 * mrSges(8,3)) - g(1) * (m(8) * (-t175 * pkin(13) + t177 + t178 + t179 + t181 + t182 + r_base(1)) + (t187 * t139 + t180 * t141) * mrSges(8,1) + (t180 * t139 - t187 * t141) * mrSges(8,2) - t175 * mrSges(8,3)) - g(2) * (m(7) * (t215 + t216 + t217 - t219 + t220 + r_base(2)) + t228 * mrSges(7,1) + t232 * mrSges(7,2) - t218 * mrSges(7,3)) - g(2) * (m(6) * t237 + t214 * mrSges(6,1) + t226 * mrSges(6,2) - t218 * mrSges(6,3)) - g(3) * (m(12) * (t246 * pkin(19) + pkin(8) - t133 - t248 + r_base(3)) + (t246 * t270 + t274 * t279) * mrSges(12,1) + (-t246 * t279 + t274 * t270) * mrSges(12,2) - t132 * mrSges(12,3)) - g(3) * (m(11) * (-t248 - t133 + pkin(8) + r_base(3)) + t246 * mrSges(11,1) + t274 * mrSges(11,2) - t132 * mrSges(11,3)) - g(3) * (m(10) * (-t297 * pkin(22) + pkin(8) - t133 + r_base(3)) + (t297 * t316 + t318 * t323) * mrSges(10,1) + (-t297 * t323 + t318 * t316) * mrSges(10,2) - t132 * mrSges(10,3)) - g(1) * (m(5) * (t179 + t181 + t182 + r_base(1)) + t163 * mrSges(5,1) + t167 * mrSges(5,2) + t180 * mrSges(5,3)) - g(1) * (m(4) * t341 + t157 * mrSges(4,1) + t161 * mrSges(4,2) + t180 * mrSges(4,3)) - g(2) * (m(14) * t237 + t359 * mrSges(14,1) + t363 * mrSges(14,2) - t218 * mrSges(14,3)) - g(2) * (m(13) * t237 + t351 * mrSges(13,1) + t356 * mrSges(13,2) - t218 * mrSges(13,3)) - g(3) * (m(4) * t374 - t103 * mrSges(4,1) - t108 * mrSges(4,2) - t132 * mrSges(4,3));
t389 = t157 * pkin(24);
t396 = t157 * t51 * t52 + t161 * t59 * t52;
t402 = -t157 * t59 * t52 + t161 * t51 * t52;
t409 = t155 * t100 + t153 * t92;
t416 = t153 * t100 - t155 * t92;
t427 = t130 - t131 - t133 + pkin(8) + r_base(3);
t431 = t110 * t19 - t115 * t2;
t435 = t110 * t2 + t115 * t19;
t437 = t431 * t352 - t435 * t357;
t441 = t435 * t352 + t431 * t357;
t471 = pkin(8) + r_base(3);
t477 = -t219 + t220 + r_base(2);
t481 = t152 * t100 - t199 * t92;
t485 = -t199 * t100 - t152 * t92;
t494 = t202 * t51 * t52 + t206 * t59 * t52;
t496 = t202 * pkin(24);
t504 = -t202 * t59 * t52 + t206 * t51 * t52;
t522 = -g(3) * (m(5) * (-t131 - t133 + pkin(8) + r_base(3)) + t110 * mrSges(5,1) + t115 * mrSges(5,2) - t132 * mrSges(5,3)) - g(1) * (m(11) * (t389 + t181 + t182 + r_base(1)) + t396 * mrSges(11,1) + t402 * mrSges(11,2) + t180 * mrSges(11,3)) - g(1) * (m(10) * (-t409 * pkin(22) + t181 + t182 + r_base(1)) + (t409 * t316 + t416 * t323) * mrSges(10,1) + (t416 * t316 - t409 * t323) * mrSges(10,2) + t180 * mrSges(10,3)) - g(3) * (m(14) * t427 + t437 * mrSges(14,1) + t441 * mrSges(14,2) - t132 * mrSges(14,3)) - g(3) * (m(13) * t427 + t431 * mrSges(13,1) + t435 * mrSges(13,2) - t132 * mrSges(13,3)) - g(1) * (m(9) * t341 + t409 * mrSges(9,1) + t416 * mrSges(9,2) + t180 * mrSges(9,3)) - g(3) * (m(9) * t374 + t297 * mrSges(9,1) + t318 * mrSges(9,2) - t132 * mrSges(9,3)) - g(1) * (m(3) * (t182 + r_base(1)) - t153 * mrSges(3,1) - t180 * mrSges(3,2) + t155 * mrSges(3,3)) - g(3) * (m(3) * t471 - t1 * mrSges(3,1) + t132 * mrSges(3,2)) - g(2) * (m(9) * t477 + t481 * mrSges(9,1) + t485 * mrSges(9,2) - t218 * mrSges(9,3)) - g(2) * (m(12) * (t494 * pkin(19) - t219 + t220 + t496 + r_base(2)) + (t494 * t270 + t504 * t279) * mrSges(12,1) + (t504 * t270 - t494 * t279) * mrSges(12,2) - t218 * mrSges(12,3)) - g(2) * (m(11) * (t496 - t219 + t220 + r_base(2)) + t494 * mrSges(11,1) + t504 * mrSges(11,2) - t218 * mrSges(11,3));
t531 = t178 + t179 + t181 + t182 + r_base(1);
t540 = t163 * t19 - t167 * t2;
t544 = t163 * t2 + t167 * t19;
t546 = t540 * t352 - t544 * t357;
t549 = pkin(3) + qJ(3) - qJ(4);
t550 = cos(t549);
t552 = -t550 * pkin(12) - t125 * pkin(14) - t120 * pkin(15) - pkin(10);
t553 = t552 ^ 2;
t556 = sin(t549);
t558 = -t556 * pkin(12) + t120 * pkin(14) - t125 * pkin(15);
t559 = t558 ^ 2;
t561 = sqrt(t553 + t559);
t562 = 0.1e1 / t561;
t563 = t552 * t562;
t565 = -t558 * t562;
t567 = -t563 * t550 + t565 * t556;
t571 = t544 * t352 + t540 * t357;
t574 = t565 * t550 + t563 * t556;
t576 = t546 * t567 + t571 * t574;
t578 = t546 * pkin(12);
t583 = t546 * t574 - t571 * t567;
t651 = t359 * t567 + t363 * t574;
t653 = t359 * pkin(12);
t658 = t359 * t574 - t363 * t567;
t671 = -g(1) * (m(7) * (t177 + t178 + t179 + t181 + t182 + r_base(1)) + t187 * mrSges(7,1) + t175 * mrSges(7,2) + t180 * mrSges(7,3)) - g(1) * (m(6) * t531 + t169 * mrSges(6,1) + t173 * mrSges(6,2) + t180 * mrSges(6,3)) - g(1) * (m(16) * (-t576 * t561 + t178 + t179 + t181 + t182 + t578 + r_base(1)) + t583 * mrSges(16,1) + t180 * mrSges(16,2) - t576 * mrSges(16,3)) - g(1) * (m(15) * (t578 + t178 + t179 + t181 + t182 + r_base(1)) + t583 * mrSges(15,1) + t576 * mrSges(15,2) + t180 * mrSges(15,3)) - g(1) * (m(14) * t531 + t546 * mrSges(14,1) + t571 * mrSges(14,2) + t180 * mrSges(14,3)) - g(1) * (m(13) * t531 + t540 * mrSges(13,1) + t544 * mrSges(13,2) + t180 * mrSges(13,3)) - g(1) * (m(12) * (t396 * pkin(19) + t181 + t182 + t389 + r_base(1)) + (t396 * t270 + t402 * t279) * mrSges(12,1) + (t402 * t270 - t396 * t279) * mrSges(12,2) + t180 * mrSges(12,3)) - g(2) * (m(10) * (-t481 * pkin(22) - t219 + t220 + r_base(2)) + (t481 * t316 + t485 * t323) * mrSges(10,1) + (t485 * t316 - t481 * t323) * mrSges(10,2) - t218 * mrSges(10,3)) - g(2) * (m(5) * (t217 - t219 + t220 + r_base(2)) + t208 * mrSges(5,1) + t212 * mrSges(5,2) - t218 * mrSges(5,3)) - g(2) * (m(4) * t477 + t202 * mrSges(4,1) + t206 * mrSges(4,2) - t218 * mrSges(4,3)) - g(2) * (m(16) * (-t651 * t561 + t216 + t217 - t219 + t220 + t653 + r_base(2)) + t658 * mrSges(16,1) - t218 * mrSges(16,2) - t651 * mrSges(16,3)) - g(2) * (m(15) * (t653 + t216 + t217 - t219 + t220 + r_base(2)) + t658 * mrSges(15,1) + t651 * mrSges(15,2) - t218 * mrSges(15,3));
t687 = t437 * t567 + t441 * t574;
t689 = t437 * pkin(12);
t694 = t437 * t574 - t441 * t567;
t750 = -g(3) * (m(7) * (t129 + t130 - t131 - t133 + pkin(8) + r_base(3)) + t138 * mrSges(7,1) + t127 * mrSges(7,2) - t132 * mrSges(7,3)) - g(3) * (m(6) * t427 + t118 * mrSges(6,1) + t124 * mrSges(6,2) - t132 * mrSges(6,3)) - g(3) * (m(16) * (-t687 * t561 + pkin(8) + t130 - t131 - t133 + t689 + r_base(3)) + t694 * mrSges(16,1) - t132 * mrSges(16,2) - t687 * mrSges(16,3)) - g(3) * (m(15) * (t689 + t130 - t131 - t133 + pkin(8) + r_base(3)) + t694 * mrSges(15,1) + t687 * mrSges(15,2) - t132 * mrSges(15,3)) - g(2) * (m(8) * (-t232 * pkin(13) + t215 + t216 + t217 - t219 + t220 + r_base(2)) + (t228 * t139 - t218 * t141) * mrSges(8,1) + (-t218 * t139 - t228 * t141) * mrSges(8,2) - t232 * mrSges(8,3)) - g(2) * (m(3) * (t220 + r_base(2)) + t199 * mrSges(3,1) + t218 * mrSges(3,2) + t152 * mrSges(3,3)) - g(2) * (m(2) * r_base(2) - t155 * mrSges(2,1) + t152 * mrSges(2,2)) - g(1) * (m(2) * r_base(1) + t152 * mrSges(2,1) + t155 * mrSges(2,2)) - g(3) * (m(2) * t471 + mrSges(2,3)) - g(3) * (m(1) * r_base(3) + mrSges(1,3)) - g(1) * (m(1) * r_base(1) + mrSges(1,1)) - g(2) * (m(1) * r_base(2) + mrSges(1,2));
t752 = t381 + t522 + t671 + t750;
U = t752;
