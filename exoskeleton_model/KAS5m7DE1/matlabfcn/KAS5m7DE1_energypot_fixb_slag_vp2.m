% Calculate potential energy for
% KAS5m7DE1
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
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7DE1_energypot_fixb_slag_vp2(qJ, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_worldframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-14 20:17:23
% EndTime: 2020-05-14 20:20:38
% DurationCPUTime: 193.80s
% Computational Cost: add. (13265256->414), mult. (17122102->505), div. (276600->4), fcn. (6865757->43), ass. (0->191)
t1 = sin(qJ(1));
t2 = sin(qJ(2));
t3 = t1 * t2;
t4 = sin(qJ(3));
t5 = cos(qJ(3));
t6 = -pkin(23) + pkin(24);
t7 = t5 * t6;
t8 = pkin(3) + qJ(3);
t9 = cos(t8);
t10 = t9 * pkin(12);
t11 = -pkin(9) + t7 + t10;
t12 = pkin(11) ^ 2;
t13 = pkin(19) ^ 2;
t14 = t9 ^ 2;
t15 = pkin(12) ^ 2;
t17 = sin(t8);
t18 = t17 ^ 2;
t21 = (-pkin(9) + t7) ^ 2;
t22 = t4 ^ 2;
t23 = t6 ^ 2;
t27 = t4 * t6;
t28 = t17 * pkin(12);
t29 = -t27 + t28;
t32 = 0.2e1 * t11 * t9 * pkin(12) + 0.2e1 * t29 * t17 * pkin(12) - t14 * t15 - t18 * t15 + t22 * t23 + t12 - t13 + t21;
t36 = 0.4e1 * t11 ^ 2 + 0.4e1 * t29 ^ 2;
t38 = t32 ^ 2;
t40 = sqrt(t12 * t36 - t38);
t43 = 0.1e1 / t36;
t45 = pkin(9) - t7 - t10 + (0.2e1 * t11 * t32 - 0.2e1 * t29 * t40) * t43;
t51 = t27 - t28 + (0.2e1 * t11 * t40 + 0.2e1 * t32 * t29) * t43;
t53 = t4 * t45 + t5 * t51;
t54 = 0.1e1 / pkin(19);
t55 = t53 * t54;
t56 = cos(pkin(7));
t57 = t56 * pkin(18);
t58 = t55 * t57;
t61 = -t4 * t51 + t5 * t45;
t62 = t61 * t54;
t63 = sin(pkin(7));
t64 = t63 * pkin(18);
t65 = t62 * t64;
t66 = pkin(17) ^ 2;
t67 = pkin(22) ^ 2;
t68 = t62 * t57;
t69 = t55 * t64;
t70 = t68 - t69;
t71 = t70 ^ 2;
t72 = t58 + t65;
t73 = t72 ^ 2;
t74 = pkin(24) ^ 2;
t75 = -pkin(24) - t68 + t69;
t78 = -0.2e1 * t75 * t70 + 0.2e1 * t72 ^ 2 + t66 - t67 - t71 - t73 + t74;
t82 = 0.4e1 * t72 ^ 2 + 0.4e1 * t75 ^ 2;
t84 = t78 ^ 2;
t86 = sqrt(t66 * t82 - t84);
t89 = 0.1e1 / t82;
t91 = -t58 - t65 - (-0.2e1 * t78 * t72 + 0.2e1 * t75 * t86) * t89;
t96 = t68 - t69 + (0.2e1 * t72 * t86 + 0.2e1 * t75 * t78) * t89 + pkin(24);
t97 = atan2(t91, t96);
t98 = t97 + pkin(6);
t99 = sin(t98);
t101 = cos(qJ(1));
t102 = cos(t98);
t104 = -t101 * t102 - t3 * t99;
t108 = t101 * t99 - t3 * t102;
t110 = t104 * t5 + t108 * t4;
t111 = cos(qJ(4));
t115 = -t104 * t4 + t108 * t5;
t116 = sin(qJ(4));
t118 = t110 * t111 + t115 * t116;
t119 = qJ(4) - qJ(3) + pkin(4);
t120 = cos(t119);
t124 = -t110 * t116 + t115 * t111;
t125 = sin(t119);
t127 = -t118 * t120 - t124 * t125;
t129 = t118 * pkin(10);
t130 = t110 * pkin(9);
t131 = t104 * pkin(23);
t132 = cos(qJ(2));
t133 = t1 * t132;
t134 = t133 * pkin(21);
t135 = t101 * pkin(16);
t140 = -t118 * t125 + t124 * t120;
t141 = cos(qJ(5));
t143 = sin(qJ(5));
t156 = t110 * t4 - t115 * t5;
t157 = sin(pkin(3));
t161 = t110 * t5 + t115 * t4;
t162 = cos(pkin(3));
t164 = t156 * t157 - t161 * t162;
t166 = pkin(3) + qJ(3) - qJ(4);
t167 = sin(t166);
t170 = t167 * pkin(12) - t120 * pkin(14) + t125 * pkin(15);
t173 = cos(t166);
t175 = -t173 * pkin(12) - t125 * pkin(14) - t120 * pkin(15) - pkin(10);
t176 = atan2(t170, t175);
t177 = t176 + pkin(3) + qJ(3) - qJ(4);
t178 = cos(t177);
t182 = t156 * t162 + t161 * t157;
t183 = sin(t177);
t185 = -t164 * t178 + t182 * t183;
t186 = t175 ^ 2;
t187 = t170 ^ 2;
t189 = sqrt(t186 + t187);
t191 = t164 * pkin(12);
t196 = t164 * t183 + t182 * t178;
t216 = t130 + t131 + t134 + t135;
t223 = atan2(-t53, t61);
t224 = cos(t223);
t226 = sin(t223);
t228 = t104 * t224 - t108 * t226;
t230 = t104 * pkin(24);
t247 = atan2((t4 * t61 * t54 - t5 * t53 * t54) * pkin(19) + t27 - t28, (t4 * t53 * t54 + t5 * t61 * t54) * pkin(19) - pkin(9) + t7 + t10);
t248 = -t247 + qJ(3) + t223;
t249 = cos(t248);
t253 = t104 * t226 + t108 * t224;
t254 = sin(t248);
t272 = sin(pkin(6));
t274 = cos(pkin(6));
t276 = t101 * t274 + t3 * t272;
t280 = 0.1e1 / pkin(22);
t281 = t91 * t280;
t283 = -t96 * t280;
t285 = t281 * t272 + t283 * t274;
t289 = -t283 * t272 + t281 * t274;
t299 = atan2(-(t289 * t272 - t285 * t274) * pkin(22) + pkin(24) + t68 - t69, (t285 * t272 + t289 * t274) * pkin(22) + t58 + t65);
t300 = t97 - t299;
t301 = sin(t300);
t305 = -t101 * t272 + t3 * t274;
t306 = cos(t300);
t323 = t132 * t99;
t325 = t132 * t102;
t327 = -t323 * t5 - t325 * t4;
t328 = t327 * pkin(9);
t329 = t323 * pkin(23);
t330 = t2 * pkin(21);
t331 = t328 - t329 - t330 + pkin(8);
t336 = t323 * t4 - t325 * t5;
t338 = t327 * t111 + t336 * t116;
t342 = t336 * t111 - t327 * t116;
t349 = -t338 * t120 - t342 * t125;
t351 = t338 * pkin(10);
t356 = t342 * t120 - t338 * t125;
t370 = -t323 * t224 + t325 * t226;
t372 = t323 * pkin(24);
t378 = -t325 * t224 - t323 * t226;
t389 = -g(1) * (m(8) * (-t127 * pkin(13) + t129 + t130 + t131 + t134 + t135) + (t133 * t143 + t140 * t141) * mrSges(8,1) + (t133 * t141 - t140 * t143) * mrSges(8,2) - t127 * mrSges(8,3)) - g(1) * (m(16) * (-t185 * t189 + t130 + t131 + t134 + t135 + t191) + t196 * mrSges(16,1) + t133 * mrSges(16,2) - t185 * mrSges(16,3)) - g(1) * (m(15) * (t191 + t130 + t131 + t134 + t135) + t196 * mrSges(15,1) + t185 * mrSges(15,2) + t133 * mrSges(15,3)) - g(1) * (m(7) * (t129 + t130 + t131 + t134 + t135) + t140 * mrSges(7,1) + t127 * mrSges(7,2) + t133 * mrSges(7,3)) - g(1) * (m(6) * t216 + t118 * mrSges(6,1) + t124 * mrSges(6,2) + t133 * mrSges(6,3)) - g(1) * (m(12) * (t228 * pkin(19) + t134 + t135 + t230) + (-t228 * t249 - t253 * t254) * mrSges(12,1) + (t228 * t254 - t253 * t249) * mrSges(12,2) + t133 * mrSges(12,3)) - g(1) * (m(11) * (t230 + t134 + t135) + t228 * mrSges(11,1) + t253 * mrSges(11,2) + t133 * mrSges(11,3)) - g(1) * (m(10) * (-t276 * pkin(22) + t134 + t135) + (t276 * t301 - t305 * t306) * mrSges(10,1) + (t276 * t306 + t305 * t301) * mrSges(10,2) + t133 * mrSges(10,3)) - g(1) * (m(14) * t216 + t164 * mrSges(14,1) + t182 * mrSges(14,2) + t133 * mrSges(14,3)) - g(3) * (m(6) * t331 + t338 * mrSges(6,1) + t342 * mrSges(6,2) - t2 * mrSges(6,3)) - g(3) * (m(8) * (-t349 * pkin(13) + pkin(8) + t328 - t329 - t330 + t351) + (t356 * t141 - t2 * t143) * mrSges(8,1) + (-t2 * t141 - t356 * t143) * mrSges(8,2) - t349 * mrSges(8,3)) - g(3) * (m(12) * (t370 * pkin(19) + pkin(8) - t330 - t372) + (-t370 * t249 - t378 * t254) * mrSges(12,1) + (-t378 * t249 + t370 * t254) * mrSges(12,2) - t2 * mrSges(12,3));
t397 = t132 * t272;
t402 = t132 * t274;
t416 = t327 * t4 - t336 * t5;
t420 = t327 * t5 + t336 * t4;
t422 = t416 * t157 - t420 * t162;
t426 = t420 * t157 + t416 * t162;
t444 = -t330 + pkin(8);
t466 = t101 * t2;
t469 = t1 * t274 - t466 * t272;
t471 = t101 * t132;
t472 = t471 * pkin(21);
t473 = t1 * pkin(16);
t479 = -t1 * t272 - t466 * t274;
t490 = -t472 + t473;
t497 = -g(3) * (m(11) * (-t372 - t330 + pkin(8)) + t370 * mrSges(11,1) + t378 * mrSges(11,2) - t2 * mrSges(11,3)) - g(3) * (m(10) * (-t397 * pkin(22) + pkin(8) - t330) + (t397 * t301 - t402 * t306) * mrSges(10,1) + (t402 * t301 + t397 * t306) * mrSges(10,2) - t2 * mrSges(10,3)) - g(3) * (m(14) * t331 + t422 * mrSges(14,1) + t426 * mrSges(14,2) - t2 * mrSges(14,3)) - g(3) * (m(13) * t331 + t416 * mrSges(13,1) + t420 * mrSges(13,2) - t2 * mrSges(13,3)) - g(3) * (m(5) * (-t329 - t330 + pkin(8)) + t327 * mrSges(5,1) + t336 * mrSges(5,2) - t2 * mrSges(5,3)) - g(3) * (m(4) * t444 - t323 * mrSges(4,1) - t325 * mrSges(4,2) - t2 * mrSges(4,3)) - g(3) * (m(3) * pkin(8) - t132 * mrSges(3,1) + t2 * mrSges(3,2)) - g(3) * (m(9) * t444 + t397 * mrSges(9,1) + t402 * mrSges(9,2) - t2 * mrSges(9,3)) - g(3) * (m(2) * pkin(8) + mrSges(2,3)) - g(3) * mrSges(1,3) - g(2) * (m(10) * (-t469 * pkin(22) - t472 + t473) + (t469 * t301 - t479 * t306) * mrSges(10,1) + (t479 * t301 + t469 * t306) * mrSges(10,2) - t471 * mrSges(10,3)) - g(2) * (m(9) * t490 + t469 * mrSges(9,1) + t479 * mrSges(9,2) - t471 * mrSges(9,3));
t513 = -t422 * t178 + t426 * t183;
t515 = t422 * pkin(12);
t520 = t426 * t178 + t422 * t183;
t553 = t134 + t135;
t577 = -g(2) * (m(3) * t1 * pkin(16) + t466 * mrSges(3,1) + t471 * mrSges(3,2) + t1 * mrSges(3,3)) - g(2) * mrSges(1,2) - g(2) * (-t101 * mrSges(2,1) + t1 * mrSges(2,2)) - g(3) * (m(16) * (-t513 * t189 + pkin(8) + t328 - t329 - t330 + t515) + t520 * mrSges(16,1) - t2 * mrSges(16,2) - t513 * mrSges(16,3)) - g(3) * (m(15) * (t515 + t328 - t329 - t330 + pkin(8)) + t520 * mrSges(15,1) + t513 * mrSges(15,2) - t2 * mrSges(15,3)) - g(3) * (m(7) * (t351 + t328 - t329 - t330 + pkin(8)) + t356 * mrSges(7,1) + t349 * mrSges(7,2) - t2 * mrSges(7,3)) - g(1) * (m(13) * t216 + t156 * mrSges(13,1) + t161 * mrSges(13,2) + t133 * mrSges(13,3)) - g(1) * (m(5) * (t131 + t134 + t135) + t110 * mrSges(5,1) + t115 * mrSges(5,2) + t133 * mrSges(5,3)) - g(1) * (m(4) * t553 + t104 * mrSges(4,1) + t108 * mrSges(4,2) + t133 * mrSges(4,3)) - g(1) * (m(3) * t101 * pkin(16) - t3 * mrSges(3,1) - t133 * mrSges(3,2) + t101 * mrSges(3,3)) - g(1) * (m(9) * t553 + t276 * mrSges(9,1) + t305 * mrSges(9,2) + t133 * mrSges(9,3)) - g(1) * (t1 * mrSges(2,1) + t101 * mrSges(2,2));
t581 = -t1 * t102 + t466 * t99;
t585 = t1 * t99 + t466 * t102;
t587 = t585 * t4 + t581 * t5;
t591 = -t581 * t4 + t585 * t5;
t593 = t587 * t111 + t591 * t116;
t597 = t591 * t111 - t587 * t116;
t599 = -t593 * t120 - t597 * t125;
t601 = t593 * pkin(10);
t602 = t587 * pkin(9);
t603 = t581 * pkin(23);
t608 = t597 * t120 - t593 * t125;
t620 = t581 * pkin(24);
t625 = t581 * t224 - t585 * t226;
t629 = t585 * t224 + t581 * t226;
t634 = t602 + t603 - t472 + t473;
t643 = t587 * t4 - t591 * t5;
t647 = t591 * t4 + t587 * t5;
t649 = t643 * t157 - t647 * t162;
t653 = t647 * t157 + t643 * t162;
t655 = -t649 * t178 + t653 * t183;
t657 = t649 * pkin(12);
t662 = t653 * t178 + t649 * t183;
t721 = -g(1) * mrSges(1,1) - g(2) * (m(8) * (-t599 * pkin(13) - t472 + t473 + t601 + t602 + t603) + (t608 * t141 - t471 * t143) * mrSges(8,1) + (-t471 * t141 - t608 * t143) * mrSges(8,2) - t599 * mrSges(8,3)) - g(2) * (m(11) * (t620 - t472 + t473) + t625 * mrSges(11,1) + t629 * mrSges(11,2) - t471 * mrSges(11,3)) - g(2) * (m(6) * t634 + t593 * mrSges(6,1) + t597 * mrSges(6,2) - t471 * mrSges(6,3)) - g(2) * (m(16) * (-t655 * t189 - t472 + t473 + t602 + t603 + t657) + t662 * mrSges(16,1) - t471 * mrSges(16,2) - t655 * mrSges(16,3)) - g(2) * (m(15) * (t657 + t602 + t603 - t472 + t473) + t662 * mrSges(15,1) + t655 * mrSges(15,2) - t471 * mrSges(15,3)) - g(2) * (m(7) * (t601 + t602 + t603 - t472 + t473) + t608 * mrSges(7,1) + t599 * mrSges(7,2) - t471 * mrSges(7,3)) - g(2) * (m(5) * (t603 - t472 + t473) + t587 * mrSges(5,1) + t591 * mrSges(5,2) - t471 * mrSges(5,3)) - g(2) * (m(4) * t490 + t581 * mrSges(4,1) + t585 * mrSges(4,2) - t471 * mrSges(4,3)) - g(2) * (m(14) * t634 + t649 * mrSges(14,1) + t653 * mrSges(14,2) - t471 * mrSges(14,3)) - g(2) * (m(13) * t634 + t643 * mrSges(13,1) + t647 * mrSges(13,2) - t471 * mrSges(13,3)) - g(2) * (m(12) * (t625 * pkin(19) - t472 + t473 + t620) + (-t625 * t249 - t629 * t254) * mrSges(12,1) + (-t629 * t249 + t625 * t254) * mrSges(12,2) - t471 * mrSges(12,3));
t723 = t389 + t497 + t577 + t721;
U = t723;
