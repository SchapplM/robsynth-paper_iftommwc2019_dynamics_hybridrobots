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
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% 
% Output:
% U [1x1]
%   Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-03 15:49
% Revision: caa0dbda1e8a16d11faaa29ba3bbef6afcd619f7 (2020-05-25)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7DE2_energypot_fixb_slag_vp2(qJ, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE2_energypot_fixb_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_worldframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-25 12:20:23
% EndTime: 2020-05-25 12:21:34
% DurationCPUTime: 49.10s
% Computational Cost: add. (3160193->414), mult. (4078245->516), div. (65930->4), fcn. (1635230->49), ass. (0->177)
t1 = cos(qJ(1));
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
t98 = t97 + pkin(6) + qJ(3);
t99 = sin(t98);
t100 = pkin(9) * t99;
t101 = t97 + pkin(6);
t102 = sin(t101);
t103 = t102 * pkin(23);
t104 = t100 + t103;
t106 = cos(qJ(2));
t107 = t1 * t106;
t108 = t107 * pkin(21);
t109 = sin(qJ(1));
t110 = cos(t98);
t111 = pkin(9) * t110;
t112 = cos(t101);
t113 = t112 * pkin(23);
t114 = -t111 - t113;
t116 = t109 * pkin(16);
t117 = t3 * t104 + t109 * t114 - t108 + t116;
t119 = t97 + pkin(6) + qJ(3) + qJ(4);
t120 = sin(t119);
t122 = cos(t119);
t134 = 0.2e1 * qJ(3);
t135 = t97 + pkin(6) + t134 + pkin(3);
t136 = sin(t135);
t138 = cos(t135);
t150 = t97 + pkin(6) + t134;
t151 = cos(t150);
t153 = sin(t150);
t164 = atan2(-t53, t61);
t165 = t97 + pkin(6) - t164;
t166 = sin(t165);
t168 = t102 * pkin(24);
t169 = pkin(19) * t166 + t168;
t171 = cos(t165);
t174 = -pkin(19) * t171 - t112 * pkin(24);
t192 = atan2((t4 * t61 * t54 - t5 * t53 * t54) * pkin(19) + t27 - t28, (t4 * t53 * t54 + t5 * t61 * t54) * pkin(19) - pkin(9) + t7 + t10);
t193 = t97 + pkin(6) - t192 + qJ(3);
t194 = sin(t193);
t196 = cos(t193);
t208 = t109 * t112;
t238 = -t108 + t116;
t250 = sin(pkin(6));
t251 = t250 * pkin(22);
t253 = cos(pkin(6));
t254 = t109 * t253;
t258 = 0.1e1 / pkin(22);
t259 = t91 * t258;
t261 = -t96 * t258;
t263 = t259 * t250 + t261 * t253;
t267 = -t261 * t250 + t259 * t253;
t277 = atan2(-(t267 * t250 - t263 * t253) * pkin(22) + pkin(24) + t68 - t69, (t263 * t250 + t267 * t253) * pkin(22) + t58 + t65);
t278 = pkin(6) + t97 - t277;
t279 = cos(t278);
t281 = sin(t278);
t315 = -g(2) * (m(6) * t117 + (-t109 * t122 + t3 * t120) * mrSges(6,1) + (t109 * t120 + t3 * t122) * mrSges(6,2) - t107 * mrSges(6,3)) - g(2) * (m(14) * t117 + (t109 * t138 - t3 * t136) * mrSges(14,1) + (-t109 * t136 - t3 * t138) * mrSges(14,2) - t107 * mrSges(14,3)) - g(2) * (m(13) * t117 + (-t109 * t153 - t3 * t151) * mrSges(13,1) + (-t109 * t151 + t3 * t153) * mrSges(13,2) - t107 * mrSges(13,3)) - g(2) * (m(12) * (t109 * t174 + t3 * t169 - t108 + t116) + (t109 * t196 - t3 * t194) * mrSges(12,1) + (-t109 * t194 - t3 * t196) * mrSges(12,2) - t107 * mrSges(12,3)) - g(2) * (m(11) * (-t208 * pkin(24) + t3 * t168 - t108 + t116) + (-t109 * t171 + t3 * t166) * mrSges(11,1) + (t109 * t166 + t3 * t171) * mrSges(11,2) - t107 * mrSges(11,3)) - g(2) * (m(5) * (-t208 * pkin(23) + t3 * t103 - t108 + t116) + (-t109 * t110 + t3 * t99) * mrSges(5,1) + (t109 * t99 + t3 * t110) * mrSges(5,2) - t107 * mrSges(5,3)) - g(2) * (m(4) * t238 + (t3 * t102 - t208) * mrSges(4,1) + (t109 * t102 + t3 * t112) * mrSges(4,2) - t107 * mrSges(4,3)) - g(2) * (m(10) * (-t254 * pkin(22) + t3 * t251 - t108 + t116) + (t109 * t281 + t3 * t279) * mrSges(10,1) + (t109 * t279 - t3 * t281) * mrSges(10,2) - t107 * mrSges(10,3)) - g(2) * (m(9) * t238 + (-t3 * t250 + t254) * mrSges(9,1) + (-t109 * t250 - t3 * t253) * mrSges(9,2) - t107 * mrSges(9,3)) - g(2) * (m(3) * t109 * pkin(16) + t3 * mrSges(3,1) + t107 * mrSges(3,2) + t109 * mrSges(3,3)) - g(2) * (-t1 * mrSges(2,1) + t109 * mrSges(2,2)) - g(2) * mrSges(1,2);
t316 = qJ(4) - qJ(3) + pkin(4);
t317 = sin(t316);
t319 = pkin(3) + qJ(3) - qJ(4);
t320 = sin(t319);
t322 = cos(t316);
t324 = t320 * pkin(12) - t322 * pkin(14) + t317 * pkin(15);
t327 = cos(t319);
t329 = -t327 * pkin(12) - t317 * pkin(14) - t322 * pkin(15) - pkin(10);
t330 = atan2(t324, t329);
t331 = t97 + pkin(6) + qJ(3) - t330 + qJ(4);
t332 = sin(t331);
t333 = t106 * t332;
t334 = t329 ^ 2;
t335 = t324 ^ 2;
t337 = sqrt(t334 + t335);
t340 = -pkin(12) * t136 + t100 + t103;
t341 = t106 * t340;
t342 = t2 * pkin(21);
t345 = cos(t331);
t346 = t106 * t345;
t359 = t109 * t2;
t361 = t109 * t106;
t362 = t361 * pkin(21);
t364 = t1 * pkin(16);
t379 = t1 * t112;
t394 = t362 + t364;
t422 = t1 * t253;
t461 = t97 + pkin(6) + 0.2e1 * qJ(4) + pkin(4);
t462 = sin(t461);
t464 = cos(t461);
t466 = t109 * t464 - t3 * t462;
t469 = pkin(10) * t120 + t100 + t103;
t470 = t3 * t469;
t472 = -pkin(10) * t122 - t111 - t113;
t473 = t109 * t472;
t478 = t109 * t462 + t3 * t464;
t479 = cos(qJ(5));
t481 = sin(qJ(5));
t492 = -g(3) * (m(16) * (t333 * t337 + pkin(8) - t341 - t342) + t346 * mrSges(16,1) - t2 * mrSges(16,2) + t333 * mrSges(16,3)) - g(3) * (m(15) * (-t341 - t342 + pkin(8)) + t346 * mrSges(15,1) - t333 * mrSges(15,2) - t2 * mrSges(15,3)) - g(1) * (m(12) * (t1 * t174 - t359 * t169 + t362 + t364) + (t1 * t196 + t359 * t194) * mrSges(12,1) + (-t1 * t194 + t359 * t196) * mrSges(12,2) + t361 * mrSges(12,3)) - g(1) * (m(5) * (-t379 * pkin(23) - t359 * t103 + t362 + t364) + (-t1 * t110 - t359 * t99) * mrSges(5,1) + (t1 * t99 - t359 * t110) * mrSges(5,2) + t361 * mrSges(5,3)) - g(1) * (m(4) * t394 + (-t359 * t102 - t379) * mrSges(4,1) + (t1 * t102 - t359 * t112) * mrSges(4,2) + t361 * mrSges(4,3)) - g(1) * (m(11) * (-t379 * pkin(24) - t359 * t168 + t362 + t364) + (-t1 * t171 - t359 * t166) * mrSges(11,1) + (t1 * t166 - t359 * t171) * mrSges(11,2) + t361 * mrSges(11,3)) - g(1) * (m(10) * (-t422 * pkin(22) - t359 * t251 + t362 + t364) + (t1 * t281 - t359 * t279) * mrSges(10,1) + (t1 * t279 + t359 * t281) * mrSges(10,2) + t361 * mrSges(10,3)) - g(1) * (m(9) * t394 + (t359 * t250 + t422) * mrSges(9,1) + (-t1 * t250 + t359 * t253) * mrSges(9,2) + t361 * mrSges(9,3)) - g(1) * (m(3) * t1 * pkin(16) - t359 * mrSges(3,1) - t361 * mrSges(3,2) + t1 * mrSges(3,3)) - g(1) * (t109 * mrSges(2,1) + t1 * mrSges(2,2)) - g(1) * mrSges(1,1) - g(2) * (m(8) * (-t466 * pkin(13) - t108 + t116 + t470 + t473) + (-t107 * t481 + t478 * t479) * mrSges(8,1) + (-t107 * t479 - t478 * t481) * mrSges(8,2) - t466 * mrSges(8,3));
t496 = -t109 * t345 + t3 * t332;
t498 = t3 * t340;
t500 = pkin(12) * t138 - t111 - t113;
t501 = t109 * t500;
t506 = -t109 * t332 - t3 * t345;
t536 = -t342 + pkin(8);
t538 = t106 * t250;
t556 = t1 * t114 - t359 * t104 + t362 + t364;
t571 = t1 * t464 + t359 * t462;
t573 = t359 * t469;
t574 = t1 * t472;
t579 = t1 * t462 - t359 * t464;
t593 = -t1 * t345 - t359 * t332;
t595 = t359 * t340;
t596 = t1 * t500;
t601 = -t1 * t332 + t359 * t345;
t614 = -g(2) * (m(16) * (-t496 * t337 - t108 + t116 + t498 + t501) + t506 * mrSges(16,1) - t107 * mrSges(16,2) - t496 * mrSges(16,3)) - g(2) * (m(15) * (t498 - t108 + t501 + t116) + t506 * mrSges(15,1) + t496 * mrSges(15,2) - t107 * mrSges(15,3)) - g(2) * (m(7) * (t470 - t108 + t473 + t116) + t478 * mrSges(7,1) + t466 * mrSges(7,2) - t107 * mrSges(7,3)) - g(3) * (m(12) * (-t106 * t169 + pkin(8) - t342) + t106 * t194 * mrSges(12,1) + t106 * t196 * mrSges(12,2) - t2 * mrSges(12,3)) - g(3) * (t106 * t253 * mrSges(9,2) + m(9) * t536 + t538 * mrSges(9,1) - t2 * mrSges(9,3)) - g(3) * (m(3) * pkin(8) - t106 * mrSges(3,1) + t2 * mrSges(3,2)) - g(3) * (m(2) * pkin(8) + mrSges(2,3)) - g(3) * mrSges(1,3) - g(1) * (m(6) * t556 + (-t1 * t122 - t359 * t120) * mrSges(6,1) + (t1 * t120 - t359 * t122) * mrSges(6,2) + t361 * mrSges(6,3)) - g(1) * (m(8) * (-t571 * pkin(13) + t362 + t364 - t573 + t574) + (t361 * t481 + t579 * t479) * mrSges(8,1) + (t361 * t479 - t579 * t481) * mrSges(8,2) - t571 * mrSges(8,3)) - g(1) * (m(16) * (-t593 * t337 + t362 + t364 - t595 + t596) + t601 * mrSges(16,1) + t361 * mrSges(16,2) - t593 * mrSges(16,3)) - g(1) * (m(15) * (-t595 + t362 + t596 + t364) + t601 * mrSges(15,1) + t593 * mrSges(15,2) + t361 * mrSges(15,3));
t646 = t106 * t462;
t648 = t106 * t469;
t651 = t106 * t464;
t671 = -t106 * t104 + pkin(8) - t342;
t680 = t106 * t102;
t734 = -g(1) * (m(7) * (-t573 + t362 + t574 + t364) + t579 * mrSges(7,1) + t571 * mrSges(7,2) + t361 * mrSges(7,3)) - g(1) * (m(14) * t556 + (t1 * t138 + t359 * t136) * mrSges(14,1) + (-t1 * t136 + t359 * t138) * mrSges(14,2) + t361 * mrSges(14,3)) - g(1) * (m(13) * t556 + (-t1 * t153 + t359 * t151) * mrSges(13,1) + (-t1 * t151 - t359 * t153) * mrSges(13,2) + t361 * mrSges(13,3)) - g(3) * (m(8) * (-t646 * pkin(13) + pkin(8) - t342 - t648) + (-t2 * t481 - t651 * t479) * mrSges(8,1) + (-t2 * t479 + t651 * t481) * mrSges(8,2) - t646 * mrSges(8,3)) - g(3) * (m(7) * (-t648 - t342 + pkin(8)) - t651 * mrSges(7,1) + t646 * mrSges(7,2) - t2 * mrSges(7,3)) - g(3) * (-t106 * t120 * mrSges(6,1) - t106 * t122 * mrSges(6,2) + m(6) * t671 - t2 * mrSges(6,3)) - g(3) * (m(11) * (-t680 * pkin(24) + pkin(8) - t342) - t106 * t166 * mrSges(11,1) - t106 * t171 * mrSges(11,2) - t2 * mrSges(11,3)) - g(3) * (m(10) * (-t538 * pkin(22) + pkin(8) - t342) - t106 * t279 * mrSges(10,1) + t106 * t281 * mrSges(10,2) - t2 * mrSges(10,3)) - g(3) * (m(5) * (-t680 * pkin(23) + pkin(8) - t342) - t106 * t99 * mrSges(5,1) - t106 * t110 * mrSges(5,2) - t2 * mrSges(5,3)) - g(3) * (-t106 * t112 * mrSges(4,2) + m(4) * t536 - t680 * mrSges(4,1) - t2 * mrSges(4,3)) - g(3) * (t106 * t136 * mrSges(14,1) + t106 * t138 * mrSges(14,2) + m(14) * t671 - t2 * mrSges(14,3)) - g(3) * (t106 * t151 * mrSges(13,1) - t106 * t153 * mrSges(13,2) + m(13) * t671 - t2 * mrSges(13,3));
t736 = t315 + t492 + t614 + t734;
U = t736;
