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

function U = KAS5m7DE2_energypot_floatb_twist_slag_vp2(qJ, r_base, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp2: r_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp2: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE2_energypot_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_floatb_twist_worldframe_par2_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-25 12:19:00
% EndTime: 2020-05-25 12:20:13
% DurationCPUTime: 47.23s
% Computational Cost: add. (3160241->452), mult. (4078250->519), div. (65930->4), fcn. (1635230->49), ass. (0->178)
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
t109 = cos(qJ(1));
t110 = cos(t98);
t111 = pkin(9) * t110;
t112 = cos(t101);
t113 = t112 * pkin(23);
t114 = -t111 - t113;
t116 = t109 * pkin(16);
t117 = -t3 * t104 + t109 * t114 + t108 + t116 + r_base(1);
t119 = t97 + pkin(6) + qJ(3) + qJ(4);
t120 = sin(t119);
t122 = cos(t119);
t134 = pkin(10) * t120 + t100 + t103;
t135 = t106 * t134;
t136 = t2 * pkin(21);
t140 = t97 + pkin(6) + 0.2e1 * qJ(4) + pkin(4);
t141 = cos(t140);
t142 = t106 * t141;
t144 = sin(t140);
t145 = t106 * t144;
t151 = -t106 * t104 + pkin(8) - t136 + r_base(3);
t160 = t109 * t2;
t161 = t160 * t134;
t162 = t109 * t106;
t163 = t162 * pkin(21);
t165 = -pkin(10) * t122 - t111 - t113;
t166 = t1 * t165;
t167 = t1 * pkin(16);
t172 = t1 * t144 + t160 * t141;
t176 = t1 * t141 - t160 * t144;
t183 = t1 * t114 + t160 * t104 - t163 + t167 + r_base(2);
t196 = t3 * t134;
t197 = t109 * t165;
t202 = t109 * t144 - t3 * t141;
t206 = t109 * t141 + t3 * t144;
t212 = 0.2e1 * qJ(3);
t213 = t97 + pkin(6) + t212 + pkin(3);
t214 = sin(t213);
t216 = cos(t213);
t228 = t97 + pkin(6) + t212;
t229 = cos(t228);
t231 = sin(t228);
t242 = atan2(-t53, t61);
t243 = t97 + pkin(6) - t242;
t244 = sin(t243);
t246 = t102 * pkin(24);
t247 = pkin(19) * t244 + t246;
t249 = cos(t243);
t252 = -pkin(19) * t249 - t112 * pkin(24);
t270 = atan2((t4 * t61 * t54 - t5 * t53 * t54) * pkin(19) + t27 - t28, (t4 * t53 * t54 + t5 * t61 * t54) * pkin(19) - pkin(9) + t7 + t10);
t271 = t97 + pkin(6) - t270 + qJ(3);
t272 = sin(t271);
t274 = cos(t271);
t286 = t1 * t112;
t316 = -t163 + t167 + r_base(2);
t328 = -g(1) * (m(6) * t117 + (-t109 * t122 - t3 * t120) * mrSges(6,1) + (t109 * t120 - t3 * t122) * mrSges(6,2) + t107 * mrSges(6,3)) - g(3) * (m(7) * (-t135 - t136 + pkin(8) + r_base(3)) - t142 * mrSges(7,1) + t145 * mrSges(7,2) - t2 * mrSges(7,3)) - g(3) * (-t106 * t120 * mrSges(6,1) - t106 * t122 * mrSges(6,2) + m(6) * t151 - t2 * mrSges(6,3)) - g(2) * (m(7) * (t161 - t163 + t166 + t167 + r_base(2)) + t172 * mrSges(7,1) + t176 * mrSges(7,2) - t162 * mrSges(7,3)) - g(2) * (m(6) * t183 + (-t1 * t122 + t160 * t120) * mrSges(6,1) + (t1 * t120 + t160 * t122) * mrSges(6,2) - t162 * mrSges(6,3)) - g(1) * (m(7) * (-t196 + t108 + t197 + t116 + r_base(1)) + t202 * mrSges(7,1) + t206 * mrSges(7,2) + t107 * mrSges(7,3)) - g(2) * (m(14) * t183 + (t1 * t216 - t160 * t214) * mrSges(14,1) + (-t1 * t214 - t160 * t216) * mrSges(14,2) - t162 * mrSges(14,3)) - g(2) * (m(13) * t183 + (-t1 * t231 - t160 * t229) * mrSges(13,1) + (-t1 * t229 + t160 * t231) * mrSges(13,2) - t162 * mrSges(13,3)) - g(2) * (m(12) * (t1 * t252 + t160 * t247 - t163 + t167 + r_base(2)) + (t1 * t274 - t160 * t272) * mrSges(12,1) + (-t1 * t272 - t160 * t274) * mrSges(12,2) - t162 * mrSges(12,3)) - g(2) * (m(11) * (-t286 * pkin(24) + t160 * t246 - t163 + t167 + r_base(2)) + (-t1 * t249 + t160 * t244) * mrSges(11,1) + (t1 * t244 + t160 * t249) * mrSges(11,2) - t162 * mrSges(11,3)) - g(2) * (m(5) * (-t286 * pkin(23) + t160 * t103 - t163 + t167 + r_base(2)) + (-t1 * t110 + t160 * t99) * mrSges(5,1) + (t1 * t99 + t160 * t110) * mrSges(5,2) - t162 * mrSges(5,3)) - g(2) * (m(4) * t316 + (t160 * t102 - t286) * mrSges(4,1) + (t1 * t102 + t160 * t112) * mrSges(4,2) - t162 * mrSges(4,3));
t341 = qJ(4) - qJ(3) + pkin(4);
t342 = sin(t341);
t344 = pkin(3) + qJ(3) - qJ(4);
t345 = sin(t344);
t347 = cos(t341);
t349 = t345 * pkin(12) - t347 * pkin(14) + t342 * pkin(15);
t352 = cos(t344);
t354 = -t352 * pkin(12) - t342 * pkin(14) - t347 * pkin(15) - pkin(10);
t355 = atan2(t349, t354);
t356 = t97 + pkin(6) + qJ(3) - t355 + qJ(4);
t357 = sin(t356);
t358 = t106 * t357;
t359 = t354 ^ 2;
t360 = t349 ^ 2;
t362 = sqrt(t359 + t360);
t365 = -pkin(12) * t214 + t100 + t103;
t366 = t106 * t365;
t369 = cos(t356);
t370 = t106 * t369;
t386 = cos(qJ(5));
t388 = sin(qJ(5));
t429 = -t1 * t369 + t160 * t357;
t431 = t160 * t365;
t433 = pkin(12) * t216 - t111 - t113;
t434 = t1 * t433;
t439 = -t1 * t357 - t160 * t369;
t454 = -t109 * t369 - t3 * t357;
t456 = t3 * t365;
t457 = t109 * t433;
t462 = -t109 * t357 + t3 * t369;
t476 = sin(pkin(6));
t478 = cos(pkin(6));
t479 = t1 * t478;
t496 = -g(1) * (m(14) * t117 + (t109 * t216 + t3 * t214) * mrSges(14,1) + (-t109 * t214 + t3 * t216) * mrSges(14,2) + t107 * mrSges(14,3)) - g(3) * (m(16) * (t358 * t362 + pkin(8) - t136 - t366 + r_base(3)) + t370 * mrSges(16,1) - t2 * mrSges(16,2) + t358 * mrSges(16,3)) - g(3) * (m(15) * (-t366 - t136 + pkin(8) + r_base(3)) + t370 * mrSges(15,1) - t358 * mrSges(15,2) - t2 * mrSges(15,3)) - g(1) * (m(8) * (-t206 * pkin(13) + t108 + t116 - t196 + t197 + r_base(1)) + (t107 * t388 + t202 * t386) * mrSges(8,1) + (t107 * t386 - t202 * t388) * mrSges(8,2) - t206 * mrSges(8,3)) - g(3) * (m(8) * (-t145 * pkin(13) + pkin(8) - t135 - t136 + r_base(3)) + (-t142 * t386 - t2 * t388) * mrSges(8,1) + (t142 * t388 - t2 * t386) * mrSges(8,2) - t145 * mrSges(8,3)) - g(2) * (m(8) * (-t176 * pkin(13) + t161 - t163 + t166 + t167 + r_base(2)) + (-t162 * t388 + t172 * t386) * mrSges(8,1) + (-t162 * t386 - t172 * t388) * mrSges(8,2) - t176 * mrSges(8,3)) - g(2) * (m(16) * (-t429 * t362 - t163 + t167 + t431 + t434 + r_base(2)) + t439 * mrSges(16,1) - t162 * mrSges(16,2) - t429 * mrSges(16,3)) - g(2) * (m(15) * (t431 - t163 + t434 + t167 + r_base(2)) + t439 * mrSges(15,1) + t429 * mrSges(15,2) - t162 * mrSges(15,3)) - g(1) * (m(16) * (-t454 * t362 + t108 + t116 - t456 + t457 + r_base(1)) + t462 * mrSges(16,1) + t107 * mrSges(16,2) - t454 * mrSges(16,3)) - g(1) * (m(15) * (-t456 + t108 + t457 + t116 + r_base(1)) + t462 * mrSges(15,1) + t454 * mrSges(15,2) + t107 * mrSges(15,3)) - g(2) * (m(9) * t316 + (-t160 * t476 + t479) * mrSges(9,1) + (-t1 * t476 - t160 * t478) * mrSges(9,2) - t162 * mrSges(9,3)) - g(1) * (m(3) * (t116 + r_base(1)) - t3 * mrSges(3,1) - t107 * mrSges(3,2) + t109 * mrSges(3,3));
t518 = pkin(8) + r_base(3);
t555 = t106 * t102;
t566 = t106 * t476;
t570 = 0.1e1 / pkin(22);
t571 = t91 * t570;
t573 = -t96 * t570;
t575 = t571 * t476 + t573 * t478;
t579 = -t573 * t476 + t571 * t478;
t589 = atan2(-(t579 * t476 - t575 * t478) * pkin(22) + pkin(24) + t68 - t69, (t575 * t476 + t579 * t478) * pkin(22) + t58 + t65);
t590 = pkin(6) + t97 - t589;
t591 = cos(t590);
t594 = sin(t590);
t610 = -g(2) * (m(3) * (t167 + r_base(2)) + t160 * mrSges(3,1) + t162 * mrSges(3,2) + t1 * mrSges(3,3)) - g(1) * (m(2) * r_base(1) + t1 * mrSges(2,1) + t109 * mrSges(2,2)) - g(2) * (m(2) * r_base(2) - t109 * mrSges(2,1) + t1 * mrSges(2,2)) - g(2) * (m(1) * r_base(2) + mrSges(1,2)) - g(3) * (m(2) * t518 + mrSges(2,3)) - g(3) * (m(1) * r_base(3) + mrSges(1,3)) - g(1) * (m(1) * r_base(1) + mrSges(1,1)) - g(1) * (m(13) * t117 + (-t109 * t231 + t3 * t229) * mrSges(13,1) + (-t109 * t229 - t3 * t231) * mrSges(13,2) + t107 * mrSges(13,3)) - g(1) * (m(12) * (t109 * t252 - t3 * t247 + t108 + t116 + r_base(1)) + (t109 * t274 + t3 * t272) * mrSges(12,1) + (-t109 * t272 + t3 * t274) * mrSges(12,2) + t107 * mrSges(12,3)) - g(3) * (m(11) * (-t555 * pkin(24) + pkin(8) - t136 + r_base(3)) - t106 * t244 * mrSges(11,1) - t106 * t249 * mrSges(11,2) - t2 * mrSges(11,3)) - g(3) * (m(10) * (-t566 * pkin(22) + pkin(8) - t136 + r_base(3)) - t106 * t591 * mrSges(10,1) + t106 * t594 * mrSges(10,2) - t2 * mrSges(10,3)) - g(3) * (m(5) * (-t555 * pkin(23) + pkin(8) - t136 + r_base(3)) - t106 * t99 * mrSges(5,1) - t106 * t110 * mrSges(5,2) - t2 * mrSges(5,3));
t611 = -t136 + pkin(8) + r_base(3);
t619 = t476 * pkin(22);
t636 = t109 * t112;
t651 = t108 + t116 + r_base(1);
t679 = t109 * t478;
t743 = -g(3) * (-t106 * t112 * mrSges(4,2) + m(4) * t611 - t555 * mrSges(4,1) - t2 * mrSges(4,3)) - g(2) * (m(10) * (-t479 * pkin(22) + t160 * t619 - t163 + t167 + r_base(2)) + (t1 * t594 + t160 * t591) * mrSges(10,1) + (t1 * t591 - t160 * t594) * mrSges(10,2) - t162 * mrSges(10,3)) - g(1) * (m(5) * (-t636 * pkin(23) - t3 * t103 + t108 + t116 + r_base(1)) + (-t109 * t110 - t3 * t99) * mrSges(5,1) + (t109 * t99 - t3 * t110) * mrSges(5,2) + t107 * mrSges(5,3)) - g(1) * (m(4) * t651 + (-t3 * t102 - t636) * mrSges(4,1) + (t109 * t102 - t3 * t112) * mrSges(4,2) + t107 * mrSges(4,3)) - g(1) * (m(11) * (-t636 * pkin(24) - t3 * t246 + t108 + t116 + r_base(1)) + (-t109 * t249 - t3 * t244) * mrSges(11,1) + (t109 * t244 - t3 * t249) * mrSges(11,2) + t107 * mrSges(11,3)) - g(1) * (m(10) * (-t679 * pkin(22) - t3 * t619 + t108 + t116 + r_base(1)) + (t109 * t594 - t3 * t591) * mrSges(10,1) + (t109 * t591 + t3 * t594) * mrSges(10,2) + t107 * mrSges(10,3)) - g(3) * (t106 * t214 * mrSges(14,1) + t106 * t216 * mrSges(14,2) + m(14) * t151 - t2 * mrSges(14,3)) - g(3) * (t106 * t229 * mrSges(13,1) - t106 * t231 * mrSges(13,2) + m(13) * t151 - t2 * mrSges(13,3)) - g(3) * (m(12) * (-t106 * t247 + pkin(8) - t136 + r_base(3)) + t106 * t272 * mrSges(12,1) + t106 * t274 * mrSges(12,2) - t2 * mrSges(12,3)) - g(1) * (m(9) * t651 + (t3 * t476 + t679) * mrSges(9,1) + (-t109 * t476 + t3 * t478) * mrSges(9,2) + t107 * mrSges(9,3)) - g(3) * (t106 * t478 * mrSges(9,2) + m(9) * t611 + t566 * mrSges(9,1) - t2 * mrSges(9,3)) - g(3) * (m(3) * t518 - t106 * mrSges(3,1) + t2 * mrSges(3,2));
t745 = t328 + t496 + t610 + t743;
U = t745;
