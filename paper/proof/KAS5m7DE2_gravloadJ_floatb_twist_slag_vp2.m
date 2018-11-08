% Calculate Gravitation load on the joints for
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
% m_mdh [16x1]
%   mass of all robot links (including the base)
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% 
% Output:
% taug [5x1]
%   joint torques required to compensate gravitation load

% Quelle: HybrDyn-Toolbox (ehem. IRT-Maple-Toolbox)
% Datum: 2018-11-06 17:34
% Revision: 51215904607767ba2b80495e06f47b388db9bd7a
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für mechatronische Systeme, Universität Hannover

function taug = KAS5m7DE2_gravloadJ_floatb_twist_slag_vp2(qJ, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE2_gravloadJ_floatb_twist_slag_vp2: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE2_gravloadJ_floatb_twist_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE2_gravloadJ_floatb_twist_slag_vp2: pkin has to be [24x1] (double)');
assert( isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE2_gravloadJ_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7DE2_gravloadJ_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From joint_gravload_floatb_twist_par2_matlab.m
% OptimizationMode: 2
% StartTime: 2018-11-06 08:20:27
% EndTime: 2018-11-06 08:42:47
% DurationCPUTime: 1275.62s
% Computational Cost: add. (45954710->491), mult. (59867915->646), div. (792237->24), fcn. (23957233->49), ass. (0->272)
t226 = pkin(3) + qJ(3);
t222 = cos(t226);
t366 = t222 * pkin(12);
t211 = 0.2e1 * t366;
t237 = cos(qJ(3));
t240 = -pkin(23) + pkin(24);
t220 = t237 * t240;
t201 = -(2 * pkin(9)) + 0.2e1 * t220 + t211;
t233 = sin(qJ(3));
t219 = t233 * t240;
t215 = -0.2e1 * t219;
t221 = sin(t226);
t367 = t221 * pkin(12);
t202 = t215 + 0.2e1 * t367;
t210 = -pkin(9) + t220;
t227 = t240 ^ 2;
t245 = pkin(11) ^ 2;
t185 = t227 * t233 ^ 2 - pkin(19) ^ 2 + t210 ^ 2 + t245 + (t201 * t222 + t202 * t221 + (-t221 ^ 2 - t222 ^ 2) * pkin(12)) * pkin(12);
t193 = t201 ^ 2 + t202 ^ 2;
t248 = sqrt(-t185 ^ 2 + t193 * t245);
t178 = t185 * t201 - t202 * t248;
t191 = 0.1e1 / t193;
t302 = t210 + t366;
t173 = t178 * t191 - t302;
t177 = t185 * t202 + t201 * t248;
t308 = t219 - t367;
t174 = t177 * t191 + t308;
t170 = t173 * t233 + t174 * t237;
t172 = t173 * t237 - t174 * t233;
t242 = 0.1e1 / pkin(19);
t375 = pkin(18) * t242;
t329 = cos(pkin(7)) * t375;
t330 = sin(pkin(7)) * t375;
t160 = -t170 * t330 + t172 * t329;
t263 = pkin(24) + t160;
t157 = 0.2e1 * t263;
t158 = t170 * t329 + t172 * t330;
t159 = 0.2e1 * t158;
t153 = t157 ^ 2 + t159 ^ 2;
t152 = 0.1e1 / t153;
t243 = pkin(17) ^ 2;
t141 = -pkin(22) ^ 2 + pkin(24) ^ 2 + t243 + (t157 - t160) * t160 + (t159 - t158) * t158;
t247 = sqrt(-t141 ^ 2 + t153 * t243);
t276 = -t141 * t157 + t159 * t247;
t134 = t152 * t276 + t263;
t136 = -t141 * t159 - t157 * t247;
t135 = -t136 * t152 - t158;
t128 = atan2(t135, t134) + pkin(6);
t126 = 0.2e1 * qJ(3) + t128;
t117 = pkin(3) + t126;
t110 = sin(t117);
t112 = cos(t117);
t119 = sin(t126);
t127 = qJ(3) + t128;
t120 = sin(t127);
t121 = cos(t126);
t122 = cos(t127);
t125 = cos(t128);
t229 = sin(pkin(6));
t231 = cos(pkin(6));
t323 = m(10) * pkin(22) - mrSges(9,1);
t124 = sin(t128);
t387 = pkin(24) * t124;
t96 = -atan2(-t170, t172) + t128;
t94 = sin(t96);
t391 = pkin(19) * t94;
t246 = 0.1e1 / pkin(22);
t107 = (t134 * t229 + t135 * t231) * t246;
t108 = (-t134 * t231 + t135 * t229) * t246;
t58 = -(t107 * t229 - t108 * t231) * pkin(22) + t263;
t60 = (t107 * t231 + t108 * t229) * pkin(22) + t158;
t42 = -atan2(t58, t60) + t128;
t40 = sin(t42);
t41 = cos(t42);
t334 = (t170 * t233 + t172 * t237) * t242;
t155 = pkin(19) * t334 + t302;
t374 = pkin(19) * t242;
t156 = (-t170 * t237 + t172 * t233) * t374 + t308;
t93 = -atan2(t156, t155) + t127;
t91 = sin(t93);
t92 = cos(t93);
t95 = cos(t96);
t460 = t229 * t323 + t125 * mrSges(4,2) - t231 * mrSges(9,2) - t121 * mrSges(13,1) - t110 * mrSges(14,1) + t119 * mrSges(13,2) - t112 * mrSges(14,2) + m(12) * (t387 + t391) - t91 * mrSges(12,1) - t92 * mrSges(12,2) + t120 * mrSges(5,1) + t122 * mrSges(5,2) + t94 * mrSges(11,1) + t95 * mrSges(11,2) + t41 * mrSges(10,1) - t40 * mrSges(10,2);
t433 = -mrSges(3,2) + mrSges(13,3) + mrSges(14,3) + mrSges(6,3);
t439 = mrSges(16,1) + mrSges(15,1);
t458 = -mrSges(11,3) - mrSges(12,3) - mrSges(5,3) - mrSges(10,3) - mrSges(16,2) - mrSges(15,3) - mrSges(7,3) - mrSges(4,3) - mrSges(9,3);
t123 = qJ(4) + t127;
t224 = -qJ(4) + t226;
t209 = cos(t224) * pkin(12);
t223 = qJ(4) - qJ(3) + pkin(4);
t216 = sin(t223);
t218 = cos(t223);
t284 = pkin(14) * t216 + pkin(15) * t218;
t196 = -pkin(10) - t209 - t284;
t306 = t218 * pkin(14) - pkin(15) * t216;
t376 = pkin(12) * sin(t224);
t197 = -t306 + t376;
t101 = -atan2(t197, t196) + t123;
t100 = cos(t101);
t116 = 0.2e1 * qJ(4) + pkin(4) + t128;
t109 = sin(t116);
t111 = cos(t116);
t114 = sin(t123);
t115 = cos(t123);
t118 = t124 * pkin(23);
t99 = sin(t101);
t456 = m(11) * t387 + m(5) * t118 + t124 * mrSges(4,1) + mrSges(6,1) * t114 + t111 * mrSges(7,1) + t99 * mrSges(15,2) + mrSges(6,2) * t115 - t109 * mrSges(7,2) - t100 * t439 + t460;
t402 = -m(4) - m(9);
t238 = cos(qJ(2));
t392 = g(3) * t238;
t198 = -t209 + t284;
t199 = t306 + t376;
t251 = t196 ^ 2;
t406 = t197 ^ 2;
t249 = sqrt(t251 + t406);
t452 = 0.2e1 * (t196 * t199 - t197 * t198) / t249;
t440 = t452 / 0.2e1;
t239 = cos(qJ(1));
t234 = sin(qJ(2));
t235 = sin(qJ(1));
t347 = t234 * t235;
t67 = t100 * t239 + t347 * t99;
t455 = t440 * t67;
t454 = t440 * t99;
t317 = m(8) * pkin(13) + mrSges(8,3);
t442 = -mrSges(7,2) + t317;
t343 = t235 * t109;
t345 = t234 * t239;
t82 = t111 * t345 + t343;
t453 = t442 * t82;
t271 = -t109 * t239 + t111 * t347;
t451 = t442 * t271;
t450 = mrSges(3,1) * t238 + t234 * t433;
t310 = m(16) * t249 + mrSges(16,3);
t449 = t310 * t99 - t456;
t447 = pkin(21) * t402 + t458;
t416 = -m(10) - m(5) - m(12) - m(11);
t401 = m(7) + m(8);
t385 = m(16) + m(15);
t301 = mrSges(15,2) - t310;
t133 = 0.1e1 / t134 ^ 2;
t203 = t215 - 0.2e1 * t367;
t204 = -0.2e1 * t220 + t211;
t182 = 0.2e1 * (-t210 * t240 + t227 * t237) * t233 + ((t202 + t203) * t222 + (-t201 + t204) * t221) * pkin(12);
t307 = t220 - t366;
t186 = 0.2e1 * t201 * t203 + 0.2e1 * t202 * t204;
t350 = t186 / t193 ^ 2;
t352 = (-0.2e1 * t182 * t185 + t186 * t245) / t248;
t335 = (t182 * t202 + t185 * t204 + t203 * t248 + t201 * t352 / 0.2e1) * t191 - t177 * t350 + t307 + t173;
t309 = -t219 - t367;
t336 = (t203 * t185 + t201 * t182 - t204 * t248 - t202 * t352 / 0.2e1) * t191 - t178 * t350 - t309 - t174;
t149 = -t233 * t335 + t237 * t336;
t150 = t233 * t336 + t237 * t335;
t273 = t149 * t329 - t150 * t330;
t143 = 0.2e1 * t273;
t337 = t149 * t330 + t150 * t329;
t145 = 0.2e1 * t337;
t131 = t143 * t160 + t145 * t158 + (-0.2e1 * t158 + t159) * t337 + (t157 - 0.2e1 * t160) * t273;
t137 = 0.2e1 * t143 * t157 + 0.2e1 * t145 * t159;
t353 = t137 / t153 ^ 2;
t368 = 0.1e1 / t247 * (-0.2e1 * t131 * t141 + t137 * t243);
t43 = (-t143 * t141 - t157 * t131 + t145 * t247 + t159 * t368 / 0.2e1) * t152 - t276 * t353 + t273;
t44 = -(-t131 * t159 - t141 * t145 - t143 * t247 - t157 * t368 / 0.2e1) * t152 + t136 * t353 - t337;
t36 = (-t43 * t135 * t133 + t44 / t134) / (t133 * t135 ^ 2 + 0.1e1);
t35 = t36 + 0.1e1;
t195 = 0.1e1 / t251;
t426 = 0.1e1 / (t195 * t406 + 0.1e1) * (t195 * t197 * t199 + 0.1e1 / t196 * t198);
t33 = t35 + t426;
t443 = t33 * t301;
t232 = sin(qJ(5));
t236 = cos(qJ(5));
t286 = mrSges(8,1) * t236 - mrSges(8,2) * t232;
t428 = mrSges(7,1) + t286;
t344 = t235 * t100;
t69 = t345 * t99 - t344;
t441 = -t69 * t452 / 0.2e1;
t86 = t114 * t235 + t115 * t345;
t84 = t114 * t239 - t115 * t347;
t438 = mrSges(15,2) * t100 + t439 * t99;
t436 = t385 + t401;
t434 = pkin(10) * t401;
t424 = t119 * mrSges(13,1) - t112 * mrSges(14,1) + t121 * mrSges(13,2) + t110 * mrSges(14,2);
t422 = t234 * t344 - t239 * t99;
t420 = -m(11) * pkin(24) - m(5) * pkin(23);
t332 = -m(6) - m(14) - m(13);
t415 = t36 * t428;
t414 = t33 * t439;
t413 = t36 * t420;
t408 = -pkin(21) * t416 - t447;
t405 = -0.2e1 * t111;
t113 = pkin(9) * t120;
t372 = t124 * t36;
t24 = pkin(23) * t372 + t35 * t113;
t34 = t36 + 0.2e1;
t378 = pkin(12) * t110;
t10 = -t34 * t378 + t24;
t369 = t125 * t36;
t30 = pkin(23) * t369;
t381 = pkin(9) * t122;
t25 = t35 * t381 + t30;
t377 = pkin(12) * t112;
t11 = -t34 * t377 + t25;
t400 = t235 * t10 + t11 * t345;
t390 = pkin(19) * t95;
t389 = pkin(21) * t238;
t388 = pkin(23) * t125;
t386 = pkin(24) * t125;
t384 = mrSges(7,1) * t109;
t383 = mrSges(7,2) * t111;
t380 = pkin(10) * t114;
t379 = pkin(10) * t115;
t373 = t124 * mrSges(4,2);
t371 = t125 * mrSges(4,1);
t364 = t236 * mrSges(8,2);
t356 = t109 * t234;
t349 = t232 * t234;
t348 = t232 * t238;
t346 = t234 * t236;
t342 = t235 * t238;
t341 = t236 * t238;
t340 = t238 * t239;
t88 = t113 + t118;
t333 = t239 * pkin(16) + pkin(21) * t342;
t331 = pkin(24) * t369;
t56 = t88 + t380;
t311 = t234 * t56 - t389;
t304 = t239 * t10 - t11 * t347;
t83 = t114 * t347 + t115 * t239;
t300 = mrSges(6,1) * t84 + mrSges(6,2) * t83;
t85 = -t114 * t345 + t115 * t235;
t299 = mrSges(6,1) * t86 + mrSges(6,2) * t85;
t297 = t40 * mrSges(10,1) + t41 * mrSges(10,2);
t89 = -t381 - t388;
t296 = -t95 * mrSges(11,1) + t94 * mrSges(11,2);
t294 = t92 * mrSges(12,1) - t91 * mrSges(12,2);
t291 = -t122 * mrSges(5,1) + t120 * mrSges(5,2);
t289 = -mrSges(6,1) * t115 + mrSges(6,2) * t114;
t281 = mrSges(4,1) - t420;
t272 = t100 * t345 + t235 * t99;
t270 = t109 * t345 - t111 * t235;
t269 = t111 * t239 + t234 * t343;
t268 = t111 * t341 + t349;
t267 = t111 * t348 - t346;
t266 = t124 * t235 + t125 * t345;
t265 = t124 * t239 - t125 * t347;
t262 = t286 * t109;
t254 = -t433 + t458;
t253 = -m(3) * pkin(16) + t229 * mrSges(9,2) - mrSges(2,2) - mrSges(3,3) - t291 - t294 - t296 - t297 - t373 + t424;
t54 = t88 - t378;
t252 = (t124 * t281 + t385 * t54 + t401 * t56 + mrSges(3,1) + t460) * t234 - mrSges(2,1);
t214 = pkin(21) * t340;
t212 = pkin(21) * t345;
t207 = t232 * t340;
t176 = 0.1e1 - t426;
t169 = 0.1e1 / t172 ^ 2;
t154 = 0.1e1 / t155 ^ 2;
t72 = t232 * t342 - t236 * t271;
t71 = t232 * t271 + t235 * t341;
t62 = -t386 - t390;
t59 = 0.1e1 / t60 ^ 2;
t57 = t89 - t379;
t55 = t89 + t377;
t38 = (t229 * t44 - t231 * t43) * t246;
t37 = (t229 * t43 + t231 * t44) * t246;
t32 = (t150 / t172 - t149 * t170 * t169) / (t169 * t170 ^ 2 + 0.1e1) + t36;
t31 = t35 + (-(((t149 * t233 - t150 * t237) * t242 + t334) * pkin(19) + t307) / t155 + (((t149 + t170) * t237 + (t150 - t172) * t233) * t374 + t309) * t156 * t154) / (t154 * t156 ^ 2 + 0.1e1);
t15 = t32 * t390 + t331;
t14 = pkin(24) * t372 + t32 * t391;
t13 = t35 * t379 + t25;
t12 = t35 * t380 + t24;
t1 = (-(-(t229 * t37 - t231 * t38) * pkin(22) + t273) / t60 + ((t229 * t38 + t231 * t37) * pkin(22) + t337) * t58 * t59) / (t58 ^ 2 * t59 + 0.1e1) + t36;
t2 = [(-mrSges(6,1) * t85 - t207 * mrSges(8,1) + mrSges(6,2) * t86 + t428 * t82 - t439 * t272 + t402 * (-pkin(16) * t235 + t214) + t442 * t270 + t301 * t69 + t332 * (-t88 * t345 + t214 + (-pkin(16) - t89) * t235) + (t416 - t436) * t214 + (-m(12) * (-pkin(16) - t62) - t371 - m(5) * (-pkin(16) + t388) + t231 * mrSges(9,1) - t253 - m(11) * (-pkin(16) + t386) - m(10) * (pkin(22) * t231 - pkin(16)) + t385 * (pkin(16) + t55) + t401 * (pkin(16) + t57)) * t235 + ((t254 - t364) * t238 + t252) * t239) * g(1) + (mrSges(6,1) * t83 - mrSges(6,2) * t84 + t271 * mrSges(7,1) - t72 * mrSges(8,1) - t71 * mrSges(8,2) + t442 * t269 - t439 * t422 + t301 * t67 + t332 * (t239 * t89 - t347 * t88) + (-m(12) * t62 + t125 * t281 + t231 * t323 + t253) * t239 + (t238 * t254 + t252) * t235 + (t402 + t332 + t416) * t333 - t385 * (t239 * t55 + t333) - t401 * (t239 * t57 + t333)) * g(2) (-m(7) * t311 - m(8) * (pkin(13) * t356 + t311) - (-t111 * t349 - t341) * mrSges(8,2) - mrSges(8,3) * t356 - (t111 * t346 - t348) * mrSges(8,1) - t385 * (t234 * t54 - t389) + t332 * (t234 * t88 - t389) + (t408 + t433) * t238 + (-mrSges(3,1) + t449) * t234) * g(3) + ((t232 * mrSges(8,1) + t364 + t408 + (-t332 + t436) * pkin(21)) * t234 + (-t332 * t88 - m(8) * (-pkin(13) * t109 - t56) + t109 * mrSges(8,3) + t286 * t111 - m(16) * (t249 * t99 - t54) - t99 * mrSges(16,3) + m(15) * t54 + m(7) * t56 + t456) * t238 + t450) * g(1) * t235 + (-t385 * (t54 * t340 + t212) - t401 * (t56 * t340 + t212) + t332 * (t340 * t88 + t212) + t416 * t212 + (-mrSges(8,1) * t268 + mrSges(8,2) * t267 + (-t109 * t317 + t449) * t238 + t447 * t234 - t450) * t239) * g(2) (-t297 * t1 - m(16) * (-t11 + t454) + m(11) * t331 - t296 * t32 + m(15) * t11 + m(12) * t15 - t294 * t31 + m(5) * t30 + (-t289 - t291) * t35 + t424 * t34 + (-t100 * t310 + t438) * t33 + t401 * t13 - t332 * t25) * t392 + (-((t235 * t41 - t345 * t40) * mrSges(10,1) + (-t235 * t40 - t345 * t41) * mrSges(10,2)) * t1 - m(16) * (t400 + t441) - ((t235 * t94 + t345 * t95) * mrSges(11,1) + (t235 * t95 - t345 * t94) * mrSges(11,2)) * t32 - ((-t235 * t91 - t345 * t92) * mrSges(12,1) + (-t235 * t92 + t345 * t91) * mrSges(12,2)) * t31 - m(12) * (t14 * t235 + t15 * t345) - m(15) * t400 - t401 * (t235 * t12 + t13 * t345) + (-(t120 * t235 + t122 * t345) * mrSges(5,1) - (-t120 * t345 + t122 * t235) * mrSges(5,2) - t299) * t35 + (-(t119 * t345 - t121 * t235) * mrSges(13,1) - (t119 * t235 + t121 * t345) * mrSges(13,2) - (-t110 * t235 - t112 * t345) * mrSges(14,1) - (t110 * t345 - t112 * t235) * mrSges(14,2)) * t34 + t266 * t413 + t270 * t415 - t272 * t443 - t69 * t414 + t332 * (t235 * t24 + t25 * t345)) * g(2) + (-m(12) * (t14 * t239 - t15 * t347) - ((t239 * t94 - t347 * t95) * mrSges(11,1) + (t239 * t95 + t347 * t94) * mrSges(11,2)) * t32 - ((-t239 * t91 + t347 * t92) * mrSges(12,1) + (-t239 * t92 - t347 * t91) * mrSges(12,2)) * t31 - ((t239 * t41 + t347 * t40) * mrSges(10,1) + (-t239 * t40 + t347 * t41) * mrSges(10,2)) * t1 - m(15) * t304 - m(16) * (t304 + t455) + (-(t120 * t239 - t122 * t347) * mrSges(5,1) - (t120 * t347 + t122 * t239) * mrSges(5,2) - t300) * t35 + (-(-t119 * t347 - t121 * t239) * mrSges(13,1) - (t119 * t239 - t121 * t347) * mrSges(13,2) - (-t110 * t239 + t112 * t347) * mrSges(14,1) - (-t110 * t347 - t112 * t239) * mrSges(14,2)) * t34 - t401 * (t239 * t12 - t13 * t347) + t265 * t413 - t269 * t415 + t422 * t443 + t67 * t414 + t332 * (t239 * t24 - t25 * t347)) * g(1) + ((t111 * t317 - t262 + t371 - t373 - t383 - t384) * t392 + (-t266 * mrSges(4,1) - (-t124 * t345 + t125 * t235) * mrSges(4,2) - t453) * g(2) + (-t265 * mrSges(4,1) - (t124 * t347 + t125 * t239) * mrSges(4,2) + t451) * g(1)) * t36 (m(16) * t441 + 0.2e1 * t270 * t428 - 0.2e1 * t453 - t434 * t86 - t299 + (-t272 * t301 - t439 * t69) * t176) * g(2) + (m(16) * t455 - 0.2e1 * t269 * t428 + 0.2e1 * t451 - t434 * t84 - t300 + (t301 * t422 + t439 * t67) * t176) * g(1) + (-m(16) * (t176 * t100 * t249 - t454) + (-mrSges(16,3) * t100 + t438) * t176 + m(7) * t379 - 0.2e1 * t383 - 0.2e1 * t384 - t289 - m(8) * (pkin(13) * t405 - t379) - mrSges(8,3) * t405 - 0.2e1 * t262) * t392, -g(3) * (mrSges(8,1) * t267 + mrSges(8,2) * t268) - g(1) * (mrSges(8,1) * t71 - mrSges(8,2) * t72) - g(2) * ((-t232 * t82 - t236 * t340) * mrSges(8,1) + (-t236 * t82 + t207) * mrSges(8,2))];
taug  = t2(:);
