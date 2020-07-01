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
% rSges [16x3]
%   center of mass of all robot links (in body frames)
%   rows: links of the robot (starting with base)
%   columns: x-, y-, z-coordinates
% 
% Output:
% U [1x1]
%   Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = KAS5m7TE_energypot_floatb_twist_slag_vp1(qJ, r_base, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp1: qJ has to be [5x1] (double)');
assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp1: r_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp1: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7TE_energypot_floatb_twist_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_floatb_twist_worldframe_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-04-13 19:09:07
% EndTime: 2020-04-13 19:14:39
% DurationCPUTime: 199.08s
% Computational Cost: add. (13660908->481), mult. (17634367->506), div. (287358->7), fcn. (7070285->28), ass. (0->179)
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
t48 = cos(qJ(3));
t49 = -pkin(23) + pkin(24);
t50 = t48 * t49;
t51 = pkin(3) + qJ(3);
t52 = cos(t51);
t53 = t52 * pkin(12);
t54 = -pkin(9) + t50 + t53;
t55 = pkin(11) ^ 2;
t56 = pkin(19) ^ 2;
t57 = t52 ^ 2;
t58 = pkin(12) ^ 2;
t60 = sin(t51);
t61 = t60 ^ 2;
t64 = (-pkin(9) + t50) ^ 2;
t65 = sin(qJ(3));
t66 = t65 ^ 2;
t67 = t49 ^ 2;
t71 = t65 * t49;
t72 = t60 * pkin(12);
t73 = -t71 + t72;
t76 = 0.2e1 * t54 * t52 * pkin(12) + 0.2e1 * t73 * t60 * pkin(12) - t57 * t58 - t61 * t58 + t66 * t67 + t55 - t56 + t64;
t80 = 0.4e1 * t54 ^ 2 + 0.4e1 * t73 ^ 2;
t82 = t76 ^ 2;
t84 = sqrt(t55 * t80 - t82);
t87 = 0.1e1 / t80;
t89 = pkin(9) - t50 - t53 + (0.2e1 * t54 * t76 - 0.2e1 * t73 * t84) * t87;
t95 = t71 - t72 + (0.2e1 * t54 * t84 + 0.2e1 * t76 * t73) * t87;
t97 = t48 * t89 - t65 * t95;
t98 = 0.1e1 / pkin(19);
t99 = t97 * t98;
t100 = cos(pkin(7));
t101 = t100 * pkin(18);
t102 = t99 * t101;
t105 = t48 * t95 + t65 * t89;
t106 = t105 * t98;
t107 = sin(pkin(7));
t108 = t107 * pkin(18);
t109 = t106 * t108;
t110 = -pkin(24) - t102 + t109;
t111 = pkin(17) ^ 2;
t112 = pkin(22) ^ 2;
t113 = t102 - t109;
t114 = t113 ^ 2;
t115 = t106 * t101;
t116 = t99 * t108;
t117 = t115 + t116;
t118 = t117 ^ 2;
t119 = pkin(24) ^ 2;
t122 = -0.2e1 * t110 * t113 + 0.2e1 * t117 ^ 2 + t111 - t112 - t114 - t118 + t119;
t126 = 0.4e1 * t110 ^ 2 + 0.4e1 * t117 ^ 2;
t128 = t122 ^ 2;
t130 = sqrt(t111 * t126 - t128);
t133 = 0.1e1 / t126;
t136 = 0.1e1 / pkin(22);
t137 = (-t102 + t109 - (0.2e1 * t110 * t122 + 0.2e1 * t117 * t130) * t133 - pkin(24)) * t136;
t138 = sin(pkin(6));
t145 = (-t115 - t116 - (0.2e1 * t110 * t130 - 0.2e1 * t122 * t117) * t133) * t136;
t146 = cos(pkin(6));
t148 = -t137 * t138 + t145 * t146;
t152 = t137 * t146 + t145 * t138;
t154 = t11 * t152 - t25 * t148;
t158 = t11 * t148 + t25 * t152;
t163 = t36 * pkin(21);
t166 = t34 * t148 + t9 * t152;
t170 = t9 * t148 - t34 * t152;
t175 = t24 * pkin(21);
t176 = t27 * t148;
t178 = -t27 * t152;
t185 = t154 * pkin(23);
t188 = t154 * t48 + t158 * t65;
t192 = -t154 * t65 + t158 * t48;
t197 = t166 * pkin(23);
t200 = t166 * t48 + t170 * t65;
t204 = -t166 * t65 + t170 * t48;
t209 = t176 * pkin(23);
t212 = -t176 * t48 - t178 * t65;
t216 = t176 * t65 - t178 * t48;
t223 = t188 * pkin(9);
t224 = cos(qJ(4));
t226 = sin(qJ(4));
t228 = t188 * t224 + t192 * t226;
t232 = -t188 * t226 + t192 * t224;
t237 = t200 * pkin(9);
t240 = t200 * t224 + t204 * t226;
t244 = -t200 * t226 + t204 * t224;
t249 = t212 * pkin(9);
t252 = t212 * t224 + t216 * t226;
t256 = -t212 * t226 + t216 * t224;
t263 = t228 * pkin(10);
t264 = qJ(4) - qJ(3) + pkin(4);
t265 = sin(t264);
t267 = cos(t264);
t269 = -t228 * t265 + t232 * t267;
t273 = -t228 * t267 - t232 * t265;
t278 = t240 * pkin(10);
t281 = -t240 * t265 + t244 * t267;
t285 = -t240 * t267 - t244 * t265;
t290 = t252 * pkin(10);
t293 = -t252 * t265 + t256 * t267;
t297 = -t252 * t267 - t256 * t265;
t305 = cos(qJ(5));
t307 = sin(qJ(5));
t346 = t11 * t146 + t25 * t138;
t350 = -t11 * t138 + t25 * t146;
t357 = -t34 * t138 + t9 * t146;
t361 = -t9 * t138 - t34 * t146;
t366 = t27 * t138;
t368 = t27 * t146;
t378 = t152 * t138 + t148 * t146;
t380 = t378 * pkin(22) + t115 + t116;
t382 = 0.1e1 / pkin(17);
t386 = t148 * t138 - t152 * t146;
t388 = -t386 * pkin(22) + pkin(24) + t102 - t109;
t391 = t378 * t380 * t382 - t386 * t388 * t382;
t397 = -t378 * t388 * t382 - t380 * t382 * t386;
t434 = t154 * pkin(24);
t439 = t158 * t105 * t98 + t154 * t97 * t98;
t445 = -t154 * t105 * t98 + t158 * t97 * t98;
t450 = t166 * pkin(24);
t455 = t170 * t105 * t98 + t166 * t97 * t98;
t461 = -t166 * t105 * t98 + t170 * t97 * t98;
t466 = t176 * pkin(24);
t469 = -t178 * t106 - t176 * t99;
t473 = t176 * t106 - t178 * t99;
t485 = t65 * t105 * t98 + t48 * t97 * t98;
t487 = t485 * pkin(19) - pkin(9) + t50 + t53;
t489 = 0.1e1 / pkin(11);
t495 = -t48 * t105 * t98 + t65 * t97 * t98;
t497 = t495 * pkin(19) + t71 - t72;
t500 = -t485 * t487 * t489 - t495 * t497 * t489;
t506 = t485 * t497 * t489 - t487 * t489 * t495;
t545 = t188 * t65 - t192 * t48;
t549 = t188 * t48 + t192 * t65;
t556 = t200 * t65 - t204 * t48;
t560 = t200 * t48 + t204 * t65;
t567 = t212 * t65 - t216 * t48;
t571 = t212 * t48 + t216 * t65;
t578 = sin(pkin(3));
t580 = cos(pkin(3));
t582 = t545 * t578 - t549 * t580;
t586 = t545 * t580 + t549 * t578;
t593 = t556 * t578 - t560 * t580;
t597 = t556 * t580 + t560 * t578;
t604 = t567 * t578 - t571 * t580;
t608 = t567 * t580 + t571 * t578;
t615 = t582 * pkin(12);
t617 = pkin(3) + qJ(3) - qJ(4);
t618 = sin(t617);
t621 = t618 * pkin(12) - t267 * pkin(14) + t265 * pkin(15);
t624 = cos(t617);
t626 = -t624 * pkin(12) - t265 * pkin(14) - t267 * pkin(15) - pkin(10);
t627 = t626 ^ 2;
t628 = t621 ^ 2;
t630 = sqrt(t627 + t628);
t631 = 0.1e1 / t630;
t632 = t621 * t631;
t634 = t626 * t631;
t636 = t634 * t618 + t632 * t624;
t640 = -t632 * t618 + t634 * t624;
t642 = t582 * t636 + t586 * t640;
t646 = -t582 * t640 + t586 * t636;
t651 = t593 * pkin(12);
t654 = t593 * t636 + t597 * t640;
t658 = -t593 * t640 + t597 * t636;
t663 = t604 * pkin(12);
t666 = t604 * t636 + t608 * t640;
t670 = -t604 * t640 + t608 * t636;
t697 = -m(1) * (g(1) * (r_base(1) + rSges(1,1)) + g(2) * (r_base(2) + rSges(1,2)) + g(3) * (r_base(3) + rSges(1,3))) - m(2) * (g(1) * (t9 * rSges(2,1) + t11 * rSges(2,2) + r_base(1)) + g(2) * (-t11 * rSges(2,1) + t9 * rSges(2,2) + r_base(2)) + g(3) * (pkin(8) + r_base(3) + rSges(2,3))) - m(3) * (g(1) * (-t25 * rSges(3,1) - t28 * rSges(3,2) + t11 * rSges(3,3) + t23 + r_base(1)) + g(2) * (t34 * rSges(3,1) + t36 * rSges(3,2) + t9 * rSges(3,3) + t33 + r_base(2)) + g(3) * (-t27 * rSges(3,1) + t24 * rSges(3,2) + pkin(8) + r_base(3))) - m(4) * (g(1) * (t154 * rSges(4,1) + t158 * rSges(4,2) + t28 * rSges(4,3) + t23 + t47 + r_base(1)) + g(2) * (t166 * rSges(4,1) + t170 * rSges(4,2) - t36 * rSges(4,3) - t163 + t33 + r_base(2)) + g(3) * (-t176 * rSges(4,1) - t178 * rSges(4,2) - t24 * rSges(4,3) + pkin(8) - t175 + r_base(3))) - m(5) * (g(1) * (t188 * rSges(5,1) + t192 * rSges(5,2) + t28 * rSges(5,3) + t185 + t23 + t47 + r_base(1)) + g(2) * (t200 * rSges(5,1) + t204 * rSges(5,2) - t36 * rSges(5,3) - t163 + t197 + t33 + r_base(2)) + g(3) * (t212 * rSges(5,1) + t216 * rSges(5,2) - t24 * rSges(5,3) + pkin(8) - t175 - t209 + r_base(3))) - m(6) * (g(1) * (t228 * rSges(6,1) + t232 * rSges(6,2) + t28 * rSges(6,3) + t185 + t223 + t23 + t47 + r_base(1)) + g(2) * (t240 * rSges(6,1) + t244 * rSges(6,2) - t36 * rSges(6,3) - t163 + t197 + t237 + t33 + r_base(2)) + g(3) * (t252 * rSges(6,1) + t256 * rSges(6,2) - t24 * rSges(6,3) + pkin(8) - t175 - t209 + t249 + r_base(3))) - m(7) * (g(1) * (t269 * rSges(7,1) + t273 * rSges(7,2) + t28 * rSges(7,3) + t185 + t223 + t23 + t263 + t47 + r_base(1)) + g(2) * (t281 * rSges(7,1) + t285 * rSges(7,2) - t36 * rSges(7,3) - t163 + t197 + t237 + t278 + t33 + r_base(2)) + g(3) * (t293 * rSges(7,1) + t297 * rSges(7,2) - t24 * rSges(7,3) + pkin(8) - t175 - t209 + t249 + t290 + r_base(3))) - m(8) * (g(1) * (-t273 * pkin(13) + t263 + t223 + t185 + t47 + t23 + r_base(1) + (t269 * t305 + t28 * t307) * rSges(8,1) + (-t269 * t307 + t28 * t305) * rSges(8,2) - t273 * rSges(8,3)) + g(2) * (-t285 * pkin(13) + t278 + t237 + t197 - t163 + t33 + r_base(2) + (t281 * t305 - t36 * t307) * rSges(8,1) + (-t281 * t307 - t36 * t305) * rSges(8,2) - t285 * rSges(8,3)) + g(3) * (-t297 * pkin(13) + t290 + t249 - t209 - t175 + pkin(8) + r_base(3) + (-t24 * t307 + t293 * t305) * rSges(8,1) + (-t24 * t305 - t293 * t307) * rSges(8,2) - t297 * rSges(8,3))) - m(9) * (g(1) * (t346 * rSges(9,1) + t350 * rSges(9,2) + t28 * rSges(9,3) + t23 + t47 + r_base(1)) + g(2) * (t357 * rSges(9,1) + t361 * rSges(9,2) - t36 * rSges(9,3) - t163 + t33 + r_base(2)) + g(3) * (t366 * rSges(9,1) + t368 * rSges(9,2) - t24 * rSges(9,3) + pkin(8) - t175 + r_base(3))) - m(10) * (g(1) * (-t346 * pkin(22) + t47 + t23 + r_base(1) + (t346 * t391 + t350 * t397) * rSges(10,1) + (-t346 * t397 + t350 * t391) * rSges(10,2) + t28 * rSges(10,3)) + g(2) * (-t357 * pkin(22) - t163 + t33 + r_base(2) + (t357 * t391 + t361 * t397) * rSges(10,1) + (-t357 * t397 + t361 * t391) * rSges(10,2) - t36 * rSges(10,3)) + g(3) * (-t366 * pkin(22) - t175 + pkin(8) + r_base(3) + (t366 * t391 + t368 * t397) * rSges(10,1) + (-t366 * t397 + t368 * t391) * rSges(10,2) - t24 * rSges(10,3))) - m(11) * (g(1) * (t439 * rSges(11,1) + t445 * rSges(11,2) + t28 * rSges(11,3) + t23 + t434 + t47 + r_base(1)) + g(2) * (t455 * rSges(11,1) + t461 * rSges(11,2) - t36 * rSges(11,3) - t163 + t33 + t450 + r_base(2)) + g(3) * (t469 * rSges(11,1) + t473 * rSges(11,2) - t24 * rSges(11,3) + pkin(8) - t175 - t466 + r_base(3))) - m(12) * (g(1) * (t439 * pkin(19) + t434 + t47 + t23 + r_base(1) + (t439 * t500 + t445 * t506) * rSges(12,1) + (-t439 * t506 + t445 * t500) * rSges(12,2) + t28 * rSges(12,3)) + g(2) * (t455 * pkin(19) + t450 - t163 + t33 + r_base(2) + (t455 * t500 + t461 * t506) * rSges(12,1) + (-t455 * t506 + t461 * t500) * rSges(12,2) - t36 * rSges(12,3)) + g(3) * (t469 * pkin(19) - t466 - t175 + pkin(8) + r_base(3) + (t469 * t500 + t473 * t506) * rSges(12,1) + (-t469 * t506 + t473 * t500) * rSges(12,2) - t24 * rSges(12,3))) - m(13) * (g(1) * (t545 * rSges(13,1) + t549 * rSges(13,2) + t28 * rSges(13,3) + t185 + t223 + t23 + t47 + r_base(1)) + g(2) * (t556 * rSges(13,1) + t560 * rSges(13,2) - t36 * rSges(13,3) - t163 + t197 + t237 + t33 + r_base(2)) + g(3) * (t567 * rSges(13,1) + t571 * rSges(13,2) - t24 * rSges(13,3) + pkin(8) - t175 - t209 + t249 + r_base(3))) - m(14) * (g(1) * (t582 * rSges(14,1) + t586 * rSges(14,2) + t28 * rSges(14,3) + t185 + t223 + t23 + t47 + r_base(1)) + g(2) * (t593 * rSges(14,1) + t597 * rSges(14,2) - t36 * rSges(14,3) - t163 + t197 + t237 + t33 + r_base(2)) + g(3) * (t604 * rSges(14,1) + t608 * rSges(14,2) - t24 * rSges(14,3) + pkin(8) - t175 - t209 + t249 + r_base(3))) - m(15) * (g(1) * (t642 * rSges(15,1) + t646 * rSges(15,2) + t28 * rSges(15,3) + t185 + t223 + t23 + t47 + t615 + r_base(1)) + g(2) * (t654 * rSges(15,1) + t658 * rSges(15,2) - t36 * rSges(15,3) - t163 + t197 + t237 + t33 + t651 + r_base(2)) + g(3) * (t666 * rSges(15,1) + t670 * rSges(15,2) - t24 * rSges(15,3) + pkin(8) - t175 - t209 + t249 + t663 + r_base(3))) - m(16) * (g(1) * (t642 * rSges(16,1) + t28 * rSges(16,2) - t646 * rSges(16,3) - t646 * t630 + t185 + t223 + t23 + t47 + t615 + r_base(1)) + g(2) * (t654 * rSges(16,1) - t36 * rSges(16,2) - t658 * rSges(16,3) - t658 * t630 - t163 + t197 + t237 + t33 + t651 + r_base(2)) + g(3) * (t666 * rSges(16,1) - t24 * rSges(16,2) - t670 * rSges(16,3) - t670 * t630 + pkin(8) - t175 - t209 + t249 + t663 + r_base(3)));
U = t697;
