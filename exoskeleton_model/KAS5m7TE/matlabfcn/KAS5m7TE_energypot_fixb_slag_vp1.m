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

function U = KAS5m7TE_energypot_fixb_slag_vp1(qJ, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_energypot_fixb_slag_vp1: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7TE_energypot_fixb_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_energypot_fixb_slag_vp1: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7TE_energypot_fixb_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7TE_energypot_fixb_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_worldframe_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-04-13 19:15:02
% EndTime: 2020-04-13 19:19:00
% DurationCPUTime: 207.49s
% Computational Cost: add. (13660860->433), mult. (17634367->506), div. (287358->7), fcn. (7070285->28), ass. (0->179)
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
t45 = cos(qJ(3));
t46 = -pkin(23) + pkin(24);
t47 = t45 * t46;
t48 = pkin(3) + qJ(3);
t49 = cos(t48);
t50 = t49 * pkin(12);
t51 = -pkin(9) + t47 + t50;
t52 = pkin(11) ^ 2;
t53 = pkin(19) ^ 2;
t54 = t49 ^ 2;
t55 = pkin(12) ^ 2;
t57 = sin(t48);
t58 = t57 ^ 2;
t61 = (-pkin(9) + t47) ^ 2;
t62 = sin(qJ(3));
t63 = t62 ^ 2;
t64 = t46 ^ 2;
t68 = t62 * t46;
t69 = t57 * pkin(12);
t70 = -t68 + t69;
t73 = 0.2e1 * t51 * t49 * pkin(12) + 0.2e1 * t70 * t57 * pkin(12) - t54 * t55 - t58 * t55 + t63 * t64 + t52 - t53 + t61;
t77 = 0.4e1 * t51 ^ 2 + 0.4e1 * t70 ^ 2;
t79 = t73 ^ 2;
t81 = sqrt(t52 * t77 - t79);
t84 = 0.1e1 / t77;
t86 = pkin(9) - t47 - t50 + (0.2e1 * t51 * t73 - 0.2e1 * t70 * t81) * t84;
t92 = t68 - t69 + (0.2e1 * t51 * t81 + 0.2e1 * t73 * t70) * t84;
t94 = t45 * t86 - t62 * t92;
t95 = 0.1e1 / pkin(19);
t96 = t94 * t95;
t97 = cos(pkin(7));
t98 = t97 * pkin(18);
t99 = t96 * t98;
t102 = t45 * t92 + t62 * t86;
t103 = t102 * t95;
t104 = sin(pkin(7));
t105 = t104 * pkin(18);
t106 = t103 * t105;
t107 = -pkin(24) - t99 + t106;
t108 = pkin(17) ^ 2;
t109 = pkin(22) ^ 2;
t110 = t99 - t106;
t111 = t110 ^ 2;
t112 = t103 * t98;
t113 = t96 * t105;
t114 = t112 + t113;
t115 = t114 ^ 2;
t116 = pkin(24) ^ 2;
t119 = -0.2e1 * t107 * t110 + 0.2e1 * t114 ^ 2 + t108 - t109 - t111 - t115 + t116;
t123 = 0.4e1 * t107 ^ 2 + 0.4e1 * t114 ^ 2;
t125 = t119 ^ 2;
t127 = sqrt(t108 * t123 - t125);
t130 = 0.1e1 / t123;
t133 = 0.1e1 / pkin(22);
t134 = (-t99 + t106 - (0.2e1 * t107 * t119 + 0.2e1 * t114 * t127) * t130 - pkin(24)) * t133;
t135 = sin(pkin(6));
t142 = (-t112 - t113 - (0.2e1 * t107 * t127 - 0.2e1 * t119 * t114) * t130) * t133;
t143 = cos(pkin(6));
t145 = -t134 * t135 + t142 * t143;
t149 = t134 * t143 + t142 * t135;
t151 = -t22 * t145 + t8 * t149;
t155 = t8 * t145 + t22 * t149;
t160 = t33 * pkin(21);
t163 = t31 * t145 + t6 * t149;
t167 = t6 * t145 - t31 * t149;
t172 = t21 * pkin(21);
t173 = t24 * t145;
t175 = -t24 * t149;
t182 = t151 * pkin(23);
t185 = t151 * t45 + t155 * t62;
t189 = -t151 * t62 + t155 * t45;
t194 = t163 * pkin(23);
t197 = t163 * t45 + t167 * t62;
t201 = -t163 * t62 + t167 * t45;
t206 = t173 * pkin(23);
t209 = -t173 * t45 - t175 * t62;
t213 = t173 * t62 - t175 * t45;
t220 = t185 * pkin(9);
t221 = cos(qJ(4));
t223 = sin(qJ(4));
t225 = t185 * t221 + t189 * t223;
t229 = -t185 * t223 + t189 * t221;
t234 = t197 * pkin(9);
t237 = t197 * t221 + t201 * t223;
t241 = -t197 * t223 + t201 * t221;
t246 = t209 * pkin(9);
t249 = t209 * t221 + t213 * t223;
t253 = -t209 * t223 + t213 * t221;
t260 = t225 * pkin(10);
t261 = qJ(4) - qJ(3) + pkin(4);
t262 = sin(t261);
t264 = cos(t261);
t266 = -t225 * t262 + t229 * t264;
t270 = -t225 * t264 - t229 * t262;
t275 = t237 * pkin(10);
t278 = -t237 * t262 + t241 * t264;
t282 = -t237 * t264 - t241 * t262;
t287 = t249 * pkin(10);
t290 = -t249 * t262 + t253 * t264;
t294 = -t249 * t264 - t253 * t262;
t302 = cos(qJ(5));
t304 = sin(qJ(5));
t343 = t22 * t135 + t8 * t143;
t347 = -t8 * t135 + t22 * t143;
t354 = -t31 * t135 + t6 * t143;
t358 = -t6 * t135 - t31 * t143;
t363 = t24 * t135;
t365 = t24 * t143;
t375 = t149 * t135 + t145 * t143;
t377 = t375 * pkin(22) + t112 + t113;
t379 = 0.1e1 / pkin(17);
t383 = t145 * t135 - t149 * t143;
t385 = -t383 * pkin(22) + pkin(24) - t106 + t99;
t388 = t375 * t377 * t379 - t383 * t385 * t379;
t394 = -t375 * t385 * t379 - t377 * t379 * t383;
t431 = t151 * pkin(24);
t436 = t155 * t102 * t95 + t151 * t94 * t95;
t442 = -t151 * t102 * t95 + t155 * t94 * t95;
t447 = t163 * pkin(24);
t452 = t167 * t102 * t95 + t163 * t94 * t95;
t458 = -t163 * t102 * t95 + t167 * t94 * t95;
t463 = t173 * pkin(24);
t466 = -t175 * t103 - t173 * t96;
t470 = t173 * t103 - t175 * t96;
t482 = t62 * t102 * t95 + t45 * t94 * t95;
t484 = t482 * pkin(19) - pkin(9) + t47 + t50;
t486 = 0.1e1 / pkin(11);
t492 = -t45 * t102 * t95 + t62 * t94 * t95;
t494 = t492 * pkin(19) + t68 - t69;
t497 = -t482 * t484 * t486 - t492 * t494 * t486;
t503 = t482 * t494 * t486 - t484 * t486 * t492;
t542 = t185 * t62 - t189 * t45;
t546 = t185 * t45 + t189 * t62;
t553 = t197 * t62 - t201 * t45;
t557 = t197 * t45 + t201 * t62;
t564 = t209 * t62 - t213 * t45;
t568 = t209 * t45 + t213 * t62;
t575 = sin(pkin(3));
t577 = cos(pkin(3));
t579 = t542 * t575 - t546 * t577;
t583 = t542 * t577 + t546 * t575;
t590 = t553 * t575 - t557 * t577;
t594 = t553 * t577 + t557 * t575;
t601 = t564 * t575 - t568 * t577;
t605 = t564 * t577 + t568 * t575;
t612 = t579 * pkin(12);
t614 = pkin(3) + qJ(3) - qJ(4);
t615 = sin(t614);
t618 = t615 * pkin(12) - t264 * pkin(14) + t262 * pkin(15);
t621 = cos(t614);
t623 = -t621 * pkin(12) - t262 * pkin(14) - t264 * pkin(15) - pkin(10);
t624 = t623 ^ 2;
t625 = t618 ^ 2;
t627 = sqrt(t624 + t625);
t628 = 0.1e1 / t627;
t629 = t618 * t628;
t631 = t623 * t628;
t633 = t631 * t615 + t629 * t621;
t637 = -t629 * t615 + t631 * t621;
t639 = t579 * t633 + t583 * t637;
t643 = -t579 * t637 + t583 * t633;
t648 = t590 * pkin(12);
t651 = t590 * t633 + t594 * t637;
t655 = -t590 * t637 + t594 * t633;
t660 = t601 * pkin(12);
t663 = t601 * t633 + t605 * t637;
t667 = -t601 * t637 + t605 * t633;
t694 = -m(1) * (rSges(1,1) * g(1) + rSges(1,2) * g(2) + g(3) * rSges(1,3)) - m(2) * (g(1) * (t6 * rSges(2,1) + t8 * rSges(2,2)) + g(2) * (-t8 * rSges(2,1) + t6 * rSges(2,2)) + g(3) * (pkin(8) + rSges(2,3))) - m(3) * (g(1) * (-t22 * rSges(3,1) - t25 * rSges(3,2) + t8 * rSges(3,3) + t20) + g(2) * (t31 * rSges(3,1) + t33 * rSges(3,2) + t6 * rSges(3,3) + t30) + g(3) * (-t24 * rSges(3,1) + t21 * rSges(3,2) + pkin(8))) - m(4) * (g(1) * (t151 * rSges(4,1) + t155 * rSges(4,2) + t25 * rSges(4,3) + t20 + t44) + g(2) * (t163 * rSges(4,1) + t167 * rSges(4,2) - t33 * rSges(4,3) - t160 + t30) + g(3) * (-t173 * rSges(4,1) - t175 * rSges(4,2) - t21 * rSges(4,3) + pkin(8) - t172)) - m(5) * (g(1) * (t185 * rSges(5,1) + t189 * rSges(5,2) + t25 * rSges(5,3) + t182 + t20 + t44) + g(2) * (t197 * rSges(5,1) + t201 * rSges(5,2) - t33 * rSges(5,3) - t160 + t194 + t30) + g(3) * (t209 * rSges(5,1) + t213 * rSges(5,2) - t21 * rSges(5,3) + pkin(8) - t172 - t206)) - m(6) * (g(1) * (t225 * rSges(6,1) + t229 * rSges(6,2) + t25 * rSges(6,3) + t182 + t20 + t220 + t44) + g(2) * (t237 * rSges(6,1) + t241 * rSges(6,2) - t33 * rSges(6,3) - t160 + t194 + t234 + t30) + g(3) * (t249 * rSges(6,1) + t253 * rSges(6,2) - t21 * rSges(6,3) + pkin(8) - t172 - t206 + t246)) - m(7) * (g(1) * (t266 * rSges(7,1) + t270 * rSges(7,2) + t25 * rSges(7,3) + t182 + t20 + t220 + t260 + t44) + g(2) * (t278 * rSges(7,1) + t282 * rSges(7,2) - t33 * rSges(7,3) - t160 + t194 + t234 + t275 + t30) + g(3) * (t290 * rSges(7,1) + t294 * rSges(7,2) - t21 * rSges(7,3) + pkin(8) - t172 - t206 + t246 + t287)) - m(8) * (g(1) * (-t270 * pkin(13) + t260 + t220 + t182 + t44 + t20 + (t25 * t304 + t266 * t302) * rSges(8,1) + (t25 * t302 - t266 * t304) * rSges(8,2) - t270 * rSges(8,3)) + g(2) * (-t282 * pkin(13) + t275 + t234 + t194 - t160 + t30 + (t278 * t302 - t33 * t304) * rSges(8,1) + (-t278 * t304 - t33 * t302) * rSges(8,2) - t282 * rSges(8,3)) + g(3) * (-t294 * pkin(13) + t287 + t246 - t206 - t172 + pkin(8) + (-t21 * t304 + t290 * t302) * rSges(8,1) + (-t21 * t302 - t290 * t304) * rSges(8,2) - t294 * rSges(8,3))) - m(9) * (g(1) * (t343 * rSges(9,1) + t347 * rSges(9,2) + t25 * rSges(9,3) + t20 + t44) + g(2) * (t354 * rSges(9,1) + t358 * rSges(9,2) - t33 * rSges(9,3) - t160 + t30) + g(3) * (t363 * rSges(9,1) + t365 * rSges(9,2) - t21 * rSges(9,3) + pkin(8) - t172)) - m(10) * (g(1) * (-t343 * pkin(22) + t44 + t20 + (t343 * t388 + t347 * t394) * rSges(10,1) + (-t343 * t394 + t347 * t388) * rSges(10,2) + t25 * rSges(10,3)) + g(2) * (-t354 * pkin(22) - t160 + t30 + (t354 * t388 + t358 * t394) * rSges(10,1) + (-t354 * t394 + t358 * t388) * rSges(10,2) - t33 * rSges(10,3)) + g(3) * (-t363 * pkin(22) - t172 + pkin(8) + (t363 * t388 + t365 * t394) * rSges(10,1) + (-t363 * t394 + t365 * t388) * rSges(10,2) - t21 * rSges(10,3))) - m(11) * (g(1) * (t436 * rSges(11,1) + t442 * rSges(11,2) + t25 * rSges(11,3) + t20 + t431 + t44) + g(2) * (t452 * rSges(11,1) + t458 * rSges(11,2) - t33 * rSges(11,3) - t160 + t30 + t447) + g(3) * (t466 * rSges(11,1) + t470 * rSges(11,2) - t21 * rSges(11,3) + pkin(8) - t172 - t463)) - m(12) * (g(1) * (t436 * pkin(19) + t431 + t44 + t20 + (t436 * t497 + t442 * t503) * rSges(12,1) + (-t436 * t503 + t442 * t497) * rSges(12,2) + t25 * rSges(12,3)) + g(2) * (t452 * pkin(19) + t447 - t160 + t30 + (t452 * t497 + t458 * t503) * rSges(12,1) + (-t452 * t503 + t458 * t497) * rSges(12,2) - t33 * rSges(12,3)) + g(3) * (t466 * pkin(19) - t463 - t172 + pkin(8) + (t466 * t497 + t470 * t503) * rSges(12,1) + (-t466 * t503 + t470 * t497) * rSges(12,2) - t21 * rSges(12,3))) - m(13) * (g(1) * (t542 * rSges(13,1) + t546 * rSges(13,2) + t25 * rSges(13,3) + t182 + t20 + t220 + t44) + g(2) * (t553 * rSges(13,1) + t557 * rSges(13,2) - t33 * rSges(13,3) - t160 + t194 + t234 + t30) + g(3) * (t564 * rSges(13,1) + t568 * rSges(13,2) - t21 * rSges(13,3) + pkin(8) - t172 - t206 + t246)) - m(14) * (g(1) * (t579 * rSges(14,1) + t583 * rSges(14,2) + t25 * rSges(14,3) + t182 + t20 + t220 + t44) + g(2) * (t590 * rSges(14,1) + t594 * rSges(14,2) - t33 * rSges(14,3) - t160 + t194 + t234 + t30) + g(3) * (t601 * rSges(14,1) + t605 * rSges(14,2) - t21 * rSges(14,3) + pkin(8) - t172 - t206 + t246)) - m(15) * (g(1) * (t639 * rSges(15,1) + t643 * rSges(15,2) + t25 * rSges(15,3) + t182 + t20 + t220 + t44 + t612) + g(2) * (t651 * rSges(15,1) + t655 * rSges(15,2) - t33 * rSges(15,3) - t160 + t194 + t234 + t30 + t648) + g(3) * (t663 * rSges(15,1) + t667 * rSges(15,2) - t21 * rSges(15,3) + pkin(8) - t172 - t206 + t246 + t660)) - m(16) * (g(1) * (t639 * rSges(16,1) + t25 * rSges(16,2) - t643 * rSges(16,3) - t643 * t627 + t182 + t20 + t220 + t44 + t612) + g(2) * (t651 * rSges(16,1) - t33 * rSges(16,2) - t655 * rSges(16,3) - t655 * t627 - t160 + t194 + t234 + t30 + t648) + g(3) * (t663 * rSges(16,1) - t21 * rSges(16,2) - t667 * rSges(16,3) - t667 * t627 + pkin(8) - t172 - t206 + t246 + t660));
U = t694;
