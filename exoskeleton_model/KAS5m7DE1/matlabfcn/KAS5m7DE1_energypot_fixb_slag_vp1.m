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

function U = KAS5m7DE1_energypot_fixb_slag_vp1(qJ, g, ...
  pkin, m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp1: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp1: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp1: pkin has to be [24x1] (double)');
assert(isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp1: m has to be [16x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [16,3]), ...
  'KAS5m7DE1_energypot_fixb_slag_vp1: rSges has to be [16x3] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_worldframe_par1_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-14 20:20:38
% EndTime: 2020-05-14 20:23:53
% DurationCPUTime: 195.27s
% Computational Cost: add. (13265256->435), mult. (17122075->476), div. (276600->4), fcn. (6865757->43), ass. (0->181)
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
t45 = sin(qJ(3));
t46 = cos(qJ(3));
t47 = -pkin(23) + pkin(24);
t48 = t46 * t47;
t49 = pkin(3) + qJ(3);
t50 = cos(t49);
t51 = t50 * pkin(12);
t52 = -pkin(9) + t48 + t51;
t53 = pkin(11) ^ 2;
t54 = pkin(19) ^ 2;
t55 = t50 ^ 2;
t56 = pkin(12) ^ 2;
t58 = sin(t49);
t59 = t58 ^ 2;
t62 = (-pkin(9) + t48) ^ 2;
t63 = t45 ^ 2;
t64 = t47 ^ 2;
t68 = t45 * t47;
t69 = t58 * pkin(12);
t70 = -t68 + t69;
t73 = 0.2e1 * pkin(12) * t50 * t52 + 0.2e1 * pkin(12) * t58 * t70 - t55 * t56 - t56 * t59 + t63 * t64 + t53 - t54 + t62;
t77 = 0.4e1 * t52 ^ 2 + 0.4e1 * t70 ^ 2;
t79 = t73 ^ 2;
t81 = sqrt(t53 * t77 - t79);
t84 = 0.1e1 / t77;
t86 = pkin(9) - t48 - t51 + (0.2e1 * t52 * t73 - 0.2e1 * t70 * t81) * t84;
t92 = t68 - t69 + (0.2e1 * t52 * t81 + 0.2e1 * t70 * t73) * t84;
t94 = t45 * t86 + t46 * t92;
t95 = 0.1e1 / pkin(19);
t96 = t94 * t95;
t97 = cos(pkin(7));
t98 = t97 * pkin(18);
t99 = t96 * t98;
t102 = -t45 * t92 + t46 * t86;
t103 = t102 * t95;
t104 = sin(pkin(7));
t105 = t104 * pkin(18);
t106 = t103 * t105;
t107 = pkin(17) ^ 2;
t108 = pkin(22) ^ 2;
t109 = t103 * t98;
t110 = t96 * t105;
t111 = t109 - t110;
t112 = t111 ^ 2;
t113 = t99 + t106;
t114 = t113 ^ 2;
t115 = pkin(24) ^ 2;
t116 = -pkin(24) - t109 + t110;
t119 = -0.2e1 * t111 * t116 + 0.2e1 * t113 ^ 2 + t107 - t108 - t112 - t114 + t115;
t123 = 0.4e1 * t113 ^ 2 + 0.4e1 * t116 ^ 2;
t125 = t119 ^ 2;
t127 = sqrt(t107 * t123 - t125);
t130 = 0.1e1 / t123;
t132 = -t99 - t106 - (-0.2e1 * t113 * t119 + 0.2e1 * t116 * t127) * t130;
t137 = t109 - t110 + (0.2e1 * t113 * t127 + 0.2e1 * t116 * t119) * t130 + pkin(24);
t138 = atan2(t132, t137);
t139 = t138 + pkin(6);
t140 = sin(t139);
t142 = cos(t139);
t144 = -t140 * t22 - t142 * t8;
t148 = t140 * t8 - t142 * t22;
t153 = t33 * pkin(21);
t156 = t140 * t31 - t142 * t6;
t160 = t140 * t6 + t142 * t31;
t165 = t21 * pkin(21);
t166 = t24 * t140;
t168 = t24 * t142;
t175 = t144 * pkin(23);
t178 = t144 * t46 + t148 * t45;
t182 = -t144 * t45 + t148 * t46;
t187 = t156 * pkin(23);
t190 = t156 * t46 + t160 * t45;
t194 = -t156 * t45 + t160 * t46;
t199 = t166 * pkin(23);
t202 = -t166 * t46 - t168 * t45;
t206 = t166 * t45 - t168 * t46;
t213 = t178 * pkin(9);
t214 = cos(qJ(4));
t216 = sin(qJ(4));
t218 = t178 * t214 + t182 * t216;
t222 = -t178 * t216 + t182 * t214;
t227 = t190 * pkin(9);
t230 = t190 * t214 + t194 * t216;
t234 = -t190 * t216 + t194 * t214;
t239 = t202 * pkin(9);
t242 = t202 * t214 + t206 * t216;
t246 = -t202 * t216 + t206 * t214;
t253 = t218 * pkin(10);
t254 = qJ(4) - qJ(3) + pkin(4);
t255 = sin(t254);
t257 = cos(t254);
t259 = -t218 * t255 + t222 * t257;
t263 = -t218 * t257 - t222 * t255;
t268 = t230 * pkin(10);
t271 = -t230 * t255 + t234 * t257;
t275 = -t230 * t257 - t234 * t255;
t280 = t242 * pkin(10);
t283 = -t242 * t255 + t246 * t257;
t287 = -t242 * t257 - t246 * t255;
t295 = cos(qJ(5));
t297 = sin(qJ(5));
t334 = sin(pkin(6));
t336 = cos(pkin(6));
t338 = t22 * t334 + t336 * t8;
t342 = t22 * t336 - t334 * t8;
t349 = -t31 * t334 + t336 * t6;
t353 = -t31 * t336 - t334 * t6;
t358 = t24 * t334;
t360 = t24 * t336;
t368 = 0.1e1 / pkin(22);
t369 = t132 * t368;
t371 = -t137 * t368;
t373 = t334 * t369 + t336 * t371;
t377 = -t334 * t371 + t336 * t369;
t387 = atan2(-(t334 * t377 - t336 * t373) * pkin(22) + pkin(24) + t109 - t110, (t334 * t373 + t336 * t377) * pkin(22) + t99 + t106);
t388 = t138 - t387;
t389 = sin(t388);
t391 = cos(t388);
t428 = t144 * pkin(24);
t429 = atan2(-t94, t102);
t430 = cos(t429);
t432 = sin(t429);
t434 = t144 * t430 - t148 * t432;
t438 = t144 * t432 + t148 * t430;
t443 = t156 * pkin(24);
t446 = t156 * t430 - t160 * t432;
t450 = t156 * t432 + t160 * t430;
t455 = t166 * pkin(24);
t458 = -t166 * t430 + t168 * t432;
t462 = -t166 * t432 - t168 * t430;
t484 = atan2((t102 * t45 * t95 - t46 * t94 * t95) * pkin(19) + t68 - t69, (t102 * t46 * t95 + t45 * t94 * t95) * pkin(19) - pkin(9) + t48 + t51);
t485 = -t484 + qJ(3) + t429;
t486 = cos(t485);
t488 = sin(t485);
t527 = t178 * t45 - t182 * t46;
t531 = t178 * t46 + t182 * t45;
t538 = t190 * t45 - t194 * t46;
t542 = t190 * t46 + t194 * t45;
t549 = t202 * t45 - t206 * t46;
t553 = t202 * t46 + t206 * t45;
t560 = sin(pkin(3));
t562 = cos(pkin(3));
t564 = t527 * t560 - t531 * t562;
t568 = t527 * t562 + t531 * t560;
t575 = t538 * t560 - t542 * t562;
t579 = t538 * t562 + t542 * t560;
t586 = t549 * t560 - t553 * t562;
t590 = t549 * t562 + t553 * t560;
t597 = t564 * pkin(12);
t599 = pkin(3) + qJ(3) - qJ(4);
t600 = sin(t599);
t603 = pkin(12) * t600 - pkin(14) * t257 + pkin(15) * t255;
t606 = cos(t599);
t608 = -pkin(12) * t606 - pkin(14) * t255 - pkin(15) * t257 - pkin(10);
t609 = atan2(t603, t608);
t610 = t609 + pkin(3) + qJ(3) - qJ(4);
t611 = sin(t610);
t613 = cos(t610);
t615 = t564 * t611 + t568 * t613;
t619 = -t564 * t613 + t568 * t611;
t624 = t575 * pkin(12);
t627 = t575 * t611 + t579 * t613;
t631 = -t575 * t613 + t579 * t611;
t636 = t586 * pkin(12);
t639 = t586 * t611 + t590 * t613;
t643 = -t586 * t613 + t590 * t611;
t650 = t608 ^ 2;
t651 = t603 ^ 2;
t653 = sqrt(t650 + t651);
t674 = -m(1) * (rSges(1,1) * g(1) + rSges(1,2) * g(2) + rSges(1,3) * g(3)) - m(2) * (g(1) * (rSges(2,1) * t6 + rSges(2,2) * t8) + g(2) * (-rSges(2,1) * t8 + rSges(2,2) * t6) + g(3) * (pkin(8) + rSges(2,3))) - m(3) * (g(1) * (-rSges(3,1) * t22 - rSges(3,2) * t25 + rSges(3,3) * t8 + t20) + g(2) * (rSges(3,1) * t31 + rSges(3,2) * t33 + rSges(3,3) * t6 + t30) + g(3) * (-rSges(3,1) * t24 + rSges(3,2) * t21 + pkin(8))) - m(4) * (g(1) * (rSges(4,1) * t144 + rSges(4,2) * t148 + rSges(4,3) * t25 + t20 + t44) + g(2) * (rSges(4,1) * t156 + rSges(4,2) * t160 - rSges(4,3) * t33 - t153 + t30) + g(3) * (-rSges(4,1) * t166 - rSges(4,2) * t168 - rSges(4,3) * t21 + pkin(8) - t165)) - m(5) * (g(1) * (rSges(5,1) * t178 + rSges(5,2) * t182 + rSges(5,3) * t25 + t175 + t20 + t44) + g(2) * (rSges(5,1) * t190 + rSges(5,2) * t194 - rSges(5,3) * t33 - t153 + t187 + t30) + g(3) * (rSges(5,1) * t202 + rSges(5,2) * t206 - rSges(5,3) * t21 + pkin(8) - t165 - t199)) - m(6) * (g(1) * (rSges(6,1) * t218 + rSges(6,2) * t222 + rSges(6,3) * t25 + t175 + t20 + t213 + t44) + g(2) * (rSges(6,1) * t230 + rSges(6,2) * t234 - rSges(6,3) * t33 - t153 + t187 + t227 + t30) + g(3) * (rSges(6,1) * t242 + rSges(6,2) * t246 - rSges(6,3) * t21 + pkin(8) - t165 - t199 + t239)) - m(7) * (g(1) * (rSges(7,1) * t259 + rSges(7,2) * t263 + rSges(7,3) * t25 + t175 + t20 + t213 + t253 + t44) + g(2) * (rSges(7,1) * t271 + rSges(7,2) * t275 - rSges(7,3) * t33 - t153 + t187 + t227 + t268 + t30) + g(3) * (rSges(7,1) * t283 + rSges(7,2) * t287 - rSges(7,3) * t21 + pkin(8) - t165 - t199 + t239 + t280)) - m(8) * (g(1) * (-t263 * pkin(13) + t253 + t213 + t175 + t44 + t20 + (t25 * t297 + t259 * t295) * rSges(8,1) + (t25 * t295 - t259 * t297) * rSges(8,2) - t263 * rSges(8,3)) + g(2) * (-t275 * pkin(13) + t268 + t227 + t187 - t153 + t30 + (t271 * t295 - t297 * t33) * rSges(8,1) + (-t271 * t297 - t295 * t33) * rSges(8,2) - t275 * rSges(8,3)) + g(3) * (-t287 * pkin(13) + t280 + t239 - t199 - t165 + pkin(8) + (-t21 * t297 + t283 * t295) * rSges(8,1) + (-t21 * t295 - t283 * t297) * rSges(8,2) - t287 * rSges(8,3))) - m(9) * (g(1) * (rSges(9,1) * t338 + rSges(9,2) * t342 + rSges(9,3) * t25 + t20 + t44) + g(2) * (rSges(9,1) * t349 + rSges(9,2) * t353 - rSges(9,3) * t33 - t153 + t30) + g(3) * (rSges(9,1) * t358 + rSges(9,2) * t360 - rSges(9,3) * t21 + pkin(8) - t165)) - m(10) * (g(1) * (-t338 * pkin(22) + t44 + t20 + (t338 * t389 - t342 * t391) * rSges(10,1) + (t338 * t391 + t342 * t389) * rSges(10,2) + t25 * rSges(10,3)) + g(2) * (-t349 * pkin(22) - t153 + t30 + (t349 * t389 - t353 * t391) * rSges(10,1) + (t349 * t391 + t353 * t389) * rSges(10,2) - t33 * rSges(10,3)) + g(3) * (-t358 * pkin(22) - t165 + pkin(8) + (t358 * t389 - t360 * t391) * rSges(10,1) + (t358 * t391 + t360 * t389) * rSges(10,2) - t21 * rSges(10,3))) - m(11) * (g(1) * (rSges(11,1) * t434 + rSges(11,2) * t438 + rSges(11,3) * t25 + t20 + t428 + t44) + g(2) * (rSges(11,1) * t446 + rSges(11,2) * t450 - rSges(11,3) * t33 - t153 + t30 + t443) + g(3) * (rSges(11,1) * t458 + rSges(11,2) * t462 - rSges(11,3) * t21 + pkin(8) - t165 - t455)) - m(12) * (g(1) * (t434 * pkin(19) + t428 + t44 + t20 + (-t434 * t486 - t438 * t488) * rSges(12,1) + (t434 * t488 - t438 * t486) * rSges(12,2) + t25 * rSges(12,3)) + g(2) * (t446 * pkin(19) + t443 - t153 + t30 + (-t446 * t486 - t450 * t488) * rSges(12,1) + (t446 * t488 - t450 * t486) * rSges(12,2) - t33 * rSges(12,3)) + g(3) * (t458 * pkin(19) - t455 - t165 + pkin(8) + (-t458 * t486 - t462 * t488) * rSges(12,1) + (t458 * t488 - t462 * t486) * rSges(12,2) - t21 * rSges(12,3))) - m(13) * (g(1) * (rSges(13,1) * t527 + rSges(13,2) * t531 + rSges(13,3) * t25 + t175 + t20 + t213 + t44) + g(2) * (rSges(13,1) * t538 + rSges(13,2) * t542 - rSges(13,3) * t33 - t153 + t187 + t227 + t30) + g(3) * (rSges(13,1) * t549 + rSges(13,2) * t553 - rSges(13,3) * t21 + pkin(8) - t165 - t199 + t239)) - m(14) * (g(1) * (rSges(14,1) * t564 + rSges(14,2) * t568 + rSges(14,3) * t25 + t175 + t20 + t213 + t44) + g(2) * (rSges(14,1) * t575 + rSges(14,2) * t579 - rSges(14,3) * t33 - t153 + t187 + t227 + t30) + g(3) * (rSges(14,1) * t586 + rSges(14,2) * t590 - rSges(14,3) * t21 + pkin(8) - t165 - t199 + t239)) - m(15) * (g(1) * (rSges(15,1) * t615 + rSges(15,2) * t619 + rSges(15,3) * t25 + t175 + t20 + t213 + t44 + t597) + g(2) * (rSges(15,1) * t627 + rSges(15,2) * t631 - rSges(15,3) * t33 - t153 + t187 + t227 + t30 + t624) + g(3) * (rSges(15,1) * t639 + rSges(15,2) * t643 - rSges(15,3) * t21 + pkin(8) - t165 - t199 + t239 + t636)) - m(16) * (g(1) * (rSges(16,1) * t615 + rSges(16,2) * t25 - rSges(16,3) * t619 - t619 * t653 + t175 + t20 + t213 + t44 + t597) + g(2) * (rSges(16,1) * t627 - rSges(16,2) * t33 - rSges(16,3) * t631 - t631 * t653 - t153 + t187 + t227 + t30 + t624) + g(3) * (rSges(16,1) * t639 - rSges(16,2) * t21 - rSges(16,3) * t643 - t643 * t653 + pkin(8) - t165 - t199 + t239 + t636));
U = t674;
