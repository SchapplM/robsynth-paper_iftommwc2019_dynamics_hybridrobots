% Calculate inertial parameters regressor of potential energy for
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
% 
% Output:
% U_reg [1x(5*10)]
%   inertial parameter regressor of Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U_reg = KAS5m7TE_energypot_fixb_reg2_slag_vp(qJ, g, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_energypot_fixb_reg2_slag_vp: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7TE_energypot_fixb_reg2_slag_vp: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_energypot_fixb_reg2_slag_vp: pkin has to be [24x1] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_regressor_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-06 10:39:44
% EndTime: 2020-05-06 10:43:10
% DurationCPUTime: 205.61s
% Computational Cost: add. (13660800->331), mult. (17634346->434), div. (287358->7), fcn. (7070285->28), ass. (0->336)
unknown=NaN(1,150);
t1 = cos(qJ(1));
t2 = g(2) * t1;
t3 = sin(qJ(1));
t4 = g(1) * t3;
t6 = g(2) * t3;
t7 = g(1) * t1;
t8 = -t6 - t7;
t9 = (g(3) * pkin(8));
t10 = cos(qJ(2));
t11 = g(3) * t10;
t12 = sin(qJ(2));
t19 = -g(3) * t12 - t2 * t10 + t4 * t10;
t23 = cos(qJ(3));
t24 = -pkin(23) + pkin(24);
t25 = t23 * t24;
t26 = pkin(3) + qJ(3);
t27 = cos(t26);
t28 = t27 * pkin(12);
t29 = -pkin(9) + t25 + t28;
t30 = pkin(11) ^ 2;
t31 = pkin(19) ^ 2;
t32 = t27 ^ 2;
t33 = pkin(12) ^ 2;
t35 = sin(t26);
t36 = t35 ^ 2;
t39 = (-pkin(9) + t25) ^ 2;
t40 = sin(qJ(3));
t41 = t40 ^ 2;
t42 = t24 ^ 2;
t46 = t40 * t24;
t47 = t35 * pkin(12);
t48 = -t46 + t47;
t51 = 0.2e1 * t29 * t27 * pkin(12) + 0.2e1 * t48 * t35 * pkin(12) - t32 * t33 - t36 * t33 + t41 * t42 + t30 - t31 + t39;
t55 = 0.4e1 * t29 ^ 2 + 0.4e1 * t48 ^ 2;
t57 = t51 ^ 2;
t59 = sqrt(t30 * t55 - t57);
t62 = 0.1e1 / t55;
t64 = pkin(9) - t25 - t28 + (0.2e1 * t29 * t51 - 0.2e1 * t48 * t59) * t62;
t70 = t46 - t47 + (0.2e1 * t29 * t59 + 0.2e1 * t51 * t48) * t62;
t72 = t23 * t64 - t40 * t70;
t73 = 0.1e1 / pkin(19);
t74 = t72 * t73;
t75 = cos(pkin(7));
t76 = t75 * pkin(18);
t77 = t74 * t76;
t80 = t23 * t70 + t40 * t64;
t81 = t80 * t73;
t82 = sin(pkin(7));
t83 = t82 * pkin(18);
t84 = t81 * t83;
t85 = -pkin(24) - t77 + t84;
t86 = pkin(17) ^ 2;
t87 = pkin(22) ^ 2;
t88 = t77 - t84;
t89 = t88 ^ 2;
t90 = t81 * t76;
t91 = t74 * t83;
t92 = t90 + t91;
t93 = t92 ^ 2;
t94 = pkin(24) ^ 2;
t97 = -0.2e1 * t85 * t88 + 0.2e1 * t92 ^ 2 + t86 - t87 - t89 - t93 + t94;
t101 = 0.4e1 * t85 ^ 2 + 0.4e1 * t92 ^ 2;
t103 = t97 ^ 2;
t105 = sqrt(t86 * t101 - t103);
t108 = 0.1e1 / t101;
t111 = 0.1e1 / pkin(22);
t112 = (-t77 + t84 - (0.2e1 * t92 * t105 + 0.2e1 * t85 * t97) * t108 - pkin(24)) * t111;
t113 = sin(pkin(6));
t120 = (-t90 - t91 - (0.2e1 * t85 * t105 - 0.2e1 * t97 * t92) * t108) * t111;
t121 = cos(pkin(6));
t123 = -t112 * t113 + t120 * t121;
t125 = t1 * t12;
t129 = t112 * t121 + t120 * t113;
t131 = t125 * t123 + t3 * t129;
t133 = t3 * t12;
t136 = t1 * t129 - t133 * t123;
t142 = t3 * t123 - t125 * t129;
t146 = t1 * t123 + t133 * t129;
t149 = t12 * pkin(21);
t152 = t1 * t10;
t153 = t152 * pkin(21);
t154 = t3 * pkin(16);
t157 = t3 * t10;
t158 = t157 * pkin(21);
t159 = t1 * pkin(16);
t162 = -g(3) * (-t149 + pkin(8)) - g(2) * (-t153 + t154) - g(1) * (t158 + t159);
t163 = t10 * t123;
t165 = -t10 * t129;
t167 = -t163 * t23 - t165 * t40;
t171 = t131 * t23 + t142 * t40;
t175 = t136 * t23 + t146 * t40;
t180 = t163 * t40 - t165 * t23;
t184 = -t131 * t40 + t142 * t23;
t188 = -t136 * t40 + t146 * t23;
t191 = t163 * pkin(23);
t194 = t131 * pkin(23);
t197 = t136 * pkin(23);
t201 = cos(qJ(4));
t203 = sin(qJ(4));
t205 = t167 * t201 + t180 * t203;
t209 = t171 * t201 + t184 * t203;
t213 = t175 * t201 + t188 * t203;
t218 = -t167 * t203 + t180 * t201;
t222 = -t171 * t203 + t184 * t201;
t226 = -t175 * t203 + t188 * t201;
t229 = t167 * pkin(9);
t232 = t171 * pkin(9);
t235 = t175 * pkin(9);
t238 = -g(3) * (t229 - t191 - t149 + pkin(8)) - g(2) * (t232 + t194 - t153 + t154) - g(1) * (t235 + t197 + t158 + t159);
t239 = qJ(4) - qJ(3) + pkin(4);
t240 = sin(t239);
t242 = cos(t239);
t244 = -t205 * t240 + t218 * t242;
t248 = -t209 * t240 + t222 * t242;
t252 = -t213 * t240 + t226 * t242;
t257 = -t205 * t242 - t218 * t240;
t261 = -t209 * t242 - t222 * t240;
t265 = -t213 * t242 - t226 * t240;
t268 = t205 * pkin(10);
t271 = t209 * pkin(10);
t274 = t213 * pkin(10);
t278 = cos(qJ(5));
t280 = sin(qJ(5));
t323 = -t125 * t113 + t3 * t121;
t327 = t1 * t121 + t133 * t113;
t333 = -t3 * t113 - t125 * t121;
t337 = -t1 * t113 + t133 * t121;
t340 = t10 * t113;
t343 = t129 * t113 + t123 * t121;
t345 = t343 * pkin(22) + t90 + t91;
t347 = 0.1e1 / pkin(17);
t351 = t123 * t113 - t129 * t121;
t353 = -t351 * pkin(22) + pkin(24) + t77 - t84;
t356 = t343 * t345 * t347 - t351 * t353 * t347;
t358 = t10 * t121;
t363 = -t343 * t353 * t347 - t345 * t347 * t351;
t401 = -t163 * t74 - t165 * t81;
t407 = t131 * t72 * t73 + t142 * t80 * t73;
t413 = t136 * t72 * t73 + t146 * t80 * t73;
t418 = t163 * t81 - t165 * t74;
t424 = -t131 * t80 * t73 + t142 * t72 * t73;
t430 = -t136 * t80 * t73 + t146 * t72 * t73;
t433 = t163 * pkin(24);
t436 = t131 * pkin(24);
t439 = t136 * pkin(24);
t447 = t23 * t72 * t73 + t40 * t80 * t73;
t449 = t447 * pkin(19) - pkin(9) + t25 + t28;
t451 = 0.1e1 / pkin(11);
t457 = -t23 * t80 * t73 + t40 * t72 * t73;
t459 = t457 * pkin(19) + t46 - t47;
t462 = -t447 * t449 * t451 - t457 * t459 * t451;
t468 = t447 * t459 * t451 - t449 * t451 * t457;
t506 = t167 * t40 - t180 * t23;
t510 = t171 * t40 - t184 * t23;
t514 = t175 * t40 - t188 * t23;
t519 = t167 * t23 + t180 * t40;
t523 = t171 * t23 + t184 * t40;
t527 = t175 * t23 + t188 * t40;
t530 = sin(pkin(3));
t532 = cos(pkin(3));
t534 = t506 * t530 - t519 * t532;
t538 = t510 * t530 - t523 * t532;
t542 = t514 * t530 - t527 * t532;
t547 = t506 * t532 + t519 * t530;
t551 = t510 * t532 + t523 * t530;
t555 = t514 * t532 + t527 * t530;
t559 = pkin(3) + qJ(3) - qJ(4);
t560 = sin(t559);
t563 = t560 * pkin(12) - t242 * pkin(14) + t240 * pkin(15);
t566 = cos(t559);
t568 = -t566 * pkin(12) - t240 * pkin(14) - t242 * pkin(15) - pkin(10);
t569 = t568 ^ 2;
t570 = t563 ^ 2;
t572 = sqrt(t569 + t570);
t573 = 0.1e1 / t572;
t574 = t563 * t573;
t576 = t568 * t573;
t578 = t576 * t560 + t574 * t566;
t582 = -t574 * t560 + t576 * t566;
t594 = -g(3) * (t534 * t578 + t547 * t582) - g(2) * (t538 * t578 + t551 * t582) - g(1) * (t542 * t578 + t555 * t582);
t597 = -t534 * t582 + t547 * t578;
t601 = -t538 * t582 + t551 * t578;
t605 = -t542 * t582 + t555 * t578;
t608 = t534 * pkin(12);
t611 = t538 * pkin(12);
t614 = t542 * pkin(12);
unknown(1,1) = 0;
unknown(1,2) = 0;
unknown(1,3) = 0;
unknown(1,4) = 0;
unknown(1,5) = 0;
unknown(1,6) = 0;
unknown(1,7) = (t2 - t4);
unknown(1,8) = t8;
unknown(1,9) = -g(3);
unknown(1,10) = -t9;
unknown(1,11) = 0;
unknown(1,12) = 0;
unknown(1,13) = 0;
unknown(1,14) = 0;
unknown(1,15) = 0;
unknown(1,16) = 0;
unknown(1,17) = (-t2 * t12 + t4 * t12 + t11);
unknown(1,18) = t19;
unknown(1,19) = t8;
unknown(1,20) = (-t6 * pkin(16) - t7 * pkin(16) - t9);
unknown(1,21) = 0;
unknown(1,22) = 0;
unknown(1,23) = 0;
unknown(1,24) = 0;
unknown(1,25) = 0;
unknown(1,26) = 0;
unknown(1,27) = (-g(1) * t136 - g(2) * t131 + t11 * t123);
unknown(1,28) = (-g(1) * t146 - g(2) * t142 - t11 * t129);
unknown(1,29) = -t19;
unknown(1,30) = t162;
unknown(1,31) = 0;
unknown(1,32) = 0;
unknown(1,33) = 0;
unknown(1,34) = 0;
unknown(1,35) = 0;
unknown(1,36) = 0;
unknown(1,37) = (-g(1) * t175 - g(2) * t171 - g(3) * t167);
unknown(1,38) = (-g(1) * t188 - g(2) * t184 - g(3) * t180);
unknown(1,39) = -t19;
unknown(1,40) = (-g(3) * (-t191 - t149 + pkin(8)) - g(2) * (t194 - t153 + t154) - g(1) * (t197 + t158 + t159));
unknown(1,41) = 0;
unknown(1,42) = 0;
unknown(1,43) = 0;
unknown(1,44) = 0;
unknown(1,45) = 0;
unknown(1,46) = 0;
unknown(1,47) = (-g(1) * t213 - g(2) * t209 - g(3) * t205);
unknown(1,48) = (-g(1) * t226 - g(2) * t222 - g(3) * t218);
unknown(1,49) = -t19;
unknown(1,50) = t238;
unknown(1,51) = 0;
unknown(1,52) = 0;
unknown(1,53) = 0;
unknown(1,54) = 0;
unknown(1,55) = 0;
unknown(1,56) = 0;
unknown(1,57) = (-g(1) * t252 - g(2) * t248 - g(3) * t244);
unknown(1,58) = (-g(1) * t265 - g(2) * t261 - g(3) * t257);
unknown(1,59) = -t19;
unknown(1,60) = (-g(3) * (t268 + t229 - t191 - t149 + pkin(8)) - g(2) * (t271 + t232 + t194 - t153 + t154) - g(1) * (t274 + t235 + t197 + t158 + t159));
unknown(1,61) = 0;
unknown(1,62) = 0;
unknown(1,63) = 0;
unknown(1,64) = 0;
unknown(1,65) = 0;
unknown(1,66) = 0;
unknown(1,67) = (-g(3) * (-t12 * t280 + t244 * t278) - g(2) * (-t152 * t280 + t248 * t278) - g(1) * (t157 * t280 + t252 * t278));
unknown(1,68) = (-g(3) * (-t12 * t278 - t244 * t280) - g(2) * (-t152 * t278 - t248 * t280) - g(1) * (t157 * t278 - t252 * t280));
unknown(1,69) = (g(1) * t265 + g(2) * t261 + g(3) * t257);
unknown(1,70) = (-g(3) * (-t257 * pkin(13) + pkin(8) - t149 - t191 + t229 + t268) - g(2) * (-t261 * pkin(13) - t153 + t154 + t194 + t232 + t271) - g(1) * (-t265 * pkin(13) + t158 + t159 + t197 + t235 + t274));
unknown(1,71) = 0;
unknown(1,72) = 0;
unknown(1,73) = 0;
unknown(1,74) = 0;
unknown(1,75) = 0;
unknown(1,76) = 0;
unknown(1,77) = (-g(1) * t327 - g(2) * t323 - t11 * t113);
unknown(1,78) = (-g(1) * t337 - g(2) * t333 - t11 * t121);
unknown(1,79) = -t19;
unknown(1,80) = t162;
unknown(1,81) = 0;
unknown(1,82) = 0;
unknown(1,83) = 0;
unknown(1,84) = 0;
unknown(1,85) = 0;
unknown(1,86) = 0;
unknown(1,87) = (-g(3) * (t340 * t356 + t358 * t363) - g(2) * (t323 * t356 + t333 * t363) - g(1) * (t327 * t356 + t337 * t363));
unknown(1,88) = (-g(3) * (-t340 * t363 + t358 * t356) - g(2) * (-t323 * t363 + t333 * t356) - g(1) * (-t327 * t363 + t337 * t356));
unknown(1,89) = -t19;
unknown(1,90) = (-g(3) * (-t340 * pkin(22) + pkin(8) - t149) - g(2) * (-t323 * pkin(22) - t153 + t154) - g(1) * (-t327 * pkin(22) + t158 + t159));
unknown(1,91) = 0;
unknown(1,92) = 0;
unknown(1,93) = 0;
unknown(1,94) = 0;
unknown(1,95) = 0;
unknown(1,96) = 0;
unknown(1,97) = (-g(1) * t413 - g(2) * t407 - g(3) * t401);
unknown(1,98) = (-g(1) * t430 - g(2) * t424 - g(3) * t418);
unknown(1,99) = -t19;
unknown(1,100) = (-g(3) * (-t433 - t149 + pkin(8)) - g(2) * (t436 - t153 + t154) - g(1) * (t439 + t158 + t159));
unknown(1,101) = 0;
unknown(1,102) = 0;
unknown(1,103) = 0;
unknown(1,104) = 0;
unknown(1,105) = 0;
unknown(1,106) = 0;
unknown(1,107) = (-g(3) * (t401 * t462 + t418 * t468) - g(2) * (t407 * t462 + t424 * t468) - g(1) * (t413 * t462 + t430 * t468));
unknown(1,108) = (-g(3) * (-t401 * t468 + t418 * t462) - g(2) * (-t407 * t468 + t424 * t462) - g(1) * (-t413 * t468 + t430 * t462));
unknown(1,109) = -t19;
unknown(1,110) = (-g(3) * (t401 * pkin(19) + pkin(8) - t149 - t433) - g(2) * (t407 * pkin(19) - t153 + t154 + t436) - g(1) * (t413 * pkin(19) + t158 + t159 + t439));
unknown(1,111) = 0;
unknown(1,112) = 0;
unknown(1,113) = 0;
unknown(1,114) = 0;
unknown(1,115) = 0;
unknown(1,116) = 0;
unknown(1,117) = (-g(1) * t514 - g(2) * t510 - g(3) * t506);
unknown(1,118) = (-g(1) * t527 - g(2) * t523 - g(3) * t519);
unknown(1,119) = -t19;
unknown(1,120) = t238;
unknown(1,121) = 0;
unknown(1,122) = 0;
unknown(1,123) = 0;
unknown(1,124) = 0;
unknown(1,125) = 0;
unknown(1,126) = 0;
unknown(1,127) = (-g(1) * t542 - g(2) * t538 - g(3) * t534);
unknown(1,128) = (-g(1) * t555 - g(2) * t551 - g(3) * t547);
unknown(1,129) = -t19;
unknown(1,130) = t238;
unknown(1,131) = 0;
unknown(1,132) = 0;
unknown(1,133) = 0;
unknown(1,134) = 0;
unknown(1,135) = 0;
unknown(1,136) = 0;
unknown(1,137) = t594;
unknown(1,138) = (-g(1) * t605 - g(2) * t601 - g(3) * t597);
unknown(1,139) = -t19;
unknown(1,140) = (-g(3) * (t608 + t229 - t191 - t149 + pkin(8)) - g(2) * (t611 + t232 + t194 - t153 + t154) - g(1) * (t614 + t235 + t197 + t158 + t159));
unknown(1,141) = 0;
unknown(1,142) = 0;
unknown(1,143) = 0;
unknown(1,144) = 0;
unknown(1,145) = 0;
unknown(1,146) = 0;
unknown(1,147) = t594;
unknown(1,148) = -t19;
unknown(1,149) = (g(1) * t605 + g(2) * t601 + g(3) * t597);
unknown(1,150) = (-g(3) * (-t597 * t572 + pkin(8) - t149 - t191 + t229 + t608) - g(2) * (-t601 * t572 - t153 + t154 + t194 + t232 + t611) - g(1) * (-t605 * t572 + t158 + t159 + t197 + t235 + t614));
U_reg = unknown;
