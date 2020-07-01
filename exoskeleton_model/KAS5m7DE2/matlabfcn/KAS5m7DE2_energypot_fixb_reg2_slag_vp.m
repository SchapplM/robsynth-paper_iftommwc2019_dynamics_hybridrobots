% Calculate inertial parameters regressor of potential energy for
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
% 
% Output:
% U_reg [1x(5*10)]
%   inertial parameter regressor of Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-03 15:49
% Revision: caa0dbda1e8a16d11faaa29ba3bbef6afcd619f7 (2020-05-25)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U_reg = KAS5m7DE2_energypot_fixb_reg2_slag_vp(qJ, g, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(3,1),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE2_energypot_fixb_reg2_slag_vp: qJ has to be [5x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7DE2_energypot_fixb_reg2_slag_vp: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE2_energypot_fixb_reg2_slag_vp: pkin has to be [24x1] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_regressor_matlab.m
% OptimizationMode: 1
% StartTime: 2020-05-31 07:57:50
% EndTime: 2020-05-31 07:58:37
% DurationCPUTime: 47.49s
% Computational Cost: add. (3160133->333), mult. (4078197->396), div. (65930->4), fcn. (1635230->49), ass. (0->323)
unknown=NaN(1,150);
t1 = sin(qJ(1));
t2 = g(1) * t1;
t3 = cos(qJ(1));
t4 = g(2) * t3;
t6 = g(1) * t3;
t7 = g(2) * t1;
t8 = -t6 - t7;
t9 = (g(3) * pkin(8));
t10 = cos(qJ(2));
t11 = g(3) * t10;
t12 = sin(qJ(2));
t19 = -g(3) * t12 + t10 * t2 - t10 * t4;
t23 = sin(qJ(3));
t24 = cos(qJ(3));
t25 = -pkin(23) + pkin(24);
t26 = t24 * t25;
t27 = pkin(3) + qJ(3);
t28 = cos(t27);
t29 = t28 * pkin(12);
t30 = -pkin(9) + t26 + t29;
t31 = pkin(11) ^ 2;
t32 = pkin(19) ^ 2;
t33 = t28 ^ 2;
t34 = pkin(12) ^ 2;
t36 = sin(t27);
t37 = t36 ^ 2;
t40 = (-pkin(9) + t26) ^ 2;
t41 = t23 ^ 2;
t42 = t25 ^ 2;
t46 = t23 * t25;
t47 = t36 * pkin(12);
t48 = -t46 + t47;
t51 = 0.2e1 * pkin(12) * t28 * t30 + 0.2e1 * pkin(12) * t36 * t48 - t33 * t34 - t34 * t37 + t41 * t42 + t31 - t32 + t40;
t55 = 0.4e1 * t30 ^ 2 + 0.4e1 * t48 ^ 2;
t57 = t51 ^ 2;
t59 = sqrt(t31 * t55 - t57);
t62 = 0.1e1 / t55;
t64 = pkin(9) - t26 - t29 + (0.2e1 * t30 * t51 - 0.2e1 * t48 * t59) * t62;
t70 = t46 - t47 + (0.2e1 * t30 * t59 + 0.2e1 * t48 * t51) * t62;
t72 = t23 * t64 + t24 * t70;
t73 = 0.1e1 / pkin(19);
t74 = t72 * t73;
t75 = cos(pkin(7));
t76 = t75 * pkin(18);
t77 = t74 * t76;
t80 = -t23 * t70 + t24 * t64;
t81 = t80 * t73;
t82 = sin(pkin(7));
t83 = t82 * pkin(18);
t84 = t81 * t83;
t85 = pkin(17) ^ 2;
t86 = pkin(22) ^ 2;
t87 = t81 * t76;
t88 = t74 * t83;
t89 = t87 - t88;
t90 = t89 ^ 2;
t91 = t77 + t84;
t92 = t91 ^ 2;
t93 = pkin(24) ^ 2;
t94 = -pkin(24) - t87 + t88;
t97 = -0.2e1 * t89 * t94 + 0.2e1 * t91 ^ 2 + t85 - t86 - t90 - t92 + t93;
t101 = 0.4e1 * t91 ^ 2 + 0.4e1 * t94 ^ 2;
t103 = t97 ^ 2;
t105 = sqrt(t101 * t85 - t103);
t108 = 0.1e1 / t101;
t110 = -t77 - t84 - (0.2e1 * t105 * t94 - 0.2e1 * t91 * t97) * t108;
t115 = t87 - t88 + (0.2e1 * t105 * t91 + 0.2e1 * t94 * t97) * t108 + pkin(24);
t116 = atan2(t110, t115);
t117 = t116 + pkin(6);
t118 = sin(t117);
t120 = t1 * t12;
t122 = cos(t117);
t123 = t3 * t122;
t126 = t3 * t12;
t128 = t1 * t122;
t142 = t12 * pkin(21);
t145 = t1 * t10;
t146 = t145 * pkin(21);
t147 = t3 * pkin(16);
t150 = t3 * t10;
t151 = t150 * pkin(21);
t152 = t1 * pkin(16);
t155 = -g(3) * (-t142 + pkin(8)) - g(1) * (t146 + t147) - g(2) * (-t151 + t152);
t156 = t116 + pkin(6) + qJ(3);
t157 = sin(t156);
t160 = cos(t156);
t179 = t10 * t118;
t183 = t118 * pkin(23);
t193 = t116 + pkin(6) + qJ(3) + qJ(4);
t194 = sin(t193);
t197 = cos(t193);
t216 = pkin(9) * t157;
t217 = t216 + t183;
t222 = pkin(9) * t160;
t223 = t122 * pkin(23);
t224 = -t222 - t223;
t232 = -g(3) * (-t10 * t217 + pkin(8) - t142) - g(1) * (-t120 * t217 + t224 * t3 + t146 + t147) - g(2) * (t1 * t224 + t126 * t217 - t151 + t152);
t234 = t116 + pkin(6) + 0.2e1 * qJ(4) + pkin(4);
t235 = cos(t234);
t238 = sin(t234);
t240 = -t120 * t235 + t238 * t3;
t244 = t1 * t238 + t126 * t235;
t247 = t11 * t238;
t250 = t120 * t238 + t235 * t3;
t254 = t1 * t235 - t126 * t238;
t258 = pkin(10) * t194 + t183 + t216;
t259 = t10 * t258;
t262 = t120 * t258;
t264 = -pkin(10) * t197 - t222 - t223;
t265 = t3 * t264;
t268 = t126 * t258;
t269 = t1 * t264;
t273 = t10 * t235;
t274 = cos(qJ(5));
t276 = sin(qJ(5));
t316 = sin(pkin(6));
t319 = cos(pkin(6));
t320 = t3 * t319;
t324 = t1 * t319;
t338 = 0.1e1 / pkin(22);
t339 = t110 * t338;
t341 = -t115 * t338;
t343 = t316 * t339 + t319 * t341;
t347 = -t316 * t341 + t319 * t339;
t357 = atan2(-(t316 * t347 - t319 * t343) * pkin(22) + pkin(24) + t87 - t88, (t316 * t343 + t319 * t347) * pkin(22) + t77 + t84);
t358 = pkin(6) + t116 - t357;
t359 = cos(t358);
t362 = sin(t358);
t385 = t316 * pkin(22);
t395 = atan2(-t72, t80);
t396 = t116 + pkin(6) - t395;
t397 = sin(t396);
t400 = cos(t396);
t422 = t118 * pkin(24);
t446 = atan2((t23 * t73 * t80 - t24 * t72 * t73) * pkin(19) + t46 - t47, (t23 * t72 * t73 + t24 * t73 * t80) * pkin(19) - pkin(9) + t26 + t29);
t447 = t116 + pkin(6) - t446 + qJ(3);
t448 = sin(t447);
t451 = cos(t447);
t471 = pkin(19) * t397 + t422;
t478 = -pkin(19) * t400 - pkin(24) * t122;
t487 = 0.2e1 * qJ(3);
t488 = t116 + pkin(6) + t487;
t489 = cos(t488);
t492 = sin(t488);
t511 = t116 + pkin(6) + t487 + pkin(3);
t512 = sin(t511);
t515 = cos(t511);
t534 = qJ(4) - qJ(3) + pkin(4);
t535 = sin(t534);
t537 = pkin(3) + qJ(3) - qJ(4);
t538 = sin(t537);
t540 = cos(t534);
t542 = pkin(12) * t538 - pkin(14) * t540 + pkin(15) * t535;
t545 = cos(t537);
t547 = -pkin(12) * t545 - pkin(14) * t535 - pkin(15) * t540 - pkin(10);
t548 = atan2(t542, t547);
t549 = t116 + pkin(6) + qJ(3) - t548 + qJ(4);
t550 = cos(t549);
t553 = sin(t549);
t561 = -t11 * t550 - g(1) * (t120 * t550 - t3 * t553) - g(2) * (-t1 * t553 - t126 * t550);
t562 = t11 * t553;
t565 = -t120 * t553 - t3 * t550;
t569 = -t1 * t550 + t126 * t553;
t573 = -pkin(12) * t512 + t183 + t216;
t574 = t10 * t573;
t577 = t120 * t573;
t579 = pkin(12) * t515 - t222 - t223;
t580 = t3 * t579;
t583 = t126 * t573;
t584 = t1 * t579;
t592 = t547 ^ 2;
t593 = t542 ^ 2;
t595 = sqrt(t592 + t593);
unknown(1,1) = 0;
unknown(1,2) = 0;
unknown(1,3) = 0;
unknown(1,4) = 0;
unknown(1,5) = 0;
unknown(1,6) = 0;
unknown(1,7) = (-t2 + t4);
unknown(1,8) = t8;
unknown(1,9) = -g(3);
unknown(1,10) = -t9;
unknown(1,11) = 0;
unknown(1,12) = 0;
unknown(1,13) = 0;
unknown(1,14) = 0;
unknown(1,15) = 0;
unknown(1,16) = 0;
unknown(1,17) = (t12 * t2 - t12 * t4 + t11);
unknown(1,18) = t19;
unknown(1,19) = t8;
unknown(1,20) = (-pkin(16) * t6 - pkin(16) * t7 - t9);
unknown(1,21) = 0;
unknown(1,22) = 0;
unknown(1,23) = 0;
unknown(1,24) = 0;
unknown(1,25) = 0;
unknown(1,26) = 0;
unknown(1,27) = (t11 * t118 - g(1) * (-t118 * t120 - t123) - g(2) * (t118 * t126 - t128));
unknown(1,28) = (t11 * t122 - g(1) * (t118 * t3 - t120 * t122) - g(2) * (t1 * t118 + t122 * t126));
unknown(1,29) = -t19;
unknown(1,30) = t155;
unknown(1,31) = 0;
unknown(1,32) = 0;
unknown(1,33) = 0;
unknown(1,34) = 0;
unknown(1,35) = 0;
unknown(1,36) = 0;
unknown(1,37) = (t11 * t157 - g(1) * (-t120 * t157 - t160 * t3) - g(2) * (-t1 * t160 + t126 * t157));
unknown(1,38) = (t11 * t160 - g(1) * (-t120 * t160 + t157 * t3) - g(2) * (t1 * t157 + t126 * t160));
unknown(1,39) = -t19;
unknown(1,40) = (-g(3) * (-pkin(23) * t179 + pkin(8) - t142) - g(1) * (-pkin(23) * t123 - t120 * t183 + t146 + t147) - g(2) * (-pkin(23) * t128 + t126 * t183 - t151 + t152));
unknown(1,41) = 0;
unknown(1,42) = 0;
unknown(1,43) = 0;
unknown(1,44) = 0;
unknown(1,45) = 0;
unknown(1,46) = 0;
unknown(1,47) = (t11 * t194 - g(1) * (-t120 * t194 - t197 * t3) - g(2) * (-t1 * t197 + t126 * t194));
unknown(1,48) = (t11 * t197 - g(1) * (-t120 * t197 + t194 * t3) - g(2) * (t1 * t194 + t126 * t197));
unknown(1,49) = -t19;
unknown(1,50) = t232;
unknown(1,51) = 0;
unknown(1,52) = 0;
unknown(1,53) = 0;
unknown(1,54) = 0;
unknown(1,55) = 0;
unknown(1,56) = 0;
unknown(1,57) = (-g(1) * t240 - g(2) * t244 + t11 * t235);
unknown(1,58) = (-g(1) * t250 - g(2) * t254 - t247);
unknown(1,59) = -t19;
unknown(1,60) = (-g(3) * (-t259 - t142 + pkin(8)) - g(1) * (-t262 + t146 + t265 + t147) - g(2) * (t268 - t151 + t269 + t152));
unknown(1,61) = 0;
unknown(1,62) = 0;
unknown(1,63) = 0;
unknown(1,64) = 0;
unknown(1,65) = 0;
unknown(1,66) = 0;
unknown(1,67) = (-g(3) * (-t12 * t276 - t273 * t274) - g(1) * (t145 * t276 + t240 * t274) - g(2) * (-t150 * t276 + t244 * t274));
unknown(1,68) = (-g(3) * (-t12 * t274 + t273 * t276) - g(1) * (t145 * t274 - t240 * t276) - g(2) * (-t150 * t274 - t244 * t276));
unknown(1,69) = (g(1) * t250 + g(2) * t254 + t247);
unknown(1,70) = (-g(3) * (-pkin(13) * t10 * t238 + pkin(8) - t142 - t259) - g(1) * (-pkin(13) * t250 + t146 + t147 - t262 + t265) - g(2) * (-pkin(13) * t254 - t151 + t152 + t268 + t269));
unknown(1,71) = 0;
unknown(1,72) = 0;
unknown(1,73) = 0;
unknown(1,74) = 0;
unknown(1,75) = 0;
unknown(1,76) = 0;
unknown(1,77) = (-t11 * t316 - g(1) * (t120 * t316 + t320) - g(2) * (-t126 * t316 + t324));
unknown(1,78) = (-t11 * t319 - g(1) * (t120 * t319 - t3 * t316) - g(2) * (-t1 * t316 - t126 * t319));
unknown(1,79) = -t19;
unknown(1,80) = t155;
unknown(1,81) = 0;
unknown(1,82) = 0;
unknown(1,83) = 0;
unknown(1,84) = 0;
unknown(1,85) = 0;
unknown(1,86) = 0;
unknown(1,87) = (t11 * t359 - g(1) * (-t120 * t359 + t3 * t362) - g(2) * (t1 * t362 + t126 * t359));
unknown(1,88) = (-t11 * t362 - g(1) * (t120 * t362 + t3 * t359) - g(2) * (t1 * t359 - t126 * t362));
unknown(1,89) = -t19;
unknown(1,90) = (-g(3) * (-pkin(22) * t10 * t316 + pkin(8) - t142) - g(1) * (-pkin(22) * t320 - t120 * t385 + t146 + t147) - g(2) * (-pkin(22) * t324 + t126 * t385 - t151 + t152));
unknown(1,91) = 0;
unknown(1,92) = 0;
unknown(1,93) = 0;
unknown(1,94) = 0;
unknown(1,95) = 0;
unknown(1,96) = 0;
unknown(1,97) = (t11 * t397 - g(1) * (-t120 * t397 - t3 * t400) - g(2) * (-t1 * t400 + t126 * t397));
unknown(1,98) = (t11 * t400 - g(1) * (-t120 * t400 + t3 * t397) - g(2) * (t1 * t397 + t126 * t400));
unknown(1,99) = -t19;
unknown(1,100) = (-g(3) * (-pkin(24) * t179 + pkin(8) - t142) - g(1) * (-pkin(24) * t123 - t120 * t422 + t146 + t147) - g(2) * (-pkin(24) * t128 + t126 * t422 - t151 + t152));
unknown(1,101) = 0;
unknown(1,102) = 0;
unknown(1,103) = 0;
unknown(1,104) = 0;
unknown(1,105) = 0;
unknown(1,106) = 0;
unknown(1,107) = (-t11 * t448 - g(1) * (t120 * t448 + t3 * t451) - g(2) * (t1 * t451 - t126 * t448));
unknown(1,108) = (-t11 * t451 - g(1) * (t120 * t451 - t3 * t448) - g(2) * (-t1 * t448 - t126 * t451));
unknown(1,109) = -t19;
unknown(1,110) = (-g(3) * (-t10 * t471 + pkin(8) - t142) - g(1) * (-t120 * t471 + t3 * t478 + t146 + t147) - g(2) * (t1 * t478 + t126 * t471 - t151 + t152));
unknown(1,111) = 0;
unknown(1,112) = 0;
unknown(1,113) = 0;
unknown(1,114) = 0;
unknown(1,115) = 0;
unknown(1,116) = 0;
unknown(1,117) = (-t11 * t489 - g(1) * (t120 * t489 - t3 * t492) - g(2) * (-t1 * t492 - t126 * t489));
unknown(1,118) = (t11 * t492 - g(1) * (-t120 * t492 - t3 * t489) - g(2) * (-t1 * t489 + t126 * t492));
unknown(1,119) = -t19;
unknown(1,120) = t232;
unknown(1,121) = 0;
unknown(1,122) = 0;
unknown(1,123) = 0;
unknown(1,124) = 0;
unknown(1,125) = 0;
unknown(1,126) = 0;
unknown(1,127) = (-t11 * t512 - g(1) * (t120 * t512 + t3 * t515) - g(2) * (t1 * t515 - t126 * t512));
unknown(1,128) = (-t11 * t515 - g(1) * (t120 * t515 - t3 * t512) - g(2) * (-t1 * t512 - t126 * t515));
unknown(1,129) = -t19;
unknown(1,130) = t232;
unknown(1,131) = 0;
unknown(1,132) = 0;
unknown(1,133) = 0;
unknown(1,134) = 0;
unknown(1,135) = 0;
unknown(1,136) = 0;
unknown(1,137) = t561;
unknown(1,138) = (-g(1) * t565 - g(2) * t569 + t562);
unknown(1,139) = -t19;
unknown(1,140) = (-g(3) * (-t574 - t142 + pkin(8)) - g(1) * (-t577 + t146 + t580 + t147) - g(2) * (t583 - t151 + t584 + t152));
unknown(1,141) = 0;
unknown(1,142) = 0;
unknown(1,143) = 0;
unknown(1,144) = 0;
unknown(1,145) = 0;
unknown(1,146) = 0;
unknown(1,147) = t561;
unknown(1,148) = -t19;
unknown(1,149) = (g(1) * t565 + g(2) * t569 - t562);
unknown(1,150) = (-g(3) * (t10 * t553 * t595 + pkin(8) - t142 - t574) - g(1) * (-t565 * t595 + t146 + t147 - t577 + t580) - g(2) * (-t569 * t595 - t151 + t152 + t583 + t584));
U_reg = unknown;
