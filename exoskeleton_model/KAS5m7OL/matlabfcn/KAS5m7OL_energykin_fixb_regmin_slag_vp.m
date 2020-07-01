% Calculate minimal parameter regressor of fixed base kinetic energy for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% qJD [13x1]
%   Generalized joint velocities
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% T_reg [1x88]
%   minimal parameter regressor of kinetic energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T_reg = KAS5m7OL_energykin_fixb_regmin_slag_vp(qJ, qJD, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energykin_fixb_regmin_slag_vp: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m7OL_energykin_fixb_regmin_slag_vp: qJD has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energykin_fixb_regmin_slag_vp: pkin has to be [19x1] (double)');

%% Symbolic Calculation
% From energy_kinetic_fixb_regressor_minpar_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:59:43
% EndTime: 2020-06-30 17:59:43
% DurationCPUTime: 0.43s
% Computational Cost: add. (3054->148), mult. (6748->284), div. (0->0), fcn. (5364->26), ass. (0->246)
unknown=NaN(1,88);
t1 = qJD(1) ^ 2;
t3 = cos(qJ(2));
t4 = t3 ^ 2;
t7 = sin(qJ(2));
t14 = qJD(2) ^ 2;
t16 = qJD(1) * t3;
t17 = pkin(11) * qJD(2);
t19 = qJD(1) * t7;
t21 = cos(qJ(3));
t22 = t3 * t21;
t24 = sin(qJ(3));
t26 = -qJD(1) * t22 + qJD(2) * t24;
t27 = t26 ^ 2;
t29 = t3 * t24;
t32 = qJD(1) * t29 + qJD(2) * t21;
t34 = -t19 + qJD(3);
t37 = t34 ^ 2;
t41 = pkin(11) * t19 + qJD(2) * pkin(16);
t42 = t41 * t24;
t43 = qJD(1) * pkin(16);
t44 = t43 * t22;
t52 = t21 * t41 + t29 * t43;
t57 = cos(qJ(4));
t59 = sin(qJ(4));
t61 = t26 * t57 + t32 * t59;
t62 = t61 ^ 2;
t66 = -t26 * t59 + t32 * t57;
t68 = -t19 + qJD(3) + qJD(4);
t71 = t68 ^ 2;
t73 = t52 * t59;
t75 = pkin(18) * t34 - t42 + t44;
t76 = t75 * t57;
t79 = pkin(11) * t16;
t80 = pkin(18) * t32;
t81 = -t79 - t80;
t87 = t52 * t57 + t59 * t75;
t90 = cos(qJ(5));
t92 = sin(qJ(5));
t94 = t61 * t90 + t66 * t92;
t95 = t94 ^ 2;
t99 = -t61 * t92 + t66 * t90;
t101 = -t19 + qJD(3) + qJD(4) + qJD(5);
t104 = t101 ^ 2;
t106 = pkin(6) * t66;
t107 = -t79 - t80 - t106;
t109 = t87 * t92;
t111 = pkin(6) * t68 - t73 + t76;
t112 = t111 * t90;
t119 = t111 * t92 + t87 * t90;
t122 = sin(qJ(6));
t124 = cos(qJ(6));
t126 = -t122 * t94 + t124 * t99;
t127 = t126 ^ 2;
t129 = t94 * t124;
t130 = t99 * t122;
t131 = -t129 - t130;
t133 = -t19 + qJD(3) + qJD(4) + qJD(5) + qJD(6);
t136 = t133 ^ 2;
t138 = pkin(7) * t99;
t139 = -t79 - t80 - t106 - t138;
t143 = pkin(7) * t101 - t109 + t112;
t145 = -t119 * t124 - t122 * t143;
t148 = t119 * t122;
t149 = t143 * t124;
t154 = cos(qJ(7));
t156 = sin(qJ(7));
t158 = t126 * t154 + t133 * t156;
t159 = t158 ^ 2;
t163 = -t126 * t156 + t133 * t154;
t165 = t129 + t130 + qJD(7);
t168 = t165 ^ 2;
t172 = pkin(10) * t133 - t148 + t149;
t175 = -pkin(10) * t126 - t106 - t138 - t79 - t80;
t177 = t154 * t175 - t156 * t172;
t183 = t154 * t172 + t156 * t175;
t189 = t183 ^ 2;
t190 = t177 ^ 2;
t191 = t145 ^ 2;
t193 = cos(qJ(8));
t194 = sin(pkin(3));
t195 = t3 * t194;
t197 = cos(pkin(3));
t199 = qJD(1) * t195 + qJD(2) * t197;
t201 = sin(qJ(8));
t202 = t3 * t197;
t205 = qJD(1) * t202 - qJD(2) * t194;
t207 = t193 * t199 + t201 * t205;
t208 = t207 ^ 2;
t212 = t193 * t205 - t199 * t201;
t214 = -t19 + qJD(8);
t217 = t214 ^ 2;
t221 = -t194 * t41 + t202 * t43;
t226 = pkin(17) * t19 - t195 * t43 - t197 * t41;
t231 = pkin(17) * t205 - t79;
t240 = cos(qJ(9));
t242 = sin(qJ(9));
t244 = t240 * t26 + t242 * t32;
t245 = t244 ^ 2;
t249 = t240 * t32 - t242 * t26;
t251 = -t19 + qJD(3) + qJD(9);
t254 = t251 ^ 2;
t256 = pkin(19) * t32;
t257 = -t79 - t256;
t259 = t52 * t242;
t261 = pkin(19) * t34 - t42 + t44;
t262 = t261 * t240;
t269 = t240 * t52 + t242 * t261;
t272 = cos(qJ(10));
t274 = sin(qJ(10));
t276 = -t244 * t272 - t249 * t274;
t277 = t276 ^ 2;
t281 = t244 * t274 - t249 * t272;
t283 = -t19 + qJD(3) + qJD(9) + qJD(10);
t286 = t283 ^ 2;
t289 = -pkin(14) * t249 - t256 - t79;
t293 = pkin(14) * t251 - t259 + t262;
t304 = sin(qJ(11));
t306 = cos(qJ(11));
t308 = t304 * t61 - t306 * t66;
t309 = t308 ^ 2;
t313 = t304 * t66 + t306 * t61;
t315 = -t19 + qJD(3) + qJD(4) + qJD(11);
t318 = t315 ^ 2;
t323 = t111 * t304 + t306 * t87;
t329 = -t111 * t306 + t304 * t87;
t332 = t329 ^ 2;
t333 = t323 ^ 2;
t334 = t107 ^ 2;
t336 = sin(qJ(12));
t337 = sin(pkin(1));
t339 = cos(pkin(1));
t341 = t308 * t337 - t313 * t339;
t343 = cos(qJ(12));
t346 = t308 * t339 + t313 * t337;
t348 = t336 * t341 - t343 * t346;
t349 = t348 ^ 2;
t353 = t336 * t346 + t341 * t343;
t355 = -t19 + qJD(3) + qJD(4) + qJD(11) + qJD(12);
t358 = t355 ^ 2;
t360 = pkin(9) * t346;
t361 = -t79 - t80 - t106 - t360;
t365 = -t323 * t339 + t329 * t337;
t366 = t365 * t343;
t370 = pkin(9) * t315 + t323 * t337 + t329 * t339;
t371 = t370 * t336;
t372 = t366 + t371;
t376 = t365 * t336;
t377 = t370 * t343;
t378 = t376 - t377;
t381 = t378 ^ 2;
t382 = t372 ^ 2;
t383 = t361 ^ 2;
t385 = -t366 - t371 + qJD(13);
t388 = -qJ(13) * t348 - t106 - t360 - t79 - t80;
t393 = qJ(13) * t355 + t376 - t377;
t399 = t393 ^ 2;
t400 = t388 ^ 2;
t401 = t385 ^ 2;
unknown(1,1) = t1 / 0.2e1;
unknown(1,2) = 0.0e0;
unknown(1,3) = 0.0e0;
unknown(1,4) = t1 * t4 / 0.2e1;
unknown(1,5) = -t3 * t1 * t7;
unknown(1,6) = -qJD(1) * t3 * qJD(2);
unknown(1,7) = qJD(1) * t7 * qJD(2);
unknown(1,8) = t14 / 0.2e1;
unknown(1,9) = t17 * t16;
unknown(1,10) = -t17 * t19;
unknown(1,11) = t27 / 0.2e1;
unknown(1,12) = t32 * t26;
unknown(1,13) = t26 * t34;
unknown(1,14) = t32 * t34;
unknown(1,15) = t37 / 0.2e1;
unknown(1,16) = t34 * (-t42 + t44) + t32 * pkin(11) * t16;
unknown(1,17) = -pkin(11) * t16 * t26 - t34 * t52;
unknown(1,18) = t62 / 0.2e1;
unknown(1,19) = t66 * t61;
unknown(1,20) = t68 * t61;
unknown(1,21) = t66 * t68;
unknown(1,22) = t71 / 0.2e1;
unknown(1,23) = t68 * (-t73 + t76) - t66 * t81;
unknown(1,24) = t61 * t81 - t68 * t87;
unknown(1,25) = t95 / 0.2e1;
unknown(1,26) = t94 * t99;
unknown(1,27) = t94 * t101;
unknown(1,28) = t99 * t101;
unknown(1,29) = t104 / 0.2e1;
unknown(1,30) = -t99 * t107 + t101 * (-t109 + t112);
unknown(1,31) = -t101 * t119 + t107 * t94;
unknown(1,32) = t127 / 0.2e1;
unknown(1,33) = t131 * t126;
unknown(1,34) = t133 * t126;
unknown(1,35) = t131 * t133;
unknown(1,36) = t136 / 0.2e1;
unknown(1,37) = -t131 * t139 + t133 * t145;
unknown(1,38) = -t133 * (-t148 + t149) + t126 * t139;
unknown(1,39) = t159 / 0.2e1;
unknown(1,40) = t163 * t158;
unknown(1,41) = t165 * t158;
unknown(1,42) = t163 * t165;
unknown(1,43) = t168 / 0.2e1;
unknown(1,44) = t145 * t163 + t165 * t177;
unknown(1,45) = -t145 * t158 - t165 * t183;
unknown(1,46) = -t158 * t177 + t163 * t183;
unknown(1,47) = t189 / 0.2e1 + t190 / 0.2e1 + t191 / 0.2e1;
unknown(1,48) = t208 / 0.2e1;
unknown(1,49) = t207 * t212;
unknown(1,50) = t207 * t214;
unknown(1,51) = t212 * t214;
unknown(1,52) = t217 / 0.2e1;
unknown(1,53) = t214 * (t193 * t226 - t201 * t221) - t212 * t231;
unknown(1,54) = t207 * t231 - t214 * (t193 * t221 + t201 * t226);
unknown(1,55) = t245 / 0.2e1;
unknown(1,56) = t244 * t249;
unknown(1,57) = t244 * t251;
unknown(1,58) = t251 * t249;
unknown(1,59) = t254 / 0.2e1;
unknown(1,60) = -t249 * t257 + t251 * (-t259 + t262);
unknown(1,61) = t244 * t257 - t251 * t269;
unknown(1,62) = t277 / 0.2e1;
unknown(1,63) = t276 * t281;
unknown(1,64) = t276 * t283;
unknown(1,65) = t281 * t283;
unknown(1,66) = t286 / 0.2e1;
unknown(1,67) = -t281 * t289 + t283 * (t269 * t274 - t272 * t293);
unknown(1,68) = t276 * t289 - t283 * (-t269 * t272 - t293 * t274);
unknown(1,69) = t309 / 0.2e1;
unknown(1,70) = t313 * t308;
unknown(1,71) = t315 * t308;
unknown(1,72) = t313 * t315;
unknown(1,73) = t318 / 0.2e1;
unknown(1,74) = -t107 * t313 + t315 * t323;
unknown(1,75) = t107 * t308 - t315 * t329;
unknown(1,76) = t332 / 0.2e1 + t333 / 0.2e1 + t334 / 0.2e1;
unknown(1,77) = t349 / 0.2e1;
unknown(1,78) = t348 * t353;
unknown(1,79) = t348 * t355;
unknown(1,80) = t353 * t355;
unknown(1,81) = t358 / 0.2e1;
unknown(1,82) = -t353 * t361 + t355 * t372;
unknown(1,83) = t348 * t361 - t355 * t378;
unknown(1,84) = t381 / 0.2e1 + t382 / 0.2e1 + t383 / 0.2e1;
unknown(1,85) = -t353 * t388 - t355 * t385;
unknown(1,86) = t348 * t385 + t353 * t393;
unknown(1,87) = -t348 * t388 + t355 * t393;
unknown(1,88) = t399 / 0.2e1 + t400 / 0.2e1 + t401 / 0.2e1;
T_reg = unknown;
