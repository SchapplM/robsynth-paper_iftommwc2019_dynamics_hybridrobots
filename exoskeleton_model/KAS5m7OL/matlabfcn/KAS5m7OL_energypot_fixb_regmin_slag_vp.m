% Calculate minimal parameter regressor of potential energy for
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% U_reg [1x88]
%   minimal parameter regressor of Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U_reg = KAS5m7OL_energypot_fixb_regmin_slag_vp(qJ, g, ...
  pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energypot_fixb_regmin_slag_vp: qJ has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_energypot_fixb_regmin_slag_vp: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energypot_fixb_regmin_slag_vp: pkin has to be [19x1] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_regressor_minpar_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:59:43
% EndTime: 2020-06-30 17:59:44
% DurationCPUTime: 0.12s
% Computational Cost: add. (531->168), mult. (408->211), div. (0->0), fcn. (452->26), ass. (0->161)
unknown=NaN(1,88);
t1 = cos(qJ(1));
t2 = t1 * g(2);
t3 = sin(qJ(1));
t4 = t3 * g(1);
t9 = cos(qJ(2));
t10 = t9 * g(3);
t11 = sin(qJ(2));
t18 = -t11 * g(3) - t9 * t2 + t9 * t4;
t19 = cos(qJ(3));
t21 = t11 * t1;
t23 = sin(qJ(3));
t27 = t11 * t3;
t43 = qJ(3) + qJ(4);
t44 = cos(t43);
t47 = sin(t43);
t66 = qJ(3) + qJ(4) + qJ(5);
t67 = cos(t66);
t70 = sin(t66);
t89 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t90 = sin(t89);
t93 = cos(t89);
t95 = -t90 * t21 + t93 * t3;
t99 = t93 * t1 + t90 * t27;
t102 = t93 * t10;
t105 = -t93 * t21 - t90 * t3;
t109 = -t90 * t1 + t93 * t27;
t112 = t90 * t9;
t113 = cos(qJ(7));
t115 = sin(qJ(7));
t120 = t9 * t1;
t125 = t9 * t3;
t149 = t44 * pkin(6);
t150 = pkin(18) * t19;
t151 = t67 * pkin(7) + t149 + t150;
t153 = pkin(16) * t11;
t158 = pkin(16) * t120;
t160 = t47 * pkin(6);
t161 = pkin(18) * t23;
t162 = t70 * pkin(7) + t160 + t161;
t164 = pkin(11) * t3;
t169 = pkin(16) * t125;
t171 = pkin(11) * t1;
t175 = pkin(3) + qJ(8);
t176 = sin(t175);
t179 = cos(t175);
t198 = qJ(3) + qJ(9);
t199 = cos(t198);
t202 = sin(t198);
t221 = qJ(3) + qJ(9) + qJ(10);
t222 = cos(t221);
t225 = sin(t221);
t244 = qJ(3) + qJ(4) + qJ(11);
t245 = sin(t244);
t248 = cos(t244);
t267 = t149 + t150;
t272 = t160 + t161;
t281 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t282 = sin(t281);
t285 = cos(t281);
t293 = -t282 * t10 - (-t282 * t21 + t285 * t3) * g(2) - (t285 * t1 + t282 * t27) * g(1);
t294 = t285 * t10;
t297 = -t285 * t21 - t282 * t3;
t301 = -t282 * t1 + t285 * t27;
t304 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t305 = cos(t304);
t307 = -t305 * pkin(9) + t149 + t150;
t308 = t307 * t9;
t311 = t307 * t21;
t312 = sin(t304);
t314 = -t312 * pkin(9) + t160 + t161;
t315 = t314 * t3;
t318 = t307 * t27;
t319 = t314 * t1;
unknown(1,1) = 0;
unknown(1,2) = (t2 - t4);
unknown(1,3) = (-t1 * g(1) - t3 * g(2));
unknown(1,4) = 0;
unknown(1,5) = 0;
unknown(1,6) = 0;
unknown(1,7) = 0;
unknown(1,8) = 0;
unknown(1,9) = (-t11 * t2 + t11 * t4 + t10);
unknown(1,10) = t18;
unknown(1,11) = 0;
unknown(1,12) = 0;
unknown(1,13) = 0;
unknown(1,14) = 0;
unknown(1,15) = 0;
unknown(1,16) = (t19 * t10 - (t19 * t21 + t23 * t3) * g(2) - (t23 * t1 - t19 * t27) * g(1));
unknown(1,17) = (-t23 * t10 - (t19 * t3 - t23 * t21) * g(2) - (t19 * t1 + t23 * t27) * g(1));
unknown(1,18) = 0;
unknown(1,19) = 0;
unknown(1,20) = 0;
unknown(1,21) = 0;
unknown(1,22) = 0;
unknown(1,23) = (t44 * t10 - (t44 * t21 + t47 * t3) * g(2) - (t47 * t1 - t44 * t27) * g(1));
unknown(1,24) = (-t47 * t10 - (-t47 * t21 + t44 * t3) * g(2) - (t44 * t1 + t47 * t27) * g(1));
unknown(1,25) = 0;
unknown(1,26) = 0;
unknown(1,27) = 0;
unknown(1,28) = 0;
unknown(1,29) = 0;
unknown(1,30) = (t67 * t10 - (t67 * t21 + t70 * t3) * g(2) - (t70 * t1 - t67 * t27) * g(1));
unknown(1,31) = (-t70 * t10 - (-t70 * t21 + t67 * t3) * g(2) - (t67 * t1 + t70 * t27) * g(1));
unknown(1,32) = 0;
unknown(1,33) = 0;
unknown(1,34) = 0;
unknown(1,35) = 0;
unknown(1,36) = 0;
unknown(1,37) = (-t99 * g(1) - t95 * g(2) - t90 * t10);
unknown(1,38) = (-t109 * g(1) - t105 * g(2) - t102);
unknown(1,39) = 0;
unknown(1,40) = 0;
unknown(1,41) = 0;
unknown(1,42) = 0;
unknown(1,43) = 0;
unknown(1,44) = (-(-t115 * t11 + t113 * t112) * g(3) - (t113 * t95 - t115 * t120) * g(2) - (t113 * t99 + t115 * t125) * g(1));
unknown(1,45) = (-(-t113 * t11 - t115 * t112) * g(3) - (-t113 * t120 - t115 * t95) * g(2) - (t113 * t125 - t115 * t99) * g(1));
unknown(1,46) = (t109 * g(1) + t105 * g(2) + t102);
unknown(1,47) = (-(-pkin(10) * t93 * t9 - t151 * t9 + pkin(5) - t153) * g(3) - (-pkin(10) * t105 + t151 * t21 + t162 * t3 - t158 + t164) * g(2) - (-pkin(10) * t109 + t162 * t1 - t151 * t27 + t169 + t171) * g(1));
unknown(1,48) = 0;
unknown(1,49) = 0;
unknown(1,50) = 0;
unknown(1,51) = 0;
unknown(1,52) = 0;
unknown(1,53) = (-t176 * t10 - (-t176 * t21 + t179 * t3) * g(2) - (t179 * t1 + t176 * t27) * g(1));
unknown(1,54) = (-t179 * t10 - (-t176 * t3 - t179 * t21) * g(2) - (-t176 * t1 + t179 * t27) * g(1));
unknown(1,55) = 0;
unknown(1,56) = 0;
unknown(1,57) = 0;
unknown(1,58) = 0;
unknown(1,59) = 0;
unknown(1,60) = (t199 * t10 - (t199 * t21 + t202 * t3) * g(2) - (t202 * t1 - t199 * t27) * g(1));
unknown(1,61) = (-t202 * t10 - (t199 * t3 - t202 * t21) * g(2) - (t199 * t1 + t202 * t27) * g(1));
unknown(1,62) = 0;
unknown(1,63) = 0;
unknown(1,64) = 0;
unknown(1,65) = 0;
unknown(1,66) = 0;
unknown(1,67) = (-t222 * t10 - (-t222 * t21 - t225 * t3) * g(2) - (-t225 * t1 + t222 * t27) * g(1));
unknown(1,68) = (t225 * t10 - (t225 * t21 - t222 * t3) * g(2) - (-t222 * t1 - t225 * t27) * g(1));
unknown(1,69) = 0;
unknown(1,70) = 0;
unknown(1,71) = 0;
unknown(1,72) = 0;
unknown(1,73) = 0;
unknown(1,74) = (t245 * t10 - (t245 * t21 - t248 * t3) * g(2) - (-t248 * t1 - t245 * t27) * g(1));
unknown(1,75) = (t248 * t10 - (t248 * t21 + t245 * t3) * g(2) - (t245 * t1 - t248 * t27) * g(1));
unknown(1,76) = (-(-t267 * t9 + pkin(5) - t153) * g(3) - (t267 * t21 + t272 * t3 - t158 + t164) * g(2) - (t272 * t1 - t267 * t27 + t169 + t171) * g(1));
unknown(1,77) = 0;
unknown(1,78) = 0;
unknown(1,79) = 0;
unknown(1,80) = 0;
unknown(1,81) = 0;
unknown(1,82) = t293;
unknown(1,83) = (-t301 * g(1) - t297 * g(2) - t294);
unknown(1,84) = (-(-t308 - t153 + pkin(5)) * g(3) - (t311 - t158 + t315 + t164) * g(2) - (-t318 + t169 + t319 + t171) * g(1));
unknown(1,85) = t293;
unknown(1,86) = -t18;
unknown(1,87) = (t301 * g(1) + t297 * g(2) + t294);
unknown(1,88) = (-(-qJ(13) * t285 * t9 + pkin(5) - t153 - t308) * g(3) - (-qJ(13) * t297 - t158 + t164 + t311 + t315) * g(2) - (-qJ(13) * t301 + t169 + t171 - t318 + t319) * g(1));
U_reg = unknown;
