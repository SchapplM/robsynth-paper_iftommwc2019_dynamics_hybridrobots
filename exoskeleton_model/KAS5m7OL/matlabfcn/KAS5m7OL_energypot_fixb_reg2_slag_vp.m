% Calculate inertial parameters regressor of potential energy for
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
% U_reg [1x(13*10)]
%   inertial parameter regressor of Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U_reg = KAS5m7OL_energypot_fixb_reg2_slag_vp(qJ, g, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_energypot_fixb_reg2_slag_vp: qJ has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_energypot_fixb_reg2_slag_vp: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_energypot_fixb_reg2_slag_vp: pkin has to be [19x1] (double)');

%% Symbolic Calculation
% From energy_potential_fixb_regressor_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:50:28
% EndTime: 2020-06-30 17:50:28
% DurationCPUTime: 0.21s
% Computational Cost: add. (739->256), mult. (710->287), div. (0->0), fcn. (731->28), ass. (0->245)
unknown=NaN(1,150);
t1 = cos(qJ(1));
t2 = g(2) * t1;
t3 = sin(qJ(1));
t4 = g(1) * t3;
t6 = g(2) * t3;
t7 = g(1) * t1;
t8 = -t6 - t7;
t9 = (g(3) * pkin(5));
t10 = cos(qJ(2));
t11 = g(3) * t10;
t12 = sin(qJ(2));
t19 = -g(3) * t12 - t2 * t10 + t4 * t10;
t23 = cos(qJ(3));
t25 = t1 * t12;
t27 = sin(qJ(3));
t28 = t3 * t27;
t31 = t3 * t12;
t33 = t1 * t27;
t47 = t12 * pkin(16);
t50 = t1 * t10;
t51 = t50 * pkin(16);
t52 = t3 * pkin(11);
t55 = t3 * t10;
t56 = t55 * pkin(16);
t57 = t1 * pkin(11);
t60 = -g(3) * (-t47 + pkin(5)) - g(2) * (-t51 + t52) - g(1) * (t56 + t57);
t61 = qJ(3) + qJ(4);
t62 = cos(t61);
t65 = sin(t61);
t84 = t10 * t23;
t88 = t23 * pkin(18);
t98 = qJ(3) + qJ(4) + qJ(5);
t99 = cos(t98);
t102 = sin(t98);
t121 = pkin(6) * t62;
t122 = t121 + t88;
t127 = pkin(6) * t65;
t128 = t27 * pkin(18);
t129 = t127 + t128;
t137 = -g(3) * (-t10 * t122 + pkin(5) - t47) - g(2) * (t25 * t122 + t3 * t129 - t51 + t52) - g(1) * (t1 * t129 - t31 * t122 + t56 + t57);
t138 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t139 = sin(t138);
t142 = cos(t138);
t144 = -t25 * t139 + t3 * t142;
t148 = t1 * t142 + t31 * t139;
t151 = t11 * t142;
t154 = -t3 * t139 - t25 * t142;
t158 = -t1 * t139 + t31 * t142;
t162 = pkin(7) * t99 + t121 + t88;
t163 = t10 * t162;
t166 = t25 * t162;
t168 = pkin(7) * t102 + t127 + t128;
t169 = t3 * t168;
t172 = t31 * t162;
t173 = t1 * t168;
t177 = t10 * t139;
t178 = cos(qJ(7));
t180 = sin(qJ(7));
t220 = sin(pkin(3));
t223 = cos(pkin(3));
t224 = t3 * t223;
t228 = t1 * t223;
t242 = pkin(3) + qJ(8);
t243 = sin(t242);
t246 = cos(t242);
t269 = t220 * pkin(17);
t279 = qJ(3) + qJ(9);
t280 = cos(t279);
t283 = sin(t279);
t305 = t23 * pkin(19);
t315 = qJ(3) + qJ(9) + qJ(10);
t316 = cos(t315);
t319 = sin(t315);
t339 = pkin(14) * t280 + t305;
t346 = pkin(14) * t283 + t27 * pkin(19);
t355 = qJ(3) + qJ(4) + qJ(11);
t356 = sin(t355);
t359 = cos(t355);
t378 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t379 = cos(t378);
t382 = sin(t378);
t401 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t402 = sin(t401);
t405 = cos(t401);
t413 = -t11 * t402 - g(2) * (-t25 * t402 + t3 * t405) - g(1) * (t1 * t405 + t31 * t402);
t414 = t11 * t405;
t417 = -t25 * t405 - t3 * t402;
t421 = -t1 * t402 + t31 * t405;
t425 = -pkin(9) * t379 + t121 + t88;
t426 = t10 * t425;
t429 = t25 * t425;
t431 = -pkin(9) * t382 + t127 + t128;
t432 = t3 * t431;
t435 = t31 * t425;
t436 = t1 * t431;
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
unknown(1,20) = (-t6 * pkin(11) - t7 * pkin(11) - t9);
unknown(1,21) = 0;
unknown(1,22) = 0;
unknown(1,23) = 0;
unknown(1,24) = 0;
unknown(1,25) = 0;
unknown(1,26) = 0;
unknown(1,27) = (t11 * t23 - g(2) * (t25 * t23 + t28) - g(1) * (-t31 * t23 + t33));
unknown(1,28) = (-t11 * t27 - g(2) * (t3 * t23 - t25 * t27) - g(1) * (t1 * t23 + t31 * t27));
unknown(1,29) = -t19;
unknown(1,30) = t60;
unknown(1,31) = 0;
unknown(1,32) = 0;
unknown(1,33) = 0;
unknown(1,34) = 0;
unknown(1,35) = 0;
unknown(1,36) = 0;
unknown(1,37) = (t11 * t62 - g(2) * (t25 * t62 + t3 * t65) - g(1) * (t1 * t65 - t31 * t62));
unknown(1,38) = (-t11 * t65 - g(2) * (-t25 * t65 + t3 * t62) - g(1) * (t1 * t62 + t31 * t65));
unknown(1,39) = -t19;
unknown(1,40) = (-g(3) * (-t84 * pkin(18) + pkin(5) - t47) - g(2) * (t28 * pkin(18) + t25 * t88 - t51 + t52) - g(1) * (t33 * pkin(18) - t31 * t88 + t56 + t57));
unknown(1,41) = 0;
unknown(1,42) = 0;
unknown(1,43) = 0;
unknown(1,44) = 0;
unknown(1,45) = 0;
unknown(1,46) = 0;
unknown(1,47) = (t11 * t99 - g(2) * (t3 * t102 + t25 * t99) - g(1) * (t1 * t102 - t31 * t99));
unknown(1,48) = (-t11 * t102 - g(2) * (-t25 * t102 + t3 * t99) - g(1) * (t1 * t99 + t31 * t102));
unknown(1,49) = -t19;
unknown(1,50) = t137;
unknown(1,51) = 0;
unknown(1,52) = 0;
unknown(1,53) = 0;
unknown(1,54) = 0;
unknown(1,55) = 0;
unknown(1,56) = 0;
unknown(1,57) = (-g(1) * t148 - g(2) * t144 - t11 * t139);
unknown(1,58) = (-g(1) * t158 - g(2) * t154 - t151);
unknown(1,59) = -t19;
unknown(1,60) = (-g(3) * (-t163 - t47 + pkin(5)) - g(2) * (t166 - t51 + t169 + t52) - g(1) * (-t172 + t56 + t173 + t57));
unknown(1,61) = 0;
unknown(1,62) = 0;
unknown(1,63) = 0;
unknown(1,64) = 0;
unknown(1,65) = 0;
unknown(1,66) = 0;
unknown(1,67) = (-g(3) * (-t12 * t180 + t177 * t178) - g(2) * (t144 * t178 - t50 * t180) - g(1) * (t148 * t178 + t55 * t180));
unknown(1,68) = (-g(3) * (-t12 * t178 - t177 * t180) - g(2) * (-t144 * t180 - t50 * t178) - g(1) * (-t148 * t180 + t55 * t178));
unknown(1,69) = (g(1) * t158 + g(2) * t154 + t151);
unknown(1,70) = (-g(3) * (-t10 * t142 * pkin(10) + pkin(5) - t163 - t47) - g(2) * (-t154 * pkin(10) + t166 + t169 - t51 + t52) - g(1) * (-t158 * pkin(10) - t172 + t173 + t56 + t57));
unknown(1,71) = 0;
unknown(1,72) = 0;
unknown(1,73) = 0;
unknown(1,74) = 0;
unknown(1,75) = 0;
unknown(1,76) = 0;
unknown(1,77) = (-t11 * t220 - g(2) * (-t25 * t220 + t224) - g(1) * (t31 * t220 + t228));
unknown(1,78) = (-t11 * t223 - g(2) * (-t3 * t220 - t25 * t223) - g(1) * (-t1 * t220 + t31 * t223));
unknown(1,79) = -t19;
unknown(1,80) = t60;
unknown(1,81) = 0;
unknown(1,82) = 0;
unknown(1,83) = 0;
unknown(1,84) = 0;
unknown(1,85) = 0;
unknown(1,86) = 0;
unknown(1,87) = (-t11 * t243 - g(2) * (-t25 * t243 + t3 * t246) - g(1) * (t1 * t246 + t31 * t243));
unknown(1,88) = (-t11 * t246 - g(2) * (-t3 * t243 - t25 * t246) - g(1) * (-t1 * t243 + t31 * t246));
unknown(1,89) = -t19;
unknown(1,90) = (-g(3) * (-t10 * t220 * pkin(17) + pkin(5) - t47) - g(2) * (-t224 * pkin(17) + t25 * t269 - t51 + t52) - g(1) * (-t228 * pkin(17) - t31 * t269 + t56 + t57));
unknown(1,91) = 0;
unknown(1,92) = 0;
unknown(1,93) = 0;
unknown(1,94) = 0;
unknown(1,95) = 0;
unknown(1,96) = 0;
unknown(1,97) = (t11 * t280 - g(2) * (t25 * t280 + t3 * t283) - g(1) * (t1 * t283 - t31 * t280));
unknown(1,98) = (-t11 * t283 - g(2) * (-t25 * t283 + t3 * t280) - g(1) * (t1 * t280 + t31 * t283));
unknown(1,99) = -t19;
unknown(1,100) = (-g(3) * (-t84 * pkin(19) + pkin(5) - t47) - g(2) * (t28 * pkin(19) + t25 * t305 - t51 + t52) - g(1) * (t33 * pkin(19) - t31 * t305 + t56 + t57));
unknown(1,101) = 0;
unknown(1,102) = 0;
unknown(1,103) = 0;
unknown(1,104) = 0;
unknown(1,105) = 0;
unknown(1,106) = 0;
unknown(1,107) = (-t11 * t316 - g(2) * (-t25 * t316 - t3 * t319) - g(1) * (-t1 * t319 + t31 * t316));
unknown(1,108) = (t11 * t319 - g(2) * (t25 * t319 - t3 * t316) - g(1) * (-t1 * t316 - t31 * t319));
unknown(1,109) = -t19;
unknown(1,110) = (-g(3) * (-t10 * t339 + pkin(5) - t47) - g(2) * (t25 * t339 + t3 * t346 - t51 + t52) - g(1) * (t1 * t346 - t31 * t339 + t56 + t57));
unknown(1,111) = 0;
unknown(1,112) = 0;
unknown(1,113) = 0;
unknown(1,114) = 0;
unknown(1,115) = 0;
unknown(1,116) = 0;
unknown(1,117) = (t11 * t356 - g(2) * (t25 * t356 - t3 * t359) - g(1) * (-t1 * t359 - t31 * t356));
unknown(1,118) = (t11 * t359 - g(2) * (t25 * t359 + t3 * t356) - g(1) * (t1 * t356 - t31 * t359));
unknown(1,119) = -t19;
unknown(1,120) = t137;
unknown(1,121) = 0;
unknown(1,122) = 0;
unknown(1,123) = 0;
unknown(1,124) = 0;
unknown(1,125) = 0;
unknown(1,126) = 0;
unknown(1,127) = (-t11 * t379 - g(2) * (-t25 * t379 - t3 * t382) - g(1) * (-t1 * t382 + t31 * t379));
unknown(1,128) = (t11 * t382 - g(2) * (t25 * t382 - t3 * t379) - g(1) * (-t1 * t379 - t31 * t382));
unknown(1,129) = -t19;
unknown(1,130) = t137;
unknown(1,131) = 0;
unknown(1,132) = 0;
unknown(1,133) = 0;
unknown(1,134) = 0;
unknown(1,135) = 0;
unknown(1,136) = 0;
unknown(1,137) = t413;
unknown(1,138) = (-g(1) * t421 - g(2) * t417 - t414);
unknown(1,139) = -t19;
unknown(1,140) = (-g(3) * (-t426 - t47 + pkin(5)) - g(2) * (t429 - t51 + t432 + t52) - g(1) * (-t435 + t56 + t436 + t57));
unknown(1,141) = 0;
unknown(1,142) = 0;
unknown(1,143) = 0;
unknown(1,144) = 0;
unknown(1,145) = 0;
unknown(1,146) = 0;
unknown(1,147) = t413;
unknown(1,148) = -t19;
unknown(1,149) = (g(1) * t421 + g(2) * t417 + t414);
unknown(1,150) = (-g(3) * (-t10 * t405 * qJ(13) + pkin(5) - t426 - t47) - g(2) * (-t417 * qJ(13) + t429 + t432 - t51 + t52) - g(1) * (-t421 * qJ(13) - t435 + t436 + t56 + t57));
U_reg = unknown;
