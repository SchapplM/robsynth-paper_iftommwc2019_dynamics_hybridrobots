% Calculate forward kinematics (homogenous transformation matrices) for fixed-base
% KAS5m3
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [6x1]
%   Generalized joint coordinates (joint angles)
% pkin [30x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8,delta8s,delta9s,l11,l12,l13,l14,l17,l18,l20,l21,l22,l4,l5,l6,delta10s,delta12s,delta17s,delta18s]';
% 
% Output:
% Tc_mdh [4x4x(13+1)]
%   homogenous transformation matrices for each (body) frame (MDH)
%   1:  mdh base (link 0) -> mdh base link 0 (unit matrix, no information)
%   ...
%   14:  mdh base (link 0) -> mdh frame (14-1), link (14-1)
%   ...
%   13+1:  mdh base (link 0) -> mdh frame (13)
% T_c_stack [(13+1)*3 x 4]
%   stacked matrices from Tc_mdh into one 2D array, last row left out.
%   Last row only contains [0 0 0 1].

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 17:47
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Tc_mdh, Tc_stack] = KAS5m3_fkine_fixb_rotmat_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(6,1),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [6 1]), ...
  'KAS5m3_fkine_fixb_rotmat_mdh_sym_varpar: qJ has to be [6x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m3_fkine_fixb_rotmat_mdh_sym_varpar: pkin has to be [30x1] (double)');

%% Symbolic Calculation
% From fkine_mdh_floatb_twist_rotmat_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-27 16:33:11
% EndTime: 2020-06-27 16:35:40
% DurationCPUTime: 145.37s
% Computational Cost: add. (9927942->282), mult. (12814746->307), div. (209026->7), fcn. (5138143->26), ass. (0->350)
unknown=NaN(42,4);
t1 = sin(qJ(1));
t2 = cos(qJ(1));
t3 = (pkin(8) + 0);
t4 = sin(qJ(2));
t5 = t1 * t4;
t6 = cos(qJ(2));
t7 = t1 * t6;
t8 = t2 * pkin(9);
t10 = t2 * t4;
t11 = t2 * t6;
t12 = t1 * pkin(9);
t14 = cos(qJ(3));
t15 = -pkin(25) + pkin(26);
t16 = t14 * t15;
t17 = pkin(29) + qJ(3);
t18 = cos(t17);
t19 = t18 * pkin(18);
t20 = -pkin(15) + t16 + t19;
t21 = pkin(17) ^ 2;
t22 = pkin(23) ^ 2;
t23 = t18 ^ 2;
t24 = pkin(18) ^ 2;
t26 = sin(t17);
t27 = t26 ^ 2;
t30 = (-pkin(15) + t16) ^ 2;
t31 = sin(qJ(3));
t32 = t31 ^ 2;
t33 = t15 ^ 2;
t37 = t31 * t15;
t38 = t26 * pkin(18);
t39 = -t37 + t38;
t42 = 0.2e1 * t20 * t18 * pkin(18) + 0.2e1 * t39 * t26 * pkin(18) - t23 * t24 - t27 * t24 + t32 * t33 + t21 - t22 + t30;
t46 = 0.4e1 * t20 ^ 2 + 0.4e1 * t39 ^ 2;
t48 = t42 ^ 2;
t50 = sqrt(t21 * t46 - t48);
t53 = 0.1e1 / t46;
t54 = (0.2e1 * t20 * t42 - 0.2e1 * t39 * t50) * t53;
t55 = pkin(15) - t16 - t19 + t54;
t60 = (0.2e1 * t20 * t50 + 0.2e1 * t42 * t39) * t53;
t61 = t37 - t38 + t60;
t63 = t14 * t55 - t31 * t61;
t64 = 0.1e1 / pkin(23);
t65 = t63 * t64;
t66 = cos(pkin(14));
t67 = t66 * pkin(22);
t68 = t65 * t67;
t71 = t14 * t61 + t31 * t55;
t72 = t71 * t64;
t73 = sin(pkin(14));
t74 = t73 * pkin(22);
t75 = t72 * t74;
t76 = -pkin(26) - t68 + t75;
t77 = pkin(21) ^ 2;
t78 = pkin(24) ^ 2;
t79 = t68 - t75;
t80 = t79 ^ 2;
t81 = t72 * t67;
t82 = t65 * t74;
t83 = t81 + t82;
t84 = t83 ^ 2;
t85 = pkin(26) ^ 2;
t88 = -0.2e1 * t76 * t79 + 0.2e1 * t83 ^ 2 + t77 - t78 - t80 - t84 + t85;
t92 = 0.4e1 * t76 ^ 2 + 0.4e1 * t83 ^ 2;
t94 = t88 ^ 2;
t96 = sqrt(t77 * t92 - t94);
t99 = 0.1e1 / t92;
t102 = 0.1e1 / pkin(24);
t103 = (-t68 + t75 - (0.2e1 * t76 * t88 + 0.2e1 * t83 * t96) * t99 - pkin(26)) * t102;
t104 = sin(pkin(13));
t111 = (-t81 - t82 - (0.2e1 * t76 * t96 - 0.2e1 * t88 * t83) * t99) * t102;
t112 = cos(pkin(13));
t114 = -t103 * t104 + t111 * t112;
t118 = t103 * t112 + t111 * t104;
t120 = -t5 * t114 + t2 * t118;
t123 = t2 * t114 + t5 * t118;
t124 = t7 * pkin(10);
t128 = t1 * t118 + t10 * t114;
t131 = t1 * t114 - t10 * t118;
t132 = t11 * pkin(10);
t134 = t6 * t114;
t135 = -t6 * t118;
t136 = t4 * pkin(10);
t140 = t120 * t14 + t123 * t31;
t143 = -t120 * t31 + t123 * t14;
t144 = t120 * pkin(4);
t148 = t128 * t14 + t131 * t31;
t151 = -t128 * t31 + t131 * t14;
t152 = t128 * pkin(4);
t156 = -t134 * t14 - t135 * t31;
t159 = t134 * t31 - t135 * t14;
t160 = t134 * pkin(4);
t162 = cos(qJ(4));
t164 = sin(qJ(4));
t166 = t140 * t162 + t143 * t164;
t169 = -t140 * t164 + t143 * t162;
t170 = t140 * pkin(5);
t174 = t148 * t162 + t151 * t164;
t177 = -t148 * t164 + t151 * t162;
t178 = t148 * pkin(5);
t182 = t156 * t162 + t159 * t164;
t185 = -t156 * t164 + t159 * t162;
t186 = t156 * pkin(5);
t188 = sin(qJ(5));
t190 = cos(qJ(5));
t192 = -t166 * t188 + t169 * t190;
t195 = -t166 * t190 - t169 * t188;
t196 = t166 * pkin(6);
t200 = -t174 * t188 + t177 * t190;
t203 = -t174 * t190 - t177 * t188;
t204 = t174 * pkin(6);
t208 = -t182 * t188 + t185 * t190;
t211 = -t182 * t190 - t185 * t188;
t212 = t182 * pkin(6);
t214 = cos(qJ(6));
t216 = sin(qJ(6));
t242 = t5 * t104 + t2 * t112;
t245 = -t2 * t104 + t5 * t112;
t246 = t7 * pkin(12);
t250 = t1 * t112 - t10 * t104;
t253 = -t1 * t104 - t10 * t112;
t254 = t11 * pkin(12);
t256 = t6 * t104;
t257 = t6 * t112;
t258 = t4 * pkin(12);
t262 = t118 * t104 + t114 * t112;
t264 = t262 * pkin(24) + t81 + t82;
t266 = 0.1e1 / pkin(21);
t270 = t114 * t104 - t118 * t112;
t272 = -t270 * pkin(24) + pkin(26) + t68 - t75;
t275 = t262 * t264 * t266 - t270 * t272 * t266;
t281 = -t262 * t272 * t266 - t264 * t266 * t270;
t309 = t120 * t63 * t64 + t123 * t71 * t64;
t314 = -t120 * t71 * t64 + t123 * t63 * t64;
t315 = t120 * pkin(1);
t321 = t128 * t63 * t64 + t131 * t71 * t64;
t326 = -t128 * t71 * t64 + t131 * t63 * t64;
t327 = t128 * pkin(1);
t331 = -t134 * t65 - t135 * t72;
t334 = t134 * t72 - t135 * t65;
t335 = t134 * pkin(1);
t341 = t14 * t63 * t64 + t31 * t71 * t64;
t343 = t341 * pkin(23) - pkin(15) + t16 + t19;
t345 = 0.1e1 / pkin(17);
t351 = -t14 * t71 * t64 + t31 * t63 * t64;
t353 = t351 * pkin(23) + t37 - t38;
t356 = -t341 * t343 * t345 - t351 * t353 * t345;
t362 = t341 * t353 * t345 - t343 * t345 * t351;
t364 = t309 * t356 + t314 * t362;
t367 = -t309 * t362 + t314 * t356;
t368 = t309 * pkin(2);
t372 = t321 * t356 + t326 * t362;
t375 = -t321 * t362 + t326 * t356;
t376 = t321 * pkin(2);
t380 = t331 * t356 + t334 * t362;
t383 = -t331 * t362 + t334 * t356;
t384 = t331 * pkin(2);
t386 = t345 * t18;
t388 = t345 * t26;
t390 = t54 * t386 + t60 * t388;
t394 = -t60 * t386 + t54 * t388;
t396 = t364 * t390 + t367 * t394;
t399 = -t364 * t394 + t367 * t390;
t401 = t364 * pkin(3) + t124 + t315 + t368 + t8 + 0;
t404 = t372 * t390 + t375 * t394;
t407 = -t372 * t394 + t375 * t390;
t409 = t372 * pkin(3) + t12 - t132 + t327 + t376 + 0;
t412 = t380 * t390 + t383 * t394;
t415 = -t380 * t394 + t383 * t390;
t417 = t380 * pkin(3) + pkin(8) - t136 - t335 + t384 + 0;
t420 = pkin(29) + qJ(3) - qJ(4);
t421 = cos(t420);
t423 = -t421 * pkin(18) - t188 * pkin(19) - t190 * pkin(20) - pkin(16);
t424 = t423 ^ 2;
t427 = sin(t420);
t429 = -t427 * pkin(18) + t190 * pkin(19) - t188 * pkin(20);
t430 = t429 ^ 2;
t432 = sqrt(t424 + t430);
t433 = 0.1e1 / t432;
t434 = t423 * t433;
t436 = -t429 * t433;
t438 = t434 * t421 - t436 * t427;
t442 = -t436 * t421 - t434 * t427;
unknown(1,1) = 1;
unknown(1,2) = 0;
unknown(1,3) = 0;
unknown(1,4) = 0;
unknown(2,1) = 0;
unknown(2,2) = 1;
unknown(2,3) = 0;
unknown(2,4) = 0;
unknown(3,1) = 0;
unknown(3,2) = 0;
unknown(3,3) = 1;
unknown(3,4) = 0;
unknown(4,1) = t1;
unknown(4,2) = t2;
unknown(4,3) = 0;
unknown(4,4) = 0;
unknown(5,1) = -t2;
unknown(5,2) = t1;
unknown(5,3) = 0;
unknown(5,4) = 0;
unknown(6,1) = 0;
unknown(6,2) = 0;
unknown(6,3) = 1;
unknown(6,4) = t3;
unknown(7,1) = -t5;
unknown(7,2) = -t7;
unknown(7,3) = t2;
unknown(7,4) = (t8 + 0);
unknown(8,1) = t10;
unknown(8,2) = t11;
unknown(8,3) = t1;
unknown(8,4) = (t12 + 0);
unknown(9,1) = -t6;
unknown(9,2) = t4;
unknown(9,3) = 0;
unknown(9,4) = t3;
unknown(10,1) = t120;
unknown(10,2) = t123;
unknown(10,3) = t7;
unknown(10,4) = (t124 + t8 + 0);
unknown(11,1) = t128;
unknown(11,2) = t131;
unknown(11,3) = -t11;
unknown(11,4) = (-t132 + t12 + 0);
unknown(12,1) = -t134;
unknown(12,2) = -t135;
unknown(12,3) = -t4;
unknown(12,4) = (-t136 + pkin(8) + 0);
unknown(13,1) = t140;
unknown(13,2) = t143;
unknown(13,3) = t7;
unknown(13,4) = (t144 + t124 + t8 + 0);
unknown(14,1) = t148;
unknown(14,2) = t151;
unknown(14,3) = -t11;
unknown(14,4) = (t152 - t132 + t12 + 0);
unknown(15,1) = t156;
unknown(15,2) = t159;
unknown(15,3) = -t4;
unknown(15,4) = (-t160 - t136 + pkin(8) + 0);
unknown(16,1) = t166;
unknown(16,2) = t169;
unknown(16,3) = t7;
unknown(16,4) = (t170 + t144 + t124 + t8 + 0);
unknown(17,1) = t174;
unknown(17,2) = t177;
unknown(17,3) = -t11;
unknown(17,4) = (t178 + t152 - t132 + t12 + 0);
unknown(18,1) = t182;
unknown(18,2) = t185;
unknown(18,3) = -t4;
unknown(18,4) = (t186 - t160 - t136 + pkin(8) + 0);
unknown(19,1) = t192;
unknown(19,2) = t195;
unknown(19,3) = t7;
unknown(19,4) = (t196 + t170 + t144 + t124 + t8 + 0);
unknown(20,1) = t200;
unknown(20,2) = t203;
unknown(20,3) = -t11;
unknown(20,4) = (t204 + t178 + t152 - t132 + t12 + 0);
unknown(21,1) = t208;
unknown(21,2) = t211;
unknown(21,3) = -t4;
unknown(21,4) = (t212 + t186 - t160 - t136 + pkin(8) + 0);
unknown(22,1) = (t192 * t214 + t7 * t216);
unknown(22,2) = (-t192 * t216 + t7 * t214);
unknown(22,3) = -t195;
unknown(22,4) = (-t195 * pkin(11) + t124 + t144 + t170 + t196 + t8 + 0);
unknown(23,1) = (-t11 * t216 + t200 * t214);
unknown(23,2) = (-t11 * t214 - t200 * t216);
unknown(23,3) = -t203;
unknown(23,4) = (-t203 * pkin(11) + t12 - t132 + t152 + t178 + t204 + 0);
unknown(24,1) = (t208 * t214 - t4 * t216);
unknown(24,2) = (-t208 * t216 - t4 * t214);
unknown(24,3) = -t211;
unknown(24,4) = (-t211 * pkin(11) + pkin(8) - t136 - t160 + t186 + t212 + 0);
unknown(25,1) = t242;
unknown(25,2) = t245;
unknown(25,3) = t7;
unknown(25,4) = (t246 + t8 + 0);
unknown(26,1) = t250;
unknown(26,2) = t253;
unknown(26,3) = -t11;
unknown(26,4) = (-t254 + t12 + 0);
unknown(27,1) = t256;
unknown(27,2) = t257;
unknown(27,3) = -t4;
unknown(27,4) = (-t258 + pkin(8) + 0);
unknown(28,1) = (t242 * t275 + t245 * t281);
unknown(28,2) = (-t242 * t281 + t245 * t275);
unknown(28,3) = t7;
unknown(28,4) = (t242 * pkin(7) + t246 + t8 + 0);
unknown(29,1) = (t250 * t275 + t253 * t281);
unknown(29,2) = (-t250 * t281 + t253 * t275);
unknown(29,3) = -t11;
unknown(29,4) = (t250 * pkin(7) + t12 - t254 + 0);
unknown(30,1) = (t256 * t275 + t257 * t281);
unknown(30,2) = (-t256 * t281 + t257 * t275);
unknown(30,3) = -t4;
unknown(30,4) = (t256 * pkin(7) + pkin(8) - t258 + 0);
unknown(31,1) = t309;
unknown(31,2) = t314;
unknown(31,3) = t7;
unknown(31,4) = (t315 + t124 + t8 + 0);
unknown(32,1) = t321;
unknown(32,2) = t326;
unknown(32,3) = -t11;
unknown(32,4) = (t327 - t132 + t12 + 0);
unknown(33,1) = t331;
unknown(33,2) = t334;
unknown(33,3) = -t4;
unknown(33,4) = (-t335 - t136 + pkin(8) + 0);
unknown(34,1) = t364;
unknown(34,2) = t367;
unknown(34,3) = t7;
unknown(34,4) = (t368 + t315 + t124 + t8 + 0);
unknown(35,1) = t372;
unknown(35,2) = t375;
unknown(35,3) = -t11;
unknown(35,4) = (t376 + t327 - t132 + t12 + 0);
unknown(36,1) = t380;
unknown(36,2) = t383;
unknown(36,3) = -t4;
unknown(36,4) = (t384 - t335 - t136 + pkin(8) + 0);
unknown(37,1) = t396;
unknown(37,2) = t399;
unknown(37,3) = t7;
unknown(37,4) = t401;
unknown(38,1) = t404;
unknown(38,2) = t407;
unknown(38,3) = -t11;
unknown(38,4) = t409;
unknown(39,1) = t412;
unknown(39,2) = t415;
unknown(39,3) = -t4;
unknown(39,4) = t417;
unknown(40,1) = (t396 * t438 + t399 * t442);
unknown(40,2) = (-t396 * t442 + t399 * t438);
unknown(40,3) = t7;
unknown(40,4) = t401;
unknown(41,1) = (t404 * t438 + t407 * t442);
unknown(41,2) = (-t404 * t442 + t407 * t438);
unknown(41,3) = -t11;
unknown(41,4) = t409;
unknown(42,1) = (t412 * t438 + t415 * t442);
unknown(42,2) = (-t412 * t442 + t415 * t438);
unknown(42,3) = -t4;
unknown(42,4) = t417;
Tc_stack = unknown;
%% Postprocessing: Reshape Output
% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)
% Fallunterscheidung der Initialisierung für symbolische Eingabe
if isa([qJ; pkin], 'double'), Tc_mdh = NaN(4,4,13+1);               % numerisch
else,                         Tc_mdh = sym('xx', [4,4,13+1]); end % symbolisch
for i = 1:13+1
  Tc_mdh(:,:,i) = [Tc_stack((i-1)*3+1 : 3*i, :);[0 0 0 1]];
end
