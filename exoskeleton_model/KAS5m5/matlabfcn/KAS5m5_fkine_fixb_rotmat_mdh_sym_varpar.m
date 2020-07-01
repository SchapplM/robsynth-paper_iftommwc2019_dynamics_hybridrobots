% Calculate forward kinematics (homogenous transformation matrices) for fixed-base
% KAS5m5
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
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
% Datum: 2020-06-27 19:16
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Tc_mdh, Tc_stack] = KAS5m5_fkine_fixb_rotmat_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m5_fkine_fixb_rotmat_mdh_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m5_fkine_fixb_rotmat_mdh_sym_varpar: pkin has to be [30x1] (double)');

%% Symbolic Calculation
% From fkine_mdh_floatb_twist_rotmat_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-27 17:57:44
% EndTime: 2020-06-27 18:00:11
% DurationCPUTime: 145.97s
% Computational Cost: add. (9928302->284), mult. (12814746->307), div. (209026->7), fcn. (5138143->26), ass. (0->351)
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
t188 = -qJ(4) + qJ(3) - pkin(30);
t189 = sin(t188);
t191 = cos(t188);
t193 = t166 * t189 + t169 * t191;
t196 = -t166 * t191 + t169 * t189;
t197 = t166 * pkin(6);
t201 = t174 * t189 + t177 * t191;
t204 = -t174 * t191 + t177 * t189;
t205 = t174 * pkin(6);
t209 = t182 * t189 + t185 * t191;
t212 = -t182 * t191 + t185 * t189;
t213 = t182 * pkin(6);
t215 = cos(qJ(5));
t217 = sin(qJ(5));
t243 = t5 * t104 + t2 * t112;
t246 = -t2 * t104 + t5 * t112;
t247 = t7 * pkin(12);
t251 = t1 * t112 - t10 * t104;
t254 = -t1 * t104 - t10 * t112;
t255 = t11 * pkin(12);
t257 = t6 * t104;
t258 = t6 * t112;
t259 = t4 * pkin(12);
t263 = t118 * t104 + t114 * t112;
t265 = t263 * pkin(24) + t81 + t82;
t267 = 0.1e1 / pkin(21);
t271 = t114 * t104 - t118 * t112;
t273 = -t271 * pkin(24) + pkin(26) + t68 - t75;
t276 = t263 * t265 * t267 - t271 * t273 * t267;
t282 = -t263 * t273 * t267 - t265 * t267 * t271;
t310 = t120 * t63 * t64 + t123 * t71 * t64;
t315 = -t120 * t71 * t64 + t123 * t63 * t64;
t316 = t120 * pkin(1);
t322 = t128 * t63 * t64 + t131 * t71 * t64;
t327 = -t128 * t71 * t64 + t131 * t63 * t64;
t328 = t128 * pkin(1);
t332 = -t134 * t65 - t135 * t72;
t335 = t134 * t72 - t135 * t65;
t336 = t134 * pkin(1);
t342 = t14 * t63 * t64 + t31 * t71 * t64;
t344 = t342 * pkin(23) - pkin(15) + t16 + t19;
t346 = 0.1e1 / pkin(17);
t352 = -t14 * t71 * t64 + t31 * t63 * t64;
t354 = t352 * pkin(23) + t37 - t38;
t357 = -t342 * t344 * t346 - t352 * t354 * t346;
t363 = t342 * t354 * t346 - t344 * t346 * t352;
t365 = t310 * t357 + t315 * t363;
t368 = -t310 * t363 + t315 * t357;
t369 = t310 * pkin(2);
t373 = t322 * t357 + t327 * t363;
t376 = -t322 * t363 + t327 * t357;
t377 = t322 * pkin(2);
t381 = t332 * t357 + t335 * t363;
t384 = -t332 * t363 + t335 * t357;
t385 = t332 * pkin(2);
t387 = t346 * t18;
t389 = t346 * t26;
t391 = t54 * t387 + t60 * t389;
t395 = -t60 * t387 + t54 * t389;
t397 = t365 * t391 + t368 * t395;
t400 = -t365 * t395 + t368 * t391;
t402 = t365 * pkin(3) + t124 + t316 + t369 + t8 + 0;
t405 = t373 * t391 + t376 * t395;
t408 = -t373 * t395 + t376 * t391;
t410 = t373 * pkin(3) + t12 - t132 + t328 + t377 + 0;
t413 = t381 * t391 + t384 * t395;
t416 = -t381 * t395 + t384 * t391;
t418 = t381 * pkin(3) + pkin(8) - t136 - t336 + t385 + 0;
t421 = pkin(29) + qJ(3) - qJ(4);
t422 = cos(t421);
t424 = -t422 * pkin(18) + t189 * pkin(19) - t191 * pkin(20) - pkin(16);
t425 = t424 ^ 2;
t428 = sin(t421);
t430 = -t428 * pkin(18) + t191 * pkin(19) + t189 * pkin(20);
t431 = t430 ^ 2;
t433 = sqrt(t425 + t431);
t434 = 0.1e1 / t433;
t435 = t424 * t434;
t437 = -t430 * t434;
t439 = t435 * t422 - t437 * t428;
t443 = -t437 * t422 - t435 * t428;
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
unknown(19,1) = t193;
unknown(19,2) = t196;
unknown(19,3) = t7;
unknown(19,4) = (t197 + t170 + t144 + t124 + t8 + 0);
unknown(20,1) = t201;
unknown(20,2) = t204;
unknown(20,3) = -t11;
unknown(20,4) = (t205 + t178 + t152 - t132 + t12 + 0);
unknown(21,1) = t209;
unknown(21,2) = t212;
unknown(21,3) = -t4;
unknown(21,4) = (t213 + t186 - t160 - t136 + pkin(8) + 0);
unknown(22,1) = (t193 * t215 + t7 * t217);
unknown(22,2) = (-t193 * t217 + t7 * t215);
unknown(22,3) = -t196;
unknown(22,4) = (-t196 * pkin(11) + t124 + t144 + t170 + t197 + t8 + 0);
unknown(23,1) = (-t11 * t217 + t201 * t215);
unknown(23,2) = (-t11 * t215 - t201 * t217);
unknown(23,3) = -t204;
unknown(23,4) = (-t204 * pkin(11) + t12 - t132 + t152 + t178 + t205 + 0);
unknown(24,1) = (t209 * t215 - t4 * t217);
unknown(24,2) = (-t209 * t217 - t4 * t215);
unknown(24,3) = -t212;
unknown(24,4) = (-t212 * pkin(11) + pkin(8) - t136 - t160 + t186 + t213 + 0);
unknown(25,1) = t243;
unknown(25,2) = t246;
unknown(25,3) = t7;
unknown(25,4) = (t247 + t8 + 0);
unknown(26,1) = t251;
unknown(26,2) = t254;
unknown(26,3) = -t11;
unknown(26,4) = (-t255 + t12 + 0);
unknown(27,1) = t257;
unknown(27,2) = t258;
unknown(27,3) = -t4;
unknown(27,4) = (-t259 + pkin(8) + 0);
unknown(28,1) = (t243 * t276 + t246 * t282);
unknown(28,2) = (-t243 * t282 + t246 * t276);
unknown(28,3) = t7;
unknown(28,4) = (t243 * pkin(7) + t247 + t8 + 0);
unknown(29,1) = (t251 * t276 + t254 * t282);
unknown(29,2) = (-t251 * t282 + t254 * t276);
unknown(29,3) = -t11;
unknown(29,4) = (t251 * pkin(7) + t12 - t255 + 0);
unknown(30,1) = (t257 * t276 + t258 * t282);
unknown(30,2) = (-t257 * t282 + t258 * t276);
unknown(30,3) = -t4;
unknown(30,4) = (t257 * pkin(7) + pkin(8) - t259 + 0);
unknown(31,1) = t310;
unknown(31,2) = t315;
unknown(31,3) = t7;
unknown(31,4) = (t316 + t124 + t8 + 0);
unknown(32,1) = t322;
unknown(32,2) = t327;
unknown(32,3) = -t11;
unknown(32,4) = (t328 - t132 + t12 + 0);
unknown(33,1) = t332;
unknown(33,2) = t335;
unknown(33,3) = -t4;
unknown(33,4) = (-t336 - t136 + pkin(8) + 0);
unknown(34,1) = t365;
unknown(34,2) = t368;
unknown(34,3) = t7;
unknown(34,4) = (t369 + t316 + t124 + t8 + 0);
unknown(35,1) = t373;
unknown(35,2) = t376;
unknown(35,3) = -t11;
unknown(35,4) = (t377 + t328 - t132 + t12 + 0);
unknown(36,1) = t381;
unknown(36,2) = t384;
unknown(36,3) = -t4;
unknown(36,4) = (t385 - t336 - t136 + pkin(8) + 0);
unknown(37,1) = t397;
unknown(37,2) = t400;
unknown(37,3) = t7;
unknown(37,4) = t402;
unknown(38,1) = t405;
unknown(38,2) = t408;
unknown(38,3) = -t11;
unknown(38,4) = t410;
unknown(39,1) = t413;
unknown(39,2) = t416;
unknown(39,3) = -t4;
unknown(39,4) = t418;
unknown(40,1) = (t397 * t439 + t400 * t443);
unknown(40,2) = (-t397 * t443 + t400 * t439);
unknown(40,3) = t7;
unknown(40,4) = t402;
unknown(41,1) = (t405 * t439 + t408 * t443);
unknown(41,2) = (-t405 * t443 + t408 * t439);
unknown(41,3) = -t11;
unknown(41,4) = t410;
unknown(42,1) = (t413 * t439 + t416 * t443);
unknown(42,2) = (-t413 * t443 + t416 * t439);
unknown(42,3) = -t4;
unknown(42,4) = t418;
Tc_stack = unknown;
%% Postprocessing: Reshape Output
% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)
% Fallunterscheidung der Initialisierung für symbolische Eingabe
if isa([qJ; pkin], 'double'), Tc_mdh = NaN(4,4,13+1);               % numerisch
else,                         Tc_mdh = sym('xx', [4,4,13+1]); end % symbolisch
for i = 1:13+1
  Tc_mdh(:,:,i) = [Tc_stack((i-1)*3+1 : 3*i, :);[0 0 0 1]];
end
