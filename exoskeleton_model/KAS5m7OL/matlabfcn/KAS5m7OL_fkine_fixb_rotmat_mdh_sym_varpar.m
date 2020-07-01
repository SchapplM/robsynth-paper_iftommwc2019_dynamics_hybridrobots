% Calculate forward kinematics (homogenous transformation matrices) for fixed-base
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% Tc_mdh [4x4x(21+1)]
%   homogenous transformation matrices for each (body) frame (MDH)
%   1:  mdh base (link 0) -> mdh base link 0 (unit matrix, no information)
%   ...
%   16:  mdh base (link 0) -> mdh frame (16-1), link (16-1)
%   ...
%   21+1:  mdh base (link 0) -> mdh frame (21)
% T_c_stack [(21+1)*3 x 4]
%   stacked matrices from Tc_mdh into one 2D array, last row left out.
%   Last row only contains [0 0 0 1].

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Tc_mdh, Tc_stack] = KAS5m7OL_fkine_fixb_rotmat_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_fkine_fixb_rotmat_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_fkine_fixb_rotmat_mdh_sym_varpar: pkin has to be [19x1] (double)');

%% Symbolic Calculation
% From fkine_mdh_floatb_twist_rotmat_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 17:46:17
% EndTime: 2020-06-30 17:46:17
% DurationCPUTime: 0.32s
% Computational Cost: add. (1158->329), mult. (813->234), div. (0->0), fcn. (1086->32), ass. (0->402)
unknown=NaN(66,4);
t1 = sin(qJ(1));
t2 = cos(qJ(1));
t3 = (pkin(5) + 0);
t4 = sin(qJ(2));
t5 = t1 * t4;
t6 = cos(qJ(2));
t7 = t1 * t6;
t8 = t2 * pkin(11);
t10 = t2 * t4;
t11 = t2 * t6;
t12 = t1 * pkin(11);
t14 = cos(qJ(3));
t16 = sin(qJ(3));
t17 = t2 * t16;
t22 = t7 * pkin(16);
t23 = t22 + t8 + 0;
t25 = t1 * t16;
t30 = t11 * pkin(16);
t31 = -t30 + t12 + 0;
t32 = t6 * t14;
t34 = t4 * pkin(16);
t35 = -t34 + pkin(5) + 0;
t36 = qJ(3) + qJ(4);
t37 = cos(t36);
t39 = sin(t36);
t45 = t14 * pkin(18);
t62 = qJ(3) + qJ(4) + qJ(5);
t63 = cos(t62);
t65 = sin(t62);
t71 = pkin(6) * t37;
t72 = t71 + t45;
t74 = pkin(6) * t39;
t75 = t16 * pkin(18);
t76 = t74 + t75;
t78 = t2 * t76 - t5 * t72 + t22 + t8 + 0;
t87 = t1 * t76 + t10 * t72 + t12 - t30 + 0;
t91 = -t6 * t72 + pkin(5) - t34 + 0;
t92 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
t93 = sin(t92);
t95 = cos(t92);
t97 = t2 * t95 + t5 * t93;
t100 = -t2 * t93 + t5 * t95;
t101 = pkin(7) * t63;
t102 = t101 + t71 + t45;
t103 = t5 * t102;
t104 = pkin(7) * t65;
t105 = t104 + t74 + t75;
t106 = t2 * t105;
t110 = t1 * t95 - t10 * t93;
t113 = -t1 * t93 - t10 * t95;
t114 = t10 * t102;
t115 = t1 * t105;
t117 = t6 * t93;
t118 = t6 * t95;
t119 = t6 * t102;
t121 = cos(qJ(7));
t123 = sin(qJ(7));
t147 = sin(pkin(3));
t149 = cos(pkin(3));
t150 = t2 * t149;
t156 = t1 * t149;
t161 = t6 * t147;
t163 = pkin(3) + qJ(8);
t164 = sin(t163);
t166 = cos(t163);
t168 = t5 * t164 + t2 * t166;
t171 = -t2 * t164 + t5 * t166;
t172 = t147 * pkin(17);
t178 = t1 * t166 - t10 * t164;
t181 = -t1 * t164 - t10 * t166;
t185 = t6 * t164;
t186 = t6 * t166;
t189 = qJ(3) + qJ(9);
t190 = cos(t189);
t192 = sin(t189);
t198 = t14 * pkin(19);
t215 = qJ(3) + qJ(9) + qJ(10);
t216 = cos(t215);
t218 = sin(t215);
t220 = -t2 * t218 + t5 * t216;
t223 = -t2 * t216 - t5 * t218;
t224 = pkin(14) * t190;
t225 = t224 + t198;
t227 = pkin(14) * t192;
t228 = t16 * pkin(19);
t229 = t227 + t228;
t234 = -t1 * t218 - t10 * t216;
t237 = -t1 * t216 + t10 * t218;
t241 = t6 * t216;
t242 = t6 * t218;
t245 = qJ(3) + qJ(4) + qJ(11);
t246 = sin(t245);
t248 = cos(t245);
t262 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
t263 = cos(t262);
t265 = sin(t262);
t267 = -t2 * t265 + t5 * t263;
t270 = -t2 * t263 - t5 * t265;
t273 = -t1 * t265 - t10 * t263;
t276 = -t1 * t263 + t10 * t265;
t277 = t6 * t263;
t278 = t6 * t265;
t279 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
t280 = sin(t279);
t282 = cos(t279);
t284 = t2 * t282 + t5 * t280;
t287 = -t2 * t280 + t5 * t282;
t289 = -pkin(9) * t263 + t45 + t71;
t290 = t5 * t289;
t292 = -pkin(9) * t265 + t74 + t75;
t293 = t2 * t292;
t294 = -t290 + t22 + t293 + t8 + 0;
t297 = t1 * t282 - t10 * t280;
t300 = -t1 * t280 - t10 * t282;
t301 = t10 * t289;
t302 = t1 * t292;
t303 = t301 - t30 + t302 + t12 + 0;
t304 = t6 * t280;
t305 = t6 * t282;
t306 = t6 * t289;
t307 = -t306 - t34 + pkin(5) + 0;
t309 = -t287 * qJ(13) + t22 - t290 + t293 + t8 + 0;
t311 = -t300 * qJ(13) + t12 - t30 + t301 + t302 + 0;
t313 = -t305 * qJ(13) + pkin(5) - t306 - t34 + 0;
t314 = qJ(3) + qJ(9) + pkin(4);
t315 = cos(t314);
t317 = sin(t314);
t324 = pkin(13) * t315 + t198;
t327 = pkin(13) * t317 + t228;
t343 = -qJ(3) - qJ(4) - qJ(5) - qJ(6) + pkin(2);
t344 = cos(t343);
t346 = sin(t343);
t353 = pkin(15) * t344 + t101 + t45 + t71;
t356 = -pkin(15) * t346 + t104 + t74 + t75;
t373 = -pkin(12) * t164 + t172;
t377 = pkin(12) * t166 - t149 * pkin(17);
t386 = -pkin(8) * t216 + t198 + t224;
t389 = -pkin(8) * t218 + t227 + t228;
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
unknown(10,1) = (-t5 * t14 + t17);
unknown(10,2) = (t2 * t14 + t5 * t16);
unknown(10,3) = t7;
unknown(10,4) = t23;
unknown(11,1) = (t10 * t14 + t25);
unknown(11,2) = (t1 * t14 - t10 * t16);
unknown(11,3) = -t11;
unknown(11,4) = t31;
unknown(12,1) = -t32;
unknown(12,2) = (t6 * t16);
unknown(12,3) = -t4;
unknown(12,4) = t35;
unknown(13,1) = (t2 * t39 - t5 * t37);
unknown(13,2) = (t2 * t37 + t5 * t39);
unknown(13,3) = t7;
unknown(13,4) = (t17 * pkin(18) - t5 * t45 + t22 + t8 + 0);
unknown(14,1) = (t1 * t39 + t10 * t37);
unknown(14,2) = (t1 * t37 - t10 * t39);
unknown(14,3) = -t11;
unknown(14,4) = (t25 * pkin(18) + t10 * t45 + t12 - t30 + 0);
unknown(15,1) = -(t6 * t37);
unknown(15,2) = (t6 * t39);
unknown(15,3) = -t4;
unknown(15,4) = (-t32 * pkin(18) + pkin(5) - t34 + 0);
unknown(16,1) = (t2 * t65 - t5 * t63);
unknown(16,2) = (t2 * t63 + t5 * t65);
unknown(16,3) = t7;
unknown(16,4) = t78;
unknown(17,1) = (t1 * t65 + t10 * t63);
unknown(17,2) = (t1 * t63 - t10 * t65);
unknown(17,3) = -t11;
unknown(17,4) = t87;
unknown(18,1) = -(t6 * t63);
unknown(18,2) = (t6 * t65);
unknown(18,3) = -t4;
unknown(18,4) = t91;
unknown(19,1) = t97;
unknown(19,2) = t100;
unknown(19,3) = t7;
unknown(19,4) = (-t103 + t22 + t106 + t8 + 0);
unknown(20,1) = t110;
unknown(20,2) = t113;
unknown(20,3) = -t11;
unknown(20,4) = (t114 - t30 + t115 + t12 + 0);
unknown(21,1) = t117;
unknown(21,2) = t118;
unknown(21,3) = -t4;
unknown(21,4) = (-t119 - t34 + pkin(5) + 0);
unknown(22,1) = (t97 * t121 + t7 * t123);
unknown(22,2) = (t7 * t121 - t97 * t123);
unknown(22,3) = -t100;
unknown(22,4) = (-t100 * pkin(10) - t103 + t106 + t22 + t8 + 0);
unknown(23,1) = (-t11 * t123 + t110 * t121);
unknown(23,2) = (-t11 * t121 - t110 * t123);
unknown(23,3) = -t113;
unknown(23,4) = (-t113 * pkin(10) + t114 + t115 + t12 - t30 + 0);
unknown(24,1) = (t117 * t121 - t4 * t123);
unknown(24,2) = (-t117 * t123 - t4 * t121);
unknown(24,3) = -t118;
unknown(24,4) = (-t118 * pkin(10) + pkin(5) - t119 - t34 + 0);
unknown(25,1) = (t5 * t147 + t150);
unknown(25,2) = (-t2 * t147 + t5 * t149);
unknown(25,3) = t7;
unknown(25,4) = t23;
unknown(26,1) = (-t10 * t147 + t156);
unknown(26,2) = (-t1 * t147 - t10 * t149);
unknown(26,3) = -t11;
unknown(26,4) = t31;
unknown(27,1) = t161;
unknown(27,2) = (t6 * t149);
unknown(27,3) = -t4;
unknown(27,4) = t35;
unknown(28,1) = t168;
unknown(28,2) = t171;
unknown(28,3) = t7;
unknown(28,4) = (-t150 * pkin(17) - t5 * t172 + t22 + t8 + 0);
unknown(29,1) = t178;
unknown(29,2) = t181;
unknown(29,3) = -t11;
unknown(29,4) = (-t156 * pkin(17) + t10 * t172 + t12 - t30 + 0);
unknown(30,1) = t185;
unknown(30,2) = t186;
unknown(30,3) = -t4;
unknown(30,4) = (-t161 * pkin(17) + pkin(5) - t34 + 0);
unknown(31,1) = (-t5 * t190 + t2 * t192);
unknown(31,2) = (t2 * t190 + t5 * t192);
unknown(31,3) = t7;
unknown(31,4) = (t17 * pkin(19) - t5 * t198 + t22 + t8 + 0);
unknown(32,1) = (t1 * t192 + t10 * t190);
unknown(32,2) = (t1 * t190 - t10 * t192);
unknown(32,3) = -t11;
unknown(32,4) = (t25 * pkin(19) + t10 * t198 + t12 - t30 + 0);
unknown(33,1) = -(t6 * t190);
unknown(33,2) = (t6 * t192);
unknown(33,3) = -t4;
unknown(33,4) = (-t32 * pkin(19) + pkin(5) - t34 + 0);
unknown(34,1) = t220;
unknown(34,2) = t223;
unknown(34,3) = t7;
unknown(34,4) = (t2 * t229 - t5 * t225 + t22 + t8 + 0);
unknown(35,1) = t234;
unknown(35,2) = t237;
unknown(35,3) = -t11;
unknown(35,4) = (t1 * t229 + t10 * t225 + t12 - t30 + 0);
unknown(36,1) = t241;
unknown(36,2) = -t242;
unknown(36,3) = -t4;
unknown(36,4) = (-t6 * t225 + pkin(5) - t34 + 0);
unknown(37,1) = (-t2 * t248 - t5 * t246);
unknown(37,2) = (t2 * t246 - t5 * t248);
unknown(37,3) = t7;
unknown(37,4) = t78;
unknown(38,1) = (-t1 * t248 + t10 * t246);
unknown(38,2) = (t1 * t246 + t10 * t248);
unknown(38,3) = -t11;
unknown(38,4) = t87;
unknown(39,1) = -(t6 * t246);
unknown(39,2) = -(t6 * t248);
unknown(39,3) = -t4;
unknown(39,4) = t91;
unknown(40,1) = t267;
unknown(40,2) = t270;
unknown(40,3) = t7;
unknown(40,4) = t78;
unknown(41,1) = t273;
unknown(41,2) = t276;
unknown(41,3) = -t11;
unknown(41,4) = t87;
unknown(42,1) = t277;
unknown(42,2) = -t278;
unknown(42,3) = -t4;
unknown(42,4) = t91;
unknown(43,1) = t284;
unknown(43,2) = t287;
unknown(43,3) = t7;
unknown(43,4) = t294;
unknown(44,1) = t297;
unknown(44,2) = t300;
unknown(44,3) = -t11;
unknown(44,4) = t303;
unknown(45,1) = t304;
unknown(45,2) = t305;
unknown(45,3) = -t4;
unknown(45,4) = t307;
unknown(46,1) = t284;
unknown(46,2) = t7;
unknown(46,3) = -t287;
unknown(46,4) = t309;
unknown(47,1) = t297;
unknown(47,2) = -t11;
unknown(47,3) = -t300;
unknown(47,4) = t311;
unknown(48,1) = t304;
unknown(48,2) = -t4;
unknown(48,3) = -t305;
unknown(48,4) = t313;
unknown(49,1) = (t2 * t317 - t5 * t315);
unknown(49,2) = (t2 * t315 + t5 * t317);
unknown(49,3) = t7;
unknown(49,4) = (t2 * t327 - t5 * t324 + t22 + t8 + 0);
unknown(50,1) = (t1 * t317 + t10 * t315);
unknown(50,2) = (t1 * t315 - t10 * t317);
unknown(50,3) = -t11;
unknown(50,4) = (t1 * t327 + t10 * t324 + t12 - t30 + 0);
unknown(51,1) = -(t6 * t315);
unknown(51,2) = (t6 * t317);
unknown(51,3) = -t4;
unknown(51,4) = (-t6 * t324 + pkin(5) - t34 + 0);
unknown(52,1) = t270;
unknown(52,2) = -t267;
unknown(52,3) = t7;
unknown(52,4) = t294;
unknown(53,1) = t276;
unknown(53,2) = -t273;
unknown(53,3) = -t11;
unknown(53,4) = t303;
unknown(54,1) = -t278;
unknown(54,2) = -t277;
unknown(54,3) = -t4;
unknown(54,4) = t307;
unknown(55,1) = (-t2 * t346 - t5 * t344);
unknown(55,2) = (t2 * t344 - t5 * t346);
unknown(55,3) = t7;
unknown(55,4) = (t2 * t356 - t5 * t353 + t22 + t8 + 0);
unknown(56,1) = (-t1 * t346 + t10 * t344);
unknown(56,2) = (t1 * t344 + t10 * t346);
unknown(56,3) = -t11;
unknown(56,4) = (t1 * t356 + t10 * t353 + t12 - t30 + 0);
unknown(57,1) = -(t6 * t344);
unknown(57,2) = -(t6 * t346);
unknown(57,3) = -t4;
unknown(57,4) = (-t6 * t353 + pkin(5) - t34 + 0);
unknown(58,1) = t168;
unknown(58,2) = t171;
unknown(58,3) = t7;
unknown(58,4) = (t2 * t377 - t5 * t373 + t22 + t8 + 0);
unknown(59,1) = t178;
unknown(59,2) = t181;
unknown(59,3) = -t11;
unknown(59,4) = (t1 * t377 + t10 * t373 + t12 - t30 + 0);
unknown(60,1) = t185;
unknown(60,2) = t186;
unknown(60,3) = -t4;
unknown(60,4) = (-t6 * t373 + pkin(5) - t34 + 0);
unknown(61,1) = t220;
unknown(61,2) = t223;
unknown(61,3) = t7;
unknown(61,4) = (t2 * t389 - t5 * t386 + t22 + t8 + 0);
unknown(62,1) = t234;
unknown(62,2) = t237;
unknown(62,3) = -t11;
unknown(62,4) = (t1 * t389 + t10 * t386 + t12 - t30 + 0);
unknown(63,1) = t241;
unknown(63,2) = -t242;
unknown(63,3) = -t4;
unknown(63,4) = (-t6 * t386 + pkin(5) - t34 + 0);
unknown(64,1) = t284;
unknown(64,2) = t287;
unknown(64,3) = t7;
unknown(64,4) = t309;
unknown(65,1) = t297;
unknown(65,2) = t300;
unknown(65,3) = -t11;
unknown(65,4) = t311;
unknown(66,1) = t304;
unknown(66,2) = t305;
unknown(66,3) = -t4;
unknown(66,4) = t313;
Tc_stack = unknown;
%% Postprocessing: Reshape Output
% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)
% Fallunterscheidung der Initialisierung für symbolische Eingabe
if isa([qJ; pkin], 'double'), Tc_mdh = NaN(4,4,21+1);               % numerisch
else,                         Tc_mdh = sym('xx', [4,4,21+1]); end % symbolisch
for i = 1:21+1
  Tc_mdh(:,:,i) = [Tc_stack((i-1)*3+1 : 3*i, :);[0 0 0 1]];
end
