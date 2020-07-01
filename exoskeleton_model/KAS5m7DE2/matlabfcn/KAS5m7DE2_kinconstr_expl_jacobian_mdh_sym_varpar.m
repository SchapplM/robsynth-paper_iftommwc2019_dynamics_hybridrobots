% Jacobian of explicit kinematic constraints of
% KAS5m7DE2
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% W [21x5]
%  Derivative of the joint coordinates w.r.t minimal coordinates
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-03 15:49
% Revision: caa0dbda1e8a16d11faaa29ba3bbef6afcd619f7 (2020-05-25)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function W = KAS5m7DE2_kinconstr_expl_jacobian_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE2_kinconstr_expl_jacobian_mdh_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE2_kinconstr_expl_jacobian_mdh_sym_varpar: pkin has to be [24x1] (double)');

%% Symbolic Calculation
% From kinconstr_expl_jacobian_matlab.m
% OptimizationMode: 2
% StartTime: 2020-05-25 11:40:46
% EndTime: 2020-05-25 11:41:16
% DurationCPUTime: 29.31s
% Computational Cost: add. (799109->135), mult. (1043196->210), div. (13096->26), fcn. (417328->18), ass. (0->110)
t167 = pkin(3) + qJ(3);
t164 = cos(t167);
t207 = t164 * pkin(12);
t156 = 0.2e1 * t207;
t174 = cos(qJ(3));
t175 = -pkin(23) + pkin(24);
t162 = t174 * t175;
t149 = -(2 * pkin(9)) + 0.2e1 * t162 + t156;
t173 = sin(qJ(3));
t161 = t173 * t175;
t157 = -0.2e1 * t161;
t163 = sin(t167);
t208 = t163 * pkin(12);
t150 = t157 + 0.2e1 * t208;
t155 = -pkin(9) + t162;
t168 = t175 ^ 2;
t179 = pkin(11) ^ 2;
t135 = t173 ^ 2 * t168 - pkin(19) ^ 2 + t155 ^ 2 + t179 + (t149 * t164 + t150 * t163 + (-t163 ^ 2 - t164 ^ 2) * pkin(12)) * pkin(12);
t141 = t149 ^ 2 + t150 ^ 2;
t181 = sqrt(-t135 ^ 2 + t141 * t179);
t129 = t135 * t150 + t149 * t181;
t139 = 0.1e1 / t141;
t196 = t161 - t208;
t126 = t129 * t139 + t196;
t124 = t173 * t126;
t130 = t135 * t149 - t150 * t181;
t193 = t155 + t207;
t125 = t130 * t139 - t193;
t122 = t125 * t174 - t124;
t119 = 0.1e1 / t122 ^ 2;
t120 = t173 * t125 + t174 * t126;
t151 = t157 - 0.2e1 * t208;
t152 = -0.2e1 * t162 + t156;
t134 = 0.2e1 * (-t155 * t175 + t168 * t174) * t173 + ((t150 + t151) * t164 + (-t149 + t152) * t163) * pkin(12);
t136 = 0.2e1 * t149 * t151 + 0.2e1 * t150 * t152;
t203 = t136 / t141 ^ 2;
t204 = (-0.2e1 * t134 * t135 + t136 * t179) / t181;
t191 = -t130 * t203 + (t151 * t135 + t149 * t134 - t152 * t181 - t150 * t204 / 0.2e1) * t139;
t197 = -t161 - t208;
t109 = t191 - t197;
t192 = -t129 * t203 + (t134 * t150 + t135 * t152 + t151 * t181 + t149 * t204 / 0.2e1) * t139;
t195 = t162 - t207;
t202 = t192 + t195 + t125;
t96 = (t109 - t126) * t174 - t202 * t173;
t97 = t173 * t109 + t174 * t202 - t124;
t217 = (t96 * t120 * t119 - t97 / t122) / (t119 * t120 ^ 2 + 0.1e1);
t166 = -qJ(4) + t167;
t154 = cos(t166) * pkin(12);
t165 = qJ(4) - qJ(3) + pkin(4);
t158 = sin(t165);
t160 = cos(t165);
t190 = pkin(14) * t158 + pkin(15) * t160;
t144 = -pkin(10) - t154 - t190;
t194 = pkin(14) * t160 - pkin(15) * t158;
t209 = sin(t166) * pkin(12);
t145 = -t194 + t209;
t146 = -t154 + t190;
t147 = t194 + t209;
t184 = t144 ^ 2;
t213 = t145 ^ 2;
t216 = (t184 + t213) ^ (-0.1e1 / 0.2e1) * (t144 * t147 - t145 * t146);
t143 = 0.1e1 / t184;
t214 = (t143 * t145 * t147 + 0.1e1 / t144 * t146) / (t143 * t213 + 0.1e1) - 0.1e1;
t177 = pkin(17) ^ 2;
t176 = 0.1e1 / pkin(19);
t211 = pkin(18) * t176;
t199 = cos(pkin(7)) * t211;
t200 = sin(pkin(7)) * t211;
t107 = -t120 * t200 + t122 * t199;
t185 = pkin(24) + t107;
t104 = 0.2e1 * t185;
t105 = t120 * t199 + t122 * t200;
t106 = 0.2e1 * t105;
t100 = t104 ^ 2 + t106 ^ 2;
t88 = -pkin(22) ^ 2 + pkin(24) ^ 2 + t177 + (t104 - t107) * t107 + (t106 - t105) * t105;
t182 = sqrt(t100 * t177 - t88 ^ 2);
t89 = t199 * t96 - t200 * t97;
t90 = 0.2e1 * t89;
t91 = t199 * t97 + t200 * t96;
t92 = 0.2e1 * t91;
t78 = t104 * t89 + t106 * t91 + (-0.2e1 * t89 + t90) * t107 + (-0.2e1 * t91 + t92) * t105;
t84 = 0.2e1 * t104 * t90 + 0.2e1 * t106 * t92;
t212 = (t177 * t84 - 0.2e1 * t78 * t88) / t182;
t210 = pkin(19) * t176;
t206 = t84 / t100 ^ 2;
t201 = (t120 * t173 + t122 * t174) * t176;
t188 = -t104 * t88 + t106 * t182;
t99 = 0.1e1 / t100;
t68 = (-t90 * t88 - t104 * t78 + t92 * t182 + t106 * t212 / 0.2e1) * t99 - t188 * t206 + t89;
t83 = -t104 * t182 - t106 * t88;
t69 = -(-t78 * t106 - t88 * t92 - t90 * t182 - t104 * t212 / 0.2e1) * t99 + t83 * t206 - t91;
t81 = t188 * t99 + t185;
t80 = 0.1e1 / t81 ^ 2;
t82 = -t83 * t99 - t105;
t189 = (-t68 * t82 * t80 + t69 / t81) / (t80 * t82 ^ 2 + 0.1e1);
t180 = 0.1e1 / pkin(22);
t172 = cos(pkin(6));
t170 = sin(pkin(6));
t128 = 0.1e1 / t130 ^ 2;
t103 = (-t120 * t174 + t122 * t173) * t210 + t196;
t102 = pkin(19) * t201 + t193;
t101 = 0.1e1 / t102 ^ 2;
t76 = (t170 * t82 - t172 * t81) * t180;
t75 = (t170 * t81 + t172 * t82) * t180;
t73 = (t170 * t76 + t172 * t75) * pkin(22) + t105;
t72 = 0.1e1 / t73 ^ 2;
t71 = -(t170 * t75 - t172 * t76) * pkin(22) + t185;
t66 = (t170 * t69 - t172 * t68) * t180;
t65 = (t170 * t68 + t172 * t69) * t180;
t1 = [1, 0, 0, 0, 0; 0, 1, 0, 0, 0; 0, 0, t189, 0, 0; 0, 0, 1, 0, 0; 0, 0, 0, 1, 0; 0, 0, -1, 1, 0; 0, 0, 0, 0, 1; 0, 0, 0, 0, 0; 0, 0, (-(-(t170 * t65 - t172 * t66) * pkin(22) + t89) / t73 + ((t170 * t66 + t172 * t65) * pkin(22) + t91) * t71 * t72) / (t71 ^ 2 * t72 + 0.1e1) + t189, 0, 0; 0, 0, -t217, 0, 0; 0, 0, 0.1e1 + (-(((t173 * t96 - t174 * t97) * t176 + t201) * pkin(19) + t195) / t102 + (((t120 + t96) * t174 + (-t122 + t97) * t173) * t210 + t197) * t103 * t101) / (t101 * t103 ^ 2 + 0.1e1) + t217, 0, 0; 0, 0, 1, 0, 0; 0, 0, 0, 0, 0; 0, 0, t214, -t214, 0; 0, 0, t216, -t216, 0; 0, 0, 0, 0, 0; 0, 0, -0.1e1 + (t192 / t130 - t191 * t129 * t128) * t141 / (t128 * t129 ^ 2 + 0.1e1), 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0;];
W = t1;
