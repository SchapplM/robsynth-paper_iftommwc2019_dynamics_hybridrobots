% Jacobian of explicit kinematic constraints of
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
% W [13x6]
%  Derivative of the joint coordinates w.r.t minimal coordinates
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 17:47
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function W = KAS5m3_kinconstr_expl_jacobian_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(6,1),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [6 1]), ...
  'KAS5m3_kinconstr_expl_jacobian_mdh_sym_varpar: qJ has to be [6x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m3_kinconstr_expl_jacobian_mdh_sym_varpar: pkin has to be [30x1] (double)');

%% Symbolic Calculation
% From kinconstr_expl_jacobian_matlab.m
% OptimizationMode: 2
% StartTime: 2020-06-27 16:26:56
% EndTime: 2020-06-27 16:27:26
% DurationCPUTime: 28.77s
% Computational Cost: add. (798962->131), mult. (1043165->212), div. (13102->26), fcn. (417305->16), ass. (0->107)
t157 = pkin(29) + qJ(3);
t154 = cos(t157);
t197 = t154 * pkin(18);
t146 = 0.2e1 * t197;
t166 = cos(qJ(3));
t167 = -pkin(25) + pkin(26);
t152 = t166 * t167;
t142 = -(2 * pkin(15)) + 0.2e1 * t152 + t146;
t164 = sin(qJ(3));
t151 = t164 * t167;
t150 = -0.2e1 * t151;
t153 = sin(t157);
t198 = t153 * pkin(18);
t143 = t150 + 0.2e1 * t198;
t147 = -pkin(15) + t152;
t162 = t167 ^ 2;
t171 = pkin(17) ^ 2;
t132 = t164 ^ 2 * t162 - pkin(23) ^ 2 + t147 ^ 2 + t171 + (t142 * t154 + t143 * t153 + (-t153 ^ 2 - t154 ^ 2) * pkin(18)) * pkin(18);
t137 = t142 ^ 2 + t143 ^ 2;
t173 = sqrt(-t132 ^ 2 + t171 * t137);
t125 = t132 * t143 + t142 * t173;
t135 = 0.1e1 / t137;
t185 = t151 - t198;
t122 = t125 * t135 + t185;
t120 = t164 * t122;
t126 = t142 * t132 - t143 * t173;
t182 = t147 + t197;
t121 = t126 * t135 - t182;
t118 = t166 * t121 - t120;
t115 = 0.1e1 / t118 ^ 2;
t116 = t164 * t121 + t166 * t122;
t144 = t150 - 0.2e1 * t198;
t145 = -0.2e1 * t152 + t146;
t131 = 0.2e1 * (-t147 * t167 + t162 * t166) * t164 + ((t143 + t144) * t154 + (-t142 + t145) * t153) * pkin(18);
t133 = 0.2e1 * t142 * t144 + 0.2e1 * t143 * t145;
t193 = t133 / t137 ^ 2;
t194 = (-0.2e1 * t132 * t131 + t171 * t133) / t173;
t180 = -t126 * t193 + (t144 * t132 + t142 * t131 - t145 * t173 - t143 * t194 / 0.2e1) * t135;
t186 = -t151 - t198;
t105 = t180 - t186;
t181 = -t125 * t193 + (t131 * t143 + t132 * t145 + t144 * t173 + t142 * t194 / 0.2e1) * t135;
t184 = t152 - t197;
t191 = t181 + t184 + t121;
t92 = (t105 - t122) * t166 - t191 * t164;
t93 = t164 * t105 + t166 * t191 - t120;
t203 = (t92 * t116 * t115 - t93 / t118) / (t116 ^ 2 * t115 + 0.1e1);
t169 = pkin(21) ^ 2;
t168 = 0.1e1 / pkin(23);
t200 = pkin(22) * t168;
t188 = cos(pkin(14)) * t200;
t189 = sin(pkin(14)) * t200;
t103 = -t116 * t189 + t118 * t188;
t176 = pkin(26) + t103;
t100 = 0.2e1 * t176;
t101 = t116 * t188 + t118 * t189;
t102 = 0.2e1 * t101;
t84 = -pkin(24) ^ 2 + pkin(26) ^ 2 + t169 + (t100 - t103) * t103 + (t102 - t101) * t101;
t96 = t100 ^ 2 + t102 ^ 2;
t174 = sqrt(t169 * t96 - t84 ^ 2);
t85 = t188 * t92 - t93 * t189;
t86 = 0.2e1 * t85;
t87 = t93 * t188 + t92 * t189;
t88 = 0.2e1 * t87;
t74 = t100 * t85 + t102 * t87 + (-0.2e1 * t85 + t86) * t103 + (-0.2e1 * t87 + t88) * t101;
t80 = 0.2e1 * t100 * t86 + 0.2e1 * t102 * t88;
t201 = (t169 * t80 - 0.2e1 * t74 * t84) / t174;
t199 = pkin(23) * t168;
t196 = t80 / t96 ^ 2;
t155 = -qJ(4) + t157;
t149 = cos(t155);
t163 = sin(qJ(5));
t165 = cos(qJ(5));
t179 = t163 * pkin(19) + t165 * pkin(20);
t140 = -t149 * pkin(18) - pkin(16) - t179;
t139 = 0.1e1 / t140 ^ 2;
t148 = sin(t155);
t183 = -t165 * pkin(19) + t163 * pkin(20);
t141 = t148 * pkin(18) + t183;
t192 = t139 * t141;
t190 = (t116 * t164 + t118 * t166) * t168;
t177 = -t100 * t84 + t102 * t174;
t95 = 0.1e1 / t96;
t64 = (-t86 * t84 - t100 * t74 + t88 * t174 + t102 * t201 / 0.2e1) * t95 - t177 * t196 + t85;
t79 = -t100 * t174 - t84 * t102;
t65 = -(-t74 * t102 - t84 * t88 - t86 * t174 - t100 * t201 / 0.2e1) * t95 + t79 * t196 - t87;
t77 = t177 * t95 + t176;
t76 = 0.1e1 / t77 ^ 2;
t78 = -t79 * t95 - t101;
t178 = (-t64 * t78 * t76 + t65 / t77) / (t76 * t78 ^ 2 + 0.1e1);
t172 = 0.1e1 / pkin(24);
t161 = cos(pkin(13));
t159 = sin(pkin(13));
t138 = 0.1e1 / t140;
t134 = 0.1e1 / (t141 ^ 2 * t139 + 0.1e1);
t127 = 0.1e1 + (t138 * t149 - t148 * t192) * t134 * pkin(18);
t124 = 0.1e1 / t126 ^ 2;
t99 = (-t116 * t166 + t118 * t164) * t199 + t185;
t98 = pkin(23) * t190 + t182;
t97 = 0.1e1 / t98 ^ 2;
t72 = (t159 * t78 - t161 * t77) * t172;
t71 = (t159 * t77 + t161 * t78) * t172;
t69 = (t72 * t159 + t71 * t161) * pkin(24) + t101;
t68 = 0.1e1 / t69 ^ 2;
t67 = -(t71 * t159 - t72 * t161) * pkin(24) + t176;
t62 = (t159 * t65 - t161 * t64) * t172;
t61 = (t159 * t64 + t161 * t65) * t172;
t1 = [1, 0, 0, 0, 0, 0; 0, 1, 0, 0, 0, 0; 0, 0, t178, 0, 0, 0; 0, 0, 1, 0, 0, 0; 0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 1; 0, 0, 0, 0, 0, 0; 0, 0, (-(-(t61 * t159 - t62 * t161) * pkin(24) + t85) / t69 + ((t62 * t159 + t61 * t161) * pkin(24) + t87) * t67 * t68) / (t67 ^ 2 * t68 + 0.1e1) + t178, 0, 0, 0; 0, 0, -t203, 0, 0, 0; 0, 0, 0.1e1 + (-(((t164 * t92 - t166 * t93) * t168 + t190) * pkin(23) + t184) / t98 + (((t116 + t92) * t166 + (-t118 + t93) * t164) * t199 + t186) * t99 * t97) / (t99 ^ 2 * t97 + 0.1e1) + t203, 0, 0, 0; 0, 0, 0.1e1 + (-t181 / t126 + t180 * t125 * t124) * t137 / (t125 ^ 2 * t124 + 0.1e1), 0, 0, 0; 0, 0, -t127, t127, (-t138 * t179 + t183 * t192) * t134, 0;];
W = t1;
