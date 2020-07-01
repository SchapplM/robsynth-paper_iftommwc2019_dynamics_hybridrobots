% Jacobian of explicit kinematic constraints of
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
% W [13x5]
%  Derivative of the joint coordinates w.r.t minimal coordinates
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:16
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function W = KAS5m5_kinconstr_expl_jacobian_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m5_kinconstr_expl_jacobian_mdh_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m5_kinconstr_expl_jacobian_mdh_sym_varpar: pkin has to be [30x1] (double)');

%% Symbolic Calculation
% From kinconstr_expl_jacobian_matlab.m
% OptimizationMode: 2
% StartTime: 2020-06-27 17:51:32
% EndTime: 2020-06-27 17:52:00
% DurationCPUTime: 28.22s
% Computational Cost: add. (799005->136), mult. (1043142->210), div. (13096->26), fcn. (417288->18), ass. (0->105)
t163 = pkin(29) + qJ(3);
t160 = cos(t163);
t199 = t160 * pkin(18);
t148 = 0.2e1 * t199;
t170 = cos(qJ(3));
t171 = -pkin(25) + pkin(26);
t158 = t170 * t171;
t144 = -(2 * pkin(15)) + 0.2e1 * t158 + t148;
t169 = sin(qJ(3));
t157 = t169 * t171;
t156 = -0.2e1 * t157;
t159 = sin(t163);
t200 = t159 * pkin(18);
t145 = t156 + 0.2e1 * t200;
t149 = -pkin(15) + t158;
t168 = t171 ^ 2;
t175 = pkin(17) ^ 2;
t132 = t169 ^ 2 * t168 - pkin(23) ^ 2 + t149 ^ 2 + t175 + (t144 * t160 + t145 * t159 + (-t159 ^ 2 - t160 ^ 2) * pkin(18)) * pkin(18);
t137 = t144 ^ 2 + t145 ^ 2;
t177 = sqrt(-t132 ^ 2 + t175 * t137);
t126 = t132 * t145 + t144 * t177;
t135 = 0.1e1 / t137;
t188 = t157 - t200;
t123 = t126 * t135 + t188;
t121 = t169 * t123;
t127 = t144 * t132 - t145 * t177;
t186 = t149 + t199;
t122 = t127 * t135 - t186;
t119 = t170 * t122 - t121;
t116 = 0.1e1 / t119 ^ 2;
t117 = t169 * t122 + t170 * t123;
t146 = t156 - 0.2e1 * t200;
t147 = -0.2e1 * t158 + t148;
t131 = 0.2e1 * (-t149 * t171 + t168 * t170) * t169 + ((t145 + t146) * t160 + (-t144 + t147) * t159) * pkin(18);
t133 = 0.2e1 * t144 * t146 + 0.2e1 * t145 * t147;
t195 = t133 / t137 ^ 2;
t196 = (-0.2e1 * t132 * t131 + t175 * t133) / t177;
t184 = -t127 * t195 + (t146 * t132 + t144 * t131 - t147 * t177 - t145 * t196 / 0.2e1) * t135;
t189 = -t157 - t200;
t106 = t184 - t189;
t185 = -t126 * t195 + (t131 * t145 + t132 * t147 + t146 * t177 + t144 * t196 / 0.2e1) * t135;
t187 = t158 - t199;
t194 = t185 + t187 + t122;
t93 = (t106 - t123) * t170 - t194 * t169;
t94 = t169 * t106 + t194 * t170 - t121;
t208 = (t93 * t117 * t116 - t94 / t119) / (t117 ^ 2 * t116 + 0.1e1);
t162 = qJ(4) - qJ(3) + pkin(30);
t151 = sin(t162);
t154 = cos(t162);
t161 = -qJ(4) + t163;
t201 = cos(t161) * pkin(18);
t140 = -t151 * pkin(19) - t154 * pkin(20) - pkin(16) - t201;
t139 = 0.1e1 / t140 ^ 2;
t202 = sin(t161) * pkin(18);
t141 = -t154 * pkin(19) + t151 * pkin(20) + t202;
t152 = -sin(t162);
t155 = cos(t162);
t206 = (t139 * t141 * (t155 * pkin(19) + t152 * pkin(20) + t202) + 0.1e1 / t140 * (-t152 * pkin(19) + t155 * pkin(20) - t201)) / (t141 ^ 2 * t139 + 0.1e1) - 0.1e1;
t173 = pkin(21) ^ 2;
t172 = 0.1e1 / pkin(23);
t204 = pkin(22) * t172;
t191 = cos(pkin(14)) * t204;
t192 = sin(pkin(14)) * t204;
t104 = -t117 * t192 + t119 * t191;
t180 = pkin(26) + t104;
t101 = 0.2e1 * t180;
t102 = t117 * t191 + t119 * t192;
t103 = 0.2e1 * t102;
t85 = -pkin(24) ^ 2 + pkin(26) ^ 2 + t173 + (t101 - t104) * t104 + (t103 - t102) * t102;
t97 = t101 ^ 2 + t103 ^ 2;
t178 = sqrt(t173 * t97 - t85 ^ 2);
t86 = t93 * t191 - t94 * t192;
t87 = 0.2e1 * t86;
t88 = t94 * t191 + t93 * t192;
t89 = 0.2e1 * t88;
t75 = t101 * t86 + t103 * t88 + (-0.2e1 * t86 + t87) * t104 + (-0.2e1 * t88 + t89) * t102;
t81 = 0.2e1 * t101 * t87 + 0.2e1 * t103 * t89;
t205 = (t173 * t81 - 0.2e1 * t75 * t85) / t178;
t203 = pkin(23) * t172;
t198 = t81 / t97 ^ 2;
t193 = (t117 * t169 + t119 * t170) * t172;
t182 = -t101 * t85 + t103 * t178;
t96 = 0.1e1 / t97;
t65 = (-t87 * t85 - t101 * t75 + t89 * t178 + t103 * t205 / 0.2e1) * t96 - t182 * t198 + t86;
t80 = -t101 * t178 - t85 * t103;
t66 = -(-t75 * t103 - t85 * t89 - t87 * t178 - t101 * t205 / 0.2e1) * t96 + t80 * t198 - t88;
t78 = t182 * t96 + t180;
t77 = 0.1e1 / t78 ^ 2;
t79 = -t80 * t96 - t102;
t183 = (-t65 * t79 * t77 + t66 / t78) / (t77 * t79 ^ 2 + 0.1e1);
t176 = 0.1e1 / pkin(24);
t167 = cos(pkin(13));
t165 = sin(pkin(13));
t125 = 0.1e1 / t127 ^ 2;
t100 = (-t117 * t170 + t119 * t169) * t203 + t188;
t99 = t193 * pkin(23) + t186;
t98 = 0.1e1 / t99 ^ 2;
t73 = (t165 * t79 - t167 * t78) * t176;
t72 = (t165 * t78 + t167 * t79) * t176;
t70 = (t73 * t165 + t72 * t167) * pkin(24) + t102;
t69 = 0.1e1 / t70 ^ 2;
t68 = -(t72 * t165 - t73 * t167) * pkin(24) + t180;
t63 = (t165 * t66 - t167 * t65) * t176;
t62 = (t165 * t65 + t167 * t66) * t176;
t1 = [1, 0, 0, 0, 0; 0, 1, 0, 0, 0; 0, 0, t183, 0, 0; 0, 0, 1, 0, 0; 0, 0, 0, 1, 0; 0, 0, -1, 1, 0; 0, 0, 0, 0, 1; 0, 0, 0, 0, 0; 0, 0, (-(-(t62 * t165 - t63 * t167) * pkin(24) + t86) / t70 + ((t63 * t165 + t62 * t167) * pkin(24) + t88) * t68 * t69) / (t68 ^ 2 * t69 + 0.1e1) + t183, 0, 0; 0, 0, -t208, 0, 0; 0, 0, 0.1e1 + (-(((t169 * t93 - t170 * t94) * t172 + t193) * pkin(23) + t187) / t99 + (((t117 + t93) * t170 + (-t119 + t94) * t169) * t203 + t189) * t100 * t98) / (t100 ^ 2 * t98 + 0.1e1) + t208, 0, 0; 0, 0, 0.1e1 + (-t185 / t127 + t184 * t126 * t125) * t137 / (t126 ^ 2 * t125 + 0.1e1), 0, 0; 0, 0, t206, -t206, 0;];
W = t1;
