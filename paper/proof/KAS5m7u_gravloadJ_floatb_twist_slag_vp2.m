% Calculate Gravitation load on the joints for
% KAS5m7u
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [20x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% m_mdh [16x1]
%   mass of all robot links (including the base)
% mrSges [16x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% 
% Output:
% taug [13x1]
%   joint torques required to compensate gravitation load

% Quelle: HybrDyn-Toolbox (ehem. IRT-Maple-Toolbox)
% Datum: 2018-11-06 18:51
% Revision: 51215904607767ba2b80495e06f47b388db9bd7a
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für mechatronische Systeme, Universität Hannover

function taug = KAS5m7u_gravloadJ_floatb_twist_slag_vp2(qJ, g, ...
  pkin, m, mrSges)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(3,1),zeros(20,1),zeros(16,1),zeros(16,3)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7u_gravloadJ_floatb_twist_slag_vp2: qJ has to be [13x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7u_gravloadJ_floatb_twist_slag_vp2: g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7u_gravloadJ_floatb_twist_slag_vp2: pkin has to be [20x1] (double)');
assert( isreal(m) && all(size(m) == [16 1]), ...
  'KAS5m7u_gravloadJ_floatb_twist_slag_vp2: m has to be [16x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [16,3]), ...
  'KAS5m7u_gravloadJ_floatb_twist_slag_vp2: mrSges has to be [16x3] (double)');

%% Symbolic Calculation
% From joint_gravload_floatb_twist_par2_matlab.m
% OptimizationMode: 2
% StartTime: 2018-11-06 18:13:29
% EndTime: 2018-11-06 18:13:37
% DurationCPUTime: 7.50s
% Computational Cost: add. (3061->422), mult. (2584->488), div. (0->0), fcn. (2505->28), ass. (0->228)
t178 = qJ(3) + qJ(9);
t170 = cos(t178);
t187 = cos(qJ(3));
t342 = pkin(20) * t187;
t132 = pkin(15) * t170 + t342;
t180 = sin(pkin(4));
t181 = cos(pkin(4));
t244 = m(10) * pkin(18) - mrSges(9,1);
t408 = m(12) * t132 - t181 * mrSges(9,2) + t180 * t244;
t383 = -mrSges(3,2) + mrSges(13,3) + mrSges(14,3) + mrSges(6,3);
t186 = cos(qJ(7));
t182 = sin(qJ(7));
t315 = mrSges(8,2) * t182;
t385 = -mrSges(8,1) * t186 + t315;
t403 = mrSges(7,1) - t385;
t401 = mrSges(15,1) + mrSges(16,1);
t406 = -mrSges(16,2) - mrSges(11,3) - mrSges(12,3) - mrSges(15,3) - mrSges(5,3) - mrSges(7,3) - mrSges(10,3) - mrSges(4,3) - mrSges(9,3);
t179 = qJ(3) + qJ(4);
t173 = qJ(11) + t179;
t157 = pkin(1) + t173;
t150 = qJ(12) + t157;
t141 = sin(t150);
t142 = cos(t150);
t148 = sin(t157);
t149 = cos(t157);
t174 = qJ(5) + t179;
t165 = qJ(6) + t174;
t152 = sin(t165);
t153 = cos(t165);
t172 = qJ(10) + t178;
t159 = sin(t172);
t160 = sin(t173);
t161 = cos(t172);
t162 = cos(t173);
t163 = sin(t174);
t164 = cos(t174);
t177 = pkin(4) + qJ(8);
t166 = sin(t177);
t167 = cos(t177);
t168 = sin(t178);
t169 = sin(t179);
t171 = cos(t179);
t175 = t187 * pkin(19);
t183 = sin(qJ(3));
t405 = m(5) * t175 + m(11) * t342 - t401 * t141 + mrSges(13,2) * t162 + mrSges(6,1) * t164 - mrSges(6,2) * t163 - mrSges(14,1) * t149 + mrSges(14,2) * t148 + t170 * mrSges(11,1) - t168 * mrSges(11,2) + t171 * mrSges(5,1) - t169 * mrSges(5,2) - t161 * mrSges(12,1) + t159 * mrSges(12,2) - t152 * mrSges(7,1) - t153 * mrSges(7,2) - t142 * mrSges(15,2) - t166 * mrSges(10,1) - t167 * mrSges(10,2) + mrSges(4,1) * t187 - mrSges(4,2) * t183 + mrSges(13,1) * t160 + t408;
t356 = -m(9) - m(4);
t404 = mrSges(7,2) - mrSges(8,3);
t374 = -m(11) * pkin(20) - m(5) * pkin(19);
t402 = t183 * t374;
t400 = mrSges(15,2) - mrSges(16,3);
t184 = sin(qJ(2));
t188 = cos(qJ(2));
t399 = mrSges(3,1) * t188 + t383 * t184;
t304 = m(16) * qJ(13);
t230 = mrSges(16,3) + t304;
t398 = -t142 * t230 - t405;
t397 = t356 * pkin(17) + t406;
t189 = cos(qJ(1));
t185 = sin(qJ(1));
t266 = t185 * t171;
t101 = -t169 * t189 + t184 * t266;
t279 = t185 * t152;
t41 = t153 * t189 + t184 * t279;
t278 = t185 * t153;
t42 = -t152 * t189 + t184 * t278;
t202 = -t403 * t42 + t404 * t41;
t273 = t185 * t163;
t74 = t164 * t189 + t184 * t273;
t272 = t185 * t164;
t75 = -t163 * t189 + t184 * t272;
t193 = -t74 * mrSges(6,1) - t75 * mrSges(6,2) + t202;
t283 = t185 * t141;
t17 = t142 * t189 + t184 * t283;
t282 = t185 * t142;
t18 = -t141 * t189 + t184 * t282;
t237 = t400 * t17 - t401 * t18;
t281 = t185 * t148;
t33 = -t149 * t189 - t184 * t281;
t280 = t185 * t149;
t34 = -t148 * t189 + t184 * t280;
t276 = t185 * t160;
t67 = t162 * t189 + t184 * t276;
t274 = t185 * t162;
t69 = t160 * t189 - t184 * t274;
t365 = -t69 * mrSges(13,1) - t33 * mrSges(14,1) - t67 * mrSges(13,2) + t34 * mrSges(14,2) + t237;
t268 = t185 * t169;
t99 = t171 * t189 + t184 * t268;
t396 = -t99 * mrSges(5,1) - t101 * mrSges(5,2) + t193 + t365;
t285 = t184 * t189;
t103 = t169 * t285 - t266;
t105 = -t171 * t285 - t268;
t43 = t152 * t285 - t278;
t44 = t153 * t285 + t279;
t201 = t403 * t44 - t404 * t43;
t76 = t163 * t285 - t272;
t77 = -t164 * t285 - t273;
t192 = t76 * mrSges(6,1) - t77 * mrSges(6,2) + t201;
t19 = t141 * t285 - t282;
t20 = t142 * t285 + t283;
t231 = -t400 * t19 + t401 * t20;
t35 = t148 * t285 - t280;
t36 = t149 * t285 + t281;
t71 = -t160 * t285 + t274;
t73 = t162 * t285 + t276;
t366 = -t73 * mrSges(13,1) - t35 * mrSges(14,1) - t71 * mrSges(13,2) - t36 * mrSges(14,2) + t231;
t395 = t103 * mrSges(5,1) - t105 * mrSges(5,2) + t192 + t366;
t262 = t186 * t188;
t293 = t153 * t188;
t295 = t152 * t188;
t239 = -t153 * mrSges(8,1) * t262 - mrSges(7,1) * t293 - mrSges(8,3) * t295;
t353 = m(8) * pkin(11);
t200 = t239 - t295 * t353 + (-mrSges(6,1) * t163 - mrSges(6,2) * t164) * t188;
t296 = t142 * t188;
t297 = t141 * t188;
t238 = -mrSges(16,3) * t297 - t401 * t296;
t364 = -t188 * t160 * mrSges(13,2) - t297 * t304 + t238;
t392 = t200 + (-mrSges(5,1) * t169 - mrSges(5,2) * t171) * t188 + t364;
t369 = -m(12) - m(10) - m(5) - m(11);
t357 = m(7) + m(8);
t340 = m(15) + m(16);
t250 = t153 * t315;
t206 = mrSges(7,2) * t152 + t250;
t379 = mrSges(13,1) * t162 + mrSges(15,2) * t141;
t387 = t206 + t379;
t384 = t340 + t357;
t382 = -m(13) - m(6);
t267 = t185 * t170;
t102 = t168 * t285 - t267;
t269 = t185 * t168;
t104 = -t170 * t285 - t269;
t275 = t185 * t161;
t70 = t159 * t285 - t275;
t277 = t185 * t159;
t72 = t161 * t285 + t277;
t328 = t70 * mrSges(12,1) + t72 * mrSges(12,2);
t378 = t102 * mrSges(11,1) - t104 * mrSges(11,2) - t328;
t100 = -t168 * t189 + t184 * t267;
t66 = -t161 * t189 - t184 * t277;
t68 = -t159 * t189 + t184 * t275;
t330 = t66 * mrSges(12,1) - t68 * mrSges(12,2);
t98 = t170 * t189 + t184 * t269;
t377 = -t98 * mrSges(11,1) - t100 * mrSges(11,2) - t330;
t251 = -m(14) + t382;
t372 = mrSges(4,1) - t374;
t361 = -t369 * pkin(17) - t397;
t350 = m(12) * pkin(15);
t347 = g(3) * t188;
t346 = pkin(11) * t41;
t345 = pkin(17) * t188;
t344 = pkin(19) * t183;
t343 = pkin(20) * t183;
t341 = t43 * pkin(11);
t311 = pkin(10) * t148;
t313 = pkin(7) * t169;
t110 = t311 - t313;
t88 = t110 - t344;
t158 = pkin(7) * t171;
t111 = -pkin(10) * t149 + t158;
t89 = t111 + t175;
t334 = t185 * t89 + t88 * t285;
t134 = t158 + t175;
t151 = pkin(8) * t164;
t107 = t151 + t134;
t312 = pkin(8) * t163;
t117 = -t312 - t313;
t106 = -t117 + t344;
t64 = t189 * t106;
t331 = t185 * t107 - t184 * t64;
t324 = t110 * t285 + t185 * t111;
t310 = pkin(15) * t168;
t131 = t310 + t343;
t319 = m(12) * t131;
t308 = t186 * mrSges(8,2);
t118 = t151 + t158;
t305 = t117 * t285 + t185 * t118;
t299 = qJ(13) * t19;
t294 = t153 * t184;
t291 = t17 * qJ(13);
t290 = t182 * t184;
t289 = t182 * t188;
t287 = t184 * t185;
t286 = t184 * t186;
t284 = t185 * t134;
t271 = t185 * t166;
t270 = t185 * t167;
t265 = t185 * t183;
t264 = t185 * t187;
t263 = t185 * t188;
t261 = t187 * t189;
t260 = t188 * t189;
t65 = t189 * t107;
t256 = t74 * pkin(8);
t254 = (mrSges(11,1) * t168 + mrSges(11,2) * t170) * t188;
t252 = t189 * pkin(12) + pkin(17) * t263;
t236 = mrSges(8,3) + t353;
t232 = t184 * t107 - t345;
t229 = pkin(7) * t251;
t228 = -mrSges(7,2) + t236;
t226 = t189 * t89 - t287 * t88;
t225 = t35 * pkin(10);
t224 = t106 * t287 + t65;
t223 = -t110 * t287 + t189 * t111;
t222 = -t117 * t287 + t189 * t118;
t221 = -mrSges(15,2) + t230;
t212 = -mrSges(12,1) * t159 - mrSges(12,2) * t161;
t209 = mrSges(14,1) * t148 + mrSges(14,2) * t149;
t207 = t76 * pkin(8);
t205 = -t152 * t262 + t290;
t204 = t152 * t289 + t286;
t114 = t183 * t285 - t264;
t112 = t184 * t265 + t261;
t203 = -m(3) * pkin(12) + t180 * mrSges(9,2) - mrSges(2,2) - mrSges(3,3);
t199 = t33 * pkin(10);
t191 = -t383 + t406;
t190 = (t107 * t357 - t187 * t374 + t340 * t89 + mrSges(3,1) + t408) * t184 - mrSges(2,1);
t156 = pkin(17) * t260;
t154 = pkin(17) * t285;
t147 = t182 * t260;
t133 = t313 + t344;
t115 = -t184 * t261 - t265;
t113 = -t183 * t189 + t184 * t264;
t94 = t167 * t285 + t271;
t93 = t166 * t285 - t270;
t92 = -t166 * t189 + t184 * t270;
t91 = t167 * t189 + t184 * t271;
t4 = t182 * t263 + t186 * t41;
t3 = -t182 * t41 + t185 * t262;
t1 = [(-t147 * mrSges(8,1) + t251 * (-t134 * t285 + t156 + (-pkin(12) - t133) * t185) + (-m(11) * (-pkin(12) - t343) - m(5) * (-pkin(12) - t344) - m(12) * (-pkin(12) - t131) + t181 * mrSges(9,1) - m(10) * (pkin(18) * t181 - pkin(12)) - t203 + t340 * (pkin(12) - t88) + t357 * (pkin(12) + t106)) * t185 - t114 * mrSges(4,2) - t115 * mrSges(4,1) - t102 * mrSges(11,2) - t103 * mrSges(5,2) - t104 * mrSges(11,1) - t105 * mrSges(5,1) - t93 * mrSges(10,1) - t94 * mrSges(10,2) + t73 * mrSges(13,2) - t76 * mrSges(6,2) - t77 * mrSges(6,1) + t70 * mrSges(12,2) - t71 * mrSges(13,1) - t72 * mrSges(12,1) + t221 * t20 + ((t191 - t308) * t188 + t190) * t189 + t356 * (-t185 * pkin(12) + t156) - t403 * t43 + t35 * mrSges(14,2) - t36 * mrSges(14,1) - t401 * t19 + (t369 - t384) * t156 + t228 * t44) * g(1) + (mrSges(6,1) * t75 - mrSges(6,2) * t74 + (t188 * t191 + t190) * t185 + (t251 + t356 + t369) * t252 + t251 * (t133 * t189 - t184 * t284) - t98 * mrSges(11,2) - t99 * mrSges(5,2) + t100 * mrSges(11,1) + t101 * mrSges(5,1) - t91 * mrSges(10,1) - t92 * mrSges(10,2) - t66 * mrSges(12,2) - t68 * mrSges(12,1) + t221 * t18 - t41 * mrSges(7,1) - t3 * mrSges(8,2) - t4 * mrSges(8,1) - t401 * t17 + (t181 * t244 + t203 - t319 + t402) * t189 + mrSges(4,1) * t113 - mrSges(4,2) * t112 + mrSges(13,1) * t67 - mrSges(13,2) * t69 - mrSges(14,1) * t34 - mrSges(14,2) * t33 + t228 * t42 - t340 * (-t189 * t88 + t252) - t357 * (t64 + t252)) * g(2) (-m(8) * (pkin(11) * t294 + t232) - (-t152 * t286 - t289) * mrSges(8,1) - (t152 * t290 - t262) * mrSges(8,2) - mrSges(8,3) * t294 - m(7) * t232 - t340 * (t184 * t89 - t345) + t251 * (t134 * t184 - t345) + (t361 + t383) * t188 + (-mrSges(3,1) + t398) * t184) * g(3) + ((t182 * mrSges(8,1) + t308 + t361 + (-t251 + t384) * pkin(17)) * t184 + (-t251 * t134 - m(8) * (-pkin(11) * t153 - t107) + t153 * mrSges(8,3) + t385 * t152 - m(16) * (-qJ(13) * t142 - t89) + t142 * mrSges(16,3) + m(15) * t89 + m(7) * t107 + t405) * t188 + t399) * g(1) * t185 + (-t340 * (t89 * t260 + t154) - t357 * (t188 * t65 + t154) + t251 * (t134 * t260 + t154) + t369 * t154 + (-mrSges(8,1) * t205 - mrSges(8,2) * t204 + t397 * t184 + t398 * t188 - t236 * t293 - t399) * t189) * g(2) (-m(14) * t133 - mrSges(4,1) * t183 - mrSges(4,2) * t187 + t209 - t212 - t319) * t347 + (-t254 + (-t106 * t357 + t382 * t133 + t340 * t88 + t387 + t402) * t188 + t392) * g(3) + (-m(16) * (-t299 + t334) - m(15) * t334 - m(12) * (-t131 * t285 + t185 * t132) - m(7) * t331 - mrSges(4,2) * t115 - m(8) * (t331 - t341) + t251 * (-t133 * t285 + t284) + t372 * t114 + t378 + t395) * g(2) + (-m(7) * t224 - m(12) * (t131 * t287 + t132 * t189) - mrSges(4,2) * t113 - m(16) * (t226 + t291) - m(15) * t226 - m(8) * (t224 + t346) + t251 * (t133 * t287 + t134 * t189) - t372 * t112 + t377 + t396) * g(1) ((t110 * t340 + t117 * t357 + t169 * t229 + t209 + t387) * t188 + t392) * g(3) + (-t103 * t229 - m(7) * t305 - m(8) * (t305 - t341) - m(15) * t324 - m(16) * (-t299 + t324) + t395) * g(2) + (t99 * t229 - m(15) * t223 - m(16) * (t223 + t291) - m(7) * t222 - m(8) * (t222 + t346) + t396) * g(1) ((-t312 * t357 + t206) * t188 + t200) * g(3) + (m(7) * t207 - m(8) * (-t207 - t341) + t192) * g(2) + (-m(7) * t256 - m(8) * (t256 + t346) + t193) * g(1) ((t250 + (mrSges(7,2) - t353) * t152) * t188 + t239) * g(3) + (m(8) * t341 + t201) * g(2) + (-m(8) * t346 + t202) * g(1), -g(2) * ((t43 * t182 - t186 * t260) * mrSges(8,1) + (t186 * t43 + t147) * mrSges(8,2)) - g(3) * (-mrSges(8,1) * t204 + mrSges(8,2) * t205) - g(1) * (mrSges(8,1) * t3 - mrSges(8,2) * t4) -g(1) * (mrSges(10,1) * t92 - mrSges(10,2) * t91) - g(2) * (-mrSges(10,1) * t94 + mrSges(10,2) * t93) - (mrSges(10,1) * t167 - mrSges(10,2) * t166) * t347, -g(3) * t254 - (m(12) * t310 + t212) * t347 + (t102 * t350 + t378) * g(2) + (-t350 * t98 + t377) * g(1), -g(1) * t330 - g(2) * t328 - t212 * t347, t209 * t347 + ((t311 * t340 + t379) * t188 + t364) * g(3) + (-m(15) * t225 - m(16) * (t225 - t299) + t366) * g(2) + (-m(15) * t199 - m(16) * (t199 + t291) + t365) * g(1) ((mrSges(15,2) - t304) * t297 + t238) * g(3) + (m(16) * t299 + t231) * g(2) + (-m(16) * t291 + t237) * g(1) (g(1) * t18 - g(2) * t20 + g(3) * t296) * m(16)];
taug  = t1(:);
