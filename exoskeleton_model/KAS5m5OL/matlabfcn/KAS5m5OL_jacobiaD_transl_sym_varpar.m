% Zeitableitung der analytischen Jacobi-Matrix (Translatorisch) für beliebiges Segment von
% KAS5m5OL
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% (Ist für translatorischen Teil egal, kennzeichnet nur den Rechenweg der Herleitung)
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% qJD [13x1]
%   Generalized joint velocities
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt (0=Basis).
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% pkin [12x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8]';
% 
% Output:
% JaD_transl [3x13]
%   Translatorischer Teil der analytischen Jacobi-Matrix (Zeitableitung)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function JaD_transl = KAS5m5OL_jacobiaD_transl_sym_varpar(qJ, qJD, link_index, r_i_i_C, ...
  pkin)


%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),uint8(0),zeros(3,1),zeros(12,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_jacobiaD_transl_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m5OL_jacobiaD_transl_sym_varpar: qJD has to be [13x1] (double)');
assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ...
	'KAS5m5OL_jacobiaD_transl_sym_varpar: Position vector r_i_i_C has to be [3x1] double');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m5OL_jacobiaD_transl_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_jacobiaD_transl_sym_varpar: pkin has to be [12x1] (double)');
JaD_transl=NaN(3,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobiaD_transl_0_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.01s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobiaD_transl_1_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.04s
	% Computational Cost: add. (2->2), mult. (8->6), div. (0->0), fcn. (4->2), ass. (0->3)
	t27 = cos(qJ(1));
	t26 = sin(qJ(1));
	t1 = [(-r_i_i_C(1) * t26 - r_i_i_C(2) * t27) * qJD(1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (r_i_i_C(1) * t27 - r_i_i_C(2) * t26) * qJD(1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobiaD_transl_2_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.07s
	% Computational Cost: add. (17->13), mult. (60->29), div. (0->0), fcn. (38->4), ass. (0->12)
	t16 = sin(qJ(2));
	t18 = cos(qJ(2));
	t20 = (r_i_i_C(1) * t18 - r_i_i_C(2) * t16) * qJD(2);
	t27 = -pkin(9) - r_i_i_C(3);
	t17 = sin(qJ(1));
	t26 = qJD(1) * t17;
	t19 = cos(qJ(1));
	t25 = qJD(1) * t19;
	t24 = qJD(2) * t17;
	t23 = qJD(2) * t19;
	t21 = r_i_i_C(1) * t16 + r_i_i_C(2) * t18;
	t1 = [-t19 * t20 + (t21 * t17 + t27 * t19) * qJD(1), (t16 * t25 + t18 * t24) * r_i_i_C(2) + (t16 * t24 - t18 * t25) * r_i_i_C(1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t17 * t20 + (t27 * t17 - t21 * t19) * qJD(1), (t16 * t26 - t18 * t23) * r_i_i_C(2) + (-t16 * t23 - t18 * t26) * r_i_i_C(1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobiaD_transl_3_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:40
	% EndTime: 2020-06-27 19:20:41
	% DurationCPUTime: 0.14s
	% Computational Cost: add. (72->34), mult. (240->68), div. (0->0), fcn. (192->6), ass. (0->30)
	t205 = cos(qJ(2));
	t201 = sin(qJ(3));
	t204 = cos(qJ(3));
	t211 = r_i_i_C(1) * t204 - r_i_i_C(2) * t201;
	t202 = sin(qJ(2));
	t222 = pkin(10) + r_i_i_C(3);
	t216 = t222 * t202;
	t226 = t211 * t205 + t216;
	t214 = qJD(3) * t202 - qJD(1);
	t224 = t214 * t201;
	t223 = t214 * t204;
	t215 = t222 * t205;
	t203 = sin(qJ(1));
	t221 = qJD(1) * t203;
	t220 = qJD(2) * t202;
	t219 = qJD(2) * t205;
	t206 = cos(qJ(1));
	t218 = qJD(2) * t206;
	t217 = qJD(3) * t205;
	t213 = qJD(1) * t202 - qJD(3);
	t212 = qJD(2) * t216;
	t210 = r_i_i_C(1) * t201 + r_i_i_C(2) * t204;
	t209 = t213 * t206;
	t208 = t210 * qJD(3);
	t207 = t213 * t203 - t205 * t218;
	t200 = t204 * t209 + (t204 * t219 - t224) * t203;
	t199 = t203 * t223 + (t203 * t219 + t209) * t201;
	t198 = t207 * t204 + t206 * t224;
	t197 = t207 * t201 - t206 * t223;
	t1 = [t198 * r_i_i_C(1) - t197 * r_i_i_C(2) - t206 * t212 + (-pkin(9) * t206 - t203 * t215) * qJD(1), -t226 * t206 * qJD(1) + (t210 * t217 + (t211 * t202 - t215) * qJD(2)) * t203, t199 * r_i_i_C(1) + t200 * r_i_i_C(2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t200 * r_i_i_C(1) + t199 * r_i_i_C(2) - t203 * t212 + (-pkin(9) * t203 + t206 * t215) * qJD(1), (-t211 * t218 - t222 * t221) * t202 + (-t211 * t221 + (t222 * qJD(2) - t208) * t206) * t205, t197 * r_i_i_C(1) + t198 * r_i_i_C(2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t226 * qJD(2) - t202 * t208, (-t201 * t217 - t204 * t220) * r_i_i_C(2) + (-t201 * t220 + t204 * t217) * r_i_i_C(1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobiaD_transl_4_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:40
	% EndTime: 2020-06-27 19:20:42
	% DurationCPUTime: 0.23s
	% Computational Cost: add. (241->51), mult. (416->86), div. (0->0), fcn. (330->8), ass. (0->50)
	t239 = sin(qJ(2));
	t242 = cos(qJ(2));
	t274 = pkin(10) + r_i_i_C(3);
	t256 = t274 * t242;
	t241 = cos(qJ(3));
	t269 = pkin(4) * qJD(3);
	t258 = t241 * t269;
	t273 = pkin(4) * t241;
	t282 = (-t239 * t273 + t256) * qJD(1) + t258;
	t237 = qJ(3) + qJ(4);
	t234 = sin(t237);
	t270 = r_i_i_C(2) * t234;
	t235 = cos(t237);
	t271 = r_i_i_C(1) * t235;
	t248 = -t270 + t271 + t273;
	t257 = t274 * t239;
	t280 = t248 * t242 + t257;
	t240 = sin(qJ(1));
	t236 = qJD(3) + qJD(4);
	t253 = t236 * t239 - qJD(1);
	t279 = t240 * t253;
	t251 = qJD(3) * t239 - qJD(1);
	t278 = t241 * t251;
	t243 = cos(qJ(1));
	t277 = t243 * t253;
	t238 = sin(qJ(3));
	t272 = r_i_i_C(1) * t234;
	t249 = r_i_i_C(2) * t235 + t272;
	t275 = t249 * t236 + t238 * t269;
	t268 = t236 * t242;
	t265 = qJD(1) * t239;
	t252 = -t236 + t265;
	t261 = qJD(2) * t243;
	t254 = t242 * t261;
	t246 = t252 * t240 - t254;
	t229 = t246 * t234 - t235 * t277;
	t230 = t234 * t277 + t246 * t235;
	t267 = t229 * r_i_i_C(1) + t230 * r_i_i_C(2);
	t262 = qJD(2) * t242;
	t255 = t240 * t262;
	t247 = t252 * t243 + t255;
	t231 = t247 * t234 + t235 * t279;
	t232 = -t234 * t279 + t247 * t235;
	t266 = t231 * r_i_i_C(1) + t232 * r_i_i_C(2);
	t264 = qJD(1) * t240;
	t263 = qJD(2) * t239;
	t250 = -qJD(3) + t265;
	t244 = -pkin(9) * qJD(1) - qJD(2) * t257 + (t251 * t238 - t241 * t262) * pkin(4);
	t233 = t268 * t271;
	t1 = [t230 * r_i_i_C(1) - t229 * r_i_i_C(2) - t282 * t240 + t244 * t243, -t280 * t243 * qJD(1) + (t275 * t242 + (t248 * t239 - t256) * qJD(2)) * t240, (t240 * t278 + (t250 * t243 + t255) * t238) * pkin(4) + t266, t266, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t232 * r_i_i_C(1) + t231 * r_i_i_C(2) + t244 * t240 + t282 * t243, (-t248 * t261 - t274 * t264) * t239 + (-t248 * t264 + (t274 * qJD(2) - t275) * t243) * t242, (-t243 * t278 + (t250 * t240 - t254) * t238) * pkin(4) + t267, t267, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t280 * qJD(2) - t239 * t275, t233 + (-t236 * t270 + t258) * t242 + (-pkin(4) * t238 - t249) * t263, -t263 * t272 + t233 + (-t234 * t268 - t235 * t263) * r_i_i_C(2), 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobiaD_transl_5_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:40
	% EndTime: 2020-06-27 19:20:42
	% DurationCPUTime: 0.28s
	% Computational Cost: add. (549->69), mult. (568->100), div. (0->0), fcn. (450->10), ass. (0->61)
	t252 = cos(qJ(3));
	t248 = qJ(3) + qJ(4);
	t245 = cos(t248);
	t247 = qJD(3) + qJD(4);
	t286 = pkin(5) * t247;
	t272 = t245 * t286;
	t282 = pkin(4) * qJD(3);
	t237 = t252 * t282 + t272;
	t240 = t252 * pkin(4) + pkin(5) * t245;
	t250 = sin(qJ(2));
	t253 = cos(qJ(2));
	t288 = pkin(10) + r_i_i_C(3);
	t269 = t288 * t253;
	t296 = (-t240 * t250 + t269) * qJD(1) + t237;
	t246 = qJ(5) + t248;
	t241 = sin(t246);
	t283 = r_i_i_C(2) * t241;
	t242 = cos(t246);
	t284 = r_i_i_C(1) * t242;
	t260 = t240 - t283 + t284;
	t270 = t288 * t250;
	t294 = t260 * t253 + t270;
	t293 = t245 * (t247 * t250 - qJD(1));
	t251 = sin(qJ(1));
	t243 = qJD(5) + t247;
	t266 = t243 * t250 - qJD(1);
	t292 = t251 * t266;
	t254 = cos(qJ(1));
	t291 = t254 * t266;
	t244 = sin(t248);
	t249 = sin(qJ(3));
	t236 = -t244 * t286 - t249 * t282;
	t285 = r_i_i_C(1) * t241;
	t262 = r_i_i_C(2) * t242 + t285;
	t289 = t262 * t243 - t236;
	t287 = pkin(5) * t244;
	t280 = t243 * t253;
	t277 = qJD(1) * t250;
	t265 = -t243 + t277;
	t273 = qJD(2) * t254;
	t267 = t253 * t273;
	t256 = t265 * t251 - t267;
	t232 = t256 * t241 - t242 * t291;
	t233 = t241 * t291 + t256 * t242;
	t279 = t232 * r_i_i_C(1) + t233 * r_i_i_C(2);
	t274 = qJD(2) * t253;
	t268 = t251 * t274;
	t257 = t265 * t254 + t268;
	t234 = t257 * t241 + t242 * t292;
	t235 = -t241 * t292 + t257 * t242;
	t278 = t234 * r_i_i_C(1) + t235 * r_i_i_C(2);
	t276 = qJD(1) * t251;
	t275 = qJD(2) * t250;
	t271 = t243 * t283;
	t263 = -t247 + t277;
	t239 = t249 * pkin(4) + t287;
	t261 = t239 * t277 + t236;
	t258 = -qJD(1) * t240 + t237 * t250 + t239 * t274;
	t255 = -t250 * t236 + (-pkin(9) - t239) * qJD(1) + (-t240 * t253 - t270) * qJD(2);
	t238 = t280 * t284;
	t1 = [t233 * r_i_i_C(1) - t232 * r_i_i_C(2) - t296 * t251 + t255 * t254, -t294 * t254 * qJD(1) + (t289 * t253 + (t260 * t250 - t269) * qJD(2)) * t251, t258 * t251 + t261 * t254 + t278, (t251 * t293 + (t263 * t254 + t268) * t244) * pkin(5) + t278, t278, 0, 0, 0, 0, 0, 0, 0, 0; -t235 * r_i_i_C(1) + t234 * r_i_i_C(2) + t255 * t251 + t296 * t254, (-t260 * t273 - t288 * t276) * t250 + (-t260 * t276 + (t288 * qJD(2) - t289) * t254) * t253, t261 * t251 - t258 * t254 + t279, (-t254 * t293 + (t263 * t251 - t267) * t244) * pkin(5) + t279, t279, 0, 0, 0, 0, 0, 0, 0, 0; 0, t294 * qJD(2) - t250 * t289, t238 + (t237 - t271) * t253 + (-t239 - t262) * t275, t238 + (-t271 + t272) * t253 + (-t262 - t287) * t275, -t275 * t285 + t238 + (-t241 * t280 - t242 * t275) * r_i_i_C(2), 0, 0, 0, 0, 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobiaD_transl_6_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:39
	% EndTime: 2020-06-27 19:20:42
	% DurationCPUTime: 0.43s
	% Computational Cost: add. (1030->87), mult. (737->115), div. (0->0), fcn. (582->12), ass. (0->72)
	t265 = qJ(3) + qJ(4);
	t262 = cos(t265);
	t263 = qJ(5) + t265;
	t258 = cos(t263);
	t264 = qJD(3) + qJD(4);
	t260 = qJD(5) + t264;
	t306 = pkin(6) * t260;
	t292 = t258 * t306;
	t308 = pkin(5) * t264;
	t246 = -t262 * t308 - t292;
	t269 = cos(qJ(3));
	t302 = pkin(4) * qJD(3);
	t244 = t269 * t302 - t246;
	t250 = pkin(5) * t262 + pkin(6) * t258;
	t248 = t269 * pkin(4) + t250;
	t267 = sin(qJ(2));
	t270 = cos(qJ(2));
	t309 = pkin(10) + r_i_i_C(3);
	t290 = t309 * t270;
	t317 = (-t248 * t267 + t290) * qJD(1) + t244;
	t259 = qJ(6) + t263;
	t253 = sin(t259);
	t254 = cos(t259);
	t303 = r_i_i_C(2) * t254;
	t282 = -r_i_i_C(1) * t253 - t303;
	t278 = -t248 - t282;
	t291 = t309 * t267;
	t315 = -t278 * t270 + t291;
	t314 = t258 * (t260 * t267 - qJD(1));
	t268 = sin(qJ(1));
	t255 = qJD(6) + t260;
	t287 = t255 * t267 - qJD(1);
	t313 = t268 * t287;
	t257 = sin(t263);
	t261 = sin(t265);
	t245 = -t257 * t306 - t261 * t308;
	t266 = sin(qJ(3));
	t243 = -t266 * t302 + t245;
	t304 = r_i_i_C(2) * t253;
	t305 = r_i_i_C(1) * t254;
	t311 = (-t304 + t305) * t255 - t243;
	t297 = qJD(1) * t267;
	t286 = -t255 + t297;
	t271 = cos(qJ(1));
	t293 = qJD(2) * t271;
	t288 = t270 * t293;
	t310 = t286 * t268 - t288;
	t307 = pkin(6) * t257;
	t300 = t255 * t270;
	t279 = t287 * t271;
	t239 = -t310 * t253 + t254 * t279;
	t240 = t253 * t279 + t310 * t254;
	t299 = t240 * r_i_i_C(1) + t239 * r_i_i_C(2);
	t294 = qJD(2) * t270;
	t289 = t268 * t294;
	t273 = t286 * t271 + t289;
	t241 = t273 * t253 + t254 * t313;
	t242 = -t253 * t313 + t273 * t254;
	t298 = t242 * r_i_i_C(1) - t241 * r_i_i_C(2);
	t296 = qJD(1) * t268;
	t295 = qJD(2) * t267;
	t284 = -t260 + t297;
	t249 = -pkin(5) * t261 - t307;
	t247 = t266 * pkin(4) - t249;
	t281 = t247 * t297 + t243;
	t280 = -t249 * t297 + t245;
	t277 = t282 * t255;
	t275 = -qJD(1) * t248 + t244 * t267 + t247 * t294;
	t274 = qJD(1) * t250 + t246 * t267 + t249 * t294;
	t272 = -t267 * t243 + (-pkin(9) - t247) * qJD(1) + (-t248 * t270 - t291) * qJD(2);
	t251 = t295 * t304;
	t1 = [t239 * r_i_i_C(1) - t240 * r_i_i_C(2) - t317 * t268 + t272 * t271, -t315 * t271 * qJD(1) + (t311 * t270 + (-t278 * t267 - t290) * qJD(2)) * t268, t275 * t268 + t281 * t271 + t298, -t274 * t268 + t280 * t271 + t298, (t268 * t314 + (t284 * t271 + t289) * t257) * pkin(6) + t298, t298, 0, 0, 0, 0, 0, 0, 0; t241 * r_i_i_C(1) + t242 * r_i_i_C(2) + t272 * t268 + t317 * t271, (t278 * t293 - t309 * t296) * t267 + (t278 * t296 + (t309 * qJD(2) - t311) * t271) * t270, t281 * t268 - t275 * t271 + t299, t280 * t268 + t274 * t271 + t299, (-t271 * t314 + (t284 * t268 - t288) * t257) * pkin(6) + t299, t299, 0, 0, 0, 0, 0, 0, 0; 0, t315 * qJD(2) - t267 * t311, t251 + (-t247 - t305) * t295 + (t244 + t277) * t270, t251 + (t249 - t305) * t295 + (-t246 + t277) * t270, t251 + (-t305 - t307) * t295 + (t277 + t292) * t270, -t300 * t303 + t251 + (-t253 * t300 - t254 * t295) * r_i_i_C(1), 0, 0, 0, 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobiaD_transl_7_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:44
	% EndTime: 2020-06-27 19:20:45
	% DurationCPUTime: 0.87s
	% Computational Cost: add. (2134->140), mult. (1535->200), div. (0->0), fcn. (1341->14), ass. (0->104)
	t391 = qJ(3) + qJ(4);
	t389 = qJ(5) + t391;
	t385 = qJ(6) + t389;
	t380 = cos(t385);
	t468 = pkin(11) + r_i_i_C(3);
	t475 = t468 * t380;
	t394 = sin(qJ(2));
	t398 = cos(qJ(2));
	t383 = sin(t389);
	t387 = sin(t391);
	t390 = qJD(3) + qJD(4);
	t386 = qJD(5) + t390;
	t465 = pkin(6) * t386;
	t467 = pkin(5) * t390;
	t361 = -t383 * t465 - t387 * t467;
	t393 = sin(qJ(3));
	t460 = pkin(4) * qJD(3);
	t355 = -t393 * t460 + t361;
	t379 = sin(t385);
	t381 = qJD(6) + t386;
	t396 = cos(qJ(7));
	t446 = qJD(2) * t396;
	t392 = sin(qJ(7));
	t457 = t380 * t392;
	t414 = t381 * t457 + t446;
	t456 = t381 * t396;
	t415 = -qJD(2) * t392 + t380 * t456;
	t420 = r_i_i_C(1) * t392 + r_i_i_C(2) * t396;
	t400 = qJD(2) * pkin(10) - t415 * r_i_i_C(1) + t414 * r_i_i_C(2) + (t420 * qJD(7) - t468 * t381) * t379 + t355;
	t384 = cos(t389);
	t388 = cos(t391);
	t374 = pkin(5) * t388 + pkin(6) * t384;
	t397 = cos(qJ(3));
	t366 = pkin(4) * t397 + t374;
	t462 = r_i_i_C(2) * t392;
	t463 = r_i_i_C(1) * t396;
	t421 = -t462 + t463;
	t406 = t421 * t379 - t366 - t475;
	t470 = t406 * qJD(2) + t421 * qJD(7);
	t474 = t470 * t394 + t400 * t398;
	t472 = t384 * (t386 * t394 - qJD(1));
	t395 = sin(qJ(1));
	t452 = t395 * t380;
	t399 = cos(qJ(1));
	t458 = t379 * t399;
	t358 = t394 * t452 - t458;
	t453 = t395 * t379;
	t454 = t394 * t399;
	t360 = -t380 * t454 - t453;
	t466 = pkin(6) * t383;
	t459 = t366 * t394;
	t455 = t381 * t398;
	t451 = t398 * t366;
	t450 = qJD(1) * t394;
	t449 = qJD(1) * t395;
	t448 = qJD(1) * t399;
	t447 = qJD(2) * t394;
	t445 = qJD(2) * t398;
	t444 = qJD(2) * t399;
	t443 = qJD(7) * t392;
	t442 = qJD(7) * t396;
	t441 = qJD(7) * t398;
	t440 = t384 * t465;
	t439 = r_i_i_C(1) * t443;
	t438 = t468 * t379;
	t437 = t379 * t455;
	t433 = t399 * t441;
	t432 = t395 * t445;
	t431 = t392 * t441;
	t430 = t398 * t444;
	t428 = t381 * t394 - qJD(1);
	t427 = -t381 + t450;
	t425 = -t386 + t450;
	t424 = r_i_i_C(2) * t380 * t442;
	t422 = r_i_i_C(2) * t447 * t457 + t437 * t462 + t455 * t475;
	t373 = -pkin(5) * t387 - t466;
	t365 = pkin(4) * t393 - t373;
	t419 = t365 * t450 + t355;
	t418 = -t373 * t450 + t361;
	t353 = -t380 * t449 + (t394 * t448 + t432) * t379 + t358 * t381;
	t417 = t395 * t441 + t353;
	t416 = -pkin(10) - t420;
	t357 = t380 * t399 + t394 * t453;
	t413 = (-t365 - pkin(9)) * qJD(1) - t394 * t355;
	t362 = -t388 * t467 - t440;
	t412 = t416 * t394;
	t411 = -t380 * t463 - t438;
	t356 = t397 * t460 - t362;
	t410 = -qJD(1) * t366 + t356 * t394 + t365 * t445;
	t409 = qJD(1) * t374 + t362 * t394 + t373 * t445;
	t408 = -qJD(7) * t357 - t395 * t447 + t398 * t448;
	t359 = t379 * t454 - t452;
	t407 = qJD(7) * t359 + t394 * t444 + t398 * t449;
	t405 = (-t379 * t456 - t380 * t443) * r_i_i_C(1) - t424;
	t354 = -t428 * t453 + (t427 * t399 + t432) * t380;
	t404 = -t358 * t439 + t354 * t463 + (-t354 * t392 - t358 * t442) * r_i_i_C(2) + t468 * t353;
	t351 = t357 * qJD(1) + t360 * t381 - t379 * t430;
	t352 = t428 * t458 + (t427 * t395 - t430) * t380;
	t403 = -t360 * t439 + t352 * t463 + (-t352 * t392 - t360 * t442) * r_i_i_C(2) + t468 * t351;
	t401 = qJD(1) * (t406 * t398 + t412);
	t377 = t396 * t433;
	t346 = t408 * t392 + t417 * t396;
	t345 = -t417 * t392 + t408 * t396;
	t1 = [(-t351 * t396 - t359 * t443 + t377) * r_i_i_C(1) + (t351 * t392 - t359 * t442) * r_i_i_C(2) + t468 * t352 + (-t356 + (t416 * t398 + t459) * qJD(1)) * t395 + (-r_i_i_C(2) * t431 + (t412 - t451) * qJD(2) + t413) * t399, -t474 * t395 + t399 * t401, t410 * t395 + t419 * t399 + t404, -t409 * t395 + t418 * t399 + t404, (t395 * t472 + (t425 * t399 + t432) * t383) * pkin(6) + t404, t404, r_i_i_C(1) * t345 - r_i_i_C(2) * t346, 0, 0, 0, 0, 0, 0; t346 * r_i_i_C(1) + t345 * r_i_i_C(2) - t468 * t354 + (t356 + (pkin(10) * t398 - t459) * qJD(1)) * t399 + ((-t394 * pkin(10) - t451) * qJD(2) + t413) * t395, t395 * t401 + t474 * t399, t419 * t395 - t410 * t399 + t403, t418 * t395 + t409 * t399 + t403, (-t399 * t472 + (t425 * t395 - t430) * t383) * pkin(6) + t403, t403, t377 * r_i_i_C(2) + (t407 * r_i_i_C(1) - t351 * r_i_i_C(2)) * t396 + ((-t351 + t433) * r_i_i_C(1) - t407 * r_i_i_C(2)) * t392, 0, 0, 0, 0, 0, 0; 0, t400 * t394 - t398 * t470, (t356 + t405) * t398 + (-t365 + t411) * t447 + t422, (-t362 + t405) * t398 + (t373 + t411) * t447 + t422, (t405 + t440) * t398 + (t411 - t466) * t447 + t422, -t398 * t424 - t438 * t447 + (-t396 * t437 + (-t394 * t446 - t431) * t380) * r_i_i_C(1) + t422, ((-t379 * t442 - t414) * r_i_i_C(1) + (t379 * t443 - t415) * r_i_i_C(2)) * t398 + t420 * t394 * (qJD(2) * t379 + qJD(7)), 0, 0, 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobiaD_transl_8_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:38
	% EndTime: 2020-06-27 19:20:40
	% DurationCPUTime: 0.20s
	% Computational Cost: add. (72->34), mult. (240->69), div. (0->0), fcn. (192->6), ass. (0->29)
	t209 = cos(qJ(2));
	t205 = sin(qJ(8));
	t208 = cos(qJ(8));
	t214 = r_i_i_C(1) * t205 + r_i_i_C(2) * t208;
	t206 = sin(qJ(2));
	t227 = pkin(12) + r_i_i_C(3);
	t221 = t227 * t206;
	t230 = -t214 * t209 + t221;
	t220 = t227 * t209;
	t207 = sin(qJ(1));
	t217 = qJD(1) * t206 - qJD(8);
	t210 = cos(qJ(1));
	t223 = qJD(2) * t210;
	t228 = t217 * t207 - t209 * t223;
	t226 = qJD(1) * t207;
	t225 = qJD(2) * t206;
	t224 = qJD(2) * t209;
	t222 = qJD(8) * t209;
	t218 = qJD(8) * t206 - qJD(1);
	t216 = qJD(2) * t221;
	t215 = r_i_i_C(1) * t208 - r_i_i_C(2) * t205;
	t213 = t218 * t210;
	t212 = t217 * t210;
	t211 = t215 * qJD(8);
	t204 = t208 * t212 + (-t218 * t205 + t208 * t224) * t207;
	t203 = t218 * t208 * t207 + (t207 * t224 + t212) * t205;
	t202 = t205 * t213 + t228 * t208;
	t201 = -t228 * t205 + t208 * t213;
	t1 = [t201 * r_i_i_C(1) - t202 * r_i_i_C(2) - t210 * t216 + (-pkin(9) * t210 - t207 * t220) * qJD(1), -t230 * t210 * qJD(1) + (t215 * t222 + (-t214 * t206 - t220) * qJD(2)) * t207, 0, 0, 0, 0, 0, r_i_i_C(1) * t204 - r_i_i_C(2) * t203, 0, 0, 0, 0, 0; t203 * r_i_i_C(1) + t204 * r_i_i_C(2) - t207 * t216 + (-pkin(9) * t207 + t210 * t220) * qJD(1), (t214 * t223 - t227 * t226) * t206 + (t214 * t226 + (t227 * qJD(2) - t211) * t210) * t209, 0, 0, 0, 0, 0, r_i_i_C(1) * t202 + r_i_i_C(2) * t201, 0, 0, 0, 0, 0; 0, t230 * qJD(2) - t206 * t211, 0, 0, 0, 0, 0, (t205 * t225 - t208 * t222) * r_i_i_C(2) + (-t205 * t222 - t208 * t225) * r_i_i_C(1), 0, 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobiaD_transl_9_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:39
	% EndTime: 2020-06-27 19:20:41
	% DurationCPUTime: 0.23s
	% Computational Cost: add. (241->52), mult. (416->87), div. (0->0), fcn. (330->8), ass. (0->49)
	t240 = sin(qJ(2));
	t243 = cos(qJ(2));
	t275 = pkin(12) + r_i_i_C(3);
	t257 = t275 * t243;
	t239 = sin(qJ(8));
	t270 = pkin(7) * qJD(8);
	t260 = t239 * t270;
	t274 = pkin(7) * t239;
	t283 = (t240 * t274 + t257) * qJD(1) - t260;
	t238 = qJ(8) + qJ(9);
	t235 = sin(t238);
	t236 = cos(t238);
	t271 = r_i_i_C(2) * t236;
	t250 = -r_i_i_C(1) * t235 - t271;
	t248 = -t250 + t274;
	t258 = t275 * t240;
	t281 = -t248 * t243 + t258;
	t241 = sin(qJ(1));
	t237 = qJD(8) + qJD(9);
	t255 = t237 * t240 - qJD(1);
	t280 = t241 * t255;
	t253 = qJD(8) * t240 - qJD(1);
	t278 = t253 * t239;
	t242 = cos(qJ(8));
	t272 = r_i_i_C(2) * t235;
	t273 = r_i_i_C(1) * t236;
	t277 = (-t272 + t273) * t237 + t242 * t270;
	t266 = qJD(1) * t240;
	t254 = -t237 + t266;
	t244 = cos(qJ(1));
	t262 = qJD(2) * t244;
	t256 = t243 * t262;
	t276 = t254 * t241 - t256;
	t269 = t237 * t243;
	t249 = t255 * t244;
	t230 = -t276 * t235 + t236 * t249;
	t231 = t235 * t249 + t276 * t236;
	t268 = t231 * r_i_i_C(1) + t230 * r_i_i_C(2);
	t263 = qJD(2) * t243;
	t247 = t241 * t263 + t254 * t244;
	t232 = t247 * t235 + t236 * t280;
	t233 = -t235 * t280 + t247 * t236;
	t267 = t233 * r_i_i_C(1) - t232 * r_i_i_C(2);
	t265 = qJD(1) * t241;
	t264 = qJD(2) * t240;
	t252 = -qJD(8) + t266;
	t245 = -pkin(9) * qJD(1) - qJD(2) * t258 + (t239 * t263 + t253 * t242) * pkin(7);
	t234 = t264 * t272;
	t1 = [t230 * r_i_i_C(1) - t231 * r_i_i_C(2) - t283 * t241 + t245 * t244, -t281 * t244 * qJD(1) + (t277 * t243 + (-t248 * t240 - t257) * qJD(2)) * t241, 0, 0, 0, 0, 0, (t252 * t244 * t242 + (t242 * t263 - t278) * t241) * pkin(7) + t267, t267, 0, 0, 0, 0; t232 * r_i_i_C(1) + t233 * r_i_i_C(2) + t245 * t241 + t283 * t244, (t248 * t262 - t275 * t265) * t240 + (t248 * t265 + (t275 * qJD(2) - t277) * t244) * t243, 0, 0, 0, 0, 0, (t244 * t278 + (t252 * t241 - t256) * t242) * pkin(7) + t268, t268, 0, 0, 0, 0; 0, t281 * qJD(2) - t240 * t277, 0, 0, 0, 0, 0, t234 + (-pkin(7) * t242 - t273) * t264 + (t250 * t237 - t260) * t243, -t269 * t271 + t234 + (-t235 * t269 - t236 * t264) * r_i_i_C(1), 0, 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobiaD_transl_10_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:40
	% EndTime: 2020-06-27 19:20:42
	% DurationCPUTime: 0.23s
	% Computational Cost: add. (241->51), mult. (416->86), div. (0->0), fcn. (330->8), ass. (0->50)
	t239 = sin(qJ(2));
	t241 = cos(qJ(3));
	t269 = pkin(1) * qJD(3);
	t256 = t241 * t269;
	t242 = cos(qJ(2));
	t274 = pkin(10) + r_i_i_C(3);
	t258 = t274 * t242;
	t270 = pkin(1) * t241;
	t282 = (-t239 * t270 + t258) * qJD(1) + t256;
	t237 = qJ(3) + qJ(10);
	t234 = sin(t237);
	t271 = r_i_i_C(2) * t234;
	t235 = cos(t237);
	t272 = r_i_i_C(1) * t235;
	t248 = t270 - t271 + t272;
	t259 = t274 * t239;
	t280 = t248 * t242 + t259;
	t240 = sin(qJ(1));
	t236 = qJD(3) + qJD(10);
	t253 = t236 * t239 - qJD(1);
	t279 = t240 * t253;
	t251 = qJD(3) * t239 - qJD(1);
	t278 = t241 * t251;
	t243 = cos(qJ(1));
	t277 = t243 * t253;
	t238 = sin(qJ(3));
	t273 = r_i_i_C(1) * t234;
	t249 = r_i_i_C(2) * t235 + t273;
	t275 = t249 * t236 + t238 * t269;
	t268 = t236 * t242;
	t265 = qJD(1) * t239;
	t252 = -t236 + t265;
	t261 = qJD(2) * t243;
	t254 = t242 * t261;
	t246 = t252 * t240 - t254;
	t229 = t246 * t234 - t235 * t277;
	t230 = t234 * t277 + t246 * t235;
	t267 = t229 * r_i_i_C(1) + t230 * r_i_i_C(2);
	t262 = qJD(2) * t242;
	t255 = t240 * t262;
	t247 = t252 * t243 + t255;
	t231 = t247 * t234 + t235 * t279;
	t232 = -t234 * t279 + t247 * t235;
	t266 = t231 * r_i_i_C(1) + t232 * r_i_i_C(2);
	t264 = qJD(1) * t240;
	t263 = qJD(2) * t239;
	t250 = -qJD(3) + t265;
	t244 = -pkin(9) * qJD(1) - qJD(2) * t259 + (t251 * t238 - t241 * t262) * pkin(1);
	t233 = t268 * t272;
	t1 = [t230 * r_i_i_C(1) - t229 * r_i_i_C(2) - t282 * t240 + t244 * t243, -t280 * t243 * qJD(1) + (t275 * t242 + (t248 * t239 - t258) * qJD(2)) * t240, (t240 * t278 + (t250 * t243 + t255) * t238) * pkin(1) + t266, 0, 0, 0, 0, 0, 0, t266, 0, 0, 0; -t232 * r_i_i_C(1) + t231 * r_i_i_C(2) + t244 * t240 + t282 * t243, (-t248 * t261 - t274 * t264) * t239 + (-t248 * t264 + (t274 * qJD(2) - t275) * t243) * t242, (-t243 * t278 + (t250 * t240 - t254) * t238) * pkin(1) + t267, 0, 0, 0, 0, 0, 0, t267, 0, 0, 0; 0, t280 * qJD(2) - t239 * t275, t233 + (-t236 * t271 + t256) * t242 + (-pkin(1) * t238 - t249) * t263, 0, 0, 0, 0, 0, 0, -t263 * t273 + t233 + (-t234 * t268 - t235 * t263) * r_i_i_C(2), 0, 0, 0;];
	JaD_transl = t1;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobiaD_transl_11_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:39
	% EndTime: 2020-06-27 19:20:41
	% DurationCPUTime: 0.33s
	% Computational Cost: add. (549->65), mult. (568->99), div. (0->0), fcn. (450->10), ass. (0->59)
	t245 = cos(qJ(3));
	t241 = qJ(3) + qJ(10);
	t238 = cos(t241);
	t240 = qJD(3) + qJD(10);
	t274 = t238 * t240;
	t276 = pkin(1) * qJD(3);
	t228 = pkin(2) * t274 + t245 * t276;
	t233 = t245 * pkin(1) + pkin(2) * t238;
	t243 = sin(qJ(2));
	t246 = cos(qJ(2));
	t282 = pkin(10) + r_i_i_C(3);
	t264 = t282 * t246;
	t290 = (-t233 * t243 + t264) * qJD(1) + t228;
	t239 = qJ(11) + t241;
	t234 = sin(t239);
	t279 = r_i_i_C(2) * t234;
	t235 = cos(t239);
	t280 = r_i_i_C(1) * t235;
	t252 = -t233 - t279 + t280;
	t265 = t282 * t243;
	t288 = -t252 * t246 + t265;
	t287 = r_i_i_C(1) * t234 + r_i_i_C(2) * t235;
	t286 = t238 * (t240 * t243 - qJD(1));
	t242 = sin(qJ(3));
	t237 = sin(t241);
	t277 = pkin(2) * t237;
	t227 = -t240 * t277 - t242 * t276;
	t236 = qJD(11) + t240;
	t250 = t287 * t236 + t227;
	t244 = sin(qJ(1));
	t271 = qJD(1) * t243;
	t259 = -t236 + t271;
	t247 = cos(qJ(1));
	t267 = qJD(2) * t247;
	t261 = t246 * t267;
	t284 = t259 * t244 - t261;
	t268 = qJD(2) * t246;
	t262 = t244 * t268;
	t283 = t259 * t247 + t262;
	t260 = -t236 * t243 + qJD(1);
	t253 = t260 * t247;
	t223 = t284 * t234 + t235 * t253;
	t224 = t234 * t253 - t284 * t235;
	t273 = -t223 * r_i_i_C(1) + t224 * r_i_i_C(2);
	t254 = t260 * t244;
	t225 = -t283 * t234 + t235 * t254;
	t226 = t234 * t254 + t283 * t235;
	t272 = t225 * r_i_i_C(1) - t226 * r_i_i_C(2);
	t270 = qJD(1) * t244;
	t269 = qJD(2) * t243;
	t266 = t236 * t280;
	t263 = t246 * t236 * t279 + t287 * t269;
	t257 = -t240 + t271;
	t232 = t242 * pkin(1) + t277;
	t255 = t232 * t271 + t227;
	t251 = -t246 * t266 + t263;
	t249 = -qJD(1) * t233 + t228 * t243 + t232 * t268;
	t248 = -t243 * t227 + (-pkin(9) - t232) * qJD(1) + (-t233 * t246 - t265) * qJD(2);
	t1 = [t224 * r_i_i_C(1) + t223 * r_i_i_C(2) - t290 * t244 + t248 * t247, -t288 * t247 * qJD(1) + (-t250 * t246 + (-t252 * t243 - t264) * qJD(2)) * t244, t249 * t244 + t255 * t247 + t272, 0, 0, 0, 0, 0, 0, (t244 * t286 + (t257 * t247 + t262) * t237) * pkin(2) + t272, t272, 0, 0; t226 * r_i_i_C(1) + t225 * r_i_i_C(2) + t248 * t244 + t290 * t247, (t252 * t267 - t282 * t270) * t243 + (t252 * t270 + (t282 * qJD(2) + t250) * t247) * t246, t255 * t244 - t249 * t247 + t273, 0, 0, 0, 0, 0, 0, (-t247 * t286 + (t257 * t244 - t261) * t237) * pkin(2) + t273, t273, 0, 0; 0, t288 * qJD(2) + t250 * t243, -t232 * t269 + (t228 - t266) * t246 + t263, 0, 0, 0, 0, 0, 0, (-t237 * t269 + t246 * t274) * pkin(2) + t251, t251, 0, 0;];
	JaD_transl = t1;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobiaD_transl_12_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:40
	% EndTime: 2020-06-27 19:20:42
	% DurationCPUTime: 0.44s
	% Computational Cost: add. (1030->81), mult. (737->113), div. (0->0), fcn. (582->12), ass. (0->69)
	t262 = qJ(3) + qJ(10);
	t258 = cos(t262);
	t259 = qJ(11) + t262;
	t253 = cos(t259);
	t261 = qJD(3) + qJD(10);
	t256 = qJD(11) + t261;
	t297 = t253 * t256;
	t300 = pkin(2) * t261;
	t237 = pkin(3) * t297 - t258 * t300;
	t286 = cos(qJ(3)) * pkin(1);
	t235 = -qJD(3) * t286 + t237;
	t242 = pkin(2) * t258 - pkin(3) * t253;
	t239 = t286 + t242;
	t263 = sin(qJ(2));
	t265 = cos(qJ(2));
	t305 = pkin(10) + r_i_i_C(3);
	t287 = t305 * t265;
	t313 = (-t239 * t263 + t287) * qJD(1) - t235;
	t254 = qJ(12) + t259;
	t247 = sin(t254);
	t302 = r_i_i_C(2) * t247;
	t248 = cos(t254);
	t303 = r_i_i_C(1) * t248;
	t272 = -t239 - t302 + t303;
	t288 = t305 * t263;
	t311 = -t272 * t265 + t288;
	t310 = r_i_i_C(1) * t247 + r_i_i_C(2) * t248;
	t309 = t253 * (t256 * t263 - qJD(1));
	t257 = sin(t262);
	t252 = sin(t259);
	t299 = pkin(3) * t252;
	t236 = t256 * t299 - t257 * t300;
	t285 = sin(qJ(3)) * pkin(1);
	t234 = -qJD(3) * t285 + t236;
	t249 = qJD(12) + t256;
	t270 = t249 * t310 + t234;
	t264 = sin(qJ(1));
	t294 = qJD(1) * t263;
	t280 = -t249 + t294;
	t266 = cos(qJ(1));
	t290 = qJD(2) * t266;
	t282 = t265 * t290;
	t307 = t264 * t280 - t282;
	t291 = qJD(2) * t265;
	t283 = t264 * t291;
	t306 = t266 * t280 + t283;
	t281 = -t249 * t263 + qJD(1);
	t273 = t281 * t266;
	t230 = t247 * t307 + t248 * t273;
	t231 = t247 * t273 - t248 * t307;
	t296 = -r_i_i_C(1) * t230 + r_i_i_C(2) * t231;
	t274 = t281 * t264;
	t232 = -t247 * t306 + t248 * t274;
	t233 = t247 * t274 + t248 * t306;
	t295 = r_i_i_C(1) * t232 - r_i_i_C(2) * t233;
	t293 = qJD(1) * t264;
	t292 = qJD(2) * t263;
	t289 = t249 * t303;
	t284 = t265 * t249 * t302 + t292 * t310;
	t241 = -pkin(2) * t257 + t299;
	t278 = t256 - t294;
	t238 = -t285 + t241;
	t276 = -t238 * t294 + t234;
	t275 = -t241 * t294 + t236;
	t271 = -t265 * t289 + t284;
	t269 = qJD(1) * t239 + t235 * t263 + t238 * t291;
	t268 = qJD(1) * t242 + t237 * t263 + t241 * t291;
	t267 = -t263 * t234 + (-pkin(9) + t238) * qJD(1) + (-t239 * t265 - t288) * qJD(2);
	t1 = [t231 * r_i_i_C(1) + t230 * r_i_i_C(2) - t264 * t313 + t267 * t266, -t311 * t266 * qJD(1) + (-t270 * t265 + (-t263 * t272 - t287) * qJD(2)) * t264, -t264 * t269 + t266 * t276 + t295, 0, 0, 0, 0, 0, 0, -t264 * t268 + t266 * t275 + t295, (-t264 * t309 + (t266 * t278 - t283) * t252) * pkin(3) + t295, t295, 0; t233 * r_i_i_C(1) + t232 * r_i_i_C(2) + t267 * t264 + t266 * t313, (t272 * t290 - t293 * t305) * t263 + (t272 * t293 + (qJD(2) * t305 + t270) * t266) * t265, t264 * t276 + t266 * t269 + t296, 0, 0, 0, 0, 0, 0, t264 * t275 + t266 * t268 + t296, (t266 * t309 + (t264 * t278 + t282) * t252) * pkin(3) + t296, t296, 0; 0, qJD(2) * t311 + t270 * t263, t238 * t292 + (-t235 - t289) * t265 + t284, 0, 0, 0, 0, 0, 0, t241 * t292 + (-t237 - t289) * t265 + t284, (t252 * t292 - t265 * t297) * pkin(3) + t271, t271, 0;];
	JaD_transl = t1;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobiaD_transl_13_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:41
	% EndTime: 2020-06-27 19:20:43
	% DurationCPUTime: 0.41s
	% Computational Cost: add. (1373->86), mult. (805->114), div. (0->0), fcn. (642->12), ass. (0->72)
	t271 = qJ(3) + qJ(10);
	t267 = cos(t271);
	t268 = qJ(11) + t271;
	t263 = cos(t268);
	t270 = qJD(3) + qJD(10);
	t265 = qJD(11) + t270;
	t305 = pkin(3) * t265;
	t296 = t263 * t305;
	t307 = pkin(2) * t270;
	t248 = -t267 * t307 + t296;
	t293 = cos(qJ(3)) * pkin(1);
	t246 = -qJD(3) * t293 + t248;
	t253 = pkin(2) * t267 - pkin(3) * t263;
	t251 = t293 + t253;
	t272 = sin(qJ(2));
	t274 = cos(qJ(2));
	t311 = pkin(10) + r_i_i_C(3);
	t294 = t311 * t274;
	t319 = (-t251 * t272 + t294) * qJD(1) - t246;
	t259 = qJ(12) + qJ(13) + t268;
	t256 = sin(t259);
	t308 = r_i_i_C(2) * t256;
	t257 = cos(t259);
	t309 = r_i_i_C(1) * t257;
	t282 = t251 - t308 + t309;
	t295 = t311 * t272;
	t317 = t282 * t274 + t295;
	t316 = t263 * (t265 * t272 - qJD(1));
	t273 = sin(qJ(1));
	t258 = qJD(12) + qJD(13) + t265;
	t289 = t258 * t272 - qJD(1);
	t315 = t273 * t289;
	t275 = cos(qJ(1));
	t314 = t275 * t289;
	t262 = sin(t268);
	t266 = sin(t271);
	t247 = t262 * t305 - t266 * t307;
	t292 = sin(qJ(3)) * pkin(1);
	t245 = -qJD(3) * t292 + t247;
	t310 = r_i_i_C(1) * t256;
	t285 = r_i_i_C(2) * t257 + t310;
	t312 = t285 * t258 - t245;
	t306 = pkin(3) * t262;
	t303 = t258 * t274;
	t302 = qJD(1) * t272;
	t288 = -t258 + t302;
	t298 = qJD(2) * t275;
	t290 = t274 * t298;
	t277 = t288 * t273 - t290;
	t240 = t277 * t256 - t257 * t314;
	t241 = t256 * t314 + t277 * t257;
	t234 = t240 * r_i_i_C(1) + t241 * r_i_i_C(2);
	t299 = qJD(2) * t274;
	t291 = t273 * t299;
	t278 = t288 * t275 + t291;
	t242 = t278 * t256 + t257 * t315;
	t243 = -t256 * t315 + t278 * t257;
	t235 = t242 * r_i_i_C(1) + t243 * r_i_i_C(2);
	t301 = qJD(1) * t273;
	t300 = qJD(2) * t272;
	t297 = t258 * t308;
	t252 = -pkin(2) * t266 + t306;
	t286 = t265 - t302;
	t250 = -t292 + t252;
	t284 = -t250 * t302 + t245;
	t283 = -t252 * t302 + t247;
	t280 = qJD(1) * t251 + t246 * t272 + t250 * t299;
	t279 = qJD(1) * t253 + t248 * t272 + t252 * t299;
	t276 = -t272 * t245 + (-pkin(9) + t250) * qJD(1) + (-t251 * t274 - t295) * qJD(2);
	t249 = t303 * t309;
	t244 = -t300 * t310 + t249 + (-t256 * t303 - t257 * t300) * r_i_i_C(2);
	t1 = [t241 * r_i_i_C(1) - t240 * r_i_i_C(2) - t319 * t273 + t276 * t275, -t317 * t275 * qJD(1) + (t312 * t274 + (t282 * t272 - t294) * qJD(2)) * t273, -t280 * t273 + t284 * t275 + t235, 0, 0, 0, 0, 0, 0, -t279 * t273 + t283 * t275 + t235, (-t273 * t316 + (t286 * t275 - t291) * t262) * pkin(3) + t235, t235, t235; -t243 * r_i_i_C(1) + t242 * r_i_i_C(2) + t276 * t273 + t319 * t275, (-t282 * t298 - t311 * t301) * t272 + (-t282 * t301 + (t311 * qJD(2) - t312) * t275) * t274, t284 * t273 + t280 * t275 + t234, 0, 0, 0, 0, 0, 0, t283 * t273 + t279 * t275 + t234, (t275 * t316 + (t286 * t273 + t290) * t262) * pkin(3) + t234, t234, t234; 0, t317 * qJD(2) - t272 * t312, t249 + (-t246 - t297) * t274 + (t250 - t285) * t300, 0, 0, 0, 0, 0, 0, t249 + (-t248 - t297) * t274 + (t252 - t285) * t300, t249 + (-t296 - t297) * t274 + (-t285 + t306) * t300, t244, t244;];
	JaD_transl = t1;
end