% Zeitableitung der Rotationsmatrix-Jacobi-Matrix für beliebiges Segment von
% KAS5m5OL
% Use Code from Maple symbolic Code Generation
% 
% Rotationsmatrix-Jacobi-Matrix: Differentieller Zusammenhang zwischen
% gestapelter Endeffektor-Rotationsmatrix und verallgemeinerten Koordinaten.
% Zeitableitung: Die Gradientenmatrix wird nochmal nach der Zeit abgeleitet.
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% qJD [13x1]
%   Generalized joint velocities
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt. (0=Basis).
%   Siehe auch: bsp_3T1R_fkine_fixb_rotmat_mdh_sym_varpar.m
% pkin [12x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8]';
% 
% Output:
% JRD_rot [9x13]
%   Zeitableitung der Jacobi-Matrix der Endeffektor-Rotationsmatrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function JRD_rot = KAS5m5OL_jacobiRD_rot_sym_varpar(qJ, qJD, link_index, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),uint8(0),zeros(12,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_jacobiRD_rot_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m5OL_jacobiRD_rot_sym_varpar: qJD has to be [13x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m5OL_jacobiRD_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_jacobiRD_rot_sym_varpar: pkin has to be [12x1] (double)');
JRD_rot=NaN(9,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobiRD_rot_0_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.04s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobiRD_rot_1_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (3->3), mult. (4->2), div. (0->0), fcn. (4->2), ass. (0->3)
	t31 = qJD(1) * sin(qJ(1));
	t30 = qJD(1) * cos(qJ(1));
	t1 = [-t31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobiRD_rot_2_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (11->9), mult. (36->13), div. (0->0), fcn. (36->4), ass. (0->14)
	t33 = sin(qJ(1));
	t40 = qJD(1) * t33;
	t35 = cos(qJ(1));
	t39 = qJD(1) * t35;
	t32 = sin(qJ(2));
	t38 = qJD(2) * t32;
	t34 = cos(qJ(2));
	t37 = qJD(2) * t34;
	t36 = qJD(2) * t35;
	t31 = t33 * t38 - t34 * t39;
	t30 = t32 * t39 + t33 * t37;
	t29 = t32 * t36 + t34 * t40;
	t28 = t32 * t40 - t34 * t36;
	t1 = [t28, t31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t30, -t29, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t29, t30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t31, t28, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t38, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t39, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobiRD_rot_3_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:41
	% EndTime: 2020-06-27 19:20:42
	% DurationCPUTime: 0.18s
	% Computational Cost: add. (48->25), mult. (173->44), div. (0->0), fcn. (173->6), ass. (0->33)
	t234 = cos(qJ(3));
	t232 = sin(qJ(2));
	t251 = qJD(3) * t232;
	t243 = -qJD(1) + t251;
	t258 = t234 * t243;
	t231 = sin(qJ(3));
	t257 = t243 * t231;
	t233 = sin(qJ(1));
	t256 = qJD(1) * t233;
	t236 = cos(qJ(1));
	t255 = qJD(1) * t236;
	t254 = qJD(2) * t232;
	t235 = cos(qJ(2));
	t253 = qJD(2) * t235;
	t252 = qJD(2) * t236;
	t250 = qJD(3) * t235;
	t249 = t235 * t255;
	t248 = t234 * t253;
	t247 = t233 * t253;
	t246 = t231 * t250;
	t245 = t234 * t250;
	t244 = t235 * t252;
	t242 = qJD(1) * t232 - qJD(3);
	t241 = t242 * t236;
	t240 = -t233 * t254 + t249;
	t239 = t232 * t252 + t235 * t256;
	t238 = t234 * t254 + t246;
	t237 = t242 * t233 - t244;
	t230 = t234 * t241 + (t248 - t257) * t233;
	t229 = t233 * t258 + (t241 + t247) * t231;
	t228 = t237 * t234 + t236 * t257;
	t227 = t237 * t231 - t236 * t258;
	t1 = [t228, t238 * t233 - t234 * t249, t229, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t230, -t239 * t234 - t236 * t246, t227, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t231 * t251 + t248, -t231 * t254 + t245, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t227, t240 * t231 + t233 * t245, t230, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t229, t239 * t231 - t236 * t245, t228, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t231 * t253 - t234 * t251, -t238, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t239, -t232 * t255 - t247, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t240, -t232 * t256 + t244, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t254, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobiRD_rot_4_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:42
	% EndTime: 2020-06-27 19:20:43
	% DurationCPUTime: 0.17s
	% Computational Cost: add. (184->25), mult. (233->44), div. (0->0), fcn. (233->6), ass. (0->34)
	t294 = sin(qJ(1));
	t291 = qJD(3) + qJD(4);
	t293 = sin(qJ(2));
	t313 = t291 * t293;
	t302 = -qJD(1) + t313;
	t315 = t294 * t302;
	t296 = cos(qJ(1));
	t314 = t296 * t302;
	t295 = cos(qJ(2));
	t312 = t291 * t295;
	t311 = qJD(1) * t294;
	t310 = qJD(1) * t296;
	t309 = qJD(2) * t293;
	t308 = qJD(2) * t295;
	t307 = qJD(2) * t296;
	t292 = qJ(3) + qJ(4);
	t289 = sin(t292);
	t306 = t289 * t312;
	t290 = cos(t292);
	t305 = t290 * t312;
	t304 = t294 * t308;
	t303 = t295 * t307;
	t301 = qJD(1) * t293 - t291;
	t300 = -t294 * t309 + t295 * t310;
	t299 = t293 * t307 + t295 * t311;
	t298 = t301 * t296 + t304;
	t297 = t301 * t294 - t303;
	t288 = -t290 * t309 - t306;
	t287 = -t289 * t309 + t305;
	t286 = -t289 * t315 + t298 * t290;
	t285 = t298 * t289 + t290 * t315;
	t284 = t289 * t314 + t297 * t290;
	t283 = t297 * t289 - t290 * t314;
	t1 = [t284, -t300 * t290 + t294 * t306, t285, t285, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t286, -t299 * t290 - t296 * t306, t283, t283, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t289 * t313 + t290 * t308, t287, t287, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t283, t300 * t289 + t294 * t305, t286, t286, 0, 0, 0, 0, 0, 0, 0, 0, 0; t285, t299 * t289 - t296 * t305, t284, t284, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t289 * t308 - t290 * t313, t288, t288, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t299, -t293 * t310 - t304, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t300, -t293 * t311 + t303, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t309, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobiRD_rot_5_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:42
	% EndTime: 2020-06-27 19:20:43
	% DurationCPUTime: 0.17s
	% Computational Cost: add. (388->25), mult. (293->44), div. (0->0), fcn. (293->6), ass. (0->34)
	t308 = sin(qJ(1));
	t305 = qJD(3) + qJD(4) + qJD(5);
	t307 = sin(qJ(2));
	t327 = t305 * t307;
	t316 = -qJD(1) + t327;
	t329 = t308 * t316;
	t310 = cos(qJ(1));
	t328 = t310 * t316;
	t309 = cos(qJ(2));
	t326 = t305 * t309;
	t325 = qJD(1) * t308;
	t324 = qJD(1) * t310;
	t323 = qJD(2) * t307;
	t322 = qJD(2) * t309;
	t321 = qJD(2) * t310;
	t306 = qJ(3) + qJ(4) + qJ(5);
	t303 = sin(t306);
	t320 = t303 * t326;
	t304 = cos(t306);
	t319 = t304 * t326;
	t318 = t308 * t322;
	t317 = t309 * t321;
	t315 = qJD(1) * t307 - t305;
	t314 = -t308 * t323 + t309 * t324;
	t313 = t307 * t321 + t309 * t325;
	t312 = t315 * t310 + t318;
	t311 = t315 * t308 - t317;
	t302 = -t304 * t323 - t320;
	t301 = -t303 * t323 + t319;
	t300 = -t303 * t329 + t312 * t304;
	t299 = t312 * t303 + t304 * t329;
	t298 = t303 * t328 + t311 * t304;
	t297 = t311 * t303 - t304 * t328;
	t1 = [t298, -t314 * t304 + t308 * t320, t299, t299, t299, 0, 0, 0, 0, 0, 0, 0, 0; -t300, -t313 * t304 - t310 * t320, t297, t297, t297, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t303 * t327 + t304 * t322, t301, t301, t301, 0, 0, 0, 0, 0, 0, 0, 0; -t297, t314 * t303 + t308 * t319, t300, t300, t300, 0, 0, 0, 0, 0, 0, 0, 0; t299, t313 * t303 - t310 * t319, t298, t298, t298, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t303 * t322 - t304 * t327, t302, t302, t302, 0, 0, 0, 0, 0, 0, 0, 0; -t313, -t307 * t324 - t318, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t314, -t307 * t325 + t317, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t323, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobiRD_rot_6_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:42
	% EndTime: 2020-06-27 19:20:42
	% DurationCPUTime: 0.12s
	% Computational Cost: add. (660->31), mult. (353->46), div. (0->0), fcn. (353->6), ass. (0->35)
	t331 = sin(qJ(1));
	t328 = qJD(3) + qJD(4) + qJD(5) + qJD(6);
	t330 = sin(qJ(2));
	t338 = qJD(1) * t330 - t328;
	t332 = cos(qJ(2));
	t333 = cos(qJ(1));
	t346 = qJD(2) * t333;
	t340 = t332 * t346;
	t353 = t338 * t331 - t340;
	t352 = t328 * t330;
	t351 = t328 * t332;
	t350 = qJD(1) * t331;
	t349 = qJD(1) * t333;
	t348 = qJD(2) * t330;
	t347 = qJD(2) * t332;
	t329 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t327 = cos(t329);
	t345 = t327 * t352;
	t326 = sin(t329);
	t344 = t326 * t351;
	t343 = t327 * t351;
	t342 = t333 * t328 * t326;
	t341 = t331 * t347;
	t339 = -qJD(1) + t352;
	t337 = t339 * t333;
	t336 = -t331 * t348 + t332 * t349;
	t335 = t330 * t346 + t332 * t350;
	t334 = t330 * t349 + t341;
	t323 = -t327 * t348 - t344;
	t322 = t326 * t348 - t343;
	t321 = -t339 * t331 * t326 + (t338 * t333 + t341) * t327;
	t320 = t334 * t326 - t327 * t350 + t331 * t345 - t342;
	t319 = t326 * t337 + t353 * t327;
	t318 = -t353 * t326 + t327 * t337;
	t1 = [t318, t336 * t326 + t331 * t343, t321, t321, t321, t321, 0, 0, 0, 0, 0, 0, 0; t320, t335 * t326 - t333 * t343, t319, t319, t319, t319, 0, 0, 0, 0, 0, 0, 0; 0, -t326 * t347 - t345, t323, t323, t323, t323, 0, 0, 0, 0, 0, 0, 0; -t319, t336 * t327 - t331 * t344, -t320, -t320, -t320, -t320, 0, 0, 0, 0, 0, 0, 0; t321, t335 * t327 + t332 * t342, t318, t318, t318, t318, 0, 0, 0, 0, 0, 0, 0; 0, t326 * t352 - t327 * t347, t322, t322, t322, t322, 0, 0, 0, 0, 0, 0, 0; -t335, -t334, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t336, -t330 * t350 + t340, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t348, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobiRD_rot_7_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:45
	% EndTime: 2020-06-27 19:20:45
	% DurationCPUTime: 0.39s
	% Computational Cost: add. (1404->57), mult. (920->105), div. (0->0), fcn. (944->8), ass. (0->67)
	t502 = sin(qJ(7));
	t505 = cos(qJ(7));
	t503 = sin(qJ(2));
	t501 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t498 = sin(t501);
	t524 = qJD(2) * t498 + qJD(7);
	t499 = cos(t501);
	t500 = qJD(3) + qJD(4) + qJD(5) + qJD(6);
	t506 = cos(qJ(2));
	t548 = t500 * t506;
	t531 = t499 * t548;
	t514 = t524 * t503 - t531;
	t525 = qJD(7) * t498 + qJD(2);
	t551 = t525 * t506;
	t509 = t514 * t502 - t505 * t551;
	t508 = t502 * t551 + t514 * t505;
	t507 = cos(qJ(1));
	t542 = t507 * t499;
	t504 = sin(qJ(1));
	t546 = t504 * t498;
	t489 = t503 * t546 + t542;
	t538 = qJD(2) * t507;
	t529 = t506 * t538;
	t530 = t503 * t542;
	t532 = t500 * t546;
	t482 = t489 * qJD(1) - t498 * t529 - t500 * t530 - t532;
	t541 = qJD(1) * t506;
	t518 = -t503 * t538 - t504 * t541;
	t543 = t507 * t498;
	t545 = t504 * t499;
	t511 = qJD(7) * (t503 * t543 - t545) - t518;
	t535 = qJD(7) * t506;
	t550 = (t507 * t535 - t482) * t502 + t511 * t505;
	t549 = t500 * t503;
	t547 = t503 * t505;
	t544 = t505 * t506;
	t540 = qJD(2) * t503;
	t539 = qJD(2) * t506;
	t537 = qJD(7) * t502;
	t536 = qJD(7) * t505;
	t534 = t498 * t502 * t506;
	t533 = t498 * t544;
	t528 = t505 * t535;
	t527 = -qJD(1) + t549;
	t526 = qJD(1) * t503 - t500;
	t515 = t504 * t539 + t526 * t507;
	t484 = t515 * t498 + t527 * t545;
	t522 = t504 * t535 + t484;
	t519 = t504 * t540 - t507 * t541;
	t517 = qJD(1) * (-t502 * t503 + t533);
	t516 = qJD(1) * (-t534 - t547);
	t512 = -qJD(7) * t489 - t519;
	t510 = -t482 * t505 - t511 * t502 + t507 * t528;
	t492 = -t530 - t546;
	t490 = t503 * t545 - t543;
	t488 = -t498 * t540 + t531;
	t487 = -t500 * t533 + (-t502 * t535 - t505 * t540) * t499;
	t486 = t500 * t534 + (t502 * t540 - t528) * t499;
	t485 = t515 * t499 - t527 * t546;
	t483 = t527 * t543 + (t526 * t504 - t529) * t499;
	t481 = t485 * t505 - t490 * t537;
	t480 = -t485 * t502 - t490 * t536;
	t479 = t483 * t505 - t492 * t537;
	t478 = -t483 * t502 - t492 * t536;
	t477 = t512 * t502 + t522 * t505;
	t476 = -t522 * t502 + t512 * t505;
	t1 = [t510, -t508 * t504 + t507 * t517, t481, t481, t481, t481, t476, 0, 0, 0, 0, 0, 0; t477, t504 * t517 + t508 * t507, t479, t479, t479, t479, t550, 0, 0, 0, 0, 0, 0; 0, -t524 * t544 + (-t499 * t500 * t505 + t525 * t502) * t503, t487, t487, t487, t487, t509, 0, 0, 0, 0, 0, 0; -t550, t509 * t504 + t507 * t516, t480, t480, t480, t480, -t477, 0, 0, 0, 0, 0, 0; t476, t504 * t516 - t509 * t507, t478, t478, t478, t478, t510, 0, 0, 0, 0, 0, 0; 0, t525 * t547 + (t499 * t549 + t524 * t506) * t502, t486, t486, t486, t486, t508, 0, 0, 0, 0, 0, 0; t483, t519 * t499 + t506 * t532, t484, t484, t484, t484, 0, 0, 0, 0, 0, 0, 0; -t485, t499 * t518 - t543 * t548, t482, t482, t482, t482, 0, 0, 0, 0, 0, 0, 0; 0, -t498 * t549 + t499 * t539, t488, t488, t488, t488, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobiRD_rot_8_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:40
	% EndTime: 2020-06-27 19:20:40
	% DurationCPUTime: 0.09s
	% Computational Cost: add. (48->24), mult. (173->45), div. (0->0), fcn. (173->6), ass. (0->32)
	t237 = sin(qJ(1));
	t236 = sin(qJ(2));
	t246 = qJD(1) * t236 - qJD(8);
	t239 = cos(qJ(2));
	t240 = cos(qJ(1));
	t256 = qJD(2) * t240;
	t248 = t239 * t256;
	t261 = t246 * t237 - t248;
	t260 = qJD(1) * t237;
	t259 = qJD(1) * t240;
	t258 = qJD(2) * t236;
	t257 = qJD(2) * t239;
	t255 = qJD(8) * t236;
	t254 = qJD(8) * t239;
	t253 = t239 * t259;
	t238 = cos(qJ(8));
	t252 = t238 * t257;
	t251 = t237 * t257;
	t235 = sin(qJ(8));
	t250 = t235 * t254;
	t249 = t238 * t254;
	t247 = -qJD(1) + t255;
	t245 = t247 * t240;
	t244 = t246 * t240;
	t243 = -t237 * t258 + t253;
	t242 = t236 * t256 + t239 * t260;
	t241 = -t238 * t258 - t250;
	t234 = t238 * t244 + (-t247 * t235 + t252) * t237;
	t233 = t247 * t238 * t237 + (t244 + t251) * t235;
	t232 = t235 * t245 + t261 * t238;
	t231 = -t261 * t235 + t238 * t245;
	t1 = [t231, t243 * t235 + t237 * t249, 0, 0, 0, 0, 0, t234, 0, 0, 0, 0, 0; t233, t242 * t235 - t240 * t249, 0, 0, 0, 0, 0, t232, 0, 0, 0, 0, 0; 0, -t235 * t257 - t238 * t255, 0, 0, 0, 0, 0, t241, 0, 0, 0, 0, 0; -t232, t241 * t237 + t238 * t253, 0, 0, 0, 0, 0, -t233, 0, 0, 0, 0, 0; t234, t242 * t238 + t240 * t250, 0, 0, 0, 0, 0, t231, 0, 0, 0, 0, 0; 0, t235 * t255 - t252, 0, 0, 0, 0, 0, t235 * t258 - t249, 0, 0, 0, 0, 0; -t242, -t236 * t259 - t251, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t243, -t236 * t260 + t248, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t258, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobiRD_rot_9_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:41
	% EndTime: 2020-06-27 19:20:42
	% DurationCPUTime: 0.16s
	% Computational Cost: add. (184->29), mult. (233->46), div. (0->0), fcn. (233->6), ass. (0->35)
	t297 = sin(qJ(1));
	t294 = qJD(8) + qJD(9);
	t296 = sin(qJ(2));
	t304 = qJD(1) * t296 - t294;
	t298 = cos(qJ(2));
	t299 = cos(qJ(1));
	t312 = qJD(2) * t299;
	t306 = t298 * t312;
	t319 = t304 * t297 - t306;
	t318 = t294 * t296;
	t317 = t294 * t298;
	t316 = qJD(1) * t297;
	t315 = qJD(1) * t299;
	t314 = qJD(2) * t296;
	t313 = qJD(2) * t298;
	t295 = qJ(8) + qJ(9);
	t293 = cos(t295);
	t311 = t293 * t318;
	t292 = sin(t295);
	t310 = t292 * t317;
	t309 = t293 * t317;
	t308 = t299 * t294 * t292;
	t307 = t297 * t313;
	t305 = -qJD(1) + t318;
	t303 = t305 * t299;
	t302 = -t297 * t314 + t298 * t315;
	t301 = t296 * t312 + t298 * t316;
	t300 = t296 * t315 + t307;
	t289 = -t293 * t314 - t310;
	t288 = t292 * t314 - t309;
	t287 = -t305 * t297 * t292 + (t304 * t299 + t307) * t293;
	t286 = t300 * t292 - t293 * t316 + t297 * t311 - t308;
	t285 = t292 * t303 + t319 * t293;
	t284 = -t319 * t292 + t293 * t303;
	t1 = [t284, t302 * t292 + t297 * t309, 0, 0, 0, 0, 0, t287, t287, 0, 0, 0, 0; t286, t301 * t292 - t299 * t309, 0, 0, 0, 0, 0, t285, t285, 0, 0, 0, 0; 0, -t292 * t313 - t311, 0, 0, 0, 0, 0, t289, t289, 0, 0, 0, 0; -t285, t302 * t293 - t297 * t310, 0, 0, 0, 0, 0, -t286, -t286, 0, 0, 0, 0; t287, t301 * t293 + t298 * t308, 0, 0, 0, 0, 0, t284, t284, 0, 0, 0, 0; 0, t292 * t318 - t293 * t313, 0, 0, 0, 0, 0, t288, t288, 0, 0, 0, 0; -t301, -t300, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t302, -t296 * t316 + t306, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t314, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobiRD_rot_10_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:42
	% EndTime: 2020-06-27 19:20:43
	% DurationCPUTime: 0.16s
	% Computational Cost: add. (184->25), mult. (233->44), div. (0->0), fcn. (233->6), ass. (0->34)
	t294 = sin(qJ(1));
	t291 = qJD(3) + qJD(10);
	t293 = sin(qJ(2));
	t313 = t291 * t293;
	t302 = -qJD(1) + t313;
	t315 = t294 * t302;
	t296 = cos(qJ(1));
	t314 = t296 * t302;
	t295 = cos(qJ(2));
	t312 = t291 * t295;
	t311 = qJD(1) * t294;
	t310 = qJD(1) * t296;
	t309 = qJD(2) * t293;
	t308 = qJD(2) * t295;
	t307 = qJD(2) * t296;
	t292 = qJ(3) + qJ(10);
	t289 = sin(t292);
	t306 = t289 * t312;
	t290 = cos(t292);
	t305 = t290 * t312;
	t304 = t294 * t308;
	t303 = t295 * t307;
	t301 = qJD(1) * t293 - t291;
	t300 = -t294 * t309 + t295 * t310;
	t299 = t293 * t307 + t295 * t311;
	t298 = t301 * t296 + t304;
	t297 = t301 * t294 - t303;
	t288 = -t290 * t309 - t306;
	t287 = -t289 * t309 + t305;
	t286 = -t289 * t315 + t298 * t290;
	t285 = t298 * t289 + t290 * t315;
	t284 = t289 * t314 + t297 * t290;
	t283 = t297 * t289 - t290 * t314;
	t1 = [t284, -t290 * t300 + t294 * t306, t285, 0, 0, 0, 0, 0, 0, t285, 0, 0, 0; -t286, -t290 * t299 - t296 * t306, t283, 0, 0, 0, 0, 0, 0, t283, 0, 0, 0; 0, -t289 * t313 + t290 * t308, t287, 0, 0, 0, 0, 0, 0, t287, 0, 0, 0; -t283, t289 * t300 + t294 * t305, t286, 0, 0, 0, 0, 0, 0, t286, 0, 0, 0; t285, t289 * t299 - t296 * t305, t284, 0, 0, 0, 0, 0, 0, t284, 0, 0, 0; 0, -t289 * t308 - t290 * t313, t288, 0, 0, 0, 0, 0, 0, t288, 0, 0, 0; -t299, -t293 * t310 - t304, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t300, -t293 * t311 + t303, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t309, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobiRD_rot_11_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:41
	% EndTime: 2020-06-27 19:20:42
	% DurationCPUTime: 0.10s
	% Computational Cost: add. (388->35), mult. (293->52), div. (0->0), fcn. (293->6), ass. (0->34)
	t305 = qJ(3) + qJ(10) + qJ(11);
	t302 = sin(t305);
	t307 = sin(qJ(1));
	t328 = t302 * t307;
	t303 = cos(t305);
	t309 = cos(qJ(1));
	t327 = t303 * t309;
	t304 = qJD(3) + qJD(10) + qJD(11);
	t306 = sin(qJ(2));
	t326 = t304 * t306;
	t308 = cos(qJ(2));
	t325 = t304 * t308;
	t324 = qJD(1) * t307;
	t323 = qJD(1) * t309;
	t322 = qJD(2) * t306;
	t321 = qJD(2) * t308;
	t320 = qJD(2) * t309;
	t319 = t304 * t328;
	t318 = t302 * t325;
	t317 = t303 * t325;
	t316 = t304 * t327;
	t315 = t307 * t321;
	t314 = t308 * t320;
	t313 = qJD(1) - t326;
	t312 = -qJD(1) * t306 + t304;
	t311 = -t307 * t322 + t308 * t323;
	t310 = t306 * t320 + t308 * t324;
	t296 = t303 * t322 + t318;
	t295 = t302 * t322 - t317;
	t294 = t303 * t315 - t306 * t319 - t316 + (t306 * t327 + t328) * qJD(1);
	t293 = t313 * t307 * t303 + (t312 * t309 - t315) * t302;
	t292 = t313 * t309 * t302 + (t312 * t307 + t314) * t303;
	t291 = -t306 * t316 - t319 - t302 * t314 + (t306 * t328 + t327) * qJD(1);
	t1 = [t292, t311 * t303 - t307 * t318, t293, 0, 0, 0, 0, 0, 0, t293, t293, 0, 0; t294, t310 * t303 + t309 * t318, -t291, 0, 0, 0, 0, 0, 0, -t291, -t291, 0, 0; 0, t302 * t326 - t303 * t321, t295, 0, 0, 0, 0, 0, 0, t295, t295, 0, 0; t291, -t311 * t302 - t307 * t317, -t294, 0, 0, 0, 0, 0, 0, -t294, -t294, 0, 0; t293, -t310 * t302 + t308 * t316, t292, 0, 0, 0, 0, 0, 0, t292, t292, 0, 0; 0, t302 * t321 + t303 * t326, t296, 0, 0, 0, 0, 0, 0, t296, t296, 0, 0; -t310, -t306 * t323 - t315, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t311, -t306 * t324 + t314, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t322, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobiRD_rot_12_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:42
	% EndTime: 2020-06-27 19:20:43
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (660->37), mult. (353->52), div. (0->0), fcn. (353->6), ass. (0->34)
	t328 = qJ(3) + qJ(10) + qJ(11) + qJ(12);
	t325 = sin(t328);
	t330 = sin(qJ(1));
	t351 = t325 * t330;
	t326 = cos(t328);
	t332 = cos(qJ(1));
	t350 = t326 * t332;
	t327 = qJD(3) + qJD(10) + qJD(11) + qJD(12);
	t329 = sin(qJ(2));
	t349 = t327 * t329;
	t331 = cos(qJ(2));
	t348 = t327 * t331;
	t347 = qJD(1) * t330;
	t346 = qJD(1) * t332;
	t345 = qJD(2) * t329;
	t344 = qJD(2) * t331;
	t343 = qJD(2) * t332;
	t342 = t327 * t351;
	t341 = t325 * t348;
	t340 = t326 * t348;
	t339 = t327 * t350;
	t338 = t330 * t344;
	t337 = t331 * t343;
	t336 = qJD(1) - t349;
	t335 = -qJD(1) * t329 + t327;
	t334 = -t330 * t345 + t331 * t346;
	t333 = t329 * t343 + t331 * t347;
	t319 = t326 * t345 + t341;
	t318 = t325 * t345 - t340;
	t317 = t326 * t338 - t329 * t342 - t339 + (t329 * t350 + t351) * qJD(1);
	t316 = t336 * t330 * t326 + (t335 * t332 - t338) * t325;
	t315 = t336 * t332 * t325 + (t335 * t330 + t337) * t326;
	t314 = -t329 * t339 - t342 - t325 * t337 + (t329 * t351 + t350) * qJD(1);
	t1 = [t315, t334 * t326 - t330 * t341, t316, 0, 0, 0, 0, 0, 0, t316, t316, t316, 0; t317, t333 * t326 + t332 * t341, -t314, 0, 0, 0, 0, 0, 0, -t314, -t314, -t314, 0; 0, t325 * t349 - t326 * t344, t318, 0, 0, 0, 0, 0, 0, t318, t318, t318, 0; t314, -t334 * t325 - t330 * t340, -t317, 0, 0, 0, 0, 0, 0, -t317, -t317, -t317, 0; t316, -t333 * t325 + t331 * t339, t315, 0, 0, 0, 0, 0, 0, t315, t315, t315, 0; 0, t325 * t344 + t326 * t349, t319, 0, 0, 0, 0, 0, 0, t319, t319, t319, 0; -t333, -t329 * t346 - t338, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t334, -t329 * t347 + t337, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t345, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobiRD_rot_13_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:43
	% EndTime: 2020-06-27 19:20:43
	% DurationCPUTime: 0.12s
	% Computational Cost: add. (1000->25), mult. (413->44), div. (0->0), fcn. (413->6), ass. (0->34)
	t331 = sin(qJ(1));
	t328 = qJD(3) + qJD(10) + qJD(11) + qJD(12) + qJD(13);
	t330 = sin(qJ(2));
	t350 = t328 * t330;
	t339 = -qJD(1) + t350;
	t352 = t331 * t339;
	t333 = cos(qJ(1));
	t351 = t333 * t339;
	t332 = cos(qJ(2));
	t349 = t328 * t332;
	t348 = qJD(1) * t331;
	t347 = qJD(1) * t333;
	t346 = qJD(2) * t330;
	t345 = qJD(2) * t332;
	t344 = qJD(2) * t333;
	t329 = qJ(3) + qJ(10) + qJ(11) + qJ(12) + qJ(13);
	t326 = sin(t329);
	t343 = t326 * t349;
	t327 = cos(t329);
	t342 = t327 * t349;
	t341 = t331 * t345;
	t340 = t332 * t344;
	t338 = qJD(1) * t330 - t328;
	t337 = -t331 * t346 + t332 * t347;
	t336 = t330 * t344 + t332 * t348;
	t335 = t338 * t333 + t341;
	t334 = t338 * t331 - t340;
	t325 = -t327 * t346 - t343;
	t324 = -t326 * t346 + t342;
	t323 = -t326 * t352 + t335 * t327;
	t322 = t335 * t326 + t327 * t352;
	t321 = t326 * t351 + t334 * t327;
	t320 = t334 * t326 - t327 * t351;
	t1 = [t321, -t337 * t327 + t331 * t343, t322, 0, 0, 0, 0, 0, 0, t322, t322, t322, t322; -t323, -t336 * t327 - t333 * t343, t320, 0, 0, 0, 0, 0, 0, t320, t320, t320, t320; 0, -t326 * t350 + t327 * t345, t324, 0, 0, 0, 0, 0, 0, t324, t324, t324, t324; -t320, t337 * t326 + t331 * t342, t323, 0, 0, 0, 0, 0, 0, t323, t323, t323, t323; t322, t336 * t326 - t333 * t342, t321, 0, 0, 0, 0, 0, 0, t321, t321, t321, t321; 0, -t326 * t345 - t327 * t350, t325, 0, 0, 0, 0, 0, 0, t325, t325, t325, t325; -t336, -t330 * t347 - t341, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t337, -t330 * t348 + t340, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t346, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JRD_rot = t1;
end