% Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix für beliebiges Segment von
% KAS5m5OL
% Use Code from Maple symbolic Code Generation
%
% Geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorgeschwindigkeit und Geschw. der verallgemeinerten Koordinaten.
% 
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
% JgD_rot [3x13]
%   Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function JgD_rot = KAS5m5OL_jacobigD_rot_sym_varpar(qJ, qJD, link_index, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),uint8(0),zeros(12,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_jacobigD_rot_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m5OL_jacobigD_rot_sym_varpar: qJD has to be [13x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m5OL_jacobigD_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_jacobigD_rot_sym_varpar: pkin has to be [12x1] (double)');
JgD_rot=NaN(3,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobigD_rot_0_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.01s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobigD_rot_1_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.01s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobigD_rot_2_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.01s
	% Computational Cost: add. (1->1), mult. (2->2), div. (0->0), fcn. (2->2), ass. (0->1)
	t1 = [0, -qJD(1) * sin(qJ(1)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, qJD(1) * cos(qJ(1)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobigD_rot_3_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (4->4), mult. (11->8), div. (0->0), fcn. (11->4), ass. (0->7)
	t77 = sin(qJ(1));
	t82 = qJD(1) * t77;
	t79 = cos(qJ(1));
	t81 = qJD(1) * t79;
	t80 = qJD(2) * sin(qJ(2));
	t78 = cos(qJ(2));
	t1 = [0, -t82, -t77 * t80 + t78 * t81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t81, t78 * t82 + t79 * t80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, -qJD(2) * t78, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobigD_rot_4_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:34
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (7->5), mult. (20->8), div. (0->0), fcn. (20->4), ass. (0->10)
	t101 = sin(qJ(1));
	t107 = qJD(1) * t101;
	t103 = cos(qJ(1));
	t106 = qJD(1) * t103;
	t105 = qJD(2) * sin(qJ(2));
	t102 = cos(qJ(2));
	t104 = qJD(2) * t102;
	t99 = -t101 * t105 + t102 * t106;
	t98 = t102 * t107 + t103 * t105;
	t1 = [0, -t107, t99, t99, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t106, t98, t98, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, -t104, -t104, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobigD_rot_5_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:34
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (10->6), mult. (29->8), div. (0->0), fcn. (29->4), ass. (0->10)
	t104 = sin(qJ(1));
	t110 = qJD(1) * t104;
	t106 = cos(qJ(1));
	t109 = qJD(1) * t106;
	t108 = qJD(2) * sin(qJ(2));
	t105 = cos(qJ(2));
	t107 = qJD(2) * t105;
	t102 = -t104 * t108 + t105 * t109;
	t101 = t105 * t110 + t106 * t108;
	t1 = [0, -t110, t102, t102, t102, 0, 0, 0, 0, 0, 0, 0, 0; 0, t109, t101, t101, t101, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, -t107, -t107, -t107, 0, 0, 0, 0, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobigD_rot_6_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:34
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (13->7), mult. (38->8), div. (0->0), fcn. (38->4), ass. (0->10)
	t110 = sin(qJ(1));
	t116 = qJD(1) * t110;
	t112 = cos(qJ(1));
	t115 = qJD(1) * t112;
	t114 = qJD(2) * sin(qJ(2));
	t111 = cos(qJ(2));
	t113 = qJD(2) * t111;
	t108 = -t110 * t114 + t111 * t115;
	t107 = t111 * t116 + t112 * t114;
	t1 = [0, -t116, t108, t108, t108, t108, 0, 0, 0, 0, 0, 0, 0; 0, t115, t107, t107, t107, t107, 0, 0, 0, 0, 0, 0, 0; 0, 0, -t113, -t113, -t113, -t113, 0, 0, 0, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobigD_rot_7_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:36
	% EndTime: 2020-06-27 19:20:36
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (73->16), mult. (68->23), div. (0->0), fcn. (68->6), ass. (0->18)
	t168 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t165 = sin(t168);
	t167 = qJD(3) + qJD(4) + qJD(5) + qJD(6);
	t169 = sin(qJ(2));
	t180 = (t167 * t169 - qJD(1)) * t165;
	t170 = sin(qJ(1));
	t179 = qJD(1) * t170;
	t172 = cos(qJ(1));
	t178 = qJD(1) * t172;
	t177 = qJD(2) * t169;
	t171 = cos(qJ(2));
	t176 = qJD(2) * t171;
	t175 = qJD(2) * t172;
	t173 = -qJD(1) * t169 + t167;
	t166 = cos(t168);
	t164 = -t170 * t177 + t171 * t178;
	t163 = t169 * t175 + t171 * t179;
	t1 = [0, -t179, t164, t164, t164, t164, t170 * t180 + (-t170 * t176 + t173 * t172) * t166, 0, 0, 0, 0, 0, 0; 0, t178, t163, t163, t163, t163, -t172 * t180 + (t173 * t170 + t171 * t175) * t166, 0, 0, 0, 0, 0, 0; 0, 0, -t176, -t176, -t176, -t176, t171 * t167 * t165 + t166 * t177, 0, 0, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobigD_rot_8_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (4->4), mult. (11->8), div. (0->0), fcn. (11->4), ass. (0->7)
	t78 = sin(qJ(1));
	t83 = qJD(1) * t78;
	t80 = cos(qJ(1));
	t82 = qJD(1) * t80;
	t81 = qJD(2) * sin(qJ(2));
	t79 = cos(qJ(2));
	t1 = [0, -t83, 0, 0, 0, 0, 0, -t78 * t81 + t79 * t82, 0, 0, 0, 0, 0; 0, t82, 0, 0, 0, 0, 0, t79 * t83 + t80 * t81, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, -qJD(2) * t79, 0, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobigD_rot_9_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (7->5), mult. (20->8), div. (0->0), fcn. (20->4), ass. (0->10)
	t106 = qJD(2) * sin(qJ(2));
	t100 = sin(qJ(1));
	t105 = qJD(1) * t100;
	t102 = cos(qJ(1));
	t104 = qJD(1) * t102;
	t101 = cos(qJ(2));
	t103 = qJD(2) * t101;
	t98 = -t100 * t106 + t101 * t104;
	t97 = t101 * t105 + t102 * t106;
	t1 = [0, -t105, 0, 0, 0, 0, 0, t98, t98, 0, 0, 0, 0; 0, t104, 0, 0, 0, 0, 0, t97, t97, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, -t103, -t103, 0, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobigD_rot_10_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:34
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (7->5), mult. (20->8), div. (0->0), fcn. (20->4), ass. (0->10)
	t101 = sin(qJ(1));
	t107 = qJD(1) * t101;
	t103 = cos(qJ(1));
	t106 = qJD(1) * t103;
	t105 = qJD(2) * sin(qJ(2));
	t102 = cos(qJ(2));
	t104 = qJD(2) * t102;
	t99 = -t101 * t105 + t102 * t106;
	t98 = t102 * t107 + t103 * t105;
	t1 = [0, -t107, t99, 0, 0, 0, 0, 0, 0, t99, 0, 0, 0; 0, t106, t98, 0, 0, 0, 0, 0, 0, t98, 0, 0, 0; 0, 0, -t104, 0, 0, 0, 0, 0, 0, -t104, 0, 0, 0;];
	JgD_rot = t1;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobigD_rot_11_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (10->6), mult. (29->8), div. (0->0), fcn. (29->4), ass. (0->10)
	t101 = sin(qJ(1));
	t107 = qJD(1) * t101;
	t103 = cos(qJ(1));
	t106 = qJD(1) * t103;
	t105 = qJD(2) * sin(qJ(2));
	t102 = cos(qJ(2));
	t104 = qJD(2) * t102;
	t99 = -t101 * t105 + t102 * t106;
	t98 = t102 * t107 + t103 * t105;
	t1 = [0, -t107, t99, 0, 0, 0, 0, 0, 0, t99, t99, 0, 0; 0, t106, t98, 0, 0, 0, 0, 0, 0, t98, t98, 0, 0; 0, 0, -t104, 0, 0, 0, 0, 0, 0, -t104, -t104, 0, 0;];
	JgD_rot = t1;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobigD_rot_12_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:34
	% EndTime: 2020-06-27 19:20:34
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (13->7), mult. (38->8), div. (0->0), fcn. (38->4), ass. (0->10)
	t107 = sin(qJ(1));
	t113 = qJD(1) * t107;
	t109 = cos(qJ(1));
	t112 = qJD(1) * t109;
	t111 = qJD(2) * sin(qJ(2));
	t108 = cos(qJ(2));
	t110 = qJD(2) * t108;
	t105 = -t107 * t111 + t108 * t112;
	t104 = t108 * t113 + t109 * t111;
	t1 = [0, -t113, t105, 0, 0, 0, 0, 0, 0, t105, t105, t105, 0; 0, t112, t104, 0, 0, 0, 0, 0, 0, t104, t104, t104, 0; 0, 0, -t110, 0, 0, 0, 0, 0, 0, -t110, -t110, -t110, 0;];
	JgD_rot = t1;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobigD_rot_13_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:34
	% EndTime: 2020-06-27 19:20:34
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (16->8), mult. (47->8), div. (0->0), fcn. (47->4), ass. (0->10)
	t111 = sin(qJ(1));
	t117 = qJD(1) * t111;
	t113 = cos(qJ(1));
	t116 = qJD(1) * t113;
	t115 = qJD(2) * sin(qJ(2));
	t112 = cos(qJ(2));
	t114 = qJD(2) * t112;
	t109 = -t111 * t115 + t112 * t116;
	t108 = t112 * t117 + t113 * t115;
	t1 = [0, -t117, t109, 0, 0, 0, 0, 0, 0, t109, t109, t109, t109; 0, t116, t108, 0, 0, 0, 0, 0, 0, t108, t108, t108, t108; 0, 0, -t114, 0, 0, 0, 0, 0, 0, -t114, -t114, -t114, -t114;];
	JgD_rot = t1;
end