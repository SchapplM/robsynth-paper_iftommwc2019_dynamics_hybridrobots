% Rotatorische Teilmatrix der geometrischen Jacobi-Matrix für beliebiges Segment von
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
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt. (0=Basis).
%   Siehe auch: bsp_3T1R_fkine_fixb_rotmat_mdh_sym_varpar.m
% pkin [12x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8]';
% 
% Output:
% Jg_rot [3x13]
%   Rotatorische Teilmatrix der geometrischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Jg_rot = KAS5m5OL_jacobig_rot_sym_varpar(qJ, link_index, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),uint8(0),zeros(12,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_jacobig_rot_sym_varpar: qJ has to be [13x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m5OL_jacobig_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_jacobig_rot_sym_varpar: pkin has to be [12x1] (double)');
Jg_rot=NaN(3,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobig_rot_0_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.01s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobig_rot_1_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.01s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobig_rot_2_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.01s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (2->2), ass. (0->1)
	t1 = [0, cos(qJ(1)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, sin(qJ(1)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobig_rot_3_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (2->2), mult. (2->2), div. (0->0), fcn. (7->4), ass. (0->4)
	t58 = cos(qJ(1));
	t57 = cos(qJ(2));
	t56 = sin(qJ(1));
	t1 = [0, t58, t56 * t57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t56, -t58 * t57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 1, 0, -sin(qJ(2)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobig_rot_4_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (4->4), mult. (4->2), div. (0->0), fcn. (12->4), ass. (0->7)
	t78 = cos(qJ(2));
	t79 = cos(qJ(1));
	t80 = t79 * t78;
	t77 = sin(qJ(1));
	t76 = sin(qJ(2));
	t75 = t77 * t78;
	t1 = [0, t79, t75, t75, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t77, -t80, -t80, 0, 0, 0, 0, 0, 0, 0, 0, 0; 1, 0, -t76, -t76, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobig_rot_5_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (6->6), mult. (6->2), div. (0->0), fcn. (17->4), ass. (0->7)
	t81 = cos(qJ(2));
	t82 = cos(qJ(1));
	t83 = t82 * t81;
	t80 = sin(qJ(1));
	t79 = sin(qJ(2));
	t78 = t80 * t81;
	t1 = [0, t82, t78, t78, t78, 0, 0, 0, 0, 0, 0, 0, 0; 0, t80, -t83, -t83, -t83, 0, 0, 0, 0, 0, 0, 0, 0; 1, 0, -t79, -t79, -t79, 0, 0, 0, 0, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobig_rot_6_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (8->8), mult. (8->2), div. (0->0), fcn. (22->4), ass. (0->7)
	t87 = cos(qJ(2));
	t88 = cos(qJ(1));
	t89 = t88 * t87;
	t86 = sin(qJ(1));
	t85 = sin(qJ(2));
	t84 = t86 * t87;
	t1 = [0, t88, t84, t84, t84, t84, 0, 0, 0, 0, 0, 0, 0; 0, t86, -t89, -t89, -t89, -t89, 0, 0, 0, 0, 0, 0, 0; 1, 0, -t85, -t85, -t85, -t85, 0, 0, 0, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobig_rot_7_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:34
	% EndTime: 2020-06-27 19:20:35
	% DurationCPUTime: 0.06s
	% Computational Cost: add. (26->12), mult. (15->8), div. (0->0), fcn. (34->6), ass. (0->11)
	t119 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t117 = cos(t119);
	t120 = sin(qJ(2));
	t125 = t117 * t120;
	t122 = cos(qJ(2));
	t123 = cos(qJ(1));
	t124 = t123 * t122;
	t121 = sin(qJ(1));
	t118 = t121 * t122;
	t116 = sin(t119);
	t1 = [0, t123, t118, t118, t118, t118, t123 * t116 - t121 * t125, 0, 0, 0, 0, 0, 0; 0, t121, -t124, -t124, -t124, -t124, t121 * t116 + t123 * t125, 0, 0, 0, 0, 0, 0; 1, 0, -t120, -t120, -t120, -t120, -t122 * t117, 0, 0, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobig_rot_8_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (2->2), mult. (2->2), div. (0->0), fcn. (7->4), ass. (0->4)
	t59 = cos(qJ(1));
	t58 = cos(qJ(2));
	t57 = sin(qJ(1));
	t1 = [0, t59, 0, 0, 0, 0, 0, t57 * t58, 0, 0, 0, 0, 0; 0, t57, 0, 0, 0, 0, 0, -t59 * t58, 0, 0, 0, 0, 0; 1, 0, 0, 0, 0, 0, 0, -sin(qJ(2)), 0, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobig_rot_9_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (4->4), mult. (4->2), div. (0->0), fcn. (12->4), ass. (0->7)
	t78 = cos(qJ(2));
	t79 = cos(qJ(1));
	t80 = t79 * t78;
	t77 = sin(qJ(1));
	t76 = sin(qJ(2));
	t75 = t77 * t78;
	t1 = [0, t79, 0, 0, 0, 0, 0, t75, t75, 0, 0, 0, 0; 0, t77, 0, 0, 0, 0, 0, -t80, -t80, 0, 0, 0, 0; 1, 0, 0, 0, 0, 0, 0, -t76, -t76, 0, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobig_rot_10_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (4->4), mult. (4->2), div. (0->0), fcn. (12->4), ass. (0->7)
	t78 = cos(qJ(2));
	t79 = cos(qJ(1));
	t80 = t79 * t78;
	t77 = sin(qJ(1));
	t76 = sin(qJ(2));
	t75 = t77 * t78;
	t1 = [0, t79, t75, 0, 0, 0, 0, 0, 0, t75, 0, 0, 0; 0, t77, -t80, 0, 0, 0, 0, 0, 0, -t80, 0, 0, 0; 1, 0, -t76, 0, 0, 0, 0, 0, 0, -t76, 0, 0, 0;];
	Jg_rot = t1;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobig_rot_11_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (6->6), mult. (6->2), div. (0->0), fcn. (17->4), ass. (0->7)
	t78 = cos(qJ(2));
	t79 = cos(qJ(1));
	t80 = t79 * t78;
	t77 = sin(qJ(1));
	t76 = sin(qJ(2));
	t75 = t77 * t78;
	t1 = [0, t79, t75, 0, 0, 0, 0, 0, 0, t75, t75, 0, 0; 0, t77, -t80, 0, 0, 0, 0, 0, 0, -t80, -t80, 0, 0; 1, 0, -t76, 0, 0, 0, 0, 0, 0, -t76, -t76, 0, 0;];
	Jg_rot = t1;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobig_rot_12_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (8->8), mult. (8->2), div. (0->0), fcn. (22->4), ass. (0->7)
	t84 = cos(qJ(2));
	t85 = cos(qJ(1));
	t86 = t85 * t84;
	t83 = sin(qJ(1));
	t82 = sin(qJ(2));
	t81 = t83 * t84;
	t1 = [0, t85, t81, 0, 0, 0, 0, 0, 0, t81, t81, t81, 0; 0, t83, -t86, 0, 0, 0, 0, 0, 0, -t86, -t86, -t86, 0; 1, 0, -t82, 0, 0, 0, 0, 0, 0, -t82, -t82, -t82, 0;];
	Jg_rot = t1;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobig_rot_13_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (10->10), mult. (10->2), div. (0->0), fcn. (27->4), ass. (0->7)
	t87 = cos(qJ(2));
	t88 = cos(qJ(1));
	t89 = t88 * t87;
	t86 = sin(qJ(1));
	t85 = sin(qJ(2));
	t84 = t86 * t87;
	t1 = [0, t88, t84, 0, 0, 0, 0, 0, 0, t84, t84, t84, t84; 0, t86, -t89, 0, 0, 0, 0, 0, 0, -t89, -t89, -t89, -t89; 1, 0, -t85, 0, 0, 0, 0, 0, 0, -t85, -t85, -t85, -t85;];
	Jg_rot = t1;
end