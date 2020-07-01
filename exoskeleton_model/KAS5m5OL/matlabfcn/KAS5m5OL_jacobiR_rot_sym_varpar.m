% Rotatorische Teilmatrix der Rotationsmatrix-Jacobi-Matrix für beliebiges Segment von
% KAS5m5OL
% Use Code from Maple symbolic Code Generation
% 
% Rotationsmatrix-Jacobi-Matrix: Differentieller Zusammenhang zwischen
% gestapelter Endeffektor-Rotationsmatrix und verallgemeinerten Koordinaten.
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
% JR_rot [9x13]
%   Jacobi-Matrix der Endeffektor-Rotationsmatrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function JR_rot = KAS5m5OL_jacobiR_rot_sym_varpar(qJ, link_index, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),uint8(0),zeros(12,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_jacobiR_rot_sym_varpar: qJ has to be [13x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m5OL_jacobiR_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_jacobiR_rot_sym_varpar: pkin has to be [12x1] (double)');
JR_rot=NaN(9,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobiR_rot_0_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobiR_rot_1_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (1->1), mult. (0->0), div. (0->0), fcn. (4->2), ass. (0->3)
	t9 = cos(qJ(1));
	t8 = sin(qJ(1));
	t1 = [t9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobiR_rot_2_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (7->7), mult. (8->4), div. (0->0), fcn. (20->4), ass. (0->9)
	t8 = sin(qJ(2));
	t9 = sin(qJ(1));
	t15 = t9 * t8;
	t11 = cos(qJ(1));
	t14 = t11 * t8;
	t10 = cos(qJ(2));
	t13 = t9 * t10;
	t12 = t11 * t10;
	t1 = [-t14, -t13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t15, t12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t12, t15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t13, -t14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobiR_rot_3_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.10s
	% Computational Cost: add. (13->11), mult. (40->20), div. (0->0), fcn. (69->6), ass. (0->18)
	t64 = sin(qJ(2));
	t65 = sin(qJ(1));
	t75 = t65 * t64;
	t66 = cos(qJ(3));
	t74 = t65 * t66;
	t63 = sin(qJ(3));
	t67 = cos(qJ(2));
	t73 = t67 * t63;
	t72 = t67 * t66;
	t68 = cos(qJ(1));
	t71 = t68 * t64;
	t70 = t68 * t66;
	t69 = t68 * t67;
	t62 = -t65 * t63 - t64 * t70;
	t61 = t63 * t71 - t74;
	t60 = -t68 * t63 + t64 * t74;
	t59 = t63 * t75 + t70;
	t1 = [t62, -t65 * t72, t59, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t60, t66 * t69, -t61, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t64 * t66, t73, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t61, t65 * t73, t60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t59, -t63 * t69, t62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t64 * t63, t72, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t69, -t75, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t65 * t67, t71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t67, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobiR_rot_4_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (51->13), mult. (54->20), div. (0->0), fcn. (93->6), ass. (0->17)
	t91 = sin(qJ(2));
	t92 = sin(qJ(1));
	t97 = t92 * t91;
	t90 = qJ(3) + qJ(4);
	t88 = sin(t90);
	t93 = cos(qJ(2));
	t86 = t93 * t88;
	t89 = cos(t90);
	t87 = t93 * t89;
	t94 = cos(qJ(1));
	t96 = t94 * t91;
	t95 = t94 * t93;
	t84 = -t92 * t88 - t89 * t96;
	t83 = t88 * t96 - t92 * t89;
	t82 = -t94 * t88 + t89 * t97;
	t81 = t88 * t97 + t94 * t89;
	t1 = [t84, -t92 * t87, t81, t81, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t82, t89 * t95, -t83, -t83, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t91 * t89, t86, t86, 0, 0, 0, 0, 0, 0, 0, 0, 0; t83, t92 * t86, t82, t82, 0, 0, 0, 0, 0, 0, 0, 0, 0; t81, -t88 * t95, t84, t84, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t91 * t88, t87, t87, 0, 0, 0, 0, 0, 0, 0, 0, 0; t95, -t97, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t92 * t93, t96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t93, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobiR_rot_5_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (109->14), mult. (68->20), div. (0->0), fcn. (117->6), ass. (0->17)
	t94 = sin(qJ(2));
	t95 = sin(qJ(1));
	t100 = t95 * t94;
	t93 = qJ(3) + qJ(4) + qJ(5);
	t91 = sin(t93);
	t96 = cos(qJ(2));
	t89 = t96 * t91;
	t92 = cos(t93);
	t90 = t96 * t92;
	t97 = cos(qJ(1));
	t99 = t97 * t94;
	t98 = t97 * t96;
	t87 = -t95 * t91 - t92 * t99;
	t86 = t91 * t99 - t95 * t92;
	t85 = t92 * t100 - t97 * t91;
	t84 = t91 * t100 + t97 * t92;
	t1 = [t87, -t95 * t90, t84, t84, t84, 0, 0, 0, 0, 0, 0, 0, 0; -t85, t92 * t98, -t86, -t86, -t86, 0, 0, 0, 0, 0, 0, 0, 0; 0, t94 * t92, t89, t89, t89, 0, 0, 0, 0, 0, 0, 0, 0; t86, t95 * t89, t85, t85, t85, 0, 0, 0, 0, 0, 0, 0, 0; t84, -t91 * t98, t87, t87, t87, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t94 * t91, t90, t90, t90, 0, 0, 0, 0, 0, 0, 0, 0; t98, -t100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t95 * t96, t99, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobiR_rot_6_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (192->23), mult. (82->20), div. (0->0), fcn. (141->6), ass. (0->18)
	t98 = sin(qJ(2));
	t99 = sin(qJ(1));
	t106 = t99 * t98;
	t100 = cos(qJ(2));
	t97 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t95 = sin(t97);
	t105 = t100 * t95;
	t101 = cos(qJ(1));
	t104 = t101 * t98;
	t103 = t99 * t100;
	t102 = t101 * t100;
	t96 = cos(t97);
	t94 = t100 * t96;
	t93 = t96 * t104 + t99 * t95;
	t92 = t95 * t104 - t99 * t96;
	t91 = -t101 * t95 + t96 * t106;
	t90 = t101 * t96 + t95 * t106;
	t1 = [t92, t95 * t103, t91, t91, t91, t91, 0, 0, 0, 0, 0, 0, 0; t90, -t95 * t102, -t93, -t93, -t93, -t93, 0, 0, 0, 0, 0, 0, 0; 0, -t98 * t95, t94, t94, t94, t94, 0, 0, 0, 0, 0, 0, 0; t93, t96 * t103, -t90, -t90, -t90, -t90, 0, 0, 0, 0, 0, 0, 0; t91, -t96 * t102, t92, t92, t92, t92, 0, 0, 0, 0, 0, 0, 0; 0, -t98 * t96, -t105, -t105, -t105, -t105, 0, 0, 0, 0, 0, 0, 0; t102, -t106, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t103, t104, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobiR_rot_7_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:35
	% EndTime: 2020-06-27 19:20:35
	% DurationCPUTime: 0.09s
	% Computational Cost: add. (336->37), mult. (199->39), div. (0->0), fcn. (306->8), ass. (0->36)
	t142 = sin(qJ(2));
	t140 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t138 = sin(t140);
	t146 = cos(qJ(1));
	t153 = t146 * t138;
	t139 = cos(t140);
	t143 = sin(qJ(1));
	t157 = t143 * t139;
	t131 = t142 * t157 - t153;
	t141 = sin(qJ(7));
	t162 = t131 * t141;
	t152 = t146 * t139;
	t158 = t143 * t138;
	t133 = -t142 * t152 - t158;
	t161 = t133 * t141;
	t160 = t142 * t141;
	t144 = cos(qJ(7));
	t159 = t142 * t144;
	t145 = cos(qJ(2));
	t156 = t145 * t141;
	t155 = t145 * t144;
	t154 = t145 * t146;
	t151 = t139 * t156;
	t132 = t142 * t153 - t157;
	t150 = t132 * t144 + t141 * t154;
	t149 = -t132 * t141 + t144 * t154;
	t148 = t138 * t155 - t160;
	t147 = t138 * t156 + t159;
	t136 = t145 * t138;
	t134 = t139 * t155;
	t130 = t142 * t158 + t152;
	t129 = t133 * t144;
	t128 = t131 * t144;
	t127 = t130 * t144 + t143 * t156;
	t126 = -t130 * t141 + t143 * t155;
	t1 = [t150, t148 * t143, t128, t128, t128, t128, t126, 0, 0, 0, 0, 0, 0; t127, -t148 * t146, t129, t129, t129, t129, -t149, 0, 0, 0, 0, 0, 0; 0, -t138 * t159 - t156, t134, t134, t134, t134, -t147, 0, 0, 0, 0, 0, 0; t149, -t147 * t143, -t162, -t162, -t162, -t162, -t127, 0, 0, 0, 0, 0, 0; t126, t147 * t146, -t161, -t161, -t161, -t161, t150, 0, 0, 0, 0, 0, 0; 0, t138 * t160 - t155, -t151, -t151, -t151, -t151, -t148, 0, 0, 0, 0, 0, 0; t133, -t145 * t157, t130, t130, t130, t130, 0, 0, 0, 0, 0, 0, 0; -t131, t145 * t152, -t132, -t132, -t132, -t132, 0, 0, 0, 0, 0, 0, 0; 0, t142 * t139, t136, t136, t136, t136, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobiR_rot_8_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.04s
	% Computational Cost: add. (15->13), mult. (40->20), div. (0->0), fcn. (69->6), ass. (0->18)
	t65 = sin(qJ(2));
	t66 = sin(qJ(1));
	t76 = t66 * t65;
	t67 = cos(qJ(8));
	t75 = t66 * t67;
	t64 = sin(qJ(8));
	t68 = cos(qJ(2));
	t74 = t68 * t64;
	t73 = t68 * t67;
	t69 = cos(qJ(1));
	t72 = t69 * t65;
	t71 = t69 * t67;
	t70 = t69 * t68;
	t63 = t66 * t64 + t65 * t71;
	t62 = t64 * t72 - t75;
	t61 = -t69 * t64 + t65 * t75;
	t60 = t64 * t76 + t71;
	t1 = [t62, t66 * t74, 0, 0, 0, 0, 0, t61, 0, 0, 0, 0, 0; t60, -t64 * t70, 0, 0, 0, 0, 0, -t63, 0, 0, 0, 0, 0; 0, -t65 * t64, 0, 0, 0, 0, 0, t73, 0, 0, 0, 0, 0; t63, t66 * t73, 0, 0, 0, 0, 0, -t60, 0, 0, 0, 0, 0; t61, -t67 * t70, 0, 0, 0, 0, 0, t62, 0, 0, 0, 0, 0; 0, -t65 * t67, 0, 0, 0, 0, 0, -t74, 0, 0, 0, 0, 0; t70, -t76, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t66 * t68, t72, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t68, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobiR_rot_9_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (54->17), mult. (54->20), div. (0->0), fcn. (93->6), ass. (0->17)
	t89 = sin(qJ(2));
	t90 = sin(qJ(1));
	t96 = t90 * t89;
	t88 = qJ(8) + qJ(9);
	t86 = sin(t88);
	t91 = cos(qJ(2));
	t95 = t91 * t86;
	t87 = cos(t88);
	t85 = t91 * t87;
	t92 = cos(qJ(1));
	t94 = t92 * t89;
	t93 = t92 * t91;
	t84 = t90 * t86 + t87 * t94;
	t83 = t86 * t94 - t90 * t87;
	t82 = -t92 * t86 + t87 * t96;
	t81 = t86 * t96 + t92 * t87;
	t1 = [t83, t90 * t95, 0, 0, 0, 0, 0, t82, t82, 0, 0, 0, 0; t81, -t86 * t93, 0, 0, 0, 0, 0, -t84, -t84, 0, 0, 0, 0; 0, -t89 * t86, 0, 0, 0, 0, 0, t85, t85, 0, 0, 0, 0; t84, t90 * t85, 0, 0, 0, 0, 0, -t81, -t81, 0, 0, 0, 0; t82, -t87 * t93, 0, 0, 0, 0, 0, t83, t83, 0, 0, 0, 0; 0, -t89 * t87, 0, 0, 0, 0, 0, -t95, -t95, 0, 0, 0, 0; t93, -t96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t90 * t91, t94, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t91, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobiR_rot_10_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (51->13), mult. (54->20), div. (0->0), fcn. (93->6), ass. (0->17)
	t91 = sin(qJ(2));
	t92 = sin(qJ(1));
	t97 = t92 * t91;
	t90 = qJ(3) + qJ(10);
	t88 = sin(t90);
	t93 = cos(qJ(2));
	t86 = t93 * t88;
	t89 = cos(t90);
	t87 = t93 * t89;
	t94 = cos(qJ(1));
	t96 = t94 * t91;
	t95 = t94 * t93;
	t84 = -t92 * t88 - t89 * t96;
	t83 = t88 * t96 - t92 * t89;
	t82 = -t94 * t88 + t89 * t97;
	t81 = t88 * t97 + t94 * t89;
	t1 = [t84, -t92 * t87, t81, 0, 0, 0, 0, 0, 0, t81, 0, 0, 0; -t82, t89 * t95, -t83, 0, 0, 0, 0, 0, 0, -t83, 0, 0, 0; 0, t91 * t89, t86, 0, 0, 0, 0, 0, 0, t86, 0, 0, 0; t83, t92 * t86, t82, 0, 0, 0, 0, 0, 0, t82, 0, 0, 0; t81, -t88 * t95, t84, 0, 0, 0, 0, 0, 0, t84, 0, 0, 0; 0, -t91 * t88, t87, 0, 0, 0, 0, 0, 0, t87, 0, 0, 0; t95, -t97, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t92 * t93, t96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t93, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobiR_rot_11_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (115->20), mult. (68->20), div. (0->0), fcn. (117->6), ass. (0->17)
	t89 = sin(qJ(2));
	t90 = sin(qJ(1));
	t97 = t90 * t89;
	t88 = qJ(3) + qJ(10) + qJ(11);
	t86 = sin(t88);
	t91 = cos(qJ(2));
	t96 = t91 * t86;
	t87 = cos(t88);
	t95 = t91 * t87;
	t92 = cos(qJ(1));
	t94 = t92 * t89;
	t93 = t92 * t91;
	t84 = t90 * t86 + t87 * t94;
	t83 = t86 * t94 - t90 * t87;
	t82 = -t92 * t86 + t87 * t97;
	t81 = -t86 * t97 - t92 * t87;
	t1 = [t84, t90 * t95, t81, 0, 0, 0, 0, 0, 0, t81, t81, 0, 0; t82, -t87 * t93, t83, 0, 0, 0, 0, 0, 0, t83, t83, 0, 0; 0, -t89 * t87, -t96, 0, 0, 0, 0, 0, 0, -t96, -t96, 0, 0; -t83, -t90 * t96, -t82, 0, 0, 0, 0, 0, 0, -t82, -t82, 0, 0; t81, t86 * t93, t84, 0, 0, 0, 0, 0, 0, t84, t84, 0, 0; 0, t89 * t86, -t95, 0, 0, 0, 0, 0, 0, -t95, -t95, 0, 0; t93, -t97, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t90 * t91, t94, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t91, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobiR_rot_12_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:34
	% DurationCPUTime: 0.13s
	% Computational Cost: add. (195->23), mult. (82->20), div. (0->0), fcn. (141->6), ass. (0->17)
	t95 = sin(qJ(2));
	t96 = sin(qJ(1));
	t103 = t96 * t95;
	t94 = qJ(3) + qJ(10) + qJ(11) + qJ(12);
	t92 = sin(t94);
	t97 = cos(qJ(2));
	t102 = t97 * t92;
	t93 = cos(t94);
	t101 = t97 * t93;
	t98 = cos(qJ(1));
	t100 = t98 * t95;
	t99 = t98 * t97;
	t90 = t93 * t100 + t96 * t92;
	t89 = t92 * t100 - t96 * t93;
	t88 = t93 * t103 - t98 * t92;
	t87 = -t92 * t103 - t98 * t93;
	t1 = [t90, t96 * t101, t87, 0, 0, 0, 0, 0, 0, t87, t87, t87, 0; t88, -t93 * t99, t89, 0, 0, 0, 0, 0, 0, t89, t89, t89, 0; 0, -t95 * t93, -t102, 0, 0, 0, 0, 0, 0, -t102, -t102, -t102, 0; -t89, -t96 * t102, -t88, 0, 0, 0, 0, 0, 0, -t88, -t88, -t88, 0; t87, t92 * t99, t90, 0, 0, 0, 0, 0, 0, t90, t90, t90, 0; 0, t95 * t92, -t101, 0, 0, 0, 0, 0, 0, -t101, -t101, -t101, 0; t99, -t103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t96 * t97, t100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t97, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobiR_rot_13_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:34
	% DurationCPUTime: 0.14s
	% Computational Cost: add. (285->16), mult. (96->20), div. (0->0), fcn. (165->6), ass. (0->18)
	t100 = sin(qJ(2));
	t101 = sin(qJ(1));
	t107 = t101 * t100;
	t102 = cos(qJ(2));
	t106 = t101 * t102;
	t103 = cos(qJ(1));
	t105 = t103 * t100;
	t104 = t103 * t102;
	t99 = qJ(3) + qJ(10) + qJ(11) + qJ(12) + qJ(13);
	t98 = cos(t99);
	t97 = sin(t99);
	t96 = t102 * t98;
	t95 = t102 * t97;
	t93 = -t101 * t97 - t105 * t98;
	t92 = -t101 * t98 + t105 * t97;
	t91 = -t103 * t97 + t107 * t98;
	t90 = t103 * t98 + t107 * t97;
	t1 = [t93, -t98 * t106, t90, 0, 0, 0, 0, 0, 0, t90, t90, t90, t90; -t91, t98 * t104, -t92, 0, 0, 0, 0, 0, 0, -t92, -t92, -t92, -t92; 0, t100 * t98, t95, 0, 0, 0, 0, 0, 0, t95, t95, t95, t95; t92, t97 * t106, t91, 0, 0, 0, 0, 0, 0, t91, t91, t91, t91; t90, -t97 * t104, t93, 0, 0, 0, 0, 0, 0, t93, t93, t93, t93; 0, -t100 * t97, t96, 0, 0, 0, 0, 0, 0, t96, t96, t96, t96; t104, -t107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t106, t105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, -t102, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	JR_rot = t1;
end