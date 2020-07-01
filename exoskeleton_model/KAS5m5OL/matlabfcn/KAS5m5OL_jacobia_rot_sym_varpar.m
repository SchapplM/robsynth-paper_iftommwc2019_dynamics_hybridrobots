% Rotatorische Teilmatrix der analytischen Jacobi-Matrix für beliebiges Segment von
% KAS5m5OL
% Use Code from Maple symbolic Code Generation
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% Zeitableitung der Winkeldarstellung des Endeffektors in Basis-Koordinaten
% 
% Winkeldarstellung: Euler-XYZ-Winkel, rotx(alpha)*roty(beta)*rotz(gamma)
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt.
%   Wie in KAS5m5OL_fkine_fixb_rotmat_mdh_sym_varpar.m (1=Basis).
% pkin [12x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8]';
% 
% Output:
% Ja_rot [3x13]
%   Rotatorische Teilmatrix der analytischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Ja_rot = KAS5m5OL_jacobia_rot_sym_varpar(qJ, link_index, ...
  pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),uint8(0),zeros(12,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_jacobia_rot_sym_varpar: qJ has to be [13x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m5OL_jacobia_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_jacobia_rot_sym_varpar: pkin has to be [12x1] (double)');
Ja_rot=NaN(3,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobia_rot_0_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.01s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobia_rot_1_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (3->0), mult. (6->0), div. (5->0), fcn. (6->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobia_rot_2_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN; NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN; NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN;];
	Ja_rot = t1;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobia_rot_3_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (86->19), mult. (224->55), div. (52->9), fcn. (332->9), ass. (0->36)
	t39 = sin(qJ(1));
	t55 = t39 ^ 2;
	t38 = sin(qJ(2));
	t41 = cos(qJ(2));
	t42 = cos(qJ(1));
	t44 = t42 * t41;
	t32 = atan2(t44, -t38);
	t30 = sin(t32);
	t31 = cos(t32);
	t22 = t30 * t44 - t31 * t38;
	t21 = 0.1e1 / t22 ^ 2;
	t54 = t21 * t41;
	t37 = sin(qJ(3));
	t46 = t42 * t37;
	t40 = cos(qJ(3));
	t49 = t39 * t40;
	t29 = -t38 * t49 + t46;
	t26 = 0.1e1 / t29 ^ 2;
	t45 = t42 * t40;
	t50 = t39 * t37;
	t27 = t38 * t50 + t45;
	t53 = t26 * t27;
	t52 = t30 * t38;
	t36 = t41 ^ 2;
	t51 = 0.1e1 / t38 ^ 2 * t36;
	t48 = t39 * t41;
	t33 = 0.1e1 / (t42 ^ 2 * t51 + 0.1e1);
	t47 = t42 * t33;
	t43 = t27 ^ 2 * t26 + 0.1e1;
	t34 = 0.1e1 / t38;
	t25 = 0.1e1 / t29;
	t24 = (0.1e1 + t51) * t47;
	t23 = 0.1e1 / t43;
	t20 = 0.1e1 / t22;
	t19 = 0.1e1 / (t55 * t36 * t21 + 0.1e1);
	t1 = [t34 * t33 * t48, t24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t20 * t44 - (t31 * t34 * t36 * t47 + (t33 - 0.1e1) * t41 * t30) * t55 * t54) * t19, (-t38 * t20 - (-t42 * t52 - t31 * t41 + (t31 * t44 + t52) * t24) * t54) * t39 * t19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((-t38 * t46 + t49) * t25 + (-t38 * t45 - t50) * t53) * t23, (-t25 * t37 - t40 * t53) * t23 * t48, t43 * t23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobia_rot_4_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.12s
	% Computational Cost: add. (158->20), mult. (251->55), div. (57->9), fcn. (367->9), ass. (0->38)
	t59 = sin(qJ(1));
	t74 = t59 ^ 2;
	t58 = sin(qJ(2));
	t60 = cos(qJ(2));
	t61 = cos(qJ(1));
	t63 = t61 * t60;
	t50 = atan2(t63, -t58);
	t48 = sin(t50);
	t49 = cos(t50);
	t41 = t48 * t63 - t49 * t58;
	t40 = 0.1e1 / t41 ^ 2;
	t73 = t40 * t60;
	t57 = qJ(3) + qJ(4);
	t52 = sin(t57);
	t65 = t61 * t52;
	t53 = cos(t57);
	t68 = t59 * t53;
	t47 = -t58 * t68 + t65;
	t44 = 0.1e1 / t47 ^ 2;
	t64 = t61 * t53;
	t69 = t59 * t52;
	t45 = t58 * t69 + t64;
	t72 = t44 * t45;
	t71 = t48 * t58;
	t56 = t60 ^ 2;
	t70 = 0.1e1 / t58 ^ 2 * t56;
	t67 = t59 * t60;
	t51 = 0.1e1 / (t61 ^ 2 * t70 + 0.1e1);
	t66 = t61 * t51;
	t62 = t45 ^ 2 * t44 + 0.1e1;
	t54 = 0.1e1 / t58;
	t43 = 0.1e1 / t47;
	t42 = (0.1e1 + t70) * t66;
	t39 = 0.1e1 / t41;
	t38 = 0.1e1 / t62;
	t37 = 0.1e1 / (t74 * t56 * t40 + 0.1e1);
	t36 = t62 * t38;
	t1 = [t54 * t51 * t67, t42, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t39 * t63 - (t49 * t54 * t56 * t66 + (t51 - 0.1e1) * t60 * t48) * t74 * t73) * t37, (-t58 * t39 - (-t61 * t71 - t49 * t60 + (t49 * t63 + t71) * t42) * t73) * t59 * t37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((-t58 * t65 + t68) * t43 + (-t58 * t64 - t69) * t72) * t38, (-t43 * t52 - t53 * t72) * t38 * t67, t36, t36, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobia_rot_5_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.12s
	% Computational Cost: add. (258->20), mult. (278->55), div. (62->9), fcn. (402->9), ass. (0->38)
	t62 = sin(qJ(1));
	t77 = t62 ^ 2;
	t61 = sin(qJ(2));
	t63 = cos(qJ(2));
	t64 = cos(qJ(1));
	t66 = t64 * t63;
	t53 = atan2(t66, -t61);
	t51 = sin(t53);
	t52 = cos(t53);
	t44 = t51 * t66 - t52 * t61;
	t43 = 0.1e1 / t44 ^ 2;
	t76 = t43 * t63;
	t57 = qJ(3) + qJ(4) + qJ(5);
	t55 = sin(t57);
	t68 = t64 * t55;
	t56 = cos(t57);
	t70 = t62 * t56;
	t50 = -t61 * t70 + t68;
	t47 = 0.1e1 / t50 ^ 2;
	t67 = t64 * t56;
	t71 = t62 * t55;
	t48 = t61 * t71 + t67;
	t75 = t47 * t48;
	t74 = t51 * t61;
	t60 = t63 ^ 2;
	t72 = 0.1e1 / t61 ^ 2 * t60;
	t54 = 0.1e1 / (t64 ^ 2 * t72 + 0.1e1);
	t73 = t54 * t64;
	t69 = t62 * t63;
	t65 = t48 ^ 2 * t47 + 0.1e1;
	t58 = 0.1e1 / t61;
	t46 = 0.1e1 / t50;
	t45 = (0.1e1 + t72) * t73;
	t42 = 0.1e1 / t44;
	t41 = 0.1e1 / (t77 * t60 * t43 + 0.1e1);
	t40 = 0.1e1 / t65;
	t39 = t65 * t40;
	t1 = [t58 * t54 * t69, t45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t42 * t66 - (t52 * t58 * t60 * t73 + (t54 - 0.1e1) * t63 * t51) * t77 * t76) * t41, (-t61 * t42 - (-t64 * t74 - t52 * t63 + (t52 * t66 + t74) * t45) * t76) * t62 * t41, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((-t61 * t68 + t70) * t46 + (-t61 * t67 - t71) * t75) * t40, (-t46 * t55 - t56 * t75) * t40 * t69, t39, t39, t39, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobia_rot_6_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (386->20), mult. (305->55), div. (67->9), fcn. (437->9), ass. (0->38)
	t68 = sin(qJ(1));
	t83 = t68 ^ 2;
	t67 = sin(qJ(2));
	t69 = cos(qJ(2));
	t70 = cos(qJ(1));
	t72 = t70 * t69;
	t58 = atan2(t72, -t67);
	t56 = sin(t58);
	t57 = cos(t58);
	t49 = t56 * t72 - t57 * t67;
	t48 = 0.1e1 / t49 ^ 2;
	t82 = t48 * t69;
	t63 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t62 = cos(t63);
	t73 = t70 * t62;
	t61 = sin(t63);
	t78 = t68 * t61;
	t53 = t67 * t78 + t73;
	t52 = 0.1e1 / t53 ^ 2;
	t74 = t70 * t61;
	t77 = t68 * t62;
	t54 = t67 * t77 - t74;
	t81 = t52 * t54;
	t80 = t56 * t67;
	t66 = t69 ^ 2;
	t79 = 0.1e1 / t67 ^ 2 * t66;
	t76 = t68 * t69;
	t59 = 0.1e1 / (t70 ^ 2 * t79 + 0.1e1);
	t75 = t70 * t59;
	t71 = t54 ^ 2 * t52 + 0.1e1;
	t64 = 0.1e1 / t67;
	t51 = 0.1e1 / t53;
	t50 = (0.1e1 + t79) * t75;
	t47 = 0.1e1 / t49;
	t46 = 0.1e1 / (t83 * t66 * t48 + 0.1e1);
	t45 = 0.1e1 / t71;
	t44 = t71 * t45;
	t1 = [t64 * t59 * t76, t50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t47 * t72 - (t57 * t64 * t66 * t75 + (t59 - 0.1e1) * t69 * t56) * t83 * t82) * t46, (-t67 * t47 - (-t70 * t80 - t57 * t69 + (t57 * t72 + t80) * t50) * t82) * t68 * t46, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((-t67 * t73 - t78) * t51 + (t67 * t74 - t77) * t81) * t45, (-t51 * t62 + t61 * t81) * t45 * t76, t44, t44, t44, t44, 0, 0, 0, 0, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobia_rot_7_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:33
	% EndTime: 2020-06-27 19:20:34
	% DurationCPUTime: 0.18s
	% Computational Cost: add. (2559->34), mult. (1322->88), div. (234->11), fcn. (1981->11), ass. (0->48)
	t101 = cos(qJ(2));
	t94 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t93 = cos(t94);
	t108 = t101 * t93;
	t102 = cos(qJ(1));
	t98 = sin(qJ(2));
	t106 = t102 * t98;
	t92 = sin(t94);
	t99 = sin(qJ(1));
	t110 = t99 * t92;
	t87 = t93 * t106 + t110;
	t79 = atan2(-t87, -t108);
	t75 = sin(t79);
	t76 = cos(t79);
	t73 = -t76 * t108 - t75 * t87;
	t71 = 0.1e1 / t73 ^ 2;
	t109 = t99 * t93;
	t84 = -t102 * t92 + t98 * t109;
	t115 = t71 * t84;
	t114 = t76 * t87;
	t100 = cos(qJ(7));
	t107 = t101 * t99;
	t83 = t102 * t93 + t98 * t110;
	t97 = sin(qJ(7));
	t81 = t83 * t100 + t97 * t107;
	t78 = 0.1e1 / t81 ^ 2;
	t105 = t100 * t101;
	t80 = -t99 * t105 + t83 * t97;
	t113 = t78 * t80;
	t112 = t84 ^ 2 * t71;
	t90 = 0.1e1 / t93;
	t95 = 0.1e1 / t101;
	t111 = t90 * t95;
	t104 = t101 * t102;
	t103 = t80 ^ 2 * t78 + 0.1e1;
	t96 = 0.1e1 / t101 ^ 2;
	t91 = 0.1e1 / t93 ^ 2;
	t86 = t92 * t106 - t109;
	t82 = 0.1e1 / (t87 ^ 2 * t96 * t91 + 0.1e1);
	t77 = 0.1e1 / t81;
	t74 = 0.1e1 / t103;
	t72 = (t87 * t90 * t96 * t98 + t102) * t82;
	t70 = 0.1e1 / t73;
	t69 = 0.1e1 / (0.1e1 + t112);
	t68 = (t87 * t91 * t92 - t86 * t90) * t95 * t82;
	t67 = (-t100 * t113 + t97 * t77) * t84 * t74;
	t66 = (t83 * t70 + (-t68 * t114 + t75 * t86 + (t68 * t75 * t93 + t76 * t92) * t101) * t115) * t69;
	t1 = [-t84 * t82 * t111, t72, t68, t68, t68, t68, 0, 0, 0, 0, 0, 0, 0; (-t87 * t70 + (t75 + (t111 * t114 - t75) * t82) * t112) * t69, (-t72 * t114 * t115 + (-t70 * t107 + (t76 * t98 + (t101 * t72 - t104) * t75) * t115) * t93) * t69, t66, t66, t66, t66, 0, 0, 0, 0, 0, 0, 0; ((-t100 * t104 + t86 * t97) * t77 - (t86 * t100 + t97 * t104) * t113) * t74, ((t101 * t92 * t97 + t100 * t98) * t77 - (t92 * t105 - t97 * t98) * t113) * t74 * t99, t67, t67, t67, t67, t103 * t74, 0, 0, 0, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobia_rot_8_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (86->19), mult. (224->55), div. (52->9), fcn. (332->9), ass. (0->36)
	t40 = sin(qJ(1));
	t56 = t40 ^ 2;
	t39 = sin(qJ(2));
	t42 = cos(qJ(2));
	t43 = cos(qJ(1));
	t45 = t43 * t42;
	t32 = atan2(t45, -t39);
	t30 = sin(t32);
	t31 = cos(t32);
	t22 = t30 * t45 - t31 * t39;
	t21 = 0.1e1 / t22 ^ 2;
	t55 = t21 * t42;
	t41 = cos(qJ(8));
	t46 = t43 * t41;
	t38 = sin(qJ(8));
	t51 = t40 * t38;
	t27 = t39 * t51 + t46;
	t26 = 0.1e1 / t27 ^ 2;
	t47 = t43 * t38;
	t50 = t40 * t41;
	t28 = t39 * t50 - t47;
	t54 = t26 * t28;
	t53 = t30 * t39;
	t37 = t42 ^ 2;
	t52 = 0.1e1 / t39 ^ 2 * t37;
	t49 = t40 * t42;
	t33 = 0.1e1 / (t43 ^ 2 * t52 + 0.1e1);
	t48 = t43 * t33;
	t44 = t28 ^ 2 * t26 + 0.1e1;
	t35 = 0.1e1 / t39;
	t25 = 0.1e1 / t27;
	t24 = (0.1e1 + t52) * t48;
	t23 = 0.1e1 / t44;
	t20 = 0.1e1 / t22;
	t19 = 0.1e1 / (t56 * t37 * t21 + 0.1e1);
	t1 = [t35 * t33 * t49, t24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t20 * t45 - (t31 * t35 * t37 * t48 + (t33 - 0.1e1) * t42 * t30) * t56 * t55) * t19, (-t39 * t20 - (-t43 * t53 - t31 * t42 + (t31 * t45 + t53) * t24) * t55) * t40 * t19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((-t39 * t46 - t51) * t25 + (t39 * t47 - t50) * t54) * t23, (-t25 * t41 + t38 * t54) * t23 * t49, 0, 0, 0, 0, 0, t44 * t23, 0, 0, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobia_rot_9_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (158->20), mult. (251->55), div. (57->9), fcn. (367->9), ass. (0->38)
	t59 = sin(qJ(1));
	t74 = t59 ^ 2;
	t58 = sin(qJ(2));
	t60 = cos(qJ(2));
	t61 = cos(qJ(1));
	t63 = t61 * t60;
	t49 = atan2(t63, -t58);
	t47 = sin(t49);
	t48 = cos(t49);
	t40 = t47 * t63 - t48 * t58;
	t39 = 0.1e1 / t40 ^ 2;
	t73 = t39 * t60;
	t57 = qJ(8) + qJ(9);
	t53 = cos(t57);
	t64 = t61 * t53;
	t52 = sin(t57);
	t69 = t59 * t52;
	t44 = t58 * t69 + t64;
	t43 = 0.1e1 / t44 ^ 2;
	t65 = t61 * t52;
	t68 = t59 * t53;
	t45 = t58 * t68 - t65;
	t72 = t43 * t45;
	t71 = t47 * t58;
	t56 = t60 ^ 2;
	t70 = 0.1e1 / t58 ^ 2 * t56;
	t67 = t59 * t60;
	t50 = 0.1e1 / (t61 ^ 2 * t70 + 0.1e1);
	t66 = t61 * t50;
	t62 = t45 ^ 2 * t43 + 0.1e1;
	t54 = 0.1e1 / t58;
	t42 = 0.1e1 / t44;
	t41 = (0.1e1 + t70) * t66;
	t38 = 0.1e1 / t40;
	t37 = 0.1e1 / t62;
	t36 = 0.1e1 / (t74 * t56 * t39 + 0.1e1);
	t35 = t62 * t37;
	t1 = [t54 * t50 * t67, t41, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t38 * t63 - (t48 * t54 * t56 * t66 + (t50 - 0.1e1) * t60 * t47) * t74 * t73) * t36, (-t58 * t38 - (-t61 * t71 - t48 * t60 + (t48 * t63 + t71) * t41) * t73) * t59 * t36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((-t58 * t64 - t69) * t42 + (t58 * t65 - t68) * t72) * t37, (-t42 * t53 + t52 * t72) * t37 * t67, 0, 0, 0, 0, 0, t35, t35, 0, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobia_rot_10_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (158->20), mult. (251->55), div. (57->9), fcn. (367->9), ass. (0->38)
	t59 = sin(qJ(1));
	t74 = t59 ^ 2;
	t58 = sin(qJ(2));
	t60 = cos(qJ(2));
	t61 = cos(qJ(1));
	t63 = t61 * t60;
	t50 = atan2(t63, -t58);
	t48 = sin(t50);
	t49 = cos(t50);
	t41 = t48 * t63 - t49 * t58;
	t40 = 0.1e1 / t41 ^ 2;
	t73 = t40 * t60;
	t54 = qJ(3) + qJ(10);
	t52 = sin(t54);
	t65 = t61 * t52;
	t53 = cos(t54);
	t68 = t59 * t53;
	t47 = -t58 * t68 + t65;
	t44 = 0.1e1 / t47 ^ 2;
	t64 = t61 * t53;
	t69 = t59 * t52;
	t45 = t58 * t69 + t64;
	t72 = t44 * t45;
	t71 = t48 * t58;
	t57 = t60 ^ 2;
	t70 = 0.1e1 / t58 ^ 2 * t57;
	t67 = t59 * t60;
	t51 = 0.1e1 / (t61 ^ 2 * t70 + 0.1e1);
	t66 = t61 * t51;
	t62 = t45 ^ 2 * t44 + 0.1e1;
	t55 = 0.1e1 / t58;
	t43 = 0.1e1 / t47;
	t42 = (0.1e1 + t70) * t66;
	t39 = 0.1e1 / t41;
	t38 = 0.1e1 / t62;
	t37 = 0.1e1 / (t74 * t57 * t40 + 0.1e1);
	t36 = t62 * t38;
	t1 = [t55 * t51 * t67, t42, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t39 * t63 - (t49 * t55 * t57 * t66 + (t51 - 0.1e1) * t60 * t48) * t74 * t73) * t37, (-t58 * t39 - (-t61 * t71 - t49 * t60 + (t49 * t63 + t71) * t42) * t73) * t59 * t37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((-t58 * t65 + t68) * t43 + (-t58 * t64 - t69) * t72) * t38, (-t43 * t52 - t53 * t72) * t38 * t67, t36, 0, 0, 0, 0, 0, 0, t36, 0, 0, 0;];
	Ja_rot = t1;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobia_rot_11_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.12s
	% Computational Cost: add. (258->20), mult. (278->55), div. (62->9), fcn. (402->9), ass. (0->38)
	t59 = sin(qJ(1));
	t74 = t59 ^ 2;
	t58 = sin(qJ(2));
	t60 = cos(qJ(2));
	t61 = cos(qJ(1));
	t63 = t61 * t60;
	t50 = atan2(t63, -t58);
	t48 = sin(t50);
	t49 = cos(t50);
	t42 = t48 * t63 - t49 * t58;
	t41 = 0.1e1 / t42 ^ 2;
	t73 = t41 * t60;
	t54 = qJ(3) + qJ(10) + qJ(11);
	t52 = sin(t54);
	t65 = t61 * t52;
	t53 = cos(t54);
	t68 = t59 * t53;
	t47 = t58 * t68 - t65;
	t45 = 0.1e1 / t47 ^ 2;
	t64 = t61 * t53;
	t69 = t59 * t52;
	t46 = t58 * t69 + t64;
	t72 = t45 * t46;
	t71 = t48 * t58;
	t57 = t60 ^ 2;
	t70 = 0.1e1 / t58 ^ 2 * t57;
	t67 = t59 * t60;
	t51 = 0.1e1 / (t61 ^ 2 * t70 + 0.1e1);
	t66 = t61 * t51;
	t62 = t46 ^ 2 * t45 + 0.1e1;
	t55 = 0.1e1 / t58;
	t44 = 0.1e1 / t47;
	t43 = (0.1e1 + t70) * t66;
	t40 = 0.1e1 / t42;
	t39 = 0.1e1 / (t74 * t57 * t41 + 0.1e1);
	t38 = 0.1e1 / t62;
	t37 = t62 * t38;
	t1 = [t55 * t51 * t67, t43, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t40 * t63 - (t49 * t55 * t57 * t66 + (t51 - 0.1e1) * t60 * t48) * t74 * t73) * t39, (-t58 * t40 - (-t61 * t71 - t49 * t60 + (t49 * t63 + t71) * t43) * t73) * t59 * t39, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((t58 * t65 - t68) * t44 - (t58 * t64 + t69) * t72) * t38, (t44 * t52 - t53 * t72) * t38 * t67, t37, 0, 0, 0, 0, 0, 0, t37, t37, 0, 0;];
	Ja_rot = t1;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobia_rot_12_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (386->20), mult. (305->55), div. (67->9), fcn. (437->9), ass. (0->38)
	t65 = sin(qJ(1));
	t80 = t65 ^ 2;
	t64 = sin(qJ(2));
	t66 = cos(qJ(2));
	t67 = cos(qJ(1));
	t69 = t67 * t66;
	t56 = atan2(t69, -t64);
	t54 = sin(t56);
	t55 = cos(t56);
	t48 = t54 * t69 - t55 * t64;
	t47 = 0.1e1 / t48 ^ 2;
	t79 = t47 * t66;
	t60 = qJ(3) + qJ(10) + qJ(11) + qJ(12);
	t58 = sin(t60);
	t71 = t67 * t58;
	t59 = cos(t60);
	t74 = t65 * t59;
	t53 = t64 * t74 - t71;
	t51 = 0.1e1 / t53 ^ 2;
	t70 = t67 * t59;
	t75 = t65 * t58;
	t52 = t64 * t75 + t70;
	t78 = t51 * t52;
	t77 = t54 * t64;
	t63 = t66 ^ 2;
	t76 = 0.1e1 / t64 ^ 2 * t63;
	t73 = t65 * t66;
	t57 = 0.1e1 / (t67 ^ 2 * t76 + 0.1e1);
	t72 = t67 * t57;
	t68 = t52 ^ 2 * t51 + 0.1e1;
	t61 = 0.1e1 / t64;
	t50 = 0.1e1 / t53;
	t49 = (0.1e1 + t76) * t72;
	t46 = 0.1e1 / t48;
	t45 = 0.1e1 / (t80 * t63 * t47 + 0.1e1);
	t44 = 0.1e1 / t68;
	t43 = t68 * t44;
	t1 = [t61 * t57 * t73, t49, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t46 * t69 - (t55 * t61 * t63 * t72 + (t57 - 0.1e1) * t66 * t54) * t80 * t79) * t45, (-t64 * t46 - (-t67 * t77 - t55 * t66 + (t55 * t69 + t77) * t49) * t79) * t65 * t45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((t64 * t71 - t74) * t50 - (t64 * t70 + t75) * t78) * t44, (t50 * t58 - t59 * t78) * t44 * t73, t43, 0, 0, 0, 0, 0, 0, t43, t43, t43, 0;];
	Ja_rot = t1;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobia_rot_13_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:32
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.09s
	% Computational Cost: add. (542->20), mult. (332->55), div. (72->9), fcn. (472->9), ass. (0->38)
	t68 = sin(qJ(1));
	t83 = t68 ^ 2;
	t67 = sin(qJ(2));
	t69 = cos(qJ(2));
	t70 = cos(qJ(1));
	t72 = t70 * t69;
	t59 = atan2(t72, -t67);
	t57 = sin(t59);
	t58 = cos(t59);
	t50 = t57 * t72 - t58 * t67;
	t49 = 0.1e1 / t50 ^ 2;
	t82 = t49 * t69;
	t63 = qJ(3) + qJ(10) + qJ(11) + qJ(12) + qJ(13);
	t61 = sin(t63);
	t74 = t70 * t61;
	t62 = cos(t63);
	t77 = t68 * t62;
	t56 = -t67 * t77 + t74;
	t53 = 0.1e1 / t56 ^ 2;
	t73 = t70 * t62;
	t78 = t68 * t61;
	t54 = t67 * t78 + t73;
	t81 = t53 * t54;
	t80 = t57 * t67;
	t66 = t69 ^ 2;
	t79 = 0.1e1 / t67 ^ 2 * t66;
	t76 = t68 * t69;
	t60 = 0.1e1 / (t70 ^ 2 * t79 + 0.1e1);
	t75 = t70 * t60;
	t71 = t54 ^ 2 * t53 + 0.1e1;
	t64 = 0.1e1 / t67;
	t52 = 0.1e1 / t56;
	t51 = (0.1e1 + t79) * t75;
	t48 = 0.1e1 / t50;
	t47 = 0.1e1 / (t83 * t66 * t49 + 0.1e1);
	t46 = 0.1e1 / t71;
	t45 = t71 * t46;
	t1 = [t64 * t60 * t76, t51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; (t48 * t72 - (t58 * t64 * t66 * t75 + (t60 - 0.1e1) * t69 * t57) * t83 * t82) * t47, (-t67 * t48 - (-t70 * t80 - t58 * t69 + (t58 * t72 + t80) * t51) * t82) * t68 * t47, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ((-t67 * t74 + t77) * t52 + (-t67 * t73 - t78) * t81) * t46, (-t52 * t61 - t62 * t81) * t46 * t76, t45, 0, 0, 0, 0, 0, 0, t45, t45, t45, t45;];
	Ja_rot = t1;
end