% Rotatorische Teilmatrix der analytischen Jacobi-Matrix für beliebiges Segment von
% KAS5m7OL
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
%   Wie in KAS5m7OL_fkine_fixb_rotmat_mdh_sym_varpar.m (1=Basis).
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% Ja_rot [3x13]
%   Rotatorische Teilmatrix der analytischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Ja_rot = KAS5m7OL_jacobia_rot_sym_varpar(qJ, link_index, ...
  pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),uint8(0),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_jacobia_rot_sym_varpar: qJ has to be [13x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m7OL_jacobia_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_jacobia_rot_sym_varpar: pkin has to be [19x1] (double)');
Ja_rot=NaN(3,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobia_rot_0_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->39)
	unknown=NaN(3,13);
	unknown(1,1) = 0;
	unknown(1,2) = 0;
	unknown(1,3) = 0;
	unknown(1,4) = 0;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = 0;
	unknown(2,3) = 0;
	unknown(2,4) = 0;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 0;
	unknown(3,2) = 0;
	unknown(3,3) = 0;
	unknown(3,4) = 0;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Ja_rot = unknown;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobia_rot_1_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (3->2), mult. (6->3), div. (5->2), fcn. (6->2), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = t1 ^ 2;
	t3 = sin(qJ(1));
	t4 = t3 ^ 2;
	t6 = t2 / t4;
	t8 = 0.1e1 / (0.1e1 + t6);
	unknown(1,1) = 0;
	unknown(1,2) = 0;
	unknown(1,3) = 0;
	unknown(1,4) = 0;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = 0;
	unknown(2,3) = 0;
	unknown(2,4) = 0;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = (t6 * t8 + t8);
	unknown(3,2) = 0;
	unknown(3,3) = 0;
	unknown(3,4) = 0;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Ja_rot = unknown;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobia_rot_2_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->39)
	unknown=NaN(3,13);
	unknown(1,1) = NaN;
	unknown(1,2) = NaN;
	unknown(1,3) = NaN;
	unknown(1,4) = NaN;
	unknown(1,5) = NaN;
	unknown(1,6) = NaN;
	unknown(1,7) = NaN;
	unknown(1,8) = NaN;
	unknown(1,9) = NaN;
	unknown(1,10) = NaN;
	unknown(1,11) = NaN;
	unknown(1,12) = NaN;
	unknown(1,13) = NaN;
	unknown(2,1) = NaN;
	unknown(2,2) = NaN;
	unknown(2,3) = NaN;
	unknown(2,4) = NaN;
	unknown(2,5) = NaN;
	unknown(2,6) = NaN;
	unknown(2,7) = NaN;
	unknown(2,8) = NaN;
	unknown(2,9) = NaN;
	unknown(2,10) = NaN;
	unknown(2,11) = NaN;
	unknown(2,12) = NaN;
	unknown(2,13) = NaN;
	unknown(3,1) = NaN;
	unknown(3,2) = NaN;
	unknown(3,3) = NaN;
	unknown(3,4) = NaN;
	unknown(3,5) = NaN;
	unknown(3,6) = NaN;
	unknown(3,7) = NaN;
	unknown(3,8) = NaN;
	unknown(3,9) = NaN;
	unknown(3,10) = NaN;
	unknown(3,11) = NaN;
	unknown(3,12) = NaN;
	unknown(3,13) = NaN;
	Ja_rot = unknown;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobia_rot_3_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (86->20), mult. (224->70), div. (52->9), fcn. (332->9), ass. (0->75)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t27 * t2 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = sin(qJ(3));
	t70 = cos(qJ(3));
	t75 = -t55 * t70 + t6 * t68;
	t76 = 0.1e1 / t75;
	t80 = -t55 * t68 - t6 * t70;
	t81 = t80 ^ 2;
	t82 = t75 ^ 2;
	t83 = 0.1e1 / t82;
	t86 = 0.1e1 / (t81 * t83 + 0.1e1);
	t92 = t83 * t86;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - (t21 * t24 * t22 + t21 * t26 * t4 - t24 * t2 - t27 * t4) * t1 * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (t1 * t70 - t67 * t68) * t76 * t86 - (-t1 * t68 - t67 * t70) * t80 * t92;
	unknown(3,2) = t3 * t70 * t80 * t83 * t86 - t3 * t68 * t76 * t86;
	unknown(3,3) = t80 ^ 2 * t92 + t86;
	unknown(3,4) = 0.0e0;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobia_rot_4_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (158->21), mult. (251->70), div. (57->9), fcn. (367->9), ass. (0->77)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t27 * t2 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = qJ(3) + qJ(4);
	t69 = sin(t68);
	t71 = cos(t68);
	t76 = -t55 * t71 + t6 * t69;
	t77 = 0.1e1 / t76;
	t81 = -t55 * t69 - t6 * t71;
	t82 = t81 ^ 2;
	t83 = t76 ^ 2;
	t84 = 0.1e1 / t83;
	t87 = 0.1e1 / (t82 * t84 + 0.1e1);
	t93 = t84 * t87;
	t106 = t81 ^ 2 * t93 + t87;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - (t21 * t24 * t22 + t21 * t26 * t4 - t24 * t2 - t27 * t4) * t1 * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (t1 * t71 - t67 * t69) * t77 * t87 - (-t1 * t69 - t67 * t71) * t81 * t93;
	unknown(3,2) = t3 * t71 * t81 * t84 * t87 - t3 * t69 * t77 * t87;
	unknown(3,3) = t106;
	unknown(3,4) = t106;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobia_rot_5_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (258->22), mult. (278->70), div. (62->9), fcn. (402->9), ass. (0->77)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t27 * t2 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = qJ(3) + qJ(4) + qJ(5);
	t69 = sin(t68);
	t71 = cos(t68);
	t76 = -t55 * t71 + t6 * t69;
	t77 = 0.1e1 / t76;
	t81 = -t55 * t69 - t6 * t71;
	t82 = t81 ^ 2;
	t83 = t76 ^ 2;
	t84 = 0.1e1 / t83;
	t87 = 0.1e1 / (t82 * t84 + 0.1e1);
	t93 = t84 * t87;
	t106 = t81 ^ 2 * t93 + t87;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - (t21 * t24 * t22 + t21 * t26 * t4 - t2 * t24 - t27 * t4) * t1 * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (t1 * t71 - t67 * t69) * t77 * t87 - (-t1 * t69 - t67 * t71) * t81 * t93;
	unknown(3,2) = t3 * t71 * t81 * t84 * t87 - t3 * t69 * t77 * t87;
	unknown(3,3) = t106;
	unknown(3,4) = t106;
	unknown(3,5) = t106;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobia_rot_6_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (386->23), mult. (305->70), div. (67->9), fcn. (437->9), ass. (0->77)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t2 * t27 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t69 = cos(t68);
	t71 = sin(t68);
	t76 = t55 * t71 + t6 * t69;
	t77 = 0.1e1 / t76;
	t81 = -t55 * t69 + t6 * t71;
	t82 = t81 ^ 2;
	t83 = t76 ^ 2;
	t84 = 0.1e1 / t83;
	t87 = 0.1e1 / (t82 * t84 + 0.1e1);
	t93 = t84 * t87;
	t106 = t81 ^ 2 * t93 + t87;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - t1 * (t21 * t24 * t22 + t21 * t26 * t4 - t24 * t2 - t27 * t4) * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (-t1 * t71 - t67 * t69) * t77 * t87 - (-t1 * t69 + t67 * t71) * t81 * t93;
	unknown(3,2) = -t3 * t71 * t81 * t84 * t87 - t3 * t69 * t77 * t87;
	unknown(3,3) = t106;
	unknown(3,4) = t106;
	unknown(3,5) = t106;
	unknown(3,6) = t106;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobia_rot_7_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (2559->36), mult. (1322->108), div. (234->11), fcn. (1981->11), ass. (0->90)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t5 = cos(t4);
	t7 = cos(qJ(1));
	t8 = sin(t4);
	t10 = t3 * t5 - t7 * t8;
	t11 = cos(qJ(2));
	t12 = 0.1e1 / t11;
	t13 = t10 * t12;
	t14 = 0.1e1 / t5;
	t15 = t7 * t2;
	t18 = -t1 * t8 - t5 * t15;
	t19 = t18 ^ 2;
	t20 = t11 ^ 2;
	t21 = 0.1e1 / t20;
	t23 = t5 ^ 2;
	t24 = 0.1e1 / t23;
	t27 = 0.1e1 / (t19 * t21 * t24 + 0.1e1);
	t28 = t14 * t27;
	t35 = -t2 * t14 * t18 * t21 * t27 + t7 * t27;
	t38 = -t1 * t5 + t15 * t8;
	t45 = -t12 * t8 * t18 * t24 * t27 - t38 * t12 * t28;
	t46 = t5 * t11;
	t47 = atan2(t18, -t46);
	t48 = cos(t47);
	t49 = t48 * t11;
	t51 = sin(t47);
	t53 = t51 * t18 - t49 * t5;
	t54 = 0.1e1 / t53;
	t56 = t10 ^ 2;
	t57 = t53 ^ 2;
	t58 = 0.1e1 / t57;
	t61 = 0.1e1 / (t56 * t58 + 0.1e1);
	t72 = t58 * t61;
	t75 = t1 * t11;
	t93 = t3 * t8 + t7 * t5;
	t105 = t93 * t54 * t61 + (t45 * t48 * t18 + t45 * t51 * t46 + t51 * t38 + t49 * t8) * t10 * t72;
	t106 = sin(qJ(7));
	t108 = t7 * t11;
	t109 = cos(qJ(7));
	t114 = t75 * t106 + t93 * t109;
	t115 = 0.1e1 / t114;
	t119 = t93 * t106 - t75 * t109;
	t120 = t119 ^ 2;
	t121 = t114 ^ 2;
	t122 = 0.1e1 / t121;
	t125 = 0.1e1 / (t120 * t122 + 0.1e1);
	t131 = t122 * t125;
	t154 = -t10 * t109 * t119 * t122 * t125 + t10 * t106 * t115 * t125;
	unknown(1,1) = -t13 * t28;
	unknown(1,2) = t35;
	unknown(1,3) = t45;
	unknown(1,4) = t45;
	unknown(1,5) = t45;
	unknown(1,6) = t45;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t18 * t54 * t61 + (-t13 * t14 * t27 * t48 * t18 - t10 * t27 * t51 + t51 * t10) * t10 * t72;
	unknown(2,2) = -t75 * t5 * t54 * t61 + (t35 * t48 * t18 + t48 * t2 * t5 + t35 * t51 * t46 - t51 * t7 * t46) * t10 * t72;
	unknown(2,3) = t105;
	unknown(2,4) = t105;
	unknown(2,5) = t105;
	unknown(2,6) = t105;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (t38 * t106 - t108 * t109) * t115 * t125 - (t108 * t106 + t38 * t109) * t119 * t131;
	unknown(3,2) = (t75 * t8 * t106 + t3 * t109) * t115 * t125 - (t75 * t8 * t109 - t3 * t106) * t119 * t131;
	unknown(3,3) = t154;
	unknown(3,4) = t154;
	unknown(3,5) = t154;
	unknown(3,6) = t154;
	unknown(3,7) = t119 ^ 2 * t131 + t125;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobia_rot_8_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (76->19), mult. (197->68), div. (47->9), fcn. (297->9), ass. (0->74)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t27 * t2 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = cos(pkin(3));
	t70 = sin(pkin(3));
	t75 = t55 * t70 + t6 * t68;
	t76 = 0.1e1 / t75;
	t80 = -t55 * t68 + t6 * t70;
	t81 = t80 ^ 2;
	t82 = t75 ^ 2;
	t83 = 0.1e1 / t82;
	t86 = 0.1e1 / (t81 * t83 + 0.1e1);
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - (t21 * t24 * t22 + t21 * t26 * t4 - t24 * t2 - t27 * t4) * t1 * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (-t1 * t70 - t67 * t68) * t76 * t86 - (-t1 * t68 + t67 * t70) * t80 * t83 * t86;
	unknown(3,2) = -t3 * t70 * t80 * t83 * t86 - t3 * t68 * t76 * t86;
	unknown(3,3) = 0.0e0;
	unknown(3,4) = 0.0e0;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobia_rot_9_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (134->21), mult. (224->70), div. (52->9), fcn. (332->9), ass. (0->76)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t27 * t2 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = pkin(3) + qJ(8);
	t69 = cos(t68);
	t71 = sin(t68);
	t76 = t55 * t71 + t6 * t69;
	t77 = 0.1e1 / t76;
	t81 = -t55 * t69 + t6 * t71;
	t82 = t81 ^ 2;
	t83 = t76 ^ 2;
	t84 = 0.1e1 / t83;
	t87 = 0.1e1 / (t82 * t84 + 0.1e1);
	t93 = t84 * t87;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - (t21 * t24 * t22 + t21 * t26 * t4 - t24 * t2 - t27 * t4) * t1 * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (-t1 * t71 - t67 * t69) * t77 * t87 - (-t1 * t69 + t67 * t71) * t81 * t93;
	unknown(3,2) = -t3 * t71 * t81 * t84 * t87 - t3 * t69 * t77 * t87;
	unknown(3,3) = 0.0e0;
	unknown(3,4) = 0.0e0;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = t81 ^ 2 * t93 + t87;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobia_rot_10_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (158->21), mult. (251->70), div. (57->9), fcn. (367->9), ass. (0->77)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t27 * t2 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = qJ(3) + qJ(9);
	t69 = sin(t68);
	t71 = cos(t68);
	t76 = -t55 * t71 + t6 * t69;
	t77 = 0.1e1 / t76;
	t81 = -t55 * t69 - t6 * t71;
	t82 = t81 ^ 2;
	t83 = t76 ^ 2;
	t84 = 0.1e1 / t83;
	t87 = 0.1e1 / (t82 * t84 + 0.1e1);
	t93 = t84 * t87;
	t106 = t81 ^ 2 * t93 + t87;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - (t21 * t24 * t22 + t21 * t26 * t4 - t24 * t2 - t27 * t4) * t1 * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (t1 * t71 - t67 * t69) * t77 * t87 - (-t1 * t69 - t67 * t71) * t81 * t93;
	unknown(3,2) = t3 * t71 * t81 * t84 * t87 - t3 * t69 * t77 * t87;
	unknown(3,3) = t106;
	unknown(3,4) = 0.0e0;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = t106;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobia_rot_11_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (258->22), mult. (278->70), div. (62->9), fcn. (402->9), ass. (0->77)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t27 * t2 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = qJ(3) + qJ(9) + qJ(10);
	t69 = sin(t68);
	t71 = cos(t68);
	t76 = t55 * t71 - t6 * t69;
	t77 = 0.1e1 / t76;
	t81 = t55 * t69 + t6 * t71;
	t82 = t81 ^ 2;
	t83 = t76 ^ 2;
	t84 = 0.1e1 / t83;
	t87 = 0.1e1 / (t82 * t84 + 0.1e1);
	t93 = t84 * t87;
	t106 = t81 ^ 2 * t93 + t87;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - (t21 * t24 * t22 + t21 * t26 * t4 - t2 * t24 - t27 * t4) * t1 * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (-t1 * t71 + t67 * t69) * t77 * t87 - (t1 * t69 + t67 * t71) * t81 * t93;
	unknown(3,2) = -t3 * t71 * t81 * t84 * t87 + t3 * t69 * t77 * t87;
	unknown(3,3) = t106;
	unknown(3,4) = 0.0e0;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = t106;
	unknown(3,10) = t106;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobia_rot_12_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (258->22), mult. (278->70), div. (62->9), fcn. (402->9), ass. (0->77)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t27 * t2 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = qJ(3) + qJ(4) + qJ(11);
	t69 = cos(t68);
	t71 = sin(t68);
	t76 = -t55 * t71 - t6 * t69;
	t77 = 0.1e1 / t76;
	t81 = t55 * t69 - t6 * t71;
	t82 = t81 ^ 2;
	t83 = t76 ^ 2;
	t84 = 0.1e1 / t83;
	t87 = 0.1e1 / (t82 * t84 + 0.1e1);
	t93 = t84 * t87;
	t106 = t81 ^ 2 * t93 + t87;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - (t21 * t24 * t22 + t21 * t26 * t4 - t2 * t24 - t27 * t4) * t1 * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (t1 * t71 + t67 * t69) * t77 * t87 - (t1 * t69 - t67 * t71) * t81 * t93;
	unknown(3,2) = t3 * t71 * t81 * t84 * t87 + t3 * t69 * t77 * t87;
	unknown(3,3) = t106;
	unknown(3,4) = t106;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = t106;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobia_rot_13_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (334->23), mult. (278->70), div. (62->9), fcn. (402->9), ass. (0->77)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t27 * t2 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
	t69 = sin(t68);
	t71 = cos(t68);
	t76 = t55 * t71 - t6 * t69;
	t77 = 0.1e1 / t76;
	t81 = t55 * t69 + t6 * t71;
	t82 = t81 ^ 2;
	t83 = t76 ^ 2;
	t84 = 0.1e1 / t83;
	t87 = 0.1e1 / (t82 * t84 + 0.1e1);
	t93 = t84 * t87;
	t106 = t81 ^ 2 * t93 + t87;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - (t21 * t24 * t22 + t21 * t26 * t4 - t2 * t24 - t27 * t4) * t1 * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (-t1 * t71 + t67 * t69) * t77 * t87 - (t1 * t69 + t67 * t71) * t81 * t93;
	unknown(3,2) = -t3 * t71 * t81 * t84 * t87 + t3 * t69 * t77 * t87;
	unknown(3,3) = t106;
	unknown(3,4) = t106;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = t106;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 14
	%% Symbolic Calculation
	% From jacobia_rot_14_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (476->24), mult. (305->70), div. (67->9), fcn. (437->9), ass. (0->77)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = cos(qJ(2));
	t3 = t1 * t2;
	t4 = sin(qJ(2));
	t5 = 0.1e1 / t4;
	t6 = cos(qJ(1));
	t7 = t6 ^ 2;
	t8 = t2 ^ 2;
	t10 = t4 ^ 2;
	t11 = 0.1e1 / t10;
	t14 = 0.1e1 / (t7 * t8 * t11 + 0.1e1);
	t21 = t8 * t6 * t11 * t14 + t6 * t14;
	t22 = t6 * t2;
	t23 = atan2(t22, -t4);
	t24 = cos(t23);
	t26 = sin(t23);
	t27 = t26 * t6;
	t29 = t2 * t27 - t24 * t4;
	t31 = t1 ^ 2;
	t33 = t29 ^ 2;
	t34 = 0.1e1 / t33;
	t37 = 0.1e1 / (t31 * t8 * t34 + 0.1e1);
	t38 = 0.1e1 / t29 * t37;
	t52 = t2 * t34 * t37;
	t55 = t1 * t4;
	t67 = t6 * t4;
	t68 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
	t69 = cos(t68);
	t71 = sin(t68);
	t76 = t55 * t71 + t6 * t69;
	t77 = 0.1e1 / t76;
	t81 = -t55 * t69 + t6 * t71;
	t82 = t81 ^ 2;
	t83 = t76 ^ 2;
	t84 = 0.1e1 / t83;
	t87 = 0.1e1 / (t82 * t84 + 0.1e1);
	t93 = t84 * t87;
	t106 = t81 ^ 2 * t93 + t87;
	unknown(1,1) = t3 * t5 * t14;
	unknown(1,2) = t21;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t22 * t38 - (t1 * t8 * t5 * t14 * t24 * t6 - t26 * t1 * t2 + t3 * t14 * t26) * t1 * t52;
	unknown(2,2) = -t55 * t38 - t1 * (t21 * t24 * t22 + t21 * t26 * t4 - t24 * t2 - t27 * t4) * t52;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = (-t1 * t71 - t67 * t69) * t77 * t87 - (-t1 * t69 + t67 * t71) * t81 * t93;
	unknown(3,2) = -t3 * t71 * t81 * t84 * t87 - t3 * t69 * t77 * t87;
	unknown(3,3) = t106;
	unknown(3,4) = t106;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = t106;
	unknown(3,12) = t106;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
elseif link_index == 15
	%% Symbolic Calculation
	% From jacobia_rot_15_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (2834->29), mult. (1049->88), div. (217->11), fcn. (1598->9), ass. (0->85)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
	t5 = cos(t4);
	t7 = cos(qJ(1));
	t8 = sin(t4);
	t10 = t3 * t5 - t7 * t8;
	t11 = cos(qJ(2));
	t12 = 0.1e1 / t11;
	t13 = t10 * t12;
	t14 = 0.1e1 / t5;
	t15 = t7 * t2;
	t18 = -t1 * t8 - t15 * t5;
	t19 = t18 ^ 2;
	t20 = t11 ^ 2;
	t21 = 0.1e1 / t20;
	t23 = t5 ^ 2;
	t24 = 0.1e1 / t23;
	t27 = 0.1e1 / (t19 * t21 * t24 + 0.1e1);
	t28 = t14 * t27;
	t35 = -t2 * t14 * t18 * t21 * t27 + t7 * t27;
	t38 = -t1 * t5 + t15 * t8;
	t45 = -t12 * t8 * t18 * t24 * t27 - t38 * t12 * t28;
	t46 = t11 * t5;
	t47 = atan2(t18, -t46);
	t48 = cos(t47);
	t49 = t48 * t11;
	t51 = sin(t47);
	t53 = t51 * t18 - t49 * t5;
	t54 = 0.1e1 / t53;
	t56 = t10 ^ 2;
	t57 = t53 ^ 2;
	t58 = 0.1e1 / t57;
	t61 = 0.1e1 / (t56 * t58 + 0.1e1);
	t72 = t58 * t61;
	t93 = t3 * t8 + t7 * t5;
	t105 = t93 * t54 * t61 + (t45 * t48 * t18 + t45 * t51 * t46 + t51 * t38 + t49 * t8) * t10 * t72;
	t108 = t1 ^ 2;
	t109 = t108 * t20;
	t110 = t93 ^ 2;
	t111 = 0.1e1 / t110;
	t114 = 0.1e1 / (t109 * t111 + 0.1e1);
	t115 = 0.1e1 / t93 * t114;
	t119 = t11 * t111 * t114;
	t128 = t10 * t1 * t119;
	unknown(1,1) = -t13 * t28;
	unknown(1,2) = t35;
	unknown(1,3) = t45;
	unknown(1,4) = t45;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = t45;
	unknown(1,12) = t45;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t18 * t54 * t61 + (-t13 * t14 * t27 * t48 * t18 - t10 * t27 * t51 + t51 * t10) * t10 * t72;
	unknown(2,2) = -t1 * t11 * t5 * t54 * t61 + (t35 * t48 * t18 + t48 * t2 * t5 + t35 * t51 * t46 - t51 * t7 * t46) * t10 * t72;
	unknown(2,3) = t105;
	unknown(2,4) = t105;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = t105;
	unknown(2,12) = t105;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = t38 * t1 * t119 - t7 * t11 * t115;
	unknown(3,2) = t109 * t8 * t111 * t114 + t3 * t115;
	unknown(3,3) = t128;
	unknown(3,4) = t128;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = t128;
	unknown(3,12) = t128;
	unknown(3,13) = 0.0e0;
	Ja_rot = unknown;
end