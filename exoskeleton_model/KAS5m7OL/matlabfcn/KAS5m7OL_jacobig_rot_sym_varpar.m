% Rotatorische Teilmatrix der geometrischen Jacobi-Matrix für beliebiges Segment von
% KAS5m7OL
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
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% Jg_rot [3x13]
%   Rotatorische Teilmatrix der geometrischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Jg_rot = KAS5m7OL_jacobig_rot_sym_varpar(qJ, link_index, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),uint8(0),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_jacobig_rot_sym_varpar: qJ has to be [13x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m7OL_jacobig_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_jacobig_rot_sym_varpar: pkin has to be [19x1] (double)');
Jg_rot=NaN(3,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobig_rot_0_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
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
	Jg_rot = unknown;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobig_rot_1_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
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
	unknown(3,1) = 1;
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
	Jg_rot = unknown;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobig_rot_2_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (2->2), ass. (0->41)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
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
	unknown(2,2) = t2;
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
	unknown(3,1) = 1;
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
	Jg_rot = unknown;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobig_rot_3_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (2->2), mult. (2->2), div. (0->0), fcn. (7->4), ass. (0->43)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = (t2 * t3);
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
	unknown(2,2) = t2;
	unknown(2,3) = -(t1 * t3);
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
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
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
	Jg_rot = unknown;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobig_rot_4_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (4->4), mult. (4->2), div. (0->0), fcn. (12->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = t1 * t3;
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = t4;
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
	unknown(2,2) = t2;
	unknown(2,3) = -t5;
	unknown(2,4) = -t5;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
	unknown(3,4) = -t6;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobig_rot_5_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (6->6), mult. (6->2), div. (0->0), fcn. (17->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = t1 * t3;
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = t4;
	unknown(1,5) = t4;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = -t5;
	unknown(2,4) = -t5;
	unknown(2,5) = -t5;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
	unknown(3,4) = -t6;
	unknown(3,5) = -t6;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobig_rot_6_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (8->8), mult. (8->2), div. (0->0), fcn. (22->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = t1 * t3;
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = t4;
	unknown(1,5) = t4;
	unknown(1,6) = t4;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = -t5;
	unknown(2,4) = -t5;
	unknown(2,5) = -t5;
	unknown(2,6) = -t5;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
	unknown(3,4) = -t6;
	unknown(3,5) = -t6;
	unknown(3,6) = -t6;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobig_rot_7_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (26->14), mult. (15->9), div. (0->0), fcn. (34->6), ass. (0->48)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = sin(qJ(2));
	t7 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t8 = cos(t7);
	t10 = sin(t7);
	t13 = t1 * t3;
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = t4;
	unknown(1,5) = t4;
	unknown(1,6) = t4;
	unknown(1,7) = (-t2 * t5 * t8 + t1 * t10);
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = -t13;
	unknown(2,4) = -t13;
	unknown(2,5) = -t13;
	unknown(2,6) = -t13;
	unknown(2,7) = (t1 * t5 * t8 + t2 * t10);
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t5;
	unknown(3,4) = -t5;
	unknown(3,5) = -t5;
	unknown(3,6) = -t5;
	unknown(3,7) = -(t3 * t8);
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobig_rot_8_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (2->2), ass. (0->41)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
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
	unknown(2,2) = t2;
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
	unknown(3,1) = 1;
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
	Jg_rot = unknown;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobig_rot_9_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (2->2), mult. (2->2), div. (0->0), fcn. (7->4), ass. (0->43)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = 0;
	unknown(1,4) = 0;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = (t2 * t3);
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = 0;
	unknown(2,4) = 0;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = -(t1 * t3);
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = 0;
	unknown(3,4) = 0;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = -t6;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobig_rot_10_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (4->4), mult. (4->2), div. (0->0), fcn. (12->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = t1 * t3;
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = 0;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = t4;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = -t5;
	unknown(2,4) = 0;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = -t5;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
	unknown(3,4) = 0;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = -t6;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobig_rot_11_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (6->6), mult. (6->2), div. (0->0), fcn. (17->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = t1 * t3;
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = 0;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = t4;
	unknown(1,10) = t4;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = -t5;
	unknown(2,4) = 0;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = -t5;
	unknown(2,10) = -t5;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
	unknown(3,4) = 0;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = -t6;
	unknown(3,10) = -t6;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobig_rot_12_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (6->6), mult. (6->2), div. (0->0), fcn. (17->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = t1 * t3;
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = t4;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = t4;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = -t5;
	unknown(2,4) = -t5;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = -t5;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
	unknown(3,4) = -t6;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = -t6;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobig_rot_13_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (6->6), mult. (6->2), div. (0->0), fcn. (17->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = t1 * t3;
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = t4;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = t4;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = -t5;
	unknown(2,4) = -t5;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = -t5;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
	unknown(3,4) = -t6;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = -t6;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 14
	%% Symbolic Calculation
	% From jacobig_rot_14_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (8->8), mult. (8->2), div. (0->0), fcn. (22->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = t1 * t3;
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = t4;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = t4;
	unknown(1,12) = t4;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = -t5;
	unknown(2,4) = -t5;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = -t5;
	unknown(2,12) = -t5;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
	unknown(3,4) = -t6;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = -t6;
	unknown(3,12) = -t6;
	unknown(3,13) = 0;
	Jg_rot = unknown;
elseif link_index == 15
	%% Symbolic Calculation
	% From jacobig_rot_15_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (8->8), mult. (8->2), div. (0->0), fcn. (22->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(1));
	t3 = cos(qJ(2));
	t4 = t2 * t3;
	t5 = t1 * t3;
	t6 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = t1;
	unknown(1,3) = t4;
	unknown(1,4) = t4;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = t4;
	unknown(1,12) = t4;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t2;
	unknown(2,3) = -t5;
	unknown(2,4) = -t5;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = -t5;
	unknown(2,12) = -t5;
	unknown(2,13) = 0;
	unknown(3,1) = 1;
	unknown(3,2) = 0;
	unknown(3,3) = -t6;
	unknown(3,4) = -t6;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = -t6;
	unknown(3,12) = -t6;
	unknown(3,13) = 0;
	Jg_rot = unknown;
end