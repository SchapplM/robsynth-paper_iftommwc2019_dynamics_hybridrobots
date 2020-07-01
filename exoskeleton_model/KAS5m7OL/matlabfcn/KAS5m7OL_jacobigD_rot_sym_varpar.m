% Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix für beliebiges Segment von
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
% qJD [13x1]
%   Generalized joint velocities
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt. (0=Basis).
%   Siehe auch: bsp_3T1R_fkine_fixb_rotmat_mdh_sym_varpar.m
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% JgD_rot [3x13]
%   Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function JgD_rot = KAS5m7OL_jacobigD_rot_sym_varpar(qJ, qJD, link_index, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),uint8(0),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_jacobigD_rot_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m7OL_jacobigD_rot_sym_varpar: qJD has to be [13x1] (double)');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m7OL_jacobigD_rot_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_jacobigD_rot_sym_varpar: pkin has to be [19x1] (double)');
JgD_rot=NaN(3,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobigD_rot_0_floatb_twist_matlab.m
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
	JgD_rot = unknown;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobigD_rot_1_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
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
	JgD_rot = unknown;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobigD_rot_2_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.05s
	% Computational Cost: add. (1->1), mult. (2->2), div. (0->0), fcn. (2->2), ass. (0->41)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t3 = cos(qJ(1));
	unknown(1,1) = 0;
	unknown(1,2) = -(qJD(1) * t1);
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
	unknown(2,2) = (qJD(1) * t3);
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
	JgD_rot = unknown;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobigD_rot_3_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (4->4), mult. (11->9), div. (0->0), fcn. (11->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = (-t1 * qJD(2) * t8 + t4 * t5);
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
	unknown(2,2) = t4;
	unknown(2,3) = (t3 * qJD(2) * t8 + t2 * t5);
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
	unknown(3,3) = -(qJD(2) * t5);
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
	JgD_rot = unknown;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobigD_rot_4_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:52
	% EndTime: 2020-06-30 18:16:52
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (7->5), mult. (20->9), div. (0->0), fcn. (20->4), ass. (0->48)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	t10 = -t1 * qJD(2) * t8 + t4 * t5;
	t14 = t3 * qJD(2) * t8 + t2 * t5;
	t15 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = t10;
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
	unknown(2,2) = t4;
	unknown(2,3) = t14;
	unknown(2,4) = t14;
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
	unknown(3,3) = -t15;
	unknown(3,4) = -t15;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobigD_rot_5_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (10->6), mult. (29->9), div. (0->0), fcn. (29->4), ass. (0->48)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	t10 = -t1 * qJD(2) * t8 + t4 * t5;
	t14 = t3 * qJD(2) * t8 + t2 * t5;
	t15 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = t10;
	unknown(1,5) = t10;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = t14;
	unknown(2,4) = t14;
	unknown(2,5) = t14;
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
	unknown(3,3) = -t15;
	unknown(3,4) = -t15;
	unknown(3,5) = -t15;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobigD_rot_6_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (13->7), mult. (38->9), div. (0->0), fcn. (38->4), ass. (0->48)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	t10 = -t1 * qJD(2) * t8 + t4 * t5;
	t14 = t3 * qJD(2) * t8 + t2 * t5;
	t15 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = t10;
	unknown(1,5) = t10;
	unknown(1,6) = t10;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = t14;
	unknown(2,4) = t14;
	unknown(2,5) = t14;
	unknown(2,6) = t14;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 0;
	unknown(3,2) = 0;
	unknown(3,3) = -t15;
	unknown(3,4) = -t15;
	unknown(3,5) = -t15;
	unknown(3,6) = -t15;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobigD_rot_7_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:52
	% EndTime: 2020-06-30 18:16:52
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (73->22), mult. (68->30), div. (0->0), fcn. (68->6), ass. (0->57)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t7 = t1 * qJD(2);
	t8 = sin(qJ(2));
	t10 = t4 * t5 - t7 * t8;
	t11 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t12 = cos(t11);
	t13 = t8 * t12;
	t15 = t5 * t12;
	t18 = qJD(3) + qJD(4) + qJD(5) + qJD(6);
	t19 = sin(t11);
	t20 = t18 * t19;
	t27 = t3 * qJD(2);
	t29 = t2 * t5 + t27 * t8;
	t38 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = t10;
	unknown(1,5) = t10;
	unknown(1,6) = t10;
	unknown(1,7) = (t1 * t8 * t20 + t3 * t18 * t12 - t4 * t13 - t7 * t15 - t2 * t19);
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = t29;
	unknown(2,4) = t29;
	unknown(2,5) = t29;
	unknown(2,6) = t29;
	unknown(2,7) = (t1 * t18 * t12 - t3 * t8 * t20 - t2 * t13 + t27 * t15 + t4 * t19);
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 0;
	unknown(3,2) = 0;
	unknown(3,3) = -t38;
	unknown(3,4) = -t38;
	unknown(3,5) = -t38;
	unknown(3,6) = -t38;
	unknown(3,7) = (qJD(2) * t8 * t12 + t5 * t18 * t19);
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobigD_rot_8_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (1->1), mult. (2->2), div. (0->0), fcn. (2->2), ass. (0->41)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t3 = cos(qJ(1));
	unknown(1,1) = 0;
	unknown(1,2) = -(qJD(1) * t1);
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
	unknown(2,2) = (qJD(1) * t3);
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
	JgD_rot = unknown;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobigD_rot_9_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.02s
	% Computational Cost: add. (4->4), mult. (11->9), div. (0->0), fcn. (11->4), ass. (0->45)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = 0;
	unknown(1,4) = 0;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = (-t1 * qJD(2) * t8 + t4 * t5);
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = 0;
	unknown(2,4) = 0;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = (t3 * qJD(2) * t8 + t2 * t5);
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
	unknown(3,8) = -(qJD(2) * t5);
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobigD_rot_10_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:52
	% EndTime: 2020-06-30 18:16:52
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (7->5), mult. (20->9), div. (0->0), fcn. (20->4), ass. (0->48)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	t10 = -t1 * qJD(2) * t8 + t4 * t5;
	t14 = t3 * qJD(2) * t8 + t2 * t5;
	t15 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = 0;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = t10;
	unknown(1,10) = 0;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = t14;
	unknown(2,4) = 0;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = t14;
	unknown(2,10) = 0;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 0;
	unknown(3,2) = 0;
	unknown(3,3) = -t15;
	unknown(3,4) = 0;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = -t15;
	unknown(3,10) = 0;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobigD_rot_11_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:52
	% EndTime: 2020-06-30 18:16:52
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (10->6), mult. (29->9), div. (0->0), fcn. (29->4), ass. (0->48)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	t10 = -t1 * qJD(2) * t8 + t4 * t5;
	t14 = t3 * qJD(2) * t8 + t2 * t5;
	t15 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = 0;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = t10;
	unknown(1,10) = t10;
	unknown(1,11) = 0;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = t14;
	unknown(2,4) = 0;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = t14;
	unknown(2,10) = t14;
	unknown(2,11) = 0;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 0;
	unknown(3,2) = 0;
	unknown(3,3) = -t15;
	unknown(3,4) = 0;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = -t15;
	unknown(3,10) = -t15;
	unknown(3,11) = 0;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobigD_rot_12_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:52
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (10->6), mult. (29->9), div. (0->0), fcn. (29->4), ass. (0->48)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	t10 = -t1 * qJD(2) * t8 + t4 * t5;
	t14 = t3 * qJD(2) * t8 + t2 * t5;
	t15 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = t10;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = t10;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = t14;
	unknown(2,4) = t14;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = t14;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 0;
	unknown(3,2) = 0;
	unknown(3,3) = -t15;
	unknown(3,4) = -t15;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = -t15;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobigD_rot_13_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:52
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (10->6), mult. (29->9), div. (0->0), fcn. (29->4), ass. (0->48)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	t10 = -t1 * qJD(2) * t8 + t4 * t5;
	t14 = t3 * qJD(2) * t8 + t2 * t5;
	t15 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = t10;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = t10;
	unknown(1,12) = 0;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = t14;
	unknown(2,4) = t14;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = t14;
	unknown(2,12) = 0;
	unknown(2,13) = 0;
	unknown(3,1) = 0;
	unknown(3,2) = 0;
	unknown(3,3) = -t15;
	unknown(3,4) = -t15;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = -t15;
	unknown(3,12) = 0;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 14
	%% Symbolic Calculation
	% From jacobigD_rot_14_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:52
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (13->7), mult. (38->9), div. (0->0), fcn. (38->4), ass. (0->48)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	t10 = -t1 * qJD(2) * t8 + t4 * t5;
	t14 = t3 * qJD(2) * t8 + t2 * t5;
	t15 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = t10;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = t10;
	unknown(1,12) = t10;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = t14;
	unknown(2,4) = t14;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = t14;
	unknown(2,12) = t14;
	unknown(2,13) = 0;
	unknown(3,1) = 0;
	unknown(3,2) = 0;
	unknown(3,3) = -t15;
	unknown(3,4) = -t15;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = -t15;
	unknown(3,12) = -t15;
	unknown(3,13) = 0;
	JgD_rot = unknown;
elseif link_index == 15
	%% Symbolic Calculation
	% From jacobigD_rot_15_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:51
	% EndTime: 2020-06-30 18:16:52
	% DurationCPUTime: 0.03s
	% Computational Cost: add. (13->7), mult. (38->9), div. (0->0), fcn. (38->4), ass. (0->48)
	unknown=NaN(3,13);
	t1 = sin(qJ(1));
	t2 = qJD(1) * t1;
	t3 = cos(qJ(1));
	t4 = qJD(1) * t3;
	t5 = cos(qJ(2));
	t8 = sin(qJ(2));
	t10 = -t1 * qJD(2) * t8 + t4 * t5;
	t14 = t3 * qJD(2) * t8 + t2 * t5;
	t15 = qJD(2) * t5;
	unknown(1,1) = 0;
	unknown(1,2) = -t2;
	unknown(1,3) = t10;
	unknown(1,4) = t10;
	unknown(1,5) = 0;
	unknown(1,6) = 0;
	unknown(1,7) = 0;
	unknown(1,8) = 0;
	unknown(1,9) = 0;
	unknown(1,10) = 0;
	unknown(1,11) = t10;
	unknown(1,12) = t10;
	unknown(1,13) = 0;
	unknown(2,1) = 0;
	unknown(2,2) = t4;
	unknown(2,3) = t14;
	unknown(2,4) = t14;
	unknown(2,5) = 0;
	unknown(2,6) = 0;
	unknown(2,7) = 0;
	unknown(2,8) = 0;
	unknown(2,9) = 0;
	unknown(2,10) = 0;
	unknown(2,11) = t14;
	unknown(2,12) = t14;
	unknown(2,13) = 0;
	unknown(3,1) = 0;
	unknown(3,2) = 0;
	unknown(3,3) = -t15;
	unknown(3,4) = -t15;
	unknown(3,5) = 0;
	unknown(3,6) = 0;
	unknown(3,7) = 0;
	unknown(3,8) = 0;
	unknown(3,9) = 0;
	unknown(3,10) = 0;
	unknown(3,11) = -t15;
	unknown(3,12) = -t15;
	unknown(3,13) = 0;
	JgD_rot = unknown;
end