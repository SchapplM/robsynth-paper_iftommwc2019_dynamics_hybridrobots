% Analytische Jacobi-Matrix (Translatorisch) für beliebiges Segment von
% KAS5m7OL
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% (Ist für translatorischen Teil egal, kennzeichnet nur den Rechenweg der Herleitung)
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt (0=Basis).
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% Ja_transl [3x13]
%   Translatorischer Teil der analytischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Ja_transl = KAS5m7OL_jacobia_transl_sym_varpar(qJ, link_index, r_i_i_C, ...
  pkin)


%% Coder Information
%#codegen
%$cgargs {zeros(13,1),uint8(0),zeros(3,1),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_jacobia_transl_sym_varpar: qJ has to be [13x1] (double)');
assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ...
	'KAS5m7OL_jacobia_transl_sym_varpar: Position vector r_i_i_C has to be [3x1] double');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m7OL_jacobia_transl_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_jacobia_transl_sym_varpar: pkin has to be [19x1] (double)');
Ja_transl=NaN(3,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobia_transl_0_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
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
	Ja_transl = unknown;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobia_transl_1_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:49
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.06s
	% Computational Cost: add. (2->2), mult. (4->4), div. (0->0), fcn. (4->2), ass. (0->41)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t3 = sin(qJ(1));
	unknown(1,1) = t1 * r_i_i_C(1) - t3 * r_i_i_C(2);
	unknown(1,2) = 0.0e0;
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
	unknown(2,1) = t3 * r_i_i_C(1) + t1 * r_i_i_C(2);
	unknown(2,2) = 0.0e0;
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
	unknown(3,1) = 0.0e0;
	unknown(3,2) = 0.0e0;
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
	Ja_transl = unknown;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobia_transl_2_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.06s
	% Computational Cost: add. (9->9), mult. (22->18), div. (0->0), fcn. (22->4), ass. (0->47)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t5 = cos(qJ(2));
	t6 = t1 * t5;
	t8 = sin(qJ(1));
	t12 = t8 * t5;
	t14 = t8 * t2;
	unknown(1,1) = -t8 * pkin(11) - t3 * r_i_i_C(1) - t6 * r_i_i_C(2) - t8 * r_i_i_C(3);
	unknown(1,2) = -t12 * r_i_i_C(1) + t14 * r_i_i_C(2);
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
	unknown(2,1) = t1 * pkin(11) - t14 * r_i_i_C(1) - t12 * r_i_i_C(2) + t1 * r_i_i_C(3);
	unknown(2,2) = t6 * r_i_i_C(1) - t3 * r_i_i_C(2);
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
	unknown(3,1) = 0.0e0;
	unknown(3,2) = t2 * r_i_i_C(1) + t5 * r_i_i_C(2);
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
	Ja_transl = unknown;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobia_transl_3_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.06s
	% Computational Cost: add. (28->24), mult. (72->46), div. (0->0), fcn. (80->6), ass. (0->55)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = cos(qJ(3));
	t6 = sin(qJ(1));
	t7 = sin(qJ(3));
	t9 = -t3 * t4 - t6 * t7;
	t13 = t3 * t7 - t6 * t4;
	t15 = cos(qJ(2));
	t16 = t1 * t15;
	t21 = t6 * t15;
	t22 = t4 * r_i_i_C(1);
	t24 = t7 * r_i_i_C(2);
	t26 = t6 * t2;
	t32 = t1 * t4 + t26 * t7;
	t36 = -t1 * t7 + t26 * t4;
	unknown(1,1) = -t6 * pkin(11) + t16 * pkin(16) + t9 * r_i_i_C(1) + t13 * r_i_i_C(2) + t16 * r_i_i_C(3);
	unknown(1,2) = -t26 * pkin(16) - t26 * r_i_i_C(3) - t21 * t22 + t21 * t24;
	unknown(1,3) = t32 * r_i_i_C(1) + t36 * r_i_i_C(2);
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
	unknown(2,1) = t1 * pkin(11) + t21 * pkin(16) - t36 * r_i_i_C(1) + t32 * r_i_i_C(2) + t21 * r_i_i_C(3);
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t16 * t22 - t16 * t24;
	unknown(2,3) = -t13 * r_i_i_C(1) + t9 * r_i_i_C(2);
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
	unknown(3,1) = 0.0e0;
	unknown(3,2) = t2 * t4 * r_i_i_C(1) - t2 * t7 * r_i_i_C(2) - t15 * pkin(16) - t15 * r_i_i_C(3);
	unknown(3,3) = t15 * t7 * r_i_i_C(1) + t15 * t4 * r_i_i_C(2);
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
	Ja_transl = unknown;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobia_transl_4_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.07s
	% Computational Cost: add. (81->41), mult. (122->66), div. (0->0), fcn. (134->8), ass. (0->66)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4);
	t5 = cos(t4);
	t7 = sin(qJ(1));
	t8 = sin(t4);
	t10 = -t3 * t5 - t7 * t8;
	t14 = t3 * t8 - t7 * t5;
	t16 = cos(qJ(2));
	t17 = t1 * t16;
	t19 = cos(qJ(3));
	t20 = t19 * pkin(18);
	t23 = sin(qJ(3));
	t28 = t7 * t16;
	t29 = t5 * r_i_i_C(1);
	t31 = t8 * r_i_i_C(2);
	t33 = t7 * t2;
	t40 = t1 * t5 + t33 * t8;
	t41 = t40 * r_i_i_C(1);
	t44 = -t1 * t8 + t33 * t5;
	t45 = t44 * r_i_i_C(2);
	t46 = t23 * pkin(18);
	t67 = -t14 * r_i_i_C(1);
	t68 = t10 * r_i_i_C(2);
	t84 = t16 * t8 * r_i_i_C(1);
	t86 = t16 * t5 * r_i_i_C(2);
	unknown(1,1) = -t7 * t23 * pkin(18) - t7 * pkin(11) + t17 * pkin(16) + t10 * r_i_i_C(1) + t14 * r_i_i_C(2) + t17 * r_i_i_C(3) - t3 * t20;
	unknown(1,2) = -t33 * pkin(16) - t33 * r_i_i_C(3) - t28 * t20 - t28 * t29 + t28 * t31;
	unknown(1,3) = t1 * t19 * pkin(18) + t33 * t46 + t41 + t45;
	unknown(1,4) = t41 + t45;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t1 * t23 * pkin(18) + t1 * pkin(11) + t28 * pkin(16) - t44 * r_i_i_C(1) + t40 * r_i_i_C(2) + t28 * r_i_i_C(3) - t33 * t20;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t17 * t20 + t17 * t29 - t17 * t31;
	unknown(2,3) = t7 * t19 * pkin(18) - t3 * t46 + t67 + t68;
	unknown(2,4) = t67 + t68;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = t2 * t19 * pkin(18) + t2 * t5 * r_i_i_C(1) - t2 * t8 * r_i_i_C(2) - t16 * pkin(16) - t16 * r_i_i_C(3);
	unknown(3,3) = t16 * t23 * pkin(18) + t84 + t86;
	unknown(3,4) = t84 + t86;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobia_transl_5_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (176->53), mult. (166->70), div. (0->0), fcn. (182->10), ass. (0->70)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4) + qJ(5);
	t5 = cos(t4);
	t7 = sin(qJ(1));
	t8 = sin(t4);
	t10 = -t3 * t5 - t7 * t8;
	t14 = t3 * t8 - t7 * t5;
	t16 = cos(qJ(2));
	t17 = t1 * t16;
	t19 = qJ(3) + qJ(4);
	t20 = cos(t19);
	t22 = cos(qJ(3));
	t24 = pkin(6) * t20 + t22 * pkin(18);
	t27 = sin(t19);
	t28 = pkin(6) * t27;
	t29 = sin(qJ(3));
	t31 = t29 * pkin(18) + t28;
	t35 = t7 * t16;
	t36 = t5 * r_i_i_C(1);
	t38 = t8 * r_i_i_C(2);
	t40 = t7 * t2;
	t47 = t1 * t5 + t40 * t8;
	t48 = t47 * r_i_i_C(1);
	t51 = -t1 * t8 + t40 * t5;
	t52 = t51 * r_i_i_C(2);
	t75 = -t14 * r_i_i_C(1);
	t76 = t10 * r_i_i_C(2);
	t94 = t16 * t8 * r_i_i_C(1);
	t96 = t16 * t5 * r_i_i_C(2);
	unknown(1,1) = -t7 * pkin(11) + t17 * pkin(16) + t10 * r_i_i_C(1) + t14 * r_i_i_C(2) + t17 * r_i_i_C(3) - t3 * t24 - t7 * t31;
	unknown(1,2) = -t40 * pkin(16) - t40 * r_i_i_C(3) - t35 * t24 - t35 * t36 + t35 * t38;
	unknown(1,3) = t1 * t24 + t40 * t31 + t48 + t52;
	unknown(1,4) = t1 * pkin(6) * t20 + t40 * t28 + t48 + t52;
	unknown(1,5) = t48 + t52;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t1 * pkin(11) + t35 * pkin(16) - t51 * r_i_i_C(1) + t47 * r_i_i_C(2) + t35 * r_i_i_C(3) + t1 * t31 - t40 * t24;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t17 * t24 + t17 * t36 - t17 * t38;
	unknown(2,3) = t7 * t24 - t3 * t31 + t75 + t76;
	unknown(2,4) = t7 * pkin(6) * t20 - t3 * t28 + t75 + t76;
	unknown(2,5) = t75 + t76;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = t2 * t5 * r_i_i_C(1) - t2 * t8 * r_i_i_C(2) - t16 * pkin(16) - t16 * r_i_i_C(3) + t2 * t24;
	unknown(3,3) = t16 * t31 + t94 + t96;
	unknown(3,4) = t16 * pkin(6) * t27 + t94 + t96;
	unknown(3,5) = t94 + t96;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobia_transl_6_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:49
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (323->68), mult. (215->77), div. (0->0), fcn. (235->12), ass. (0->78)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t5 = sin(t4);
	t7 = sin(qJ(1));
	t8 = cos(t4);
	t10 = t3 * t5 - t7 * t8;
	t14 = t3 * t8 + t7 * t5;
	t16 = cos(qJ(2));
	t17 = t1 * t16;
	t19 = qJ(3) + qJ(4) + qJ(5);
	t20 = cos(t19);
	t21 = pkin(7) * t20;
	t22 = qJ(3) + qJ(4);
	t23 = cos(t22);
	t24 = pkin(6) * t23;
	t25 = cos(qJ(3));
	t27 = t25 * pkin(18) + t21 + t24;
	t30 = sin(t19);
	t31 = pkin(7) * t30;
	t32 = sin(t22);
	t33 = pkin(6) * t32;
	t34 = sin(qJ(3));
	t36 = t34 * pkin(18) + t31 + t33;
	t40 = t7 * t16;
	t41 = t5 * r_i_i_C(1);
	t43 = t8 * r_i_i_C(2);
	t45 = t7 * t2;
	t52 = -t1 * t5 + t45 * t8;
	t53 = t52 * r_i_i_C(1);
	t56 = -t1 * t8 - t45 * t5;
	t57 = t56 * r_i_i_C(2);
	t61 = -t31 - t33;
	t63 = t21 + t24;
	t85 = -t14 * r_i_i_C(1);
	t86 = t10 * r_i_i_C(2);
	t107 = t16 * t8 * r_i_i_C(1);
	t109 = t16 * t5 * r_i_i_C(2);
	unknown(1,1) = -t7 * pkin(11) + t17 * pkin(16) + t10 * r_i_i_C(1) + t14 * r_i_i_C(2) + t17 * r_i_i_C(3) - t3 * t27 - t7 * t36;
	unknown(1,2) = -t45 * pkin(16) - t45 * r_i_i_C(3) - t40 * t27 + t40 * t41 + t40 * t43;
	unknown(1,3) = t1 * t27 + t45 * t36 + t53 + t57;
	unknown(1,4) = t1 * t63 - t45 * t61 + t53 + t57;
	unknown(1,5) = t1 * pkin(7) * t20 + t45 * t31 + t53 + t57;
	unknown(1,6) = t53 + t57;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t1 * pkin(11) + t40 * pkin(16) - t56 * r_i_i_C(1) + t52 * r_i_i_C(2) + t40 * r_i_i_C(3) + t1 * t36 - t45 * t27;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t17 * t27 - t17 * t41 - t17 * t43;
	unknown(2,3) = t7 * t27 - t3 * t36 + t85 + t86;
	unknown(2,4) = t3 * t61 + t7 * t63 + t85 + t86;
	unknown(2,5) = t7 * pkin(7) * t20 - t3 * t31 + t85 + t86;
	unknown(2,6) = t85 + t86;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = -t2 * t5 * r_i_i_C(1) - t2 * t8 * r_i_i_C(2) - t16 * pkin(16) - t16 * r_i_i_C(3) + t2 * t27;
	unknown(3,3) = t16 * t36 + t107 - t109;
	unknown(3,4) = -t16 * t61 + t107 - t109;
	unknown(3,5) = t16 * pkin(7) * t30 + t107 - t109;
	unknown(3,6) = t107 - t109;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobia_transl_7_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (587->115), mult. (406->128), div. (0->0), fcn. (466->14), ass. (0->96)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4) + qJ(5) + qJ(6);
	t5 = sin(t4);
	t7 = sin(qJ(1));
	t8 = cos(t4);
	t10 = t3 * t5 - t7 * t8;
	t11 = cos(qJ(7));
	t13 = cos(qJ(2));
	t14 = t1 * t13;
	t15 = sin(qJ(7));
	t16 = t14 * t15;
	t20 = t14 * t11;
	t25 = -t3 * t8 - t7 * t5;
	t28 = qJ(3) + qJ(4) + qJ(5);
	t29 = cos(t28);
	t30 = pkin(7) * t29;
	t31 = qJ(3) + qJ(4);
	t32 = cos(t31);
	t33 = pkin(6) * t32;
	t34 = cos(qJ(3));
	t36 = t34 * pkin(18) + t30 + t33;
	t39 = sin(t28);
	t40 = pkin(7) * t39;
	t41 = sin(t31);
	t42 = pkin(6) * t41;
	t43 = sin(qJ(3));
	t45 = t43 * pkin(18) + t40 + t42;
	t49 = t7 * t13;
	t50 = t5 * t11;
	t52 = t7 * t2;
	t56 = t5 * t15;
	t61 = t8 * r_i_i_C(3);
	t63 = t8 * pkin(10);
	t70 = -t1 * t5 + t52 * t8;
	t72 = t70 * t11 * r_i_i_C(1);
	t74 = t70 * t15 * r_i_i_C(2);
	t77 = t1 * t8 + t52 * t5;
	t78 = t77 * r_i_i_C(3);
	t79 = -t77 * pkin(10);
	t83 = -t40 - t42;
	t85 = t30 + t33;
	t95 = t49 * t11 - t77 * t15;
	t99 = -t77 * t11 - t49 * t15;
	t125 = t25 * t11 * r_i_i_C(1);
	t127 = t25 * t15 * r_i_i_C(2);
	t128 = -t10 * r_i_i_C(3);
	t129 = t10 * pkin(10);
	t148 = t2 * t5;
	t157 = t2 * t8;
	t163 = t13 * t8;
	t165 = t163 * t11 * r_i_i_C(1);
	t167 = t163 * t15 * r_i_i_C(2);
	t168 = t13 * t5;
	t169 = t168 * r_i_i_C(3);
	t170 = t168 * pkin(10);
	unknown(1,1) = (t10 * t11 + t16) * r_i_i_C(1) + (-t10 * t15 + t20) * r_i_i_C(2) + t25 * r_i_i_C(3) + t25 * pkin(10) - t3 * t36 + t14 * pkin(16) - t7 * t45 - t7 * pkin(11);
	unknown(1,2) = (-t52 * t15 + t49 * t50) * r_i_i_C(1) + (-t52 * t11 - t49 * t56) * r_i_i_C(2) - t49 * t61 - t49 * t63 - t49 * t36 - t52 * pkin(16);
	unknown(1,3) = t1 * t36 + t52 * t45 + t72 - t74 + t78 - t79;
	unknown(1,4) = t1 * t85 - t52 * t83 + t72 - t74 + t78 - t79;
	unknown(1,5) = t1 * pkin(7) * t29 + t52 * t40 + t72 - t74 + t78 - t79;
	unknown(1,6) = t72 - t74 + t78 - t79;
	unknown(1,7) = t95 * r_i_i_C(1) + t99 * r_i_i_C(2);
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = -t70 * pkin(10) + t1 * pkin(11) + t49 * pkin(16) - t99 * r_i_i_C(1) + t95 * r_i_i_C(2) - t70 * r_i_i_C(3) + t1 * t45 - t52 * t36;
	unknown(2,2) = (-t14 * t50 + t3 * t15) * r_i_i_C(1) + (t3 * t11 + t14 * t56) * r_i_i_C(2) + t14 * t61 + t14 * t63 + t14 * t36 + t3 * pkin(16);
	unknown(2,3) = -t3 * t45 + t7 * t36 + t125 - t127 + t128 - t129;
	unknown(2,4) = t3 * t83 + t7 * t85 + t125 - t127 + t128 - t129;
	unknown(2,5) = t7 * pkin(7) * t29 - t3 * t40 + t125 - t127 + t128 - t129;
	unknown(2,6) = t125 - t127 + t128 - t129;
	unknown(2,7) = (t10 * t15 - t20) * r_i_i_C(1) + (t10 * t11 + t16) * r_i_i_C(2);
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = (-t148 * t11 - t13 * t15) * r_i_i_C(1) + (-t13 * t11 + t148 * t15) * r_i_i_C(2) + t157 * r_i_i_C(3) + t157 * pkin(10) + t2 * t36 - t13 * pkin(16);
	unknown(3,3) = t13 * t45 + t165 - t167 + t169 + t170;
	unknown(3,4) = -t13 * t83 + t165 - t167 + t169 + t170;
	unknown(3,5) = t13 * pkin(7) * t39 + t165 - t167 + t169 + t170;
	unknown(3,6) = t165 - t167 + t169 + t170;
	unknown(3,7) = (-t2 * t11 - t168 * t15) * r_i_i_C(1) + (-t168 * t11 + t2 * t15) * r_i_i_C(2);
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobia_transl_8_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.06s
	% Computational Cost: add. (21->21), mult. (52->38), div. (0->0), fcn. (56->6), ass. (0->51)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = sin(pkin(3));
	t6 = sin(qJ(1));
	t7 = cos(pkin(3));
	t15 = cos(qJ(2));
	t16 = t1 * t15;
	t21 = t6 * t15;
	t22 = t4 * r_i_i_C(1);
	t24 = t7 * r_i_i_C(2);
	t26 = t6 * t2;
	unknown(1,1) = (t3 * t4 - t6 * t7) * r_i_i_C(1) + (t3 * t7 + t6 * t4) * r_i_i_C(2) + t16 * r_i_i_C(3) + t16 * pkin(16) - t6 * pkin(11);
	unknown(1,2) = -t26 * pkin(16) - t26 * r_i_i_C(3) + t21 * t22 + t21 * t24;
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
	unknown(2,1) = (t1 * t7 + t26 * t4) * r_i_i_C(1) + (-t1 * t4 + t26 * t7) * r_i_i_C(2) + t21 * r_i_i_C(3) + t21 * pkin(16) + t1 * pkin(11);
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) - t16 * t22 - t16 * t24;
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
	unknown(3,1) = 0.0e0;
	unknown(3,2) = -t2 * t4 * r_i_i_C(1) - t2 * t7 * r_i_i_C(2) - t15 * pkin(16) - t15 * r_i_i_C(3);
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
	Ja_transl = unknown;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobia_transl_9_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.07s
	% Computational Cost: add. (59->32), mult. (90->57), div. (0->0), fcn. (98->8), ass. (0->59)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = pkin(3) + qJ(8);
	t5 = sin(t4);
	t7 = sin(qJ(1));
	t8 = cos(t4);
	t10 = t3 * t5 - t7 * t8;
	t14 = t3 * t8 + t7 * t5;
	t16 = cos(qJ(2));
	t17 = t1 * t16;
	t19 = sin(pkin(3));
	t20 = t19 * pkin(17);
	t23 = cos(pkin(3));
	t28 = t7 * t16;
	t29 = t5 * r_i_i_C(1);
	t31 = t8 * r_i_i_C(2);
	t33 = t7 * t2;
	t40 = -t1 * t5 + t33 * t8;
	t44 = -t1 * t8 - t33 * t5;
	unknown(1,1) = t7 * t23 * pkin(17) - t7 * pkin(11) + t17 * pkin(16) + t10 * r_i_i_C(1) + t14 * r_i_i_C(2) + t17 * r_i_i_C(3) - t3 * t20;
	unknown(1,2) = -t33 * pkin(16) - t33 * r_i_i_C(3) - t28 * t20 + t28 * t29 + t28 * t31;
	unknown(1,3) = 0.0e0;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = t40 * r_i_i_C(1) + t44 * r_i_i_C(2);
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = -t1 * t23 * pkin(17) + t1 * pkin(11) + t28 * pkin(16) - t44 * r_i_i_C(1) + t40 * r_i_i_C(2) + t28 * r_i_i_C(3) - t33 * t20;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t17 * t20 - t17 * t29 - t17 * t31;
	unknown(2,3) = 0.0e0;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = -t14 * r_i_i_C(1) + t10 * r_i_i_C(2);
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = t2 * t19 * pkin(17) - t2 * t5 * r_i_i_C(1) - t2 * t8 * r_i_i_C(2) - t16 * pkin(16) - t16 * r_i_i_C(3);
	unknown(3,3) = 0.0e0;
	unknown(3,4) = 0.0e0;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = t16 * t8 * r_i_i_C(1) - t16 * t5 * r_i_i_C(2);
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobia_transl_10_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:51
	% DurationCPUTime: 0.07s
	% Computational Cost: add. (81->41), mult. (122->66), div. (0->0), fcn. (134->8), ass. (0->66)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(9);
	t5 = cos(t4);
	t7 = sin(qJ(1));
	t8 = sin(t4);
	t10 = -t3 * t5 - t7 * t8;
	t14 = t3 * t8 - t7 * t5;
	t16 = cos(qJ(2));
	t17 = t1 * t16;
	t19 = cos(qJ(3));
	t20 = t19 * pkin(19);
	t23 = sin(qJ(3));
	t28 = t7 * t16;
	t29 = t5 * r_i_i_C(1);
	t31 = t8 * r_i_i_C(2);
	t33 = t7 * t2;
	t40 = t1 * t5 + t33 * t8;
	t41 = t40 * r_i_i_C(1);
	t44 = -t1 * t8 + t33 * t5;
	t45 = t44 * r_i_i_C(2);
	t46 = t23 * pkin(19);
	t67 = -t14 * r_i_i_C(1);
	t68 = t10 * r_i_i_C(2);
	t84 = t16 * t8 * r_i_i_C(1);
	t86 = t16 * t5 * r_i_i_C(2);
	unknown(1,1) = -t7 * t23 * pkin(19) - t7 * pkin(11) + t17 * pkin(16) + t10 * r_i_i_C(1) + t14 * r_i_i_C(2) + t17 * r_i_i_C(3) - t3 * t20;
	unknown(1,2) = -t33 * pkin(16) - t33 * r_i_i_C(3) - t28 * t20 - t28 * t29 + t28 * t31;
	unknown(1,3) = t1 * t19 * pkin(19) + t33 * t46 + t41 + t45;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = t41 + t45;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t1 * t23 * pkin(19) + t1 * pkin(11) + t28 * pkin(16) - t44 * r_i_i_C(1) + t40 * r_i_i_C(2) + t28 * r_i_i_C(3) - t33 * t20;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t17 * t20 + t17 * t29 - t17 * t31;
	unknown(2,3) = t7 * t19 * pkin(19) - t3 * t46 + t67 + t68;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = t67 + t68;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = t2 * t19 * pkin(19) + t2 * t5 * r_i_i_C(1) - t2 * t8 * r_i_i_C(2) - t16 * pkin(16) - t16 * r_i_i_C(3);
	unknown(3,3) = t16 * t23 * pkin(19) + t84 + t86;
	unknown(3,4) = 0.0e0;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = t84 + t86;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobia_transl_11_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.07s
	% Computational Cost: add. (176->53), mult. (166->70), div. (0->0), fcn. (182->10), ass. (0->70)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(9) + qJ(10);
	t5 = cos(t4);
	t7 = sin(qJ(1));
	t8 = sin(t4);
	t10 = t3 * t5 + t7 * t8;
	t14 = -t3 * t8 + t7 * t5;
	t16 = cos(qJ(2));
	t17 = t1 * t16;
	t19 = qJ(3) + qJ(9);
	t20 = cos(t19);
	t22 = cos(qJ(3));
	t24 = pkin(14) * t20 + t22 * pkin(19);
	t27 = sin(t19);
	t28 = pkin(14) * t27;
	t29 = sin(qJ(3));
	t31 = t29 * pkin(19) + t28;
	t35 = t7 * t16;
	t36 = t5 * r_i_i_C(1);
	t38 = t8 * r_i_i_C(2);
	t40 = t7 * t2;
	t47 = -t1 * t5 - t40 * t8;
	t48 = t47 * r_i_i_C(1);
	t51 = t1 * t8 - t40 * t5;
	t52 = t51 * r_i_i_C(2);
	t75 = -t14 * r_i_i_C(1);
	t76 = t10 * r_i_i_C(2);
	t94 = t16 * t8 * r_i_i_C(1);
	t96 = t16 * t5 * r_i_i_C(2);
	unknown(1,1) = -t7 * pkin(11) + t17 * pkin(16) + t10 * r_i_i_C(1) + t14 * r_i_i_C(2) + t17 * r_i_i_C(3) - t3 * t24 - t7 * t31;
	unknown(1,2) = -t40 * pkin(16) - t40 * r_i_i_C(3) - t35 * t24 + t35 * t36 - t35 * t38;
	unknown(1,3) = t1 * t24 + t40 * t31 + t48 + t52;
	unknown(1,4) = 0.0e0;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = t1 * pkin(14) * t20 + t40 * t28 + t48 + t52;
	unknown(1,10) = t48 + t52;
	unknown(1,11) = 0.0e0;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t1 * pkin(11) + t35 * pkin(16) - t51 * r_i_i_C(1) + t47 * r_i_i_C(2) + t35 * r_i_i_C(3) + t1 * t31 - t40 * t24;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t17 * t24 - t17 * t36 + t17 * t38;
	unknown(2,3) = t7 * t24 - t3 * t31 + t75 + t76;
	unknown(2,4) = 0.0e0;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = t7 * pkin(14) * t20 - t3 * t28 + t75 + t76;
	unknown(2,10) = t75 + t76;
	unknown(2,11) = 0.0e0;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = -t2 * t5 * r_i_i_C(1) + t2 * t8 * r_i_i_C(2) - t16 * pkin(16) - t16 * r_i_i_C(3) + t2 * t24;
	unknown(3,3) = t16 * t31 - t94 - t96;
	unknown(3,4) = 0.0e0;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = t16 * pkin(14) * t27 - t94 - t96;
	unknown(3,10) = -t94 - t96;
	unknown(3,11) = 0.0e0;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobia_transl_12_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:49
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (176->53), mult. (166->70), div. (0->0), fcn. (182->10), ass. (0->70)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4) + qJ(11);
	t5 = sin(t4);
	t7 = sin(qJ(1));
	t8 = cos(t4);
	t10 = -t3 * t5 + t7 * t8;
	t14 = -t3 * t8 - t7 * t5;
	t16 = cos(qJ(2));
	t17 = t1 * t16;
	t19 = qJ(3) + qJ(4);
	t20 = cos(t19);
	t22 = cos(qJ(3));
	t24 = pkin(6) * t20 + t22 * pkin(18);
	t27 = sin(t19);
	t28 = pkin(6) * t27;
	t29 = sin(qJ(3));
	t31 = t29 * pkin(18) + t28;
	t35 = t7 * t16;
	t36 = t5 * r_i_i_C(1);
	t38 = t8 * r_i_i_C(2);
	t40 = t7 * t2;
	t47 = t1 * t5 - t40 * t8;
	t48 = t47 * r_i_i_C(1);
	t51 = t1 * t8 + t40 * t5;
	t52 = t51 * r_i_i_C(2);
	t75 = -t14 * r_i_i_C(1);
	t76 = t10 * r_i_i_C(2);
	t94 = t16 * t8 * r_i_i_C(1);
	t96 = t16 * t5 * r_i_i_C(2);
	unknown(1,1) = -t7 * pkin(11) + t17 * pkin(16) + t10 * r_i_i_C(1) + t14 * r_i_i_C(2) + t17 * r_i_i_C(3) - t3 * t24 - t7 * t31;
	unknown(1,2) = -t40 * pkin(16) - t40 * r_i_i_C(3) - t35 * t24 - t35 * t36 - t35 * t38;
	unknown(1,3) = t1 * t24 + t40 * t31 + t48 + t52;
	unknown(1,4) = t1 * pkin(6) * t20 + t40 * t28 + t48 + t52;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = t48 + t52;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t1 * pkin(11) + t35 * pkin(16) - t51 * r_i_i_C(1) + t47 * r_i_i_C(2) + t35 * r_i_i_C(3) + t1 * t31 - t40 * t24;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t17 * t24 + t17 * t36 + t17 * t38;
	unknown(2,3) = t7 * t24 - t3 * t31 + t75 + t76;
	unknown(2,4) = t7 * pkin(6) * t20 - t3 * t28 + t75 + t76;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = t75 + t76;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = t2 * t5 * r_i_i_C(1) + t2 * t8 * r_i_i_C(2) - t16 * pkin(16) - t16 * r_i_i_C(3) + t2 * t24;
	unknown(3,3) = t16 * t31 - t94 + t96;
	unknown(3,4) = t16 * pkin(6) * t27 - t94 + t96;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = -t94 + t96;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobia_transl_13_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:49
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (220->54), mult. (166->70), div. (0->0), fcn. (182->10), ass. (0->70)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
	t5 = cos(t4);
	t7 = sin(qJ(1));
	t8 = sin(t4);
	t10 = t3 * t5 + t7 * t8;
	t14 = -t3 * t8 + t7 * t5;
	t16 = cos(qJ(2));
	t17 = t1 * t16;
	t19 = qJ(3) + qJ(4);
	t20 = cos(t19);
	t22 = cos(qJ(3));
	t24 = pkin(6) * t20 + t22 * pkin(18);
	t27 = sin(t19);
	t28 = pkin(6) * t27;
	t29 = sin(qJ(3));
	t31 = t29 * pkin(18) + t28;
	t35 = t7 * t16;
	t36 = t5 * r_i_i_C(1);
	t38 = t8 * r_i_i_C(2);
	t40 = t7 * t2;
	t47 = -t1 * t5 - t40 * t8;
	t48 = t47 * r_i_i_C(1);
	t51 = t1 * t8 - t40 * t5;
	t52 = t51 * r_i_i_C(2);
	t75 = -t14 * r_i_i_C(1);
	t76 = t10 * r_i_i_C(2);
	t94 = t16 * t8 * r_i_i_C(1);
	t96 = t16 * t5 * r_i_i_C(2);
	unknown(1,1) = -t7 * pkin(11) + t17 * pkin(16) + t10 * r_i_i_C(1) + t14 * r_i_i_C(2) + t17 * r_i_i_C(3) - t3 * t24 - t7 * t31;
	unknown(1,2) = -t40 * pkin(16) - t40 * r_i_i_C(3) - t35 * t24 + t35 * t36 - t35 * t38;
	unknown(1,3) = t1 * t24 + t40 * t31 + t48 + t52;
	unknown(1,4) = t1 * pkin(6) * t20 + t40 * t28 + t48 + t52;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = t48 + t52;
	unknown(1,12) = 0.0e0;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t1 * pkin(11) + t35 * pkin(16) - t51 * r_i_i_C(1) + t47 * r_i_i_C(2) + t35 * r_i_i_C(3) + t1 * t31 - t40 * t24;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t17 * t24 - t17 * t36 + t17 * t38;
	unknown(2,3) = t7 * t24 - t3 * t31 + t75 + t76;
	unknown(2,4) = t7 * pkin(6) * t20 - t3 * t28 + t75 + t76;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = t75 + t76;
	unknown(2,12) = 0.0e0;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = -t2 * t5 * r_i_i_C(1) + t2 * t8 * r_i_i_C(2) - t16 * pkin(16) - t16 * r_i_i_C(3) + t2 * t24;
	unknown(3,3) = t16 * t31 - t94 - t96;
	unknown(3,4) = t16 * pkin(6) * t27 - t94 - t96;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = -t94 - t96;
	unknown(3,12) = 0.0e0;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 14
	%% Symbolic Calculation
	% From jacobia_transl_14_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.08s
	% Computational Cost: add. (399->70), mult. (215->77), div. (0->0), fcn. (235->12), ass. (0->78)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
	t5 = sin(t4);
	t7 = sin(qJ(1));
	t8 = cos(t4);
	t10 = t3 * t5 - t7 * t8;
	t14 = t3 * t8 + t7 * t5;
	t16 = cos(qJ(2));
	t17 = t1 * t16;
	t19 = qJ(3) + qJ(4);
	t20 = cos(t19);
	t21 = pkin(6) * t20;
	t22 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
	t23 = cos(t22);
	t24 = pkin(9) * t23;
	t25 = cos(qJ(3));
	t27 = t25 * pkin(18) + t21 - t24;
	t30 = sin(t19);
	t31 = pkin(6) * t30;
	t32 = sin(t22);
	t33 = pkin(9) * t32;
	t34 = sin(qJ(3));
	t36 = t34 * pkin(18) + t31 - t33;
	t40 = t7 * t16;
	t41 = t5 * r_i_i_C(1);
	t43 = t8 * r_i_i_C(2);
	t45 = t7 * t2;
	t52 = -t1 * t5 + t45 * t8;
	t53 = t52 * r_i_i_C(1);
	t56 = -t1 * t8 - t45 * t5;
	t57 = t56 * r_i_i_C(2);
	t61 = -t31 + t33;
	t63 = t21 - t24;
	t85 = -t14 * r_i_i_C(1);
	t86 = t10 * r_i_i_C(2);
	t107 = t16 * t8 * r_i_i_C(1);
	t109 = t16 * t5 * r_i_i_C(2);
	unknown(1,1) = -t7 * pkin(11) + t17 * pkin(16) + t10 * r_i_i_C(1) + t14 * r_i_i_C(2) + t17 * r_i_i_C(3) - t3 * t27 - t7 * t36;
	unknown(1,2) = -t45 * pkin(16) - t45 * r_i_i_C(3) - t40 * t27 + t40 * t41 + t40 * t43;
	unknown(1,3) = t1 * t27 + t45 * t36 + t53 + t57;
	unknown(1,4) = t1 * t63 - t45 * t61 + t53 + t57;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = -t1 * pkin(9) * t23 - t45 * t33 + t53 + t57;
	unknown(1,12) = t53 + t57;
	unknown(1,13) = 0.0e0;
	unknown(2,1) = t1 * pkin(11) + t40 * pkin(16) - t56 * r_i_i_C(1) + t52 * r_i_i_C(2) + t40 * r_i_i_C(3) + t1 * t36 - t45 * t27;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(3) + t17 * t27 - t17 * t41 - t17 * t43;
	unknown(2,3) = t7 * t27 - t3 * t36 + t85 + t86;
	unknown(2,4) = t3 * t61 + t7 * t63 + t85 + t86;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = -t7 * pkin(9) * t23 + t3 * t33 + t85 + t86;
	unknown(2,12) = t85 + t86;
	unknown(2,13) = 0.0e0;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = -t2 * t5 * r_i_i_C(1) - t2 * t8 * r_i_i_C(2) - t16 * pkin(16) - t16 * r_i_i_C(3) + t2 * t27;
	unknown(3,3) = t16 * t36 + t107 - t109;
	unknown(3,4) = -t16 * t61 + t107 - t109;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = -t16 * pkin(9) * t32 + t107 - t109;
	unknown(3,12) = t107 - t109;
	unknown(3,13) = 0.0e0;
	Ja_transl = unknown;
elseif link_index == 15
	%% Symbolic Calculation
	% From jacobia_transl_15_floatb_twist_matlab.m
	% OptimizationMode: 1
	% StartTime: 2020-06-30 18:16:50
	% EndTime: 2020-06-30 18:16:50
	% DurationCPUTime: 0.09s
	% Computational Cost: add. (557->91), mult. (278->86), div. (0->0), fcn. (313->12), ass. (0->85)
	unknown=NaN(3,13);
	t1 = cos(qJ(1));
	t2 = sin(qJ(2));
	t3 = t1 * t2;
	t4 = qJ(3) + qJ(4) + qJ(11) + pkin(1) + qJ(12);
	t5 = sin(t4);
	t7 = sin(qJ(1));
	t8 = cos(t4);
	t10 = t3 * t5 - t7 * t8;
	t12 = cos(qJ(2));
	t13 = t1 * t12;
	t17 = -t3 * t8 - t7 * t5;
	t20 = qJ(3) + qJ(4);
	t21 = cos(t20);
	t22 = pkin(6) * t21;
	t23 = qJ(3) + qJ(4) + qJ(11) + pkin(1);
	t24 = cos(t23);
	t25 = pkin(9) * t24;
	t26 = cos(qJ(3));
	t28 = t26 * pkin(18) + t22 - t25;
	t31 = sin(t20);
	t32 = pkin(6) * t31;
	t33 = sin(t23);
	t34 = pkin(9) * t33;
	t35 = sin(qJ(3));
	t37 = t35 * pkin(18) + t32 - t34;
	t41 = t7 * t12;
	t42 = t5 * r_i_i_C(1);
	t44 = t7 * t2;
	t46 = t8 * r_i_i_C(3);
	t48 = t8 * qJ(13);
	t55 = -t1 * t5 + t44 * t8;
	t56 = t55 * r_i_i_C(1);
	t59 = t1 * t8 + t44 * t5;
	t60 = t59 * r_i_i_C(3);
	t61 = -t59 * qJ(13);
	t65 = -t32 + t34;
	t67 = t22 - t25;
	t91 = t17 * r_i_i_C(1);
	t92 = -t10 * r_i_i_C(3);
	t93 = t10 * qJ(13);
	t108 = t2 * t8;
	t114 = t12 * t8;
	t115 = t114 * r_i_i_C(1);
	t116 = t12 * t5;
	t117 = t116 * r_i_i_C(3);
	t118 = t116 * qJ(13);
	unknown(1,1) = -t7 * pkin(11) + t13 * pkin(16) + t10 * r_i_i_C(1) + t13 * r_i_i_C(2) + t17 * r_i_i_C(3) + t17 * qJ(13) - t3 * t28 - t7 * t37;
	unknown(1,2) = -t44 * pkin(16) - t44 * r_i_i_C(2) - t41 * t28 + t41 * t42 - t41 * t46 - t41 * t48;
	unknown(1,3) = t1 * t28 + t44 * t37 + t56 + t60 - t61;
	unknown(1,4) = t1 * t67 - t44 * t65 + t56 + t60 - t61;
	unknown(1,5) = 0.0e0;
	unknown(1,6) = 0.0e0;
	unknown(1,7) = 0.0e0;
	unknown(1,8) = 0.0e0;
	unknown(1,9) = 0.0e0;
	unknown(1,10) = 0.0e0;
	unknown(1,11) = -t1 * pkin(9) * t24 - t44 * t34 + t56 + t60 - t61;
	unknown(1,12) = t56 + t60 - t61;
	unknown(1,13) = -t55;
	unknown(2,1) = t1 * pkin(11) + t41 * pkin(16) + t59 * r_i_i_C(1) + t41 * r_i_i_C(2) - t55 * r_i_i_C(3) - t55 * qJ(13) + t1 * t37 - t44 * t28;
	unknown(2,2) = t3 * pkin(16) + t3 * r_i_i_C(2) + t13 * t28 - t13 * t42 + t13 * t46 + t13 * t48;
	unknown(2,3) = t7 * t28 - t3 * t37 + t91 + t92 - t93;
	unknown(2,4) = t3 * t65 + t7 * t67 + t91 + t92 - t93;
	unknown(2,5) = 0.0e0;
	unknown(2,6) = 0.0e0;
	unknown(2,7) = 0.0e0;
	unknown(2,8) = 0.0e0;
	unknown(2,9) = 0.0e0;
	unknown(2,10) = 0.0e0;
	unknown(2,11) = -t7 * pkin(9) * t24 + t3 * t34 + t91 + t92 - t93;
	unknown(2,12) = t91 + t92 - t93;
	unknown(2,13) = -t17;
	unknown(3,1) = 0.0e0;
	unknown(3,2) = -t2 * t5 * r_i_i_C(1) - t12 * pkin(16) - t12 * r_i_i_C(2) + t108 * r_i_i_C(3) + t108 * qJ(13) + t2 * t28;
	unknown(3,3) = t12 * t37 + t115 + t117 + t118;
	unknown(3,4) = -t12 * t65 + t115 + t117 + t118;
	unknown(3,5) = 0.0e0;
	unknown(3,6) = 0.0e0;
	unknown(3,7) = 0.0e0;
	unknown(3,8) = 0.0e0;
	unknown(3,9) = 0.0e0;
	unknown(3,10) = 0.0e0;
	unknown(3,11) = -t12 * pkin(9) * t33 + t115 + t117 + t118;
	unknown(3,12) = t115 + t117 + t118;
	unknown(3,13) = -t114;
	Ja_transl = unknown;
end