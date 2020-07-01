% Analytische Jacobi-Matrix (Translatorisch) für beliebiges Segment von
% KAS5m5OL
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
% pkin [12x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8]';
% 
% Output:
% Ja_transl [3x13]
%   Translatorischer Teil der analytischen Jacobi-Matrix

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Ja_transl = KAS5m5OL_jacobia_transl_sym_varpar(qJ, link_index, r_i_i_C, ...
  pkin)


%% Coder Information
%#codegen
%$cgargs {zeros(13,1),uint8(0),zeros(3,1),zeros(12,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_jacobia_transl_sym_varpar: qJ has to be [13x1] (double)');
assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ...
	'KAS5m5OL_jacobia_transl_sym_varpar: Position vector r_i_i_C has to be [3x1] double');
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
	'KAS5m5OL_jacobia_transl_sym_varpar: link_index has to be [1x1] uint8');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_jacobia_transl_sym_varpar: pkin has to be [12x1] (double)');
Ja_transl=NaN(3,13);
if link_index == 0
	%% Symbolic Calculation
	% From jacobia_transl_0_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:30
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.06s
	% Computational Cost: add. (0->0), mult. (0->0), div. (0->0), fcn. (0->0), ass. (0->1)
	t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_transl = t1;
elseif link_index == 1
	%% Symbolic Calculation
	% From jacobia_transl_1_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:30
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.06s
	% Computational Cost: add. (2->2), mult. (4->4), div. (0->0), fcn. (4->2), ass. (0->3)
	t2 = cos(qJ(1));
	t1 = sin(qJ(1));
	t3 = [t2 * r_i_i_C(1) - t1 * r_i_i_C(2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; t1 * r_i_i_C(1) + t2 * r_i_i_C(2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_transl = t3;
elseif link_index == 2
	%% Symbolic Calculation
	% From jacobia_transl_2_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.07s
	% Computational Cost: add. (9->6), mult. (22->10), div. (0->0), fcn. (22->4), ass. (0->8)
	t7 = pkin(9) + r_i_i_C(3);
	t1 = sin(qJ(2));
	t3 = cos(qJ(2));
	t6 = r_i_i_C(1) * t3 - r_i_i_C(2) * t1;
	t5 = t1 * r_i_i_C(1) + t3 * r_i_i_C(2);
	t4 = cos(qJ(1));
	t2 = sin(qJ(1));
	t8 = [-t7 * t2 - t5 * t4, -t6 * t2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t5 * t2 + t7 * t4, t6 * t4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_transl = t8;
elseif link_index == 3
	%% Symbolic Calculation
	% From jacobia_transl_3_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (28->18), mult. (72->31), div. (0->0), fcn. (80->6), ass. (0->19)
	t15 = pkin(10) + r_i_i_C(3);
	t9 = cos(qJ(2));
	t12 = t15 * t9;
	t5 = sin(qJ(3));
	t8 = cos(qJ(3));
	t11 = r_i_i_C(1) * t8 - r_i_i_C(2) * t5;
	t6 = sin(qJ(2));
	t18 = t11 * t9 + t15 * t6;
	t7 = sin(qJ(1));
	t17 = t7 * t5;
	t16 = t7 * t8;
	t10 = cos(qJ(1));
	t14 = t10 * t5;
	t13 = t10 * t8;
	t4 = -t6 * t13 - t17;
	t3 = t6 * t14 - t16;
	t2 = t6 * t16 - t14;
	t1 = t6 * t17 + t13;
	t19 = [-t7 * pkin(9) + t4 * r_i_i_C(1) + t3 * r_i_i_C(2) + t10 * t12, -t18 * t7, r_i_i_C(1) * t1 + r_i_i_C(2) * t2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; pkin(9) * t10 - t2 * r_i_i_C(1) + t1 * r_i_i_C(2) + t7 * t12, t18 * t10, -r_i_i_C(1) * t3 + r_i_i_C(2) * t4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t11 * t6 - t12, (r_i_i_C(1) * t5 + r_i_i_C(2) * t8) * t9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_transl = t19;
elseif link_index == 4
	%% Symbolic Calculation
	% From jacobia_transl_4_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (81->27), mult. (122->42), div. (0->0), fcn. (134->8), ass. (0->30)
	t18 = cos(qJ(2));
	t33 = pkin(10) + r_i_i_C(3);
	t35 = t33 * t18;
	t15 = sin(qJ(2));
	t13 = qJ(3) + qJ(4);
	t11 = sin(t13);
	t12 = cos(t13);
	t17 = cos(qJ(3));
	t29 = pkin(4) * t17;
	t20 = r_i_i_C(1) * t12 - r_i_i_C(2) * t11 + t29;
	t34 = t33 * t15 + t20 * t18;
	t19 = cos(qJ(1));
	t23 = t19 * t12;
	t16 = sin(qJ(1));
	t26 = t16 * t11;
	t5 = t15 * t26 + t23;
	t24 = t19 * t11;
	t25 = t16 * t12;
	t6 = t15 * t25 - t24;
	t32 = t5 * r_i_i_C(1) + t6 * r_i_i_C(2);
	t7 = t15 * t24 - t25;
	t8 = -t15 * t23 - t26;
	t31 = -t7 * r_i_i_C(1) + t8 * r_i_i_C(2);
	t14 = sin(qJ(3));
	t30 = pkin(4) * t14;
	t28 = (r_i_i_C(1) * t11 + r_i_i_C(2) * t12) * t18;
	t27 = t14 * t15;
	t22 = pkin(9) + t30;
	t21 = -t15 * t29 + t35;
	t1 = [t8 * r_i_i_C(1) + t7 * r_i_i_C(2) - t22 * t16 + t21 * t19, -t34 * t16, (t16 * t27 + t17 * t19) * pkin(4) + t32, t32, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t6 * r_i_i_C(1) + t5 * r_i_i_C(2) + t21 * t16 + t22 * t19, t34 * t19, (t16 * t17 - t19 * t27) * pkin(4) + t31, t31, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, t20 * t15 - t35, t18 * t30 + t28, t28, 0, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_transl = t1;
elseif link_index == 5
	%% Symbolic Calculation
	% From jacobia_transl_5_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.12s
	% Computational Cost: add. (176->35), mult. (166->50), div. (0->0), fcn. (182->10), ass. (0->33)
	t21 = cos(qJ(2));
	t35 = pkin(10) + r_i_i_C(3);
	t38 = t35 * t21;
	t19 = sin(qJ(2));
	t18 = qJ(3) + qJ(4);
	t16 = cos(t18);
	t10 = pkin(5) * t16 + cos(qJ(3)) * pkin(4);
	t17 = qJ(5) + t18;
	t13 = sin(t17);
	t14 = cos(t17);
	t24 = r_i_i_C(1) * t14 - r_i_i_C(2) * t13 + t10;
	t37 = t35 * t19 + t24 * t21;
	t15 = sin(t18);
	t32 = pkin(5) * t15;
	t9 = t32 + sin(qJ(3)) * pkin(4);
	t36 = pkin(9) + t9;
	t22 = cos(qJ(1));
	t26 = t22 * t14;
	t20 = sin(qJ(1));
	t29 = t20 * t13;
	t5 = t19 * t29 + t26;
	t27 = t22 * t13;
	t28 = t20 * t14;
	t6 = t19 * t28 - t27;
	t34 = t5 * r_i_i_C(1) + t6 * r_i_i_C(2);
	t7 = t19 * t27 - t28;
	t8 = -t19 * t26 - t29;
	t33 = -t7 * r_i_i_C(1) + t8 * r_i_i_C(2);
	t31 = t19 * t20;
	t30 = t19 * t22;
	t25 = (r_i_i_C(1) * t13 + r_i_i_C(2) * t14) * t21;
	t23 = -t19 * t10 + t38;
	t1 = [t8 * r_i_i_C(1) + t7 * r_i_i_C(2) - t36 * t20 + t23 * t22, -t37 * t20, t22 * t10 + t9 * t31 + t34, (t15 * t31 + t16 * t22) * pkin(5) + t34, t34, 0, 0, 0, 0, 0, 0, 0, 0; -t6 * r_i_i_C(1) + t5 * r_i_i_C(2) + t23 * t20 + t36 * t22, t37 * t22, t20 * t10 - t9 * t30 + t33, (-t15 * t30 + t16 * t20) * pkin(5) + t33, t33, 0, 0, 0, 0, 0, 0, 0, 0; 0, t24 * t19 - t38, t21 * t9 + t25, t21 * t32 + t25, t25, 0, 0, 0, 0, 0, 0, 0, 0;];
	Ja_transl = t1;
elseif link_index == 6
	%% Symbolic Calculation
	% From jacobia_transl_6_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:30
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.16s
	% Computational Cost: add. (323->46), mult. (215->57), div. (0->0), fcn. (235->12), ass. (0->37)
	t26 = cos(qJ(2));
	t40 = pkin(10) + r_i_i_C(3);
	t43 = t40 * t26;
	t24 = sin(qJ(2));
	t23 = qJ(3) + qJ(4);
	t22 = qJ(5) + t23;
	t19 = cos(t22);
	t12 = pkin(6) * t19 + pkin(5) * cos(t23);
	t10 = cos(qJ(3)) * pkin(4) + t12;
	t20 = qJ(6) + t22;
	t15 = sin(t20);
	t16 = cos(t20);
	t29 = r_i_i_C(1) * t15 + r_i_i_C(2) * t16 - t10;
	t42 = t40 * t24 - t29 * t26;
	t18 = sin(t22);
	t37 = pkin(6) * t18;
	t11 = -pkin(5) * sin(t23) - t37;
	t9 = sin(qJ(3)) * pkin(4) - t11;
	t41 = pkin(9) + t9;
	t27 = cos(qJ(1));
	t30 = t27 * t16;
	t25 = sin(qJ(1));
	t33 = t25 * t15;
	t5 = t24 * t33 + t30;
	t31 = t27 * t15;
	t32 = t25 * t16;
	t6 = t24 * t32 - t31;
	t39 = t6 * r_i_i_C(1) - t5 * r_i_i_C(2);
	t7 = t24 * t31 - t32;
	t8 = t24 * t30 + t33;
	t38 = -t8 * r_i_i_C(1) + t7 * r_i_i_C(2);
	t36 = r_i_i_C(2) * t15;
	t35 = t24 * t25;
	t34 = t24 * t27;
	t28 = -t24 * t10 + t43;
	t13 = t26 * t16 * r_i_i_C(1);
	t1 = [t7 * r_i_i_C(1) + t8 * r_i_i_C(2) - t41 * t25 + t28 * t27, -t42 * t25, t27 * t10 + t9 * t35 + t39, -t11 * t35 + t27 * t12 + t39, (t18 * t35 + t19 * t27) * pkin(6) + t39, t39, 0, 0, 0, 0, 0, 0, 0; t5 * r_i_i_C(1) + t6 * r_i_i_C(2) + t28 * t25 + t41 * t27, t42 * t27, t25 * t10 - t9 * t34 + t38, t11 * t34 + t25 * t12 + t38, (-t18 * t34 + t19 * t25) * pkin(6) + t38, t38, 0, 0, 0, 0, 0, 0, 0; 0, -t29 * t24 - t43, t13 + (t9 - t36) * t26, t13 + (-t11 - t36) * t26, t13 + (-t36 + t37) * t26, -t26 * t36 + t13, 0, 0, 0, 0, 0, 0, 0;];
	Ja_transl = t1;
elseif link_index == 7
	%% Symbolic Calculation
	% From jacobia_transl_7_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:33
	% DurationCPUTime: 0.27s
	% Computational Cost: add. (587->62), mult. (406->89), div. (0->0), fcn. (466->14), ass. (0->49)
	t31 = sin(qJ(2));
	t34 = cos(qJ(2));
	t30 = sin(qJ(7));
	t33 = cos(qJ(7));
	t42 = t33 * r_i_i_C(2) + pkin(10);
	t39 = t30 * r_i_i_C(1) + t42;
	t29 = qJ(3) + qJ(4);
	t28 = qJ(5) + t29;
	t25 = cos(t28);
	t14 = pkin(6) * t25 + pkin(5) * cos(t29);
	t12 = cos(qJ(3)) * pkin(4) + t14;
	t26 = qJ(6) + t28;
	t21 = sin(t26);
	t22 = cos(t26);
	t61 = pkin(11) + r_i_i_C(3);
	t59 = t30 * r_i_i_C(2);
	t64 = t33 * r_i_i_C(1) - t59;
	t62 = t64 * t21 - t61 * t22 - t12;
	t65 = -t39 * t31 + t62 * t34;
	t24 = sin(t28);
	t60 = pkin(6) * t24;
	t35 = cos(qJ(1));
	t46 = t35 * t21;
	t32 = sin(qJ(1));
	t50 = t32 * t22;
	t9 = t31 * t46 - t50;
	t57 = t9 * t30;
	t13 = -pkin(5) * sin(t29) - t60;
	t11 = sin(qJ(3)) * pkin(4) - t13;
	t56 = -t11 - pkin(9);
	t54 = t31 * t12;
	t53 = t31 * t32;
	t52 = t31 * t35;
	t51 = t32 * t21;
	t49 = t34 * t30;
	t48 = t34 * t33;
	t47 = t34 * t35;
	t45 = t35 * t22;
	t44 = t22 * t59;
	t43 = t22 * r_i_i_C(1) * t48 + t61 * t21 * t34;
	t41 = t30 * t47 + t9 * t33;
	t7 = t31 * t51 + t45;
	t8 = t31 * t50 - t46;
	t38 = t61 * t7 + t64 * t8;
	t10 = t31 * t45 + t51;
	t37 = -t64 * t10 - t61 * t9;
	t2 = t32 * t49 + t7 * t33;
	t1 = -t7 * t30 + t32 * t48;
	t3 = [t41 * r_i_i_C(1) - r_i_i_C(2) * t57 + (t42 * t34 - t54) * t35 + t56 * t32 - t61 * t10, t65 * t32, t11 * t53 + t35 * t12 + t38, -t13 * t53 + t35 * t14 + t38, (t24 * t53 + t25 * t35) * pkin(6) + t38, t38, t1 * r_i_i_C(1) - t2 * r_i_i_C(2), 0, 0, 0, 0, 0, 0; t2 * r_i_i_C(1) + t1 * r_i_i_C(2) - t61 * t8 - t56 * t35 + (t34 * pkin(10) - t54) * t32, -t65 * t35, -t11 * t52 + t32 * t12 + t37, t13 * t52 + t32 * t14 + t37, (-t24 * t52 + t25 * t32) * pkin(6) + t37, t37, (-t33 * t47 + t57) * r_i_i_C(1) + t41 * r_i_i_C(2), 0, 0, 0, 0, 0, 0; 0, -t31 * t62 - t39 * t34, (t11 - t44) * t34 + t43, (-t13 - t44) * t34 + t43, (-t44 + t60) * t34 + t43, -t34 * t44 + t43, (-t21 * t49 - t31 * t33) * r_i_i_C(1) + (-t21 * t48 + t31 * t30) * r_i_i_C(2), 0, 0, 0, 0, 0, 0;];
	Ja_transl = t3;
elseif link_index == 8
	%% Symbolic Calculation
	% From jacobia_transl_8_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.09s
	% Computational Cost: add. (28->18), mult. (72->31), div. (0->0), fcn. (80->6), ass. (0->19)
	t15 = pkin(12) + r_i_i_C(3);
	t9 = cos(qJ(2));
	t12 = t15 * t9;
	t5 = sin(qJ(8));
	t8 = cos(qJ(8));
	t11 = r_i_i_C(1) * t5 + r_i_i_C(2) * t8;
	t6 = sin(qJ(2));
	t18 = t11 * t9 - t15 * t6;
	t7 = sin(qJ(1));
	t17 = t7 * t5;
	t16 = t7 * t8;
	t10 = cos(qJ(1));
	t14 = t10 * t5;
	t13 = t10 * t8;
	t4 = t6 * t13 + t17;
	t3 = t6 * t14 - t16;
	t2 = t6 * t16 - t14;
	t1 = t6 * t17 + t13;
	t19 = [-t7 * pkin(9) + t3 * r_i_i_C(1) + t4 * r_i_i_C(2) + t10 * t12, t18 * t7, 0, 0, 0, 0, 0, r_i_i_C(1) * t2 - r_i_i_C(2) * t1, 0, 0, 0, 0, 0; pkin(9) * t10 + t1 * r_i_i_C(1) + t2 * r_i_i_C(2) + t7 * t12, -t18 * t10, 0, 0, 0, 0, 0, -r_i_i_C(1) * t4 + r_i_i_C(2) * t3, 0, 0, 0, 0, 0; 0, -t11 * t6 - t12, 0, 0, 0, 0, 0, (r_i_i_C(1) * t8 - r_i_i_C(2) * t5) * t9, 0, 0, 0, 0, 0;];
	Ja_transl = t19;
elseif link_index == 9
	%% Symbolic Calculation
	% From jacobia_transl_9_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:30
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (81->28), mult. (122->42), div. (0->0), fcn. (134->8), ass. (0->31)
	t17 = cos(qJ(2));
	t32 = pkin(12) + r_i_i_C(3);
	t34 = t32 * t17;
	t14 = sin(qJ(2));
	t12 = qJ(8) + qJ(9);
	t10 = sin(t12);
	t11 = cos(t12);
	t13 = sin(qJ(8));
	t29 = pkin(7) * t13;
	t19 = r_i_i_C(1) * t10 + r_i_i_C(2) * t11 + t29;
	t33 = t32 * t14 - t19 * t17;
	t18 = cos(qJ(1));
	t22 = t18 * t11;
	t15 = sin(qJ(1));
	t25 = t15 * t10;
	t5 = t14 * t25 + t22;
	t23 = t18 * t10;
	t24 = t15 * t11;
	t6 = t14 * t24 - t23;
	t31 = t6 * r_i_i_C(1) - t5 * r_i_i_C(2);
	t7 = t14 * t23 - t24;
	t8 = t14 * t22 + t25;
	t30 = -t8 * r_i_i_C(1) + t7 * r_i_i_C(2);
	t16 = cos(qJ(8));
	t28 = pkin(7) * t16;
	t27 = r_i_i_C(2) * t10;
	t26 = t14 * t16;
	t21 = pkin(9) + t28;
	t20 = t14 * t29 + t34;
	t9 = t17 * t11 * r_i_i_C(1);
	t1 = [t7 * r_i_i_C(1) + t8 * r_i_i_C(2) - t21 * t15 + t20 * t18, -t33 * t15, 0, 0, 0, 0, 0, (-t13 * t18 + t15 * t26) * pkin(7) + t31, t31, 0, 0, 0, 0; t5 * r_i_i_C(1) + t6 * r_i_i_C(2) + t20 * t15 + t21 * t18, t33 * t18, 0, 0, 0, 0, 0, (-t13 * t15 - t18 * t26) * pkin(7) + t30, t30, 0, 0, 0, 0; 0, -t19 * t14 - t34, 0, 0, 0, 0, 0, t9 + (-t27 + t28) * t17, -t17 * t27 + t9, 0, 0, 0, 0;];
	Ja_transl = t1;
elseif link_index == 10
	%% Symbolic Calculation
	% From jacobia_transl_10_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.11s
	% Computational Cost: add. (81->27), mult. (122->42), div. (0->0), fcn. (134->8), ass. (0->30)
	t18 = cos(qJ(2));
	t33 = pkin(10) + r_i_i_C(3);
	t35 = t33 * t18;
	t15 = sin(qJ(2));
	t13 = qJ(3) + qJ(10);
	t11 = sin(t13);
	t12 = cos(t13);
	t17 = cos(qJ(3));
	t28 = pkin(1) * t17;
	t20 = r_i_i_C(1) * t12 - r_i_i_C(2) * t11 + t28;
	t34 = t33 * t15 + t20 * t18;
	t19 = cos(qJ(1));
	t23 = t19 * t12;
	t16 = sin(qJ(1));
	t26 = t16 * t11;
	t5 = t15 * t26 + t23;
	t24 = t19 * t11;
	t25 = t16 * t12;
	t6 = t15 * t25 - t24;
	t32 = t5 * r_i_i_C(1) + t6 * r_i_i_C(2);
	t7 = t15 * t24 - t25;
	t8 = -t15 * t23 - t26;
	t31 = -t7 * r_i_i_C(1) + t8 * r_i_i_C(2);
	t30 = (r_i_i_C(1) * t11 + r_i_i_C(2) * t12) * t18;
	t14 = sin(qJ(3));
	t29 = pkin(1) * t14;
	t27 = t14 * t15;
	t22 = pkin(9) + t29;
	t21 = -t15 * t28 + t35;
	t1 = [t8 * r_i_i_C(1) + t7 * r_i_i_C(2) - t22 * t16 + t21 * t19, -t34 * t16, (t16 * t27 + t17 * t19) * pkin(1) + t32, 0, 0, 0, 0, 0, 0, t32, 0, 0, 0; -t6 * r_i_i_C(1) + t5 * r_i_i_C(2) + t21 * t16 + t22 * t19, t34 * t19, (t16 * t17 - t19 * t27) * pkin(1) + t31, 0, 0, 0, 0, 0, 0, t31, 0, 0, 0; 0, t20 * t15 - t35, t18 * t29 + t30, 0, 0, 0, 0, 0, 0, t30, 0, 0, 0;];
	Ja_transl = t1;
elseif link_index == 11
	%% Symbolic Calculation
	% From jacobia_transl_11_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:30
	% EndTime: 2020-06-27 19:20:31
	% DurationCPUTime: 0.12s
	% Computational Cost: add. (176->35), mult. (166->50), div. (0->0), fcn. (182->10), ass. (0->33)
	t19 = cos(qJ(2));
	t33 = pkin(10) + r_i_i_C(3);
	t36 = t33 * t19;
	t17 = sin(qJ(2));
	t16 = qJ(3) + qJ(10);
	t14 = cos(t16);
	t10 = pkin(2) * t14 + cos(qJ(3)) * pkin(1);
	t15 = qJ(11) + t16;
	t11 = sin(t15);
	t12 = cos(t15);
	t22 = r_i_i_C(1) * t12 - r_i_i_C(2) * t11 - t10;
	t35 = t33 * t17 - t22 * t19;
	t13 = sin(t16);
	t30 = pkin(2) * t13;
	t9 = t30 + sin(qJ(3)) * pkin(1);
	t34 = pkin(9) + t9;
	t20 = cos(qJ(1));
	t24 = t20 * t12;
	t18 = sin(qJ(1));
	t27 = t18 * t11;
	t5 = -t17 * t27 - t24;
	t25 = t20 * t11;
	t26 = t18 * t12;
	t6 = t17 * t26 - t25;
	t32 = t5 * r_i_i_C(1) - t6 * r_i_i_C(2);
	t7 = t17 * t25 - t26;
	t8 = t17 * t24 + t27;
	t31 = t7 * r_i_i_C(1) + t8 * r_i_i_C(2);
	t29 = t17 * t18;
	t28 = t17 * t20;
	t23 = -r_i_i_C(1) * t11 - r_i_i_C(2) * t12;
	t21 = -t17 * t10 + t36;
	t1 = [t8 * r_i_i_C(1) - t7 * r_i_i_C(2) - t34 * t18 + t21 * t20, -t35 * t18, t20 * t10 + t9 * t29 + t32, 0, 0, 0, 0, 0, 0, (t13 * t29 + t14 * t20) * pkin(2) + t32, t32, 0, 0; t6 * r_i_i_C(1) + t5 * r_i_i_C(2) + t21 * t18 + t34 * t20, t35 * t20, t18 * t10 - t9 * t28 + t31, 0, 0, 0, 0, 0, 0, (-t13 * t28 + t14 * t18) * pkin(2) + t31, t31, 0, 0; 0, -t22 * t17 - t36, (t23 + t9) * t19, 0, 0, 0, 0, 0, 0, (t23 + t30) * t19, t23 * t19, 0, 0;];
	Ja_transl = t1;
elseif link_index == 12
	%% Symbolic Calculation
	% From jacobia_transl_12_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.16s
	% Computational Cost: add. (323->43), mult. (215->57), div. (0->0), fcn. (235->12), ass. (0->36)
	t25 = cos(qJ(2));
	t39 = pkin(10) + r_i_i_C(3);
	t42 = t39 * t25;
	t23 = sin(qJ(2));
	t22 = qJ(3) + qJ(10);
	t20 = qJ(11) + t22;
	t18 = cos(t20);
	t12 = -pkin(3) * t18 + pkin(2) * cos(t22);
	t10 = cos(qJ(3)) * pkin(1) + t12;
	t19 = qJ(12) + t20;
	t13 = sin(t19);
	t14 = cos(t19);
	t28 = r_i_i_C(1) * t14 - r_i_i_C(2) * t13 - t10;
	t41 = t39 * t23 - t28 * t25;
	t17 = sin(t20);
	t36 = pkin(3) * t17;
	t11 = -pkin(2) * sin(t22) + t36;
	t9 = -sin(qJ(3)) * pkin(1) + t11;
	t40 = pkin(9) - t9;
	t26 = cos(qJ(1));
	t30 = t26 * t14;
	t24 = sin(qJ(1));
	t33 = t24 * t13;
	t5 = -t23 * t33 - t30;
	t31 = t26 * t13;
	t32 = t24 * t14;
	t6 = t23 * t32 - t31;
	t38 = t5 * r_i_i_C(1) - t6 * r_i_i_C(2);
	t7 = t23 * t31 - t32;
	t8 = t23 * t30 + t33;
	t37 = t7 * r_i_i_C(1) + t8 * r_i_i_C(2);
	t35 = t23 * t24;
	t34 = t23 * t26;
	t29 = -r_i_i_C(1) * t13 - r_i_i_C(2) * t14;
	t27 = -t23 * t10 + t42;
	t1 = [t8 * r_i_i_C(1) - t7 * r_i_i_C(2) - t40 * t24 + t27 * t26, -t41 * t24, t26 * t10 - t9 * t35 + t38, 0, 0, 0, 0, 0, 0, -t11 * t35 + t26 * t12 + t38, (-t17 * t35 - t18 * t26) * pkin(3) + t38, t38, 0; t6 * r_i_i_C(1) + t5 * r_i_i_C(2) + t27 * t24 + t40 * t26, t41 * t26, t24 * t10 + t9 * t34 + t37, 0, 0, 0, 0, 0, 0, t11 * t34 + t24 * t12 + t37, (t17 * t34 - t18 * t24) * pkin(3) + t37, t37, 0; 0, -t28 * t23 - t42, (t29 - t9) * t25, 0, 0, 0, 0, 0, 0, (-t11 + t29) * t25, (t29 - t36) * t25, t29 * t25, 0;];
	Ja_transl = t1;
elseif link_index == 13
	%% Symbolic Calculation
	% From jacobia_transl_13_floatb_twist_matlab.m
	% OptimizationMode: 2
	% StartTime: 2020-06-27 19:20:31
	% EndTime: 2020-06-27 19:20:32
	% DurationCPUTime: 0.17s
	% Computational Cost: add. (424->43), mult. (235->57), div. (0->0), fcn. (259->12), ass. (0->36)
	t30 = cos(qJ(2));
	t42 = pkin(10) + r_i_i_C(3);
	t44 = t42 * t30;
	t28 = sin(qJ(2));
	t27 = qJ(3) + qJ(10);
	t25 = qJ(11) + t27;
	t24 = cos(t25);
	t15 = -pkin(3) * t24 + pkin(2) * cos(t27);
	t13 = cos(qJ(3)) * pkin(1) + t15;
	t20 = qJ(12) + qJ(13) + t25;
	t18 = sin(t20);
	t19 = cos(t20);
	t33 = r_i_i_C(1) * t19 - r_i_i_C(2) * t18 + t13;
	t43 = t42 * t28 + t33 * t30;
	t31 = cos(qJ(1));
	t34 = t31 * t19;
	t29 = sin(qJ(1));
	t37 = t29 * t18;
	t7 = t28 * t37 + t34;
	t35 = t31 * t18;
	t36 = t29 * t19;
	t8 = t28 * t36 - t35;
	t1 = t7 * r_i_i_C(1) + t8 * r_i_i_C(2);
	t10 = -t28 * t34 - t37;
	t9 = t28 * t35 - t36;
	t2 = -t9 * r_i_i_C(1) + t10 * r_i_i_C(2);
	t23 = sin(t25);
	t40 = pkin(3) * t23;
	t14 = -pkin(2) * sin(t27) + t40;
	t12 = -sin(qJ(3)) * pkin(1) + t14;
	t41 = pkin(9) - t12;
	t39 = t28 * t29;
	t38 = t28 * t31;
	t11 = (r_i_i_C(1) * t18 + r_i_i_C(2) * t19) * t30;
	t32 = -t28 * t13 + t44;
	t3 = [t10 * r_i_i_C(1) + t9 * r_i_i_C(2) - t41 * t29 + t32 * t31, -t43 * t29, -t12 * t39 + t31 * t13 + t1, 0, 0, 0, 0, 0, 0, -t14 * t39 + t31 * t15 + t1, (-t23 * t39 - t24 * t31) * pkin(3) + t1, t1, t1; -t8 * r_i_i_C(1) + t7 * r_i_i_C(2) + t32 * t29 + t41 * t31, t43 * t31, t12 * t38 + t29 * t13 + t2, 0, 0, 0, 0, 0, 0, t14 * t38 + t29 * t15 + t2, (t23 * t38 - t24 * t29) * pkin(3) + t2, t2, t2; 0, t33 * t28 - t44, -t30 * t12 + t11, 0, 0, 0, 0, 0, 0, -t30 * t14 + t11, -t30 * t40 + t11, t11, t11;];
	Ja_transl = t3;
end