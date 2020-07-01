% Calculate forward kinematics (homogenous transformation matrices) for fixed-base
% KAS5m5OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [12x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8]';
% 
% Output:
% Tc_mdh [4x4x(13+1)]
%   homogenous transformation matrices for each (body) frame (MDH)
%   1:  mdh base (link 0) -> mdh base link 0 (unit matrix, no information)
%   ...
%   14:  mdh base (link 0) -> mdh frame (14-1), link (14-1)
%   ...
%   13+1:  mdh base (link 0) -> mdh frame (13)
% T_c_stack [(13+1)*3 x 4]
%   stacked matrices from Tc_mdh into one 2D array, last row left out.
%   Last row only contains [0 0 0 1].

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Tc_mdh, Tc_stack] = KAS5m5OL_fkine_fixb_rotmat_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(12,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m5OL_fkine_fixb_rotmat_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [12 1]), ...
  'KAS5m5OL_fkine_fixb_rotmat_mdh_sym_varpar: pkin has to be [12x1] (double)');

%% Symbolic Calculation
% From fkine_mdh_floatb_twist_rotmat_matlab.m
% OptimizationMode: 2
% StartTime: 2020-06-27 19:20:22
% EndTime: 2020-06-27 19:20:23
% DurationCPUTime: 0.43s
% Computational Cost: add. (554->141), mult. (448->173), div. (0->0), fcn. (613->26), ass. (0->78)
t65 = cos(qJ(2));
t92 = pkin(10) * t65;
t60 = sin(qJ(2));
t61 = sin(qJ(1));
t91 = t61 * t60;
t62 = cos(qJ(8));
t90 = t61 * t62;
t64 = cos(qJ(3));
t89 = t61 * t64;
t35 = t61 * t65;
t56 = qJ(3) + qJ(4);
t46 = qJ(5) + t56;
t38 = qJ(6) + t46;
t24 = sin(t38);
t88 = t65 * t24;
t25 = cos(t38);
t87 = t65 * t25;
t57 = sin(qJ(8));
t86 = t65 * t57;
t85 = t65 * t64;
t66 = cos(qJ(1));
t84 = t66 * t60;
t83 = t66 * t62;
t82 = t66 * t64;
t81 = t66 * t65;
t54 = qJ(3) + qJ(10);
t39 = sin(t54);
t59 = sin(qJ(3));
t15 = t59 * pkin(1) + pkin(2) * t39;
t40 = cos(t54);
t16 = t64 * pkin(1) + pkin(2) * t40;
t43 = sin(t56);
t17 = t59 * pkin(4) + pkin(5) * t43;
t45 = cos(t56);
t18 = t64 * pkin(4) + pkin(5) * t45;
t53 = pkin(8) + 0;
t80 = t61 * pkin(9) + 0;
t79 = t66 * pkin(9) + 0;
t41 = qJ(11) + t54;
t78 = pkin(12) * t35 + t79;
t77 = pkin(10) * t35 + t79;
t32 = qJ(12) + t41;
t76 = -t60 * pkin(10) + t53;
t75 = -t60 * pkin(12) + t53;
t74 = -t57 * t84 + t90;
t73 = t57 * t91 + t83;
t72 = t61 * t59 + t60 * t82;
t71 = t66 * t59 - t60 * t89;
t70 = -pkin(10) * t81 + t80;
t69 = -pkin(12) * t81 + t80;
t36 = sin(t46);
t13 = pkin(6) * t36 + t17;
t37 = cos(t46);
t14 = pkin(6) * t37 + t18;
t68 = t61 * t13 + t14 * t84 + t70;
t67 = t66 * t13 - t14 * t91 + t77;
t63 = cos(qJ(7));
t58 = sin(qJ(7));
t55 = qJ(8) + qJ(9);
t44 = cos(t55);
t42 = sin(t55);
t31 = cos(t41);
t30 = sin(t41);
t23 = qJ(13) + t32;
t22 = cos(t32);
t21 = sin(t32);
t20 = cos(t23);
t19 = sin(t23);
t12 = -pkin(3) * t31 + t16;
t11 = -pkin(3) * t30 + t15;
t8 = t61 * t24 + t25 * t84;
t7 = -t24 * t84 + t61 * t25;
t6 = -t66 * t24 + t25 * t91;
t5 = t24 * t91 + t66 * t25;
t3 = -t65 * t12 + t76;
t2 = t61 * t11 + (t12 * t60 - t92) * t66 + t80;
t1 = t66 * t11 - t12 * t91 + t77;
t4 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0; t61, t66, 0, 0; -t66, t61, 0, 0; 0, 0, 1, t53; -t91, -t35, t66, t79; t84, t81, t61, t80; -t65, t60, 0, t53; t71, t59 * t91 + t82, t35, t77; t72, -t59 * t84 + t89, -t81, t70; -t85, t65 * t59, -t60, t76; t66 * t43 - t45 * t91, t43 * t91 + t66 * t45, t35, t71 * pkin(4) + t77; t61 * t43 + t45 * t84, -t43 * t84 + t61 * t45, -t81, t72 * pkin(4) + t70; -t65 * t45, t65 * t43, -t60, -pkin(4) * t85 + t76; t66 * t36 - t37 * t91, t36 * t91 + t66 * t37, t35, t66 * t17 - t18 * t91 + t77; t61 * t36 + t37 * t84, -t36 * t84 + t61 * t37, -t81, t61 * t17 + (t18 * t60 - t92) * t66 + t80; -t65 * t37, t65 * t36, -t60, -t65 * t18 + t76; t5, t6, t35, t67; t7, -t8, -t81, t68; t88, t87, -t60, -t65 * t14 + t76; t58 * t35 + t5 * t63, t63 * t35 - t5 * t58, -t6, -t6 * pkin(11) + t67; -t58 * t81 + t7 * t63, -t7 * t58 - t63 * t81, t8, t8 * pkin(11) + t68; -t60 * t58 + t63 * t88, -t58 * t88 - t60 * t63, -t87, (-pkin(11) * t25 - t14) * t65 + t76; t73, -t66 * t57 + t60 * t90, t35, t78; t74, -t61 * t57 - t60 * t83, -t81, t69; t86, t65 * t62, -t60, t75; t42 * t91 + t66 * t44, -t66 * t42 + t44 * t91, t35, t73 * pkin(7) + t78; -t42 * t84 + t61 * t44, -t61 * t42 - t44 * t84, -t81, t74 * pkin(7) + t69; t65 * t42, t65 * t44, -t60, pkin(7) * t86 + t75; t66 * t39 - t40 * t91, t39 * t91 + t66 * t40, t35, t71 * pkin(1) + t77; t61 * t39 + t40 * t84, -t39 * t84 + t61 * t40, -t81, t72 * pkin(1) + t70; -t65 * t40, t65 * t39, -t60, -pkin(1) * t85 + t76; -t66 * t30 + t31 * t91, -t30 * t91 - t66 * t31, t35, t66 * t15 - t16 * t91 + t77; -t61 * t30 - t31 * t84, t30 * t84 - t61 * t31, -t81, t61 * t15 + (t16 * t60 - t92) * t66 + t80; t65 * t31, -t65 * t30, -t60, -t65 * t16 + t76; -t66 * t21 + t22 * t91, -t21 * t91 - t66 * t22, t35, t1; -t61 * t21 - t22 * t84, t21 * t84 - t61 * t22, -t81, t2; t65 * t22, -t65 * t21, -t60, t3; t66 * t19 - t20 * t91, t19 * t91 + t66 * t20, t35, t1; t61 * t19 + t20 * t84, -t19 * t84 + t61 * t20, -t81, t2; -t65 * t20, t65 * t19, -t60, t3;];
Tc_stack = t4;
%% Postprocessing: Reshape Output
% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)
% Fallunterscheidung der Initialisierung für symbolische Eingabe
if isa([qJ; pkin], 'double'), Tc_mdh = NaN(4,4,13+1);               % numerisch
else,                         Tc_mdh = sym('xx', [4,4,13+1]); end % symbolisch
for i = 1:13+1
  Tc_mdh(:,:,i) = [Tc_stack((i-1)*3+1 : 3*i, :);[0 0 0 1]];
end
