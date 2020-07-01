% Explicit kinematic constraints of
% KAS5m5
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% pkin [30x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8,delta8s,delta9s,l11,l12,l13,l14,l17,l18,l20,l21,l22,l4,l5,l6,delta10s,delta12s,delta17s,delta18s]';
% 
% Output:
% jv [13x1]
%   Joint variables (rotation around z or translation in z-direction according to MDH)
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:16
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function jv = KAS5m5_kinconstr_expl_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m5_kinconstr_expl_mdh_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m5_kinconstr_expl_mdh_sym_varpar: pkin has to be [30x1] (double)');

%% Symbolic Calculation
% From kinconstr_expl_matlab.m
% OptimizationMode: 2
% StartTime: 2020-06-27 17:51:28
% EndTime: 2020-06-27 17:51:32
% DurationCPUTime: 3.24s
% Computational Cost: add. (68078->62), mult. (87830->86), div. (1422->4), fcn. (35218->22), ass. (0->49)
t46 = cos(qJ(3));
t48 = -pkin(25) + pkin(26);
t30 = t46 * t48 - pkin(15);
t40 = pkin(29) + qJ(3);
t37 = cos(t40);
t28 = t37 * pkin(18) + t30;
t36 = sin(t40);
t45 = sin(qJ(3));
t29 = t36 * pkin(18) - t45 * t48;
t27 = 0.4e1 * t28 ^ 2 + 0.4e1 * t29 ^ 2;
t61 = 0.2e1 / t27;
t53 = pkin(17) ^ 2;
t25 = t45 ^ 2 * t48 ^ 2 - pkin(23) ^ 2 + t30 ^ 2 + t53 + (-t36 ^ 2 - t37 ^ 2) * pkin(18) ^ 2 + 0.2e1 * (t28 * t37 + t29 * t36) * pkin(18);
t24 = sqrt(-t25 ^ 2 + t53 * t27);
t23 = (-t24 * t29 + t25 * t28) * t61;
t20 = t23 - t28;
t22 = (t24 * t28 + t25 * t29) * t61;
t21 = t22 - t29;
t18 = t45 * t20 + t46 * t21;
t19 = t46 * t20 - t45 * t21;
t50 = 0.1e1 / pkin(23);
t59 = pkin(22) * t50;
t56 = cos(pkin(14)) * t59;
t57 = sin(pkin(14)) * t59;
t13 = -t18 * t57 + t19 * t56;
t55 = pkin(26) + t13;
t10 = 0.2e1 * t55;
t11 = t18 * t56 + t19 * t57;
t12 = 0.2e1 * t11;
t51 = pkin(21) ^ 2;
t7 = -pkin(24) ^ 2 + pkin(26) ^ 2 + t51 + (t10 - t13) * t13 + (t12 - t11) * t11;
t9 = t10 ^ 2 + t12 ^ 2;
t6 = sqrt(t51 * t9 - t7 ^ 2);
t8 = 0.1e1 / t9;
t4 = (-t10 * t7 + t12 * t6) * t8 + t55;
t5 = -(-t10 * t6 - t7 * t12) * t8 - t11;
t60 = atan2(t5, t4) - pi / 0.2e1;
t58 = pkin(23) * t50;
t38 = -qJ(4) + t40;
t54 = 0.1e1 / pkin(24);
t44 = cos(pkin(13));
t42 = sin(pkin(13));
t39 = qJ(4) - qJ(3) + pkin(30);
t33 = cos(t39);
t32 = sin(t39);
t14 = atan2(-t18, t19);
t2 = (-t4 * t44 + t42 * t5) * t54;
t1 = (t4 * t42 + t44 * t5) * t54;
t3 = [qJ(1); qJ(2); pkin(13) + t60; qJ(3); qJ(4); t39; qJ(5); pkin(13); -atan2(-(t1 * t42 - t2 * t44) * pkin(24) + t55, (t1 * t44 + t2 * t42) * pkin(24) + t11) + t60; -t14; -atan2((-t18 * t46 + t19 * t45) * t58 - t29, (t18 * t45 + t19 * t46) * t58 + t28) + qJ(3) + t14; -atan2(t22, t23) + t40; -atan2(t32 * pkin(20) + sin(t38) * pkin(18) - t33 * pkin(19), -t32 * pkin(19) - pkin(16) - t33 * pkin(20) - cos(t38) * pkin(18)) + pi - t38;];
jv = t3(:);
