% Explicit kinematic constraints of
% KAS5m7DE1
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% jv [21x1]
%   Joint variables (rotation around z or translation in z-direction according to MDH)
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function jv = KAS5m7DE1_kinconstr_expl_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7DE1_kinconstr_expl_mdh_sym_varpar: qJ has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_kinconstr_expl_mdh_sym_varpar: pkin has to be [24x1] (double)');

%% Symbolic Calculation
% From kinconstr_expl_matlab.m
% OptimizationMode: 2
% StartTime: 2020-05-14 19:20:07
% EndTime: 2020-05-14 19:20:10
% DurationCPUTime: 3.26s
% Computational Cost: add. (68099->66), mult. (87840->90), div. (1422->4), fcn. (35226->24), ass. (0->51)
t49 = cos(qJ(3));
t51 = -pkin(23) + pkin(24);
t33 = t49 * t51 - pkin(9);
t43 = pkin(3) + qJ(3);
t40 = cos(t43);
t31 = t40 * pkin(12) + t33;
t39 = sin(t43);
t48 = sin(qJ(3));
t32 = t39 * pkin(12) - t48 * t51;
t27 = 0.4e1 * t31 ^ 2 + 0.4e1 * t32 ^ 2;
t64 = 0.2e1 / t27;
t56 = pkin(11) ^ 2;
t25 = t48 ^ 2 * t51 ^ 2 - pkin(19) ^ 2 + t33 ^ 2 + t56 + (-t39 ^ 2 - t40 ^ 2) * pkin(12) ^ 2 + 0.2e1 * (t31 * t40 + t32 * t39) * pkin(12);
t24 = sqrt(-t25 ^ 2 + t56 * t27);
t23 = (-t24 * t32 + t25 * t31) * t64;
t20 = t23 - t31;
t22 = (t24 * t31 + t25 * t32) * t64;
t21 = t22 - t32;
t18 = t48 * t20 + t49 * t21;
t19 = t49 * t20 - t48 * t21;
t53 = 0.1e1 / pkin(19);
t62 = pkin(18) * t53;
t59 = cos(pkin(7)) * t62;
t60 = sin(pkin(7)) * t62;
t13 = -t18 * t60 + t19 * t59;
t58 = pkin(24) + t13;
t10 = 0.2e1 * t58;
t11 = t18 * t59 + t19 * t60;
t12 = 0.2e1 * t11;
t54 = pkin(17) ^ 2;
t7 = -pkin(22) ^ 2 + pkin(24) ^ 2 + t54 + (t10 - t13) * t13 + (t12 - t11) * t11;
t9 = t10 ^ 2 + t12 ^ 2;
t6 = sqrt(t54 * t9 - t7 ^ 2);
t8 = 0.1e1 / t9;
t4 = (-t10 * t7 + t12 * t6) * t8 + t58;
t5 = -(-t10 * t6 - t7 * t12) * t8 - t11;
t63 = atan2(t5, t4) - pi / 0.2e1;
t61 = pkin(19) * t53;
t42 = -qJ(4) + t43;
t57 = 0.1e1 / pkin(22);
t47 = cos(pkin(6));
t45 = sin(pkin(6));
t41 = qJ(4) - qJ(3) + pkin(4);
t36 = cos(t41);
t35 = sin(t41);
t29 = -t36 * pkin(14) + t35 * pkin(15) + sin(t42) * pkin(12);
t28 = -t35 * pkin(14) - pkin(10) - t36 * pkin(15) - cos(t42) * pkin(12);
t14 = atan2(-t18, t19);
t2 = (-t4 * t47 + t45 * t5) * t57;
t1 = (t4 * t45 + t47 * t5) * t57;
t3 = [qJ(1); qJ(2); pkin(6) + t63; qJ(3); qJ(4); t41; qJ(5); -pkin(6) - pi + (2 * pkin(21)); -atan2(-(t1 * t45 - t2 * t47) * pkin(22) + t58, (t1 * t47 + t2 * t45) * pkin(22) + t11) + t63; -t14; -atan2((-t18 * t49 + t19 * t48) * t61 - t32, (t18 * t48 + t19 * t49) * t61 + t31) + qJ(3) + t14; qJ(3); -pkin(3) + pi; -atan2(t29, t28) + pi - t42; sqrt(t28 ^ 2 + t29 ^ 2); 0; atan2(t22, t23) - t43; 0; 0; 0; 0;];
jv = t3(:);
