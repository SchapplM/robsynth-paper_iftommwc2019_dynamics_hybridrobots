% Explicit kinematic constraints of
% KAS5m3
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [6x1]
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
% Datum: 2020-06-27 17:47
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function jv = KAS5m3_kinconstr_expl_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(6,1),zeros(30,1)}
assert(isreal(qJ) && all(size(qJ) == [6 1]), ...
  'KAS5m3_kinconstr_expl_mdh_sym_varpar: qJ has to be [6x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m3_kinconstr_expl_mdh_sym_varpar: pkin has to be [30x1] (double)');

%% Symbolic Calculation
% From kinconstr_expl_matlab.m
% OptimizationMode: 2
% StartTime: 2020-06-27 16:26:51
% EndTime: 2020-06-27 16:26:56
% DurationCPUTime: 3.40s
% Computational Cost: add. (68068->61), mult. (87830->86), div. (1422->4), fcn. (35218->22), ass. (0->48)
t45 = cos(qJ(3));
t47 = -pkin(25) + pkin(26);
t30 = t45 * t47 - pkin(15);
t37 = pkin(29) + qJ(3);
t35 = cos(t37);
t28 = t35 * pkin(18) + t30;
t34 = sin(t37);
t43 = sin(qJ(3));
t29 = t34 * pkin(18) - t43 * t47;
t27 = 0.4e1 * t28 ^ 2 + 0.4e1 * t29 ^ 2;
t60 = 0.2e1 / t27;
t52 = pkin(17) ^ 2;
t25 = t43 ^ 2 * t47 ^ 2 - pkin(23) ^ 2 + t30 ^ 2 + t52 + (-t34 ^ 2 - t35 ^ 2) * pkin(18) ^ 2 + 0.2e1 * (t28 * t35 + t29 * t34) * pkin(18);
t24 = sqrt(-t25 ^ 2 + t52 * t27);
t23 = (-t24 * t29 + t25 * t28) * t60;
t20 = t23 - t28;
t22 = (t24 * t28 + t25 * t29) * t60;
t21 = t22 - t29;
t18 = t43 * t20 + t45 * t21;
t19 = t45 * t20 - t43 * t21;
t49 = 0.1e1 / pkin(23);
t58 = pkin(22) * t49;
t55 = cos(pkin(14)) * t58;
t56 = sin(pkin(14)) * t58;
t13 = -t18 * t56 + t19 * t55;
t54 = pkin(26) + t13;
t10 = 0.2e1 * t54;
t11 = t18 * t55 + t19 * t56;
t12 = 0.2e1 * t11;
t50 = pkin(21) ^ 2;
t7 = -pkin(24) ^ 2 + pkin(26) ^ 2 + t50 + (t10 - t13) * t13 + (t12 - t11) * t11;
t9 = t10 ^ 2 + t12 ^ 2;
t6 = sqrt(t50 * t9 - t7 ^ 2);
t8 = 0.1e1 / t9;
t4 = (-t10 * t7 + t12 * t6) * t8 + t54;
t5 = -(-t10 * t6 - t7 * t12) * t8 - t11;
t59 = atan2(t5, t4) - pi / 0.2e1;
t57 = pkin(23) * t49;
t36 = -qJ(4) + t37;
t53 = 0.1e1 / pkin(24);
t44 = cos(qJ(5));
t42 = sin(qJ(5));
t41 = cos(pkin(13));
t39 = sin(pkin(13));
t14 = atan2(-t18, t19);
t2 = (t39 * t5 - t4 * t41) * t53;
t1 = (t39 * t4 + t41 * t5) * t53;
t3 = [qJ(1); qJ(2); pkin(13) + t59; qJ(3); qJ(4); qJ(5); qJ(6); pkin(13); -atan2(-(t1 * t39 - t2 * t41) * pkin(24) + t54, (t1 * t41 + t2 * t39) * pkin(24) + t11) + t59; -t14; -atan2((-t18 * t45 + t19 * t43) * t57 - t29, (t18 * t43 + t19 * t45) * t57 + t28) + qJ(3) + t14; -atan2(t22, t23) + t37; -atan2(t42 * pkin(20) + sin(t36) * pkin(18) - t44 * pkin(19), -t42 * pkin(19) - pkin(16) - t44 * pkin(20) - cos(t36) * pkin(18)) + pi - t36;];
jv = t3(:);
