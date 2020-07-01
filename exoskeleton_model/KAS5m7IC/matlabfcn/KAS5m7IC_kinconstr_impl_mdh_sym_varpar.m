% Implicit kinematic constraints of
% KAS5m7IC
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [20x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% h [(no of constraints)x1]
%   Implicit constraint equations (e.g. closed loops)
%   In a valid robot configuration qJ this has to be zero

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:50
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function h = KAS5m7IC_kinconstr_impl_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(20,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7IC_kinconstr_impl_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7IC_kinconstr_impl_mdh_sym_varpar: pkin has to be [20x1] (double)');

%% Symbolic Calculation
% From kinconstr_impl_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 18:48:50
% EndTime: 2020-06-30 18:48:50
% DurationCPUTime: 0.07s
% Computational Cost: add. (46->35), mult. (24->24), div. (0->0), fcn. (24->24), ass. (0->39)
unknown=NaN(8,1);
t1 = qJ(4) + qJ(11) + pkin(1);
t2 = cos(t1);
t4 = cos(qJ(4));
t6 = qJ(9) + qJ(10);
t7 = cos(t6);
t9 = cos(qJ(9));
t12 = sin(t1);
t14 = sin(qJ(4));
t16 = sin(t6);
t18 = sin(qJ(9));
t21 = qJ(3) + qJ(9) + pkin(5);
t22 = cos(t21);
t24 = cos(qJ(3));
t26 = pkin(4) + qJ(8);
t27 = sin(t26);
t29 = sin(pkin(4));
t32 = sin(t21);
t34 = sin(qJ(3));
t36 = cos(t26);
t38 = cos(pkin(4));
t41 = -qJ(5) - qJ(6) + pkin(3);
t42 = cos(t41);
t44 = cos(qJ(5));
t46 = qJ(11) + pkin(1) + qJ(12);
t47 = cos(t46);
t49 = qJ(11) + pkin(1);
t50 = cos(t49);
t53 = sin(t41);
t55 = sin(qJ(5));
t57 = sin(t46);
t59 = sin(t49);
unknown(1,1) = pkin(7) * t4 + pkin(9) * t7 - pkin(10) * t2 - pkin(15) * t9 + pkin(19) - pkin(20);
unknown(2,1) = pkin(7) * t14 + pkin(9) * t16 - pkin(10) * t12 - pkin(15) * t18;
unknown(3,1) = pkin(13) * t27 + pkin(14) * t22 - pkin(18) * t29 + pkin(20) * t24;
unknown(4,1) = -pkin(13) * t36 + pkin(14) * t32 + pkin(18) * t38 + pkin(20) * t34;
unknown(5,1) = pkin(8) * t44 + pkin(10) * t50 + pkin(16) * t42 - qJ(13) * t47;
unknown(6,1) = pkin(8) * t55 + pkin(10) * t59 - pkin(16) * t53 - qJ(13) * t57;
unknown(7,1) = -qJ(4) + qJ(11);
unknown(8,1) = -qJ(6) + qJ(5) - qJ(4) + pkin(2);
h = unknown;
