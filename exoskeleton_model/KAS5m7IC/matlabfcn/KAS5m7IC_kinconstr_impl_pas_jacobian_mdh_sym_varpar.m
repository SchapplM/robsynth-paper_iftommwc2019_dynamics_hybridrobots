% Jacobian of implicit kinematic constraints of
% KAS5m7IC
% with respect to passive joint coordinates
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
% Phi_p [(no of constraints)x(no of passive joints)] 

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:50
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi_p = KAS5m7IC_kinconstr_impl_pas_jacobian_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(20,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7IC_kinconstr_impl_pas_jacobian_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7IC_kinconstr_impl_pas_jacobian_mdh_sym_varpar: pkin has to be [20x1] (double)');

%% Symbolic Calculation
% From kinconstr_impl_passive_jacobian_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 18:48:50
% EndTime: 2020-06-30 18:48:50
% DurationCPUTime: 0.05s
% Computational Cost: add. (48->23), mult. (24->18), div. (0->0), fcn. (26->18), ass. (0->95)
unknown=NaN(8,8);
t1 = qJ(9) + qJ(10);
t2 = sin(t1);
t3 = pkin(9) * t2;
t4 = sin(qJ(9));
t7 = qJ(4) + qJ(11) + pkin(1);
t8 = sin(t7);
t10 = cos(t1);
t11 = pkin(9) * t10;
t12 = cos(qJ(9));
t15 = cos(t7);
t17 = qJ(3) + qJ(9) + pkin(5);
t18 = sin(t17);
t19 = t18 * pkin(14);
t20 = sin(qJ(3));
t23 = pkin(4) + qJ(8);
t24 = cos(t23);
t26 = cos(t17);
t27 = t26 * pkin(14);
t28 = cos(qJ(3));
t31 = sin(t23);
t33 = -qJ(5) - qJ(6) + pkin(3);
t34 = sin(t33);
t36 = qJ(11) + pkin(1) + qJ(12);
t37 = sin(t36);
t38 = qJ(13) * t37;
t39 = qJ(11) + pkin(1);
t40 = sin(t39);
t43 = cos(t36);
t44 = cos(t33);
t46 = qJ(13) * t43;
t47 = cos(t39);
unknown(1,1) = 0;
unknown(1,2) = 0;
unknown(1,3) = 0;
unknown(1,4) = (pkin(15) * t4 - t3);
unknown(1,5) = -t3;
unknown(1,6) = (pkin(10) * t8);
unknown(1,7) = 0;
unknown(1,8) = 0;
unknown(2,1) = 0;
unknown(2,2) = 0;
unknown(2,3) = 0;
unknown(2,4) = (-pkin(15) * t12 + t11);
unknown(2,5) = t11;
unknown(2,6) = -(pkin(10) * t15);
unknown(2,7) = 0;
unknown(2,8) = 0;
unknown(3,1) = (-pkin(20) * t20 - t19);
unknown(3,2) = 0;
unknown(3,3) = (t24 * pkin(13));
unknown(3,4) = -t19;
unknown(3,5) = 0;
unknown(3,6) = 0;
unknown(3,7) = 0;
unknown(3,8) = 0;
unknown(4,1) = (pkin(20) * t28 + t27);
unknown(4,2) = 0;
unknown(4,3) = (t31 * pkin(13));
unknown(4,4) = t27;
unknown(4,5) = 0;
unknown(4,6) = 0;
unknown(4,7) = 0;
unknown(4,8) = 0;
unknown(5,1) = 0;
unknown(5,2) = (t34 * pkin(16));
unknown(5,3) = 0;
unknown(5,4) = 0;
unknown(5,5) = 0;
unknown(5,6) = (-pkin(10) * t40 + t38);
unknown(5,7) = t38;
unknown(5,8) = -t43;
unknown(6,1) = 0;
unknown(6,2) = (t44 * pkin(16));
unknown(6,3) = 0;
unknown(6,4) = 0;
unknown(6,5) = 0;
unknown(6,6) = (pkin(10) * t47 - t46);
unknown(6,7) = -t46;
unknown(6,8) = -t37;
unknown(7,1) = 0;
unknown(7,2) = 0;
unknown(7,3) = 0;
unknown(7,4) = 0;
unknown(7,5) = 0;
unknown(7,6) = 1;
unknown(7,7) = 0;
unknown(7,8) = 0;
unknown(8,1) = 0;
unknown(8,2) = -1;
unknown(8,3) = 0;
unknown(8,4) = 0;
unknown(8,5) = 0;
unknown(8,6) = 0;
unknown(8,7) = 0;
unknown(8,8) = 0;
Phi_p = unknown;
