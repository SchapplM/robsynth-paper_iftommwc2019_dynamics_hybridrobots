% Jacobian of implicit kinematic constraints of
% KAS5m7IC
% with respect to active joint coordinates
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
% Phi_a [(no of constraints)x(no of active joints)] 

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:50
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi_a = KAS5m7IC_kinconstr_impl_act_jacobian_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(20,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7IC_kinconstr_impl_act_jacobian_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7IC_kinconstr_impl_act_jacobian_mdh_sym_varpar: pkin has to be [20x1] (double)');

%% Symbolic Calculation
% From kinconstr_impl_active_jacobian_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 18:48:50
% EndTime: 2020-06-30 18:48:50
% DurationCPUTime: 0.03s
% Computational Cost: add. (12->8), mult. (8->8), div. (0->0), fcn. (8->8), ass. (0->50)
unknown=NaN(8,5);
t1 = qJ(4) + qJ(11) + pkin(1);
t2 = sin(t1);
t4 = sin(qJ(4));
t7 = cos(t1);
t9 = cos(qJ(4));
t12 = -qJ(5) - qJ(6) + pkin(3);
t13 = sin(t12);
t15 = sin(qJ(5));
t18 = cos(t12);
t20 = cos(qJ(5));
unknown(1,1) = 0;
unknown(1,2) = 0;
unknown(1,3) = (-pkin(7) * t4 + pkin(10) * t2);
unknown(1,4) = 0;
unknown(1,5) = 0;
unknown(2,1) = 0;
unknown(2,2) = 0;
unknown(2,3) = (pkin(7) * t9 - pkin(10) * t7);
unknown(2,4) = 0;
unknown(2,5) = 0;
unknown(3,1) = 0;
unknown(3,2) = 0;
unknown(3,3) = 0;
unknown(3,4) = 0;
unknown(3,5) = 0;
unknown(4,1) = 0;
unknown(4,2) = 0;
unknown(4,3) = 0;
unknown(4,4) = 0;
unknown(4,5) = 0;
unknown(5,1) = 0;
unknown(5,2) = 0;
unknown(5,3) = 0;
unknown(5,4) = (-pkin(8) * t15 + t13 * pkin(16));
unknown(5,5) = 0;
unknown(6,1) = 0;
unknown(6,2) = 0;
unknown(6,3) = 0;
unknown(6,4) = (pkin(8) * t20 + t18 * pkin(16));
unknown(6,5) = 0;
unknown(7,1) = 0;
unknown(7,2) = 0;
unknown(7,3) = -1;
unknown(7,4) = 0;
unknown(7,5) = 0;
unknown(8,1) = 0;
unknown(8,2) = 0;
unknown(8,3) = -1;
unknown(8,4) = 1;
unknown(8,5) = 0;
Phi_a = unknown;
