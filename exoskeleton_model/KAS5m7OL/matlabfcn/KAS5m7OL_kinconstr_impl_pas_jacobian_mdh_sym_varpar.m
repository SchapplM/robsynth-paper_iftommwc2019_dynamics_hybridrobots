% Jacobian of implicit kinematic constraints of
% KAS5m7OL
% with respect to passive joint coordinates
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [20x1]
%   kinematic parameters (e.g. lengths of the links)
% 
% Output:
% Phi_p [(21-13)x(no of passive joints)] 

% Quelle: IRT-Maple-Repo
% Datum: 2018-03-02 01:48
% Revision: 9baab17e09a318b4af8cfa3a1be0887325bda075
% (C) Institut für Regelungstechnik, Universität Hannover

function Phi_p = KAS5m7OL_kinconstr_impl_pas_jacobian_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_kinconstr_impl_pas_jacobian_mdh_sym_varpar: qJ has to be [13x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7OL_kinconstr_impl_pas_jacobian_mdh_sym_varpar: pkin has to be [20x1] double');
%% Variable Initialization

qJ1s = qJ(1);
qJ2s = qJ(2);
qJ3s = qJ(3);
qJ4s = qJ(4);
qJ5s = qJ(5);
qJ6s = qJ(6);
qJ7s = qJ(7);
qJ8s = qJ(8);
qJ9s = qJ(9);
qJ10s = qJ(10);
qJ11s = qJ(11);
qJ12s = qJ(12);
qJ13s = qJ(13);

delta17 = pkin(1);
delta18 = pkin(2);
delta20 = pkin(3);
delta8 = pkin(4);
delta9 = pkin(5);
l1 = pkin(6);
l11 = pkin(7);
l12 = pkin(8);
l13 = pkin(9);
l14 = pkin(10);
l15 = pkin(11);
l2 = pkin(12);
l20 = pkin(13);
l21 = pkin(14);
l22 = pkin(15);
l23 = pkin(16);
l3 = pkin(17);
l4 = pkin(18);
l5 = pkin(19);
l6 = pkin(20);

%% Symbolic Calculation
% From kinconstr_impl_passive_jacobian_matlab.m
% OptimizationMode: 2
% StartTime: 2018-03-01 18:36:49
% EndTime: 2018-03-01 18:36:49
% DurationCPUTime: 0.06s
% Computational Cost: add. (48->19), mult. (24->18), div. (0->0), fcn. (26->18), ass. (0->16)
t28 = qJ3s + qJ9s + delta9;
t34 = l21 * sin(t28);
t31 = qJ9s + qJ10s;
t33 = sin(t31) * l13;
t29 = qJ11s + delta17;
t24 = qJ12s + t29;
t21 = cos(t24);
t32 = t21 * qJ13s;
t30 = delta8 + qJ8s;
t26 = -qJ5s - qJ6s + delta20;
t25 = qJ4s + t29;
t22 = cos(t31) * l13;
t20 = sin(t24);
t19 = l21 * cos(t28);
t18 = t20 * qJ13s;
t1 = [0 0 0 -t33 + sin(qJ9s) * l22 -t33 sin(t25) * l14 0 0; 0 0 0 t22 - cos(qJ9s) * l22 t22 -cos(t25) * l14 0 0; -t34 - sin(qJ3s) * l6 0 cos(t30) * l20 -t34 0 0 0 0; t19 + cos(qJ3s) * l6 0 sin(t30) * l20 t19 0 0 0 0; 0 l23 * sin(t26) 0 0 0 t18 - sin(t29) * l14 t18 -t21; 0 l23 * cos(t26) 0 0 0 -t32 + cos(t29) * l14 -t32 -t20; 0 0 0 0 0 1 0 0; 0 -1 0 0 0 0 0 0;];
Phi_p  = t1 ;
