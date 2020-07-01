% Jacobian time derivative of explicit kinematic constraints of
% KAS5m7OL
% with respect to active joint coordinates
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% qJD [13x1]
%   Generalized joint velocities
% pkin [20x1]
%   kinematic parameters (e.g. lengths of the links)
% 
% Output:
% PhiD_a [(21-13)x(no. of active joints)] 

% Quelle: IRT-Maple-Repo
% Datum: 2018-03-02 01:48
% Revision: 9baab17e09a318b4af8cfa3a1be0887325bda075
% (C) Institut für Regelungstechnik, Universität Hannover

function PhiD_a = KAS5m7OL_kinconstr_impl_act_jacobianD_mdh_sym_varpar(qJ, qJD, pkin)
%% Coder Information
%#codegen
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_kinconstr_impl_act_jacobianD_mdh_sym_varpar: qJ has to be [13x1] double');
assert(isa(qJD,'double') && isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m7OL_kinconstr_impl_act_jacobianD_mdh_sym_varpar: qJD has to be [13x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7OL_kinconstr_impl_act_jacobianD_mdh_sym_varpar: pkin has to be [20x1] double');
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

qJD1s = qJD(1);
qJD2s = qJD(2);
qJD3s = qJD(3);
qJD4s = qJD(4);
qJD5s = qJD(5);
qJD6s = qJD(6);
qJD7s = qJD(7);
qJD8s = qJD(8);
qJD9s = qJD(9);
qJD10s = qJD(10);
qJD11s = qJD(11);
qJD12s = qJD(12);
qJD13s = qJD(13);

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
% From kinconstr_impl_active_jacobianD_matlab.m
% OptimizationMode: 2
% StartTime: 2018-03-01 18:36:49
% EndTime: 2018-03-01 18:36:49
% DurationCPUTime: 0.02s
% Computational Cost: add. (16->8), mult. (16->12), div. (0->0), fcn. (8->8), ass. (0->7)
t17 = l14 * (qJD4s + qJD11s);
t16 = l23 * (-qJD5s - qJD6s);
t15 = l11 * qJD4s;
t14 = l12 * qJD5s;
t11 = -qJ5s - qJ6s + delta20;
t10 = qJ4s + qJ11s + delta17;
t1 = [0 0 cos(t10) * t17 - cos(qJ4s) * t15 0 0; 0 0 sin(t10) * t17 - sin(qJ4s) * t15 0 0; 0 0 0 0 0; 0 0 0 0 0; 0 0 0 cos(t11) * t16 - cos(qJ5s) * t14 0; 0 0 0 -sin(t11) * t16 - sin(qJ5s) * t14 0; 0 0 0 0 0; 0 0 0 0 0;];
PhiD_a  = t1 ;
