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

function PhiD_a = KAS5m7OL_kinconstr_impl_pas_jacobianD_mdh_sym_varpar(qJ, qJD, pkin)
%% Coder Information
%#codegen
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_kinconstr_impl_pas_jacobianD_mdh_sym_varpar: qJ has to be [13x1] double');
assert(isa(qJD,'double') && isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m7OL_kinconstr_impl_pas_jacobianD_mdh_sym_varpar: qJD has to be [13x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7OL_kinconstr_impl_pas_jacobianD_mdh_sym_varpar: pkin has to be [20x1] double');
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
% From kinconstr_impl_passive_jacobianD_matlab.m
% OptimizationMode: 2
% StartTime: 2018-03-01 18:36:49
% EndTime: 2018-03-01 18:36:49
% DurationCPUTime: 0.05s
% Computational Cost: add. (79->27), mult. (54->31), div. (0->0), fcn. (30->18), ass. (0->26)
t70 = l13 * (qJD9s + qJD10s);
t69 = l14 * (qJD4s + qJD11s);
t68 = l21 * (qJD3s + qJD9s);
t67 = l23 * (-qJD5s - qJD6s);
t49 = qJ11s + delta17;
t42 = qJ12s + t49;
t38 = sin(t42);
t39 = cos(t42);
t48 = qJD11s + qJD12s;
t62 = qJ13s * t48;
t66 = t38 * qJD13s + t39 * t62;
t65 = l6 * qJD3s;
t64 = l20 * qJD8s;
t63 = l22 * qJD9s;
t61 = l14 * qJD11s;
t47 = qJ3s + qJ9s + delta9;
t60 = sin(t47) * t68;
t59 = cos(t47) * t68;
t55 = qJ9s + qJ10s;
t58 = sin(t55) * t70;
t57 = cos(t55) * t70;
t56 = -t39 * qJD13s + t38 * t62;
t54 = delta8 + qJ8s;
t44 = -qJ5s - qJ6s + delta20;
t43 = qJ4s + t49;
t1 = [0 0 0 -t57 + cos(qJ9s) * t63 -t57 cos(t43) * t69 0 0; 0 0 0 -t58 + sin(qJ9s) * t63 -t58 sin(t43) * t69 0 0; -t59 - cos(qJ3s) * t65 0 -sin(t54) * t64 -t59 0 0 0 0; -t60 - sin(qJ3s) * t65 0 cos(t54) * t64 -t60 0 0 0 0; 0 cos(t44) * t67 0 0 0 -cos(t49) * t61 + t66 t66 t48 * t38; 0 -sin(t44) * t67 0 0 0 -sin(t49) * t61 + t56 t56 -t48 * t39; 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0;];
PhiD_a  = t1 ;
