% Implicit kinematic constraints of
% KAS5m7OL
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [20x1]
%   kinematic parameters (e.g. lengths of the links)
% 
% Output:
% h [(13x1]
%   Implicit constraint equations (e.g. closed loops)
%

% Quelle: IRT-Maple-Repo
% Datum: 2018-03-02 01:48
% Revision: 9baab17e09a318b4af8cfa3a1be0887325bda075
% (C) Institut für Regelungstechnik, Universität Hannover

function h = KAS5m7OL_kinconstr_impl_mdh_sym_varpar(qJ, pkin)
%% Coder Information
%#codegen
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_kinconstr_impl_mdh_sym_varpar: qJ has to be [13x1] double');
assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7OL_kinconstr_impl_mdh_sym_varpar: pkin has to be [20x1] double');
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
% From kinconstr_impl_matlab.m
% OptimizationMode: 2
% StartTime: 2018-03-01 18:36:48
% EndTime: 2018-03-01 18:36:48
% DurationCPUTime: 0.06s
% Computational Cost: add. (46->31), mult. (24->24), div. (0->0), fcn. (24->24), ass. (0->8)
t5 = qJ11s + delta17;
t7 = qJ9s + qJ10s;
t6 = delta8 + qJ8s;
t4 = qJ3s + qJ9s + delta9;
t3 = -qJ5s - qJ6s + delta20;
t2 = qJ4s + t5;
t1 = qJ12s + t5;
t8 = [-cos(t2) * l14 + cos(qJ4s) * l11 + l5 + cos(t7) * l13 - cos(qJ9s) * l22 - l6; -sin(t2) * l14 + sin(qJ4s) * l11 + sin(t7) * l13 - sin(qJ9s) * l22; l21 * cos(t4) + cos(qJ3s) * l6 + sin(t6) * l20 - sin(delta8) * l4; l21 * sin(t4) + sin(qJ3s) * l6 - cos(t6) * l20 + cos(delta8) * l4; l23 * cos(t3) + cos(qJ5s) * l12 - cos(t1) * qJ13s + cos(t5) * l14; -l23 * sin(t3) + sin(qJ5s) * l12 - sin(t1) * qJ13s + sin(t5) * l14; -qJ4s + qJ11s; -qJ6s + qJ5s - qJ4s + delta18;];
h  = t8 (:);
