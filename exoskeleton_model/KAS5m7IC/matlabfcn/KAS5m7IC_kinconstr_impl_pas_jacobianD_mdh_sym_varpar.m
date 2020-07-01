% Jacobian time derivative of explicit kinematic constraints of
% KAS5m7IC
% with respect to passive joint coordinates
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% qJD [13x1]
%   Generalized joint velocities
% pkin [20x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% PhiD_p [(no of constraints)x(no. of passive joints)] 

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:50
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function PhiD_p = KAS5m7IC_kinconstr_impl_pas_jacobianD_mdh_sym_varpar(qJ, qJD, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(13,1),zeros(20,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7IC_kinconstr_impl_pas_jacobianD_mdh_sym_varpar: qJ has to be [13x1] (double)');
assert(isreal(qJD) && all(size(qJD) == [13 1]), ...
  'KAS5m7IC_kinconstr_impl_pas_jacobianD_mdh_sym_varpar: qJD has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [20 1]), ...
  'KAS5m7IC_kinconstr_impl_pas_jacobianD_mdh_sym_varpar: pkin has to be [20x1] (double)');

%% Symbolic Calculation
% From kinconstr_impl_passive_jacobianD_matlab.m
% OptimizationMode: 1
% StartTime: 2020-06-30 18:48:50
% EndTime: 2020-06-30 18:48:50
% DurationCPUTime: 0.09s
% Computational Cost: add. (78->33), mult. (54->36), div. (0->0), fcn. (30->18), ass. (0->105)
unknown=NaN(8,8);
t1 = qJD(9) + qJD(10);
t2 = qJ(9) + qJ(10);
t3 = cos(t2);
t5 = pkin(9) * t3 * t1;
t6 = cos(qJ(9));
t10 = qJD(4) + qJD(11);
t11 = qJ(4) + qJ(11) + pkin(1);
t12 = cos(t11);
t15 = sin(t2);
t17 = pkin(9) * t15 * t1;
t18 = sin(qJ(9));
t22 = sin(t11);
t26 = (qJD(3) + qJD(9)) * pkin(14);
t27 = qJ(3) + qJ(9) + pkin(5);
t28 = cos(t27);
t29 = t28 * t26;
t30 = cos(qJ(3));
t34 = qJD(8) * pkin(13);
t35 = pkin(4) + qJ(8);
t36 = sin(t35);
t38 = sin(t27);
t39 = t38 * t26;
t40 = sin(qJ(3));
t44 = cos(t35);
t46 = -qJD(5) - qJD(6);
t48 = -qJ(5) - qJ(6) + pkin(3);
t49 = cos(t48);
t51 = qJD(11) + qJD(12);
t52 = qJ(11) + pkin(1) + qJ(12);
t53 = cos(t52);
t54 = t53 * t51;
t55 = qJ(13) * t54;
t56 = sin(t52);
t57 = qJD(13) * t56;
t58 = qJ(11) + pkin(1);
t59 = cos(t58);
t64 = t56 * t51;
t66 = sin(t48);
t68 = qJ(13) * t64;
t69 = qJD(13) * t53;
t70 = sin(t58);
unknown(1,1) = 0;
unknown(1,2) = 0;
unknown(1,3) = 0;
unknown(1,4) = (pkin(15) * t6 * qJD(9) - t5);
unknown(1,5) = -t5;
unknown(1,6) = (pkin(10) * t12 * t10);
unknown(1,7) = 0;
unknown(1,8) = 0;
unknown(2,1) = 0;
unknown(2,2) = 0;
unknown(2,3) = 0;
unknown(2,4) = (pkin(15) * t18 * qJD(9) - t17);
unknown(2,5) = -t17;
unknown(2,6) = (pkin(10) * t22 * t10);
unknown(2,7) = 0;
unknown(2,8) = 0;
unknown(3,1) = (-pkin(20) * t30 * qJD(3) - t29);
unknown(3,2) = 0;
unknown(3,3) = -(t36 * t34);
unknown(3,4) = -t29;
unknown(3,5) = 0;
unknown(3,6) = 0;
unknown(3,7) = 0;
unknown(3,8) = 0;
unknown(4,1) = (-pkin(20) * t40 * qJD(3) - t39);
unknown(4,2) = 0;
unknown(4,3) = (t44 * t34);
unknown(4,4) = -t39;
unknown(4,5) = 0;
unknown(4,6) = 0;
unknown(4,7) = 0;
unknown(4,8) = 0;
unknown(5,1) = 0;
unknown(5,2) = (t49 * t46 * pkin(16));
unknown(5,3) = 0;
unknown(5,4) = 0;
unknown(5,5) = 0;
unknown(5,6) = (-pkin(10) * t59 * qJD(11) + t55 + t57);
unknown(5,7) = (t55 + t57);
unknown(5,8) = t64;
unknown(6,1) = 0;
unknown(6,2) = -(t66 * t46 * pkin(16));
unknown(6,3) = 0;
unknown(6,4) = 0;
unknown(6,5) = 0;
unknown(6,6) = (-pkin(10) * t70 * qJD(11) + t68 - t69);
unknown(6,7) = (t68 - t69);
unknown(6,8) = -t54;
unknown(7,1) = 0;
unknown(7,2) = 0;
unknown(7,3) = 0;
unknown(7,4) = 0;
unknown(7,5) = 0;
unknown(7,6) = 0;
unknown(7,7) = 0;
unknown(7,8) = 0;
unknown(8,1) = 0;
unknown(8,2) = 0;
unknown(8,3) = 0;
unknown(8,4) = 0;
unknown(8,5) = 0;
unknown(8,6) = 0;
unknown(8,7) = 0;
unknown(8,8) = 0;
PhiD_p = unknown;
