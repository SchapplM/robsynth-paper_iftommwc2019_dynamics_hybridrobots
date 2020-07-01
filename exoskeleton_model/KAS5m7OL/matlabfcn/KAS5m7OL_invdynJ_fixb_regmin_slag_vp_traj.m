% Calculate time series of minimal parameter regressor of inv. dyn. joint torques for
% KAS5m7OL
%
% Input:
% Q [NTx13]
%   Trajektorie von Gelenkpositionen (NT Zeitschritte in den Zeilen)
% QD [NTx13]
%   Trajektorie von Gelenkgeschwindigkeiten
% QDD [NTx13]
%   Trajektorie von Gelenkbeschleunigungen
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
%
% Output:
% RV_Traj [NTx405]
%   time series of regressor matrices as vectors
%   see KAS5m7OL_invdynJ_fixb_regmin2vec.m

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function RV_Traj = KAS5m7OL_invdynJ_fixb_regmin_slag_vp_traj(Q, QD, QDD, g, pkin)
%% Coder Information
%#codegen
%$cgargs {coder.newtype('double',[inf,13]),
%$cgargs  coder.newtype('double',[inf,13]),
%$cgargs  coder.newtype('double',[inf,13]),
%$cgargs  zeros(3,1), zeros(19,1)}
assert(isreal(Q) && all(size(Q,2) == 13), ...
  'KAS5m7OL_invdynJ_fixb_regmin_slag_vp_traj: Q needs to be [NTx13] (double)');
assert(isreal(QD) && all(size(QD,2) == 13), ...
  'KAS5m7OL_invdynJ_fixb_regmin_slag_vp_traj: QD needs to be [NTx13] (double)');
assert(isreal(QDD) && all(size(QDD,2) == 13), ...
  'KAS5m7OL_invdynJ_fixb_regmin_slag_vp_traj: QDD needs to be [NTx13] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_invdynJ_fixb_regmin_slag_vp_traj: Gravity vector g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_invdynJ_fixb_regmin_slag_vp_traj: Kinematic parameters pkin have to be [19x1] (double)');
  
%% Trajektorie der Regressor-Vektoren aufbauen
RV_Traj = NaN(size(Q,1), 405);
for ii = 1:size(Q,1)
  RV_Traj(ii,:) = KAS5m7OL_invdynJ_fixb_regmin2vec( ...
    KAS5m7OL_invdynJ_fixb_regmin_slag_vp(Q(ii,:)', QD(ii,:)', QDD(ii,:)', g, pkin) );
end
