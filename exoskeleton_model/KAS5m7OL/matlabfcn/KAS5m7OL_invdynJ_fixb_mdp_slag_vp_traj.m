% Inverse Dynamik für komplette Trajektorie für
% KAS5m7OL
%
% Eingabe:
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
% MDP [88x1]
%   Minimal dynamic parameter vector (fixed base model)
%   see KAS5m7OL_convert_par2_MPV_fixb.m
%
% Ausgabe:
% TAU [NTx13]
%   Time series of inverse Dynamics joint torque

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function TAU = KAS5m7OL_invdynJ_fixb_mdp_slag_vp_traj(Q, QD, QDD, g, pkin, MDP)

%% Coder Information
%#codegen
%$cgargs {coder.newtype('double',[inf,13]),
%$cgargs  coder.newtype('double',[inf,13]),
%$cgargs  coder.newtype('double',[inf,13]),
%$cgargs  zeros(3,1), zeros(19,1), zeros(88,1)}
assert(isreal(Q) && all(size(Q,2) == 13), ...
  'KAS5m7OL_invdynJ_fixb_mdp_slag_vp_traj: Q needs to be [NTx13] (double)');
assert(isreal(QD) && all(size(QD,2) == 13), ...
  'KAS5m7OL_invdynJ_fixb_mdp_slag_vp_traj: QD needs to be [NTx13] (double)');
assert(isreal(QDD) && all(size(QDD,2) == 13), ...
  'KAS5m7OL_invdynJ_fixb_mdp_slag_vp_traj: QDD needs to be [NTx13] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'KAS5m7OL_invdynJ_fixb_mdp_slag_vp_traj: Gravity vector g has to be [3x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_invdynJ_fixb_mdp_slag_vp_traj: Kinematic parameters pkin have to be [19x1] (double)');
assert(isreal(MDP) && all(size(MDP) == [88 1]), ...
  'KAS5m7OL_invdynJ_fixb_mdp_slag_vp_traj: Dynamics parameter vector MDP has to be [88x1] (double)');

%% Inverse Dynamik für jeden Zeitschritt der Trajektorie berechnen
TAU = NaN(size(Q));
for k = 1:size(Q,1)
  TAU(k,:) = KAS5m7OL_invdynJ_fixb_mdp_slag_vp(Q(k,:)', QD(k,:)', QDD(k,:)', g, pkin, MDP);
end
