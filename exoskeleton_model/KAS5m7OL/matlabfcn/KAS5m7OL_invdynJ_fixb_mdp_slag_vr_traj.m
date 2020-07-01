% Inverse Dynamik für komplette Trajektorie für
% KAS5m7OL
%
% Eingabe:
% RV_Traj [NTx405]
%   time series of regressor matrices as vectors
%   Number of time steps (NT) in rows
%   see KAS5m7OL_invdynJ_fixb_regmin2vec.m
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

function TAU = KAS5m7OL_invdynJ_fixb_mdp_slag_vr_traj(RV_Traj, MDP)

%% Coder Information
%#codegen
%$cgargs {coder.newtype('double',[inf,405]), zeros(88,1)}
assert(isreal(RV_Traj) && all(size(RV_Traj,2) == 405), ...
  'KAS5m7OL_invdynJ_fixb_mdp_slag_vr_traj: RV_Traj needs to be [NTx405] (double)');
assert(isreal(MDP) && all(size(MDP) == [88 1]), ...
  'KAS5m7OL_invdynJ_fixb_mdp_slag_vr_traj: Dynamics parameter vector MDP has to be [88x1] (double)');

%% Inverse Dynamik für jeden Zeitschritt der Trajektorie berechnen
TAU = NaN(size(RV_Traj,1), 13);
for ii = 1:size(RV_Traj,1)
  TAU(ii,:) = KAS5m7OL_invdynJ_fixb_mdp_slag_vr(RV_Traj(ii,:), MDP);
end
