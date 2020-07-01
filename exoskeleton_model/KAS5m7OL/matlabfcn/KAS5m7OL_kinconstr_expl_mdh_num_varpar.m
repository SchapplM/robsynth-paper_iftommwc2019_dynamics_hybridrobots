% Explicit kinematic constraints of
% palh1m1TE
% Numeric calculation from joint transformation
% 
% Input:
% qJ [13x1]
%   Generalized joint coordinates (joint angles)
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% jv [21x1]
%   Joint variables (rotation around z or translation in z-direction according to MDH)
%
% Sources:
% Berechnungen Schappler; 30.11.2018

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function jv = KAS5m7OL_kinconstr_expl_mdh_num_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(13,1),zeros(19,1)}
assert(isreal(qJ) && all(size(qJ) == [13 1]), ...
  'KAS5m7OL_kinconstr_expl_mdh_num_varpar: qJ has to be [13x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_kinconstr_expl_mdh_num_varpar: pkin has to be [19x1] (double)');

%% Berechnung
% Kinematikparameter und Struktureigenschaften
[beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh] = KAS5m7OL_pkin2mdhparam(pkin);
[~, sigma_mdh] = KAS5m7OL_structural_kinematic_parameters();

% Kinematik: Einzel-Gelenktransformationen
T = KAS5m7OL_joint_trafo_rotmat_mdh_sym_varpar(qJ, pkin);

jv = zeros(21, 1);
for i = 1:21
  % MDH-Parameter
  sigma = sigma_mdh(i);
  beta = beta_mdh(i);
  b = b_mdh(i);
  alpha = alpha_mdh(i);
  a = a_mdh(i);
  theta = theta_mdh(i);
  d = d_mdh(i);
  q_offset = qoffset_mdh(i);
  % Transformationsschritte außer theta und d
  T_1 = trotz(beta) * transl([0;0;b]) * trotx(alpha) * transl([a;0;0]);
  % MDH-Transformation so umformen, dass nur noch die Transformation
  % in die Richtung des Gelenks (theta und d) übrig bleibt
  if sigma == 0 % Drehgelenk
    % Gelenkvariable theta aus Rotationsmatrix berechnen
    R_ztheta = T_1(1:3,1:3)' * T(1:3,1:3,i);
    theta = atan2(R_ztheta(2,1), R_ztheta(1,1));
    jv(i) = theta - q_offset;
  elseif sigma == 1 % Schubgelenk
    % Gelenkvariable d aus Translations-Teil der Transformationsmatrix
    T_Tzd = invtr(T_1 * trotz(theta)) * T(:,:,i);
    d = T_Tzd(3,4);
    jv(i) = d - q_offset;
  end
end
