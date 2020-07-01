% Explicit kinematic constraints of
% palh1m1TE
% Numeric calculation from joint transformation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
% 
% Output:
% jv [21x1]
%   Joint variables (rotation around z or translation in z-direction according to MDH)
%
% Sources:
% Berechnungen Schappler; 30.11.2018

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function jv = KAS5m7TE_kinconstr_expl_mdh_num_varpar(qJ, pkin)
%% Coder Information
%#codegen
%$cgargs {zeros(5,1),zeros(24,1)}
assert(isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m7TE_kinconstr_expl_mdh_num_varpar: qJ has to be [5x1] (double)');
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7TE_kinconstr_expl_mdh_num_varpar: pkin has to be [24x1] (double)');

%% Berechnung
% Kinematikparameter und Struktureigenschaften
[beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh] = KAS5m7TE_pkin2mdhparam(pkin);
[~, sigma_mdh] = KAS5m7TE_structural_kinematic_parameters();

% Kinematik: Einzel-Gelenktransformationen
T = KAS5m7TE_joint_trafo_rotmat_mdh_sym_varpar(qJ, pkin);

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
