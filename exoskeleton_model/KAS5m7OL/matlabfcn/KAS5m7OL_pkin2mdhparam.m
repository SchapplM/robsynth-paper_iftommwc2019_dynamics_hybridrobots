% Convert vector of kinematic parameters to modified DH parameters of
% KAS5m7OL
%
% Input:
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
%
% Output: MDH parameter in order of transformation
% beta_mdh [21x1]
%   Rotation around z
% b_mdh [21x1]
%   Translation along z
% alpha_mdh [21x1]
%   Rotation around x
% a_mdh [21x1]
%   Translation along x
% theta_mdh [21x1]
%   Rotation around z
% d_mdh [21x1]
%   Translation along z
% qoffset_mdh [21x1]
%   Offset on joint coordinate q

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh] = KAS5m7OL_pkin2mdhparam(pkin)

%% Init
%#codegen
%$cgargs {zeros(19,1)}
assert(isreal(pkin) && all(size(pkin) == [19 1]), ...
  'KAS5m7OL_pkin2mdhparam: Kinematic parameters pkin have to be [19x1] (double)');

%% Zuweisung der Parameter


% Aus parameters_mdh_beta_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; pkin(4); 0; -pi / 0.2e1 - pkin(2); 0; 0; 0;];
beta_mdh = t1;

% Aus parameters_mdh_b_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
b_mdh = t1;

% Aus parameters_mdh_alpha_matlab.m
t2 = -pi / 0.2e1;
t1 = pi / 0.2e1;
t3 = [0; t2; t1; 0; 0; 0; t1; t1; 0; 0; 0; 0; 0; 0; t1; 0; 0; 0; 0; 0; t2;];
alpha_mdh = t3;

% Aus parameters_mdh_a_matlab.m
t1 = [0; 0; 0; pkin(18); pkin(6); pkin(7); 0; 0; -pkin(17); pkin(19); pkin(14); pkin(6); 0; pkin(9); 0; pkin(13); pkin(9); pkin(15); pkin(12); pkin(8); 0;];
a_mdh = t1;

% Aus parameters_mdh_theta_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; (2 * pkin(3)) + pi; 0; 0; 0; 0; (2 * pkin(1)) - pi; 0; 0; 0; 0; 0; 0; 0; 0;];
theta_mdh = t1;

% Aus parameters_mdh_d_matlab.m
t1 = [pkin(5); pkin(11); pkin(16); 0; 0; 0; pkin(10); -pkin(16); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
d_mdh = t1;

% Aus parameters_mdh_qoffset_matlab.m
t4 = -pi / 0.2e1;
t3 = pi / 0.2e1;
t1 = [t4; t3; 0; 0; 0; t3; 0; t3; 0; 0; pi; t4; t4; 0.3e1 / 0.2e1 * pi; 0; 0; t3; 0; 0; 0; 0;];
qoffset_mdh = t1;
