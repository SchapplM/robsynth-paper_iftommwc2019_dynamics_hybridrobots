% Convert vector of kinematic parameters to modified DH parameters of
% KAS5m7DE1
%
% Input:
% pkin [24x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta10,delta12,delta17,delta18,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l17,l18,l2,l20,l21,l22,l23,l3,l4,l5,l6]';
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
% Datum: 2020-05-25 11:30
% Revision: 91226b68921adecbf67aba0faa97e308f05cdafe (2020-05-14)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh] = KAS5m7DE1_pkin2mdhparam(pkin)

%% Init
%#codegen
%$cgargs {zeros(24,1)}
assert(isreal(pkin) && all(size(pkin) == [24 1]), ...
  'KAS5m7DE1_pkin2mdhparam: Kinematic parameters pkin have to be [24x1] (double)');

%% Zuweisung der Parameter


% Aus parameters_mdh_beta_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; pkin(7); 0; -pi / 0.2e1 - pkin(5); 0; 0; 0;];
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
t1 = [0; 0; 0; pkin(23); pkin(9); pkin(10); 0; 0; -pkin(22); pkin(24); pkin(19); pkin(9); 0; pkin(12); 0; pkin(18); pkin(12); pkin(20); pkin(17); pkin(11); 0;];
a_mdh = t1;

% Aus parameters_mdh_theta_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; (2 * pkin(6)) + pi; 0; 0; 0; 0; (2 * pkin(3)) - pi; 0; 0; 0; 0; 0; 0; 0; 0;];
theta_mdh = t1;

% Aus parameters_mdh_d_matlab.m
t1 = [pkin(8); pkin(16); pkin(21); 0; 0; 0; pkin(13); -pkin(21); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
d_mdh = t1;

% Aus parameters_mdh_qoffset_matlab.m
t4 = -pi / 0.2e1;
t3 = pi / 0.2e1;
t1 = [t4; t3; 0; 0; 0; t3; 0; t3; 0; 0; pi; t4; t4; 0.3e1 / 0.2e1 * pi; 0; 0; 0; t3; 0; 0; 0;];
qoffset_mdh = t1;
