% Convert vector of kinematic parameters to modified DH parameters of
% KAS5m5
%
% Input:
% pkin [30x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8,delta8s,delta9s,l11,l12,l13,l14,l17,l18,l20,l21,l22,l4,l5,l6,delta10s,delta12s,delta17s,delta18s]';
%
% Output: MDH parameter in order of transformation
% beta_mdh [13x1]
%   Rotation around z
% b_mdh [13x1]
%   Translation along z
% alpha_mdh [13x1]
%   Rotation around x
% a_mdh [13x1]
%   Translation along x
% theta_mdh [13x1]
%   Rotation around z
% d_mdh [13x1]
%   Translation along z
% qoffset_mdh [13x1]
%   Offset on joint coordinate q

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:16
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh] = KAS5m5_pkin2mdhparam(pkin)

%% Init
%#codegen
%$cgargs {zeros(30,1)}
assert(isreal(pkin) && all(size(pkin) == [30 1]), ...
  'KAS5m5_pkin2mdhparam: Kinematic parameters pkin have to be [30x1] (double)');

%% Zuweisung der Parameter


% Aus parameters_mdh_beta_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
beta_mdh = t1;

% Aus parameters_mdh_b_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
b_mdh = t1;

% Aus parameters_mdh_alpha_matlab.m
t1 = pi / 0.2e1;
t2 = [0; -pi / 0.2e1; t1; 0; 0; 0; t1; t1; 0; 0; 0; 0; 0;];
alpha_mdh = t2;

% Aus parameters_mdh_a_matlab.m
t1 = [0; 0; 0; pkin(4); pkin(5); pkin(6); 0; 0; pkin(7); pkin(1); pkin(2); pkin(3); 0;];
a_mdh = t1;

% Aus parameters_mdh_theta_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
theta_mdh = t1;

% Aus parameters_mdh_d_matlab.m
t1 = [pkin(8); pkin(9); pkin(10); 0; 0; 0; pkin(11); pkin(12); 0; 0; 0; 0; 0;];
d_mdh = t1;

% Aus parameters_mdh_qoffset_matlab.m
t2 = pi / 0.2e1;
t1 = [-pi / 0.2e1; t2; 0; 0; 0; t2; 0; t2; 0; 0; pi; 0; pi;];
qoffset_mdh = t1;
