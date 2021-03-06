% Convert vector of modified DH parameters to kinematic parameter vector for
% KAS5m5OL
%
% Input:
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
%
% Output:
% pkin [12x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[a10,a11,a12,a4,a5,a6,a9,d1,d2,d3,d7,d8]';

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function pkin = KAS5m5OL_mdhparam2pkin(beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh)

% Aus parameter_kin_from_mdh_matlab.m
t1 = [a_mdh(10); a_mdh(11); a_mdh(12); a_mdh(4); a_mdh(5); a_mdh(6); a_mdh(9); d_mdh(1); d_mdh(2); d_mdh(3); d_mdh(7); d_mdh(8);];
pkin = t1;
