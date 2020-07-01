% Convert vector of modified DH parameters to kinematic parameter vector for
% KAS5m7OL
%
% Input:
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
%
% Output:
% pkin [19x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[delta17,delta20,delta8,delta9,l1,l11,l12,l13,l14,l15,l2,l20,l21,l22,l23,l3,l4,l5,l6]';

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function pkin = KAS5m7OL_mdhparam2pkin(beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh)

% Aus parameter_kin_from_mdh_matlab.m
t1 = -pi / 0.2e1;
t2 = [theta_mdh(13) + pi / 0.2e1; -beta_mdh(18) + t1; theta_mdh(8) + t1; beta_mdh(16); d_mdh(1); a_mdh(5); a_mdh(6); a_mdh(20); a_mdh(14); d_mdh(7); d_mdh(2); a_mdh(19); a_mdh(16); a_mdh(11); a_mdh(18); d_mdh(3); -a_mdh(9); a_mdh(4); a_mdh(10);];
pkin = t2;
