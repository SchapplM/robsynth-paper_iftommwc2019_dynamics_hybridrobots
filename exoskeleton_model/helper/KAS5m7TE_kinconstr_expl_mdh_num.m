% Explicit kinematic constraints of
% KAS5m7
% Use Code from Maple symbolic Code Generation
% 
% Input:
% qJ [5x1]
%   Generalized joint coordinates (joint angles)
% 
% Output:
% jv [21x1]
%   Joint variables (rotation around z or translation in z-direction according to MDH)

% TODO: delta14 Vorzeichen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-02
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function jv = KAS5m7TE_kinconstr_expl_mdh_num(qJ)
%% Coder Information
%#codegen
assert(isa(qJ,'double') && isreal(qJ) && all(size(qJ) == [5 1]), ...
  'KAS5m5_kinconstr_expl_mdh_sym_varpar: qJ has to be [5x1] double');

% Variable Initialization

q_m5 = qJ;
q_m3 = KAS5_convert_q(q_m5, 5, 3);
[Ls, w_par] = KAS5m7_kine_parallel(q_m5);

alpha = w_par(1);
beta1 = w_par(2);
beta2 = w_par(3);
gamma3 = w_par(4);
gamma5 = w_par(5);
delta = w_par(6:end-1);
rho3 = w_par(end);

rho1 = q_m3(1);
rho2 = q_m3(2);
rho4 = q_m3(3);
rho5 = q_m3(4);
rho6 = q_m3(5);
rho7 = q_m3(6);

jv = [rho1; rho2; rho3; rho4; rho5; rho6; rho7; ... % 1-7
  delta(8); -delta(6); -delta(16); -delta(3); delta(19); delta(17); -beta1; Ls; ... % 8-15
  delta(14); beta2; delta(21); zeros(3,1)]; % 16-21
