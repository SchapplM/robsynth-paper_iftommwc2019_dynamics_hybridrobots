% Direkte Kinematik für KAS5 m4 (MDH-Konvention, numerische Berechnung)
% 
% Input:
% q [7x1]
%   Joint Angles [rad]
% 
% Output:
% T_c_mdh [4x4x14]
%   homogenious transformation matrices for each body frame (MDH)
%   1: KAS-Basis -> MDH Basis (link 0)
%   2: KAS-Basis  -> MDH Segment 1 (link 1)
%   3: KAS-Basis  -> MDH Segment 2 (link 2)
%   4: KAS-Basis  -> MDH Segment 3 (link 3)
%   ...
%   8: KAS-Basis  -> MDH Segment 7 (link 7)
%   9: KAS-Basis  -> Parallelstruktur. Segment 8 (Körper 8)
%   ...
%   14: KAS-Basis  -> Parallelstruktur. Segment 13 (Körper 13)
%   15: KAS-Basis  -> Endeffektor

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_mdh = KAS5_m4_fkine_mdh_num(q, w_par)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [7 1]), ...
  'Joint angles q have to be [7x1] double');
assert(isa(w_par,'double') && isreal(w_par) && all(size(w_par) == [24 1]), ...
  'w_par has to be [24x1] double');

[a_mdh, d_mdh, alpha_mdh, q_offset_mdh, v] = KAS5_m4_parameter_mdh();
l = KAS5_parameter_kinematic();

alpha = w_par(1);
beta1 = w_par(2);
beta2 = w_par(3);
gamma3 = w_par(4);
gamma5 = w_par(5);
delta = w_par(6:end-1);
theta3 = w_par(end);
%% Berechnung
T_base = eye(4);
T_c_mdh = NaN(4,4,15);
T_c_mdh(:,:,1) = T_base;

q_mdh = [q; delta(8); -delta(6); -delta(16); -delta(3); -beta2; -beta1];

% [2] Equ. (9), (7)
% Alle Gelenkwinkel durchgehen
for i = 1:13
  j = v(i)+1;
  T_c_mdh(:,:,i+1) = T_c_mdh(:,:,j) * trotx(alpha_mdh(i)) * ...
                     transl([a_mdh(i);0;0]) * trotz(q_mdh(i)+q_offset_mdh(i)) ...
                     * transl([0;0;d_mdh(i)]);
end

% Endeffektor-Trafo
T_c_mdh(:,:,15) = T_c_mdh(:,:,8)*transl([0;0;l(19)]);