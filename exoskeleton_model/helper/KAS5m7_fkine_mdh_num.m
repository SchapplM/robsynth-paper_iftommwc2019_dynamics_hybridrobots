% Direkte Kinematik für KAS5 m7 (MDH-Konvention, numerische Berechnung)
% 
% Input:
% q [7x1]
%   Joint Angles [rad]
% 
% Output:
% T_c_mdh [4x4x14]
%   homogenious transformation matrices for each body frame (MDH)
%   1: KAS-Basis -> MDH Basis (Körper 0)
%   2: KAS-Basis  -> MDH Segment 1 (Körper 1)
%   3: KAS-Basis  -> MDH Segment 2 (Körper 2)
%   4: KAS-Basis  -> MDH Segment 3 (Körper 3)
%   ...
%   8: KAS-Basis  -> MDH Segment 7 (Körper 7)
%   9: KAS-Basis  -> Parallelstruktur. Segment 8 (Körper 8)
%   ...
%   15: KAS-Basis  -> Parallelstruktur. Segment 15 (Körper 15)
%   16-18: KAS-Basis  -> Erstes Koordinatensystem für Schnittgelenk
%   19-21: KAS-Basis  -> Zweites Koordinatensystem für Schnittgelenk
%   23: KAS-Basis  -> Endeffektor

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-02
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover


function T_c_mdh = KAS5m7_fkine_mdh_num(q)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [5 1]), ...
  'Joint angles q have to be [5x1] double');

[sigma_mdh, beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, q_offset_mdh, v] = KAS5m7_parameter_mdh();
l = KAS5_parameter_kinematic();

jv = KAS5m7TE_kinconstr_expl_mdh_num(q);
%% Berechnung
T_base = eye(4);
T_c_mdh = NaN(4,4,23);
T_c_mdh(:,:,1) = T_base;

q_mdh = jv + q_offset_mdh;

% [2] Equ. (9), (7)
% Alle Gelenkwinkel durchgehen
for i = 1:21
  j = v(i)+1;
  T_c_mdh(:,:,i+1) = T_c_mdh(:,:,j) * ...
                       transl([b_mdh(i);0;0]) * trotz(beta_mdh(i)) * ...
                       trotx(alpha_mdh(i)) * ...
                       transl([a_mdh(i);0;0]);
  if sigma_mdh(i) == 0
    % Drehgelenk
    T_c_mdh(:,:,i+1) = T_c_mdh(:,:,i+1) * trotz(q_mdh(i)) * transl([0;0;d_mdh(i)]);
  else
    % Schubgelenk
    T_c_mdh(:,:,i+1) = T_c_mdh(:,:,i+1) * trotz(theta_mdh(i)) * transl([0;0;q_mdh(i)]);
  end
end

% Endeffektor-Trafo
T_c_mdh(:,:,23) = T_c_mdh(:,:,8)*transl([0;0;l(19)]);