% Direkte Kinematik für KAS5m2 (MDH-Konvention, symbolische Berechnung)
% 
% Input:
% q [7x1]
%   Joint Angles [rad]
% 
% Output:
% T_c_mdh [4x4x9]
%   homogenious transformation matrices for each body frame (MDH)
%   1: KAS-Basis -> MDH Basis (link 0)
%   2: KAS-Basis  -> MDH Segment 1 (link 1)
%   3: KAS-Basis  -> MDH Segment 2 (link 2)
%   4: KAS-Basis  -> MDH Segment 3 (link 3)
%   ...
%   8: KAS-Basis  -> MDH Segment 7 (link 7)
%   9: KAS-Basis  -> Endeffektor (link 7)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_mdh = KAS5m2_fkine_mdh_sym(q)
%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [7 1]), ...
  'Joint angles q have to be [7x1] double');

[a_mdh, d_mdh, ~, ~] = KAS5m2_parameter_mdh();
l = KAS5_parameter_kinematic();

T_c_mdh = NaN(4,4,9);
%% Berechnung
T_ges = KAS5m2_fkine_mdh_sym_varpar(q, a_mdh, d_mdh);

% Aus Maple-Format (2-dimensionaler Tensor) exportieren (3-dimensionaler Tensor)
for i = 1:8
  T_c_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);
end

% Endeffektor-Trafo
T_c_mdh(:,:,9) = T_c_mdh(:,:,8)*transl([0;0;l(19)]);