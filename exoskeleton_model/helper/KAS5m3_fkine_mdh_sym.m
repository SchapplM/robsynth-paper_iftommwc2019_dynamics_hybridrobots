% Direkte Kinematik für KAS5m3 (MDH-Konvention, symbolische Berechnung)
% 
% Input:
% q [6x1]
%   Gelenkwinkel, verallgemeinerte Koordinaten von KAS5m3. Definition
%   siehe Dokumentation
% 
% Output:
% T_c_mdh [4x4x15]
%   homogenious transformation matrices for each body frame (MDH)
%   1:  KAS-Basis -> MDH Basis (link 0)
%   2:  KAS-Basis  -> MDH Segment  1 (Hauptstruktur Segment 1)
%   3:  KAS-Basis  -> MDH Segment  2 (Hauptstruktur Segment 2)
%   4:  KAS-Basis  -> MDH Segment  3 (Hauptstruktur Segment 3)
%   ...
%   8:  KAS-Basis  -> MDH Segment  7 (Hauptstruktur Segment 7)
%   9:  KAS-Basis  -> MDH Segment  8 (Nebenstruktur Segment 1)
%   10: KAS-Basis  -> MDH Segment  9 (Nebenstruktur Segment 2)
%   11: KAS-Basis  -> MDH Segment 10 (Nebenstruktur Segment 3)
%   ...
%   14: KAS-Basis  -> MDH Segment 13 (Kompensationsfeder)
%   15: KAS-Basis  -> Endeffektor (Hauptstruktur Segment 7)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover


function T_c_mdh = KAS5m3_fkine_mdh_sym(q)
%% Init
% # codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [6 1]), ...
  'Gelenkwinkel q gefordert als [6x1] double');

[a_mdh, d_mdh, ~, ~] = KAS5_m4_parameter_mdh();
[l_const, w_const] = KAS5_parameter_kinematic();

T_c_mdh = NaN(4,4,15);
%% Berechnung
T_ges = KAS5m3_fkine_mdh_sym_varpar(q, a_mdh, d_mdh, l_const, w_const);

% Aus Maple-Format (2-dimensionaler Tensor) exportieren (3-dimensionaler Tensor)
for i = 1:14
  T_c_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);
end

% Endeffektor-Trafo
T_c_mdh(:,:,15) = T_c_mdh(:,:,8)*transl([0;0;l_const(19)]);