% Kinematik der Parallelstruktur zur Unterstützung des KAS5
% Berechne die Position aller Punkte der Parallelstruktur
% 
% Unabhängig von Modell m3 oder m4
% 
% Eingabe:
% w_par
%   Winkel der Parallelstruktur
% T_c_mdh
%   Transformationsmatrizen zu allen MDH-Koordinatensystemen
% 
% Ausgabe:
% r_P
%   Position aller zusätzlicher Punkte im Basis-KS (MDH)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [r_P] = KAS5m3_kine_parallel_hilfspunkte(l_const, w_par, T_c_mdh)

%% Initialisierung
gamma3 = w_par(4);
gamma5 = w_par(5);
delta9 = w_par(14);
delta16 = w_par(21);
%% Berechnung von Punkten
r_P = NaN(8,3);

% Punkt A (ausgehend von K6)
P_tmp = T_c_mdh(:,:,7) * [-l_const(17); -l_const(18); 0; 1];
r_P(1,:) = P_tmp(1:3);

% Punkt B (ausgehend von K5)
P_tmp = T_c_mdh(:,:,6) * trotz(-gamma5) * [l_const(14); 0; 0; 1];
r_P(2,:) = P_tmp(1:3);

% Punkt C (ausgehend von K3)
P_tmp = T_c_mdh(:,:,4) * transl([l_const(6); 0; 0]) * trotz(-delta16) * [l_const(22); 0; 0; 1];
r_P(3,:) = P_tmp(1:3);

% Punkt D (ausgehend von K3)
P_tmp = T_c_mdh(:,:,4) * transl([l_const(6); 0; 0]) * [0;0;0;1];
r_P(4,:) = P_tmp(1:3);

% Punkt E (ausgehend von K3)
P_tmp = T_c_mdh(:,:,4) * transl([l_const(6); 0; 0]) * trotz(-delta16 + delta9) * [l_const(21); 0; 0; 1];
r_P(5,:) = P_tmp(1:3);

% Punkt F (ausgehend von K3)
P_tmp = T_c_mdh(:,:,4) * trotz(-gamma3) * [l_const(4); 0; 0; 1];
r_P(6,:) = P_tmp(1:3);

% Punkt H (ausgehend von K6)
P_tmp = T_c_mdh(:,:,7) * [0; -l_const(18); 0; 1];
r_P(8,:) = P_tmp(1:3);