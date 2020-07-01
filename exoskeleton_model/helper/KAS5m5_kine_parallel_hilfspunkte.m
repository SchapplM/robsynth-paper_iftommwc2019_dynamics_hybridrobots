% Kinematik der Parallelstruktur zur Unterstützung des KAS5 mit Modell m5
% Berechne die Position aller Punkte der Parallelstruktur
% 
% Unabhängig von Modell m3 oder m5
% 
% Eingabe:
% l_const
%   Konstante Winkel
% T_c_mdh_m5
%   Transformationsmatrizen zu allen MDH-Koordinatensystemen der Modelle
%   KAS5m3, KAS5m5
% 
% Ausgabe:
% r_P_m5
%   Position aller zusätzlicher Punkte der Parallelstruktur im Basis-KS (MDH)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-02
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function r_P_m5 = KAS5m5_kine_parallel_hilfspunkte(l_const, T_c_mdh_m5)

%% Berechnung von Punkten
r_P_m5 = NaN(8,3);

% Punkt A (ausgehend von K6)
P_tmp = T_c_mdh_m5(:,:,7) * [-l_const(17); -l_const(18); 0; 1];
r_P_m5(1,:) = P_tmp(1:3);

% Punkt B (in KS12, KS13)
r_P_m5(2,:) = T_c_mdh_m5(1:3,4,13);

% Punkt C (in KS11)
r_P_m5(3,:) = T_c_mdh_m5(1:3,4,12);

% Punkt D (in KS10)
r_P_m5(4,:) = T_c_mdh_m5(1:3,4,11);

% Punkt E (ausgehend von KS9)
P_tmp = T_c_mdh_m5(:,:,10) * transl([l_const(20); 0; 0]);
r_P_m5(5,:) = P_tmp(1:3,4);

% Punkt F (in KS9)
r_P_m5(6,:) = T_c_mdh_m5(1:3,4,10);

% Punkt H (ausgehend von K6)
P_tmp = T_c_mdh_m5(:,:,7) * [0; -l_const(18); 0; 1];
r_P_m5(8,:) = P_tmp(1:3);