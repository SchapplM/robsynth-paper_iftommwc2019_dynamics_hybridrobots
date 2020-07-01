% Kinematik der Parallelstruktur zur Unterstützung des KAS5
% Berechne nur die tatsächlich verwendete Konfiguration
% 
% Modell m3: Kurbel N7 ist starr mit Körper K2 gekoppelt (über die Achse)
% 
% Version 2 der Berechnung:
% Erst Berechnung der Schnittpunkte der Kreise mit allgemeiner Formel. 
% Dann Berechnung der Winkel mit Trigonometrie
% 
% 
% Anmerkungen:
% * Die Position aller Punkte wird mit berechnet (im Algorithmus sowieso
% enthalten)

% Eingabe:
% q [6x1]
%   Gelenkwinkel, verallgemeinerte Koordinaten von KAS5m3. Definition
%   siehe Dokumentation
% 
% Ausgabe:
% l_par
%   Längen der Parallelstruktur
% w_par [24x1]
%   Winkel der Parallelstruktur

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [l_par, w_par, r_P, err] = KAS5m3_kine_parallel_v2(q)

%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [6 1]), ...
  'Gelenkwinkel q gefordert als [6x1] double');
%% Init
% Konstante, bekannte Größen bestimmen
delta = NaN(18,1);
r_P = NaN(8,3);
w_par = NaN(23,1);
l_par = NaN;

[l_const, w_const] = KAS5_parameter_kinematic();
l4 = l_const(4);
l12 = l_const(12);
l14 = l_const(14);
l17 = l_const(17);
l18 = l_const(18);
l22 = l_const(22);

rho = NaN(7,1);
rho([1,2,4,5,6,7]) = q;

% Konstante Winkel
delta(9) = w_const(1);
delta(10) = w_const(2);
delta(12) = w_const(3);
delta(17) = w_const(4);
delta(8) = w_const(5);
delta(18) = w_const(6);
%% Berechnungen
% Gl. (30) Berechne delta7 (Setzt Zahnradkontakt voraus!)
delta(7) = delta(17) + rho(4);

% Gl. (2) Berechne gamma5
gamma5 = - delta(7) + pi + rho(5);

% Punkt C aus Kreisschnittpunkt
% codegen/KAS5m3_sym_codegen_kinematik_parallelstruktur_hilfsberechnungen.mw
% codeexport/KAS5_kinematik_parallel_r_4_G5D.m
r_4_G5_D = [-l_const(11) + cos(rho(4))*(-l_const(5) + l_const(6)); ...
            -sin(rho(4))*(-l_const(5) + l_const(6));...
             0];
r_4_G5_B = l_const(14)*[-cos(delta(7)); -sin(delta(7)); 0];
[r_4_G5_C_2Alt, err] = intersect_circles(r_4_G5_B(1:2), r_4_G5_D(1:2), l_const(13), l_const(22));
if err
  return
end
% Wähle Lösung Nummer 2 aus. Die Nummerierung ist entgegengesetzt zur
% anderen Berechnungsmethode! (Keine Lösung für anderen Hebelmechanismus
% mit Lösung 1.
r_4_G5_C = [r_4_G5_C_2Alt(:,2); 0];
r_4_D_C = - r_4_G5_D + r_4_G5_C;
R_3_4 = rotz(rho(4));
r_3_D_C = R_3_4 * r_4_D_C;

% Gl. (m3.5)
delta(16) = atan2(-r_3_D_C(2), r_3_D_C(1));


% Punkt F aus Kreisschnittpunkt
r_3_D_E = l_const(21) * [cos(-delta(9)+delta(16)); -sin(-delta(9)+delta(16)); 0];
r_3_D_G3 = [-l_const(6); 0; 0];

[r_3_D_F_2Alt, err] = intersect_circles(r_3_D_E(1:2), r_3_D_G3(1:2), l_const(20), l_const(4));
if err
  return
end

r_3_D_F = [r_3_D_F_2Alt(:,2); 0];

r_3_F_G3 = -r_3_D_F + r_3_D_G3;

% Gl. (m3.6)
gamma3 = atan2(r_3_F_G3(2), -r_3_F_G3(1));

% Gl. (22)
delta(4) = pi/2 - delta(16) - delta(12);

% Gl. (3)
delta(11) = delta(10) - delta(4);

% Gl. (m3.7)
% codegen/KAS5m3_kinematik_parallelstruktur.mw
% codeexport/KAS5_kinematik_parallel_beta2.m
r_4_G5_C_x = r_4_G5_C(1);
r_4_G5_C_y = r_4_G5_C(2);
beta2 = atan2((sin(delta(7)) * l14 + r_4_G5_C_y), (cos(delta(7)) * l14 + r_4_G5_C_x)) - delta(7);

% Gl. (m3.9)
% codegen/KAS5m3_sym_codegen_kinematik_parallelstruktur_hilfsberechnungen.mw
% codeexport/KAS5_kinematik_parallel_delta3.m
r_4_D_B_x = -r_4_G5_D(1) + r_4_G5_B(1);
r_4_D_B_y = -r_4_G5_D(2) + r_4_G5_B(2);

t7 = rho(4) + delta(16);
delta(3) = atan2((sin(t7) * l22 + r_4_D_B_y), (cos(t7) * l22 - r_4_D_B_x)) - t7;

%% Berechnungen Schulter-Oberarm
% % Gl. (16)
% Warnung: In Tests noch nicht alle Gleichungen konsistent!
% Wahrscheinlich Fehler in Vorzeichen o.ä.
% NaN lassen, damit bei Tests keine Fehler auftreten
% delta(13) = pi/2 - delta(11) - delta(14)

% Gl. (11)
rho(3) = gamma3 + delta(8) - pi/2;

% Gl. (m3.8)
% codegen/KAS5m3_sym_codegen_kinematik_parallelstruktur_hilfsberechnungen.mw
% codeexport/KAS5_kinematik_parallel_beta2.m
r_3_G3_E_x = -r_3_D_G3(1) + r_3_D_E(1);
r_3_G3_E_y = -r_3_D_G3(2) + r_3_D_E(2);
t1 = rho(3) - delta(8);
delta(6) = atan2((sin(t1) * l4 + r_3_G3_E_x), (cos(t1) * l4 + r_3_G3_E_y)) - t1;

%% Berechnungen im Hilfsdreieck
% Gl. (19)
% Warnung: In Tests noch nicht alle Gleichungen konsistent!
% Wahrscheinlich Fehler in Vorzeichen o.ä.
% delta(5) = pi-delta(10) - delta(14) - pi/2;

% Gl. (18)
% Warnung: In Tests noch nicht alle Gleichungen konsistent!
% Wahrscheinlich Fehler in Vorzeichen o.ä.
% delta(15) = pi/2 - delta(5);
%% Berechnungen Unterarm
% r_4_G5_B siehe oben
% Berechnung Federlänge aus Punkten
% codegen/KAS5m3_kinematik_parallelstruktur.mw
% codeexport/KAS5_kinematik_parallel_r_4_G5_A.m
t3 = rho(5) + (rho(6)+pi/2);
t2 = cos(t3);
t1 = sin(t3);
r_4_G5_A = [cos(rho(5)) * l12 - t2 * l17 + t1 * l18; sin(rho(5)) * l12 - t1 * l17 - t2 * l18; 0;];

r_4_A_B = - r_4_G5_A + r_4_G5_B;
l_par = sqrt(r_4_A_B(1)^2 + r_4_A_B(2)^2);

% Gl. (29)
% codegen/KAS5m2_kinematik_parallelstruktur_0FG.mw
% codeexport/KAS5_kinematik_parallel_beta1_Fcn_gamma5.m
t2 = cos(rho(6)+pi/2);
t1 = sin(rho(6)+pi/2);
beta1 = atan2(-(t2 * l18 - sin(gamma5) * l14 + t1 * l17), (t2 * l17 + cos(gamma5) * l14 - t1 * l18 - l12)) - gamma5;

% 
% % Gl. (28)
% % codegen/KAS5m2_kinematik_parallelstruktur_0FG.mw
% alpha = (3*pi - beta1 - gamma5 - pi - rho(6) - pi/2);

%% Ergebnisse speichern
w_par = [NaN; beta1; beta2; gamma3; gamma5; delta; rho(3)];


%% Hilfspunkte berechnen
% Berechne die Position der Hilfspunkte im Basis-Koordinatensystem

% direkte Kinematik mit neuen Gelenkwinkeln
q_KAS5m2 = rho;
T_c_mdh = KAS5m2_fkine_mdh_num(q_KAS5m2);

% Punkt A (ausgehend von K6)
P_tmp = T_c_mdh(:,:,7) * [-l_const(17); -l_const(18); 0; 1];
r_P(1,:) = P_tmp(1:3);

% Punkt B (ausgehend von K4)
% G5 ist der Ursprung von KS5 (T_c_mdh(1:3,4,6))
% T_c_mdh(1:3, 1:3, 5) rotiert KS4 nach KS0
P_tmp = T_c_mdh(1:3,4,6) + T_c_mdh(1:3, 1:3, 5)*r_4_G5_B;
r_P(2,:) = P_tmp(1:3);

% Punkt C (ausgehend von K3)
P_tmp = T_c_mdh(:,:,4) * transl([l_const(6); 0; 0]) * trotz(-delta(16)) * [l_const(22); 0; 0; 1];
r_P(3,:) = P_tmp(1:3);

% Punkt D (ausgehend von K3)
P_tmp = T_c_mdh(:,:,4) * [-r_3_D_G3; 1];
r_P(4,:) = P_tmp(1:3);

% Punkt E (ausgehend von K3)
P_tmp = T_c_mdh(:,:,4) * transl([l_const(6); 0; 0]) * trotz(-delta(16) + delta(9)) * [l_const(21); 0; 0; 1];
r_P(5,:) = P_tmp(1:3);

% Punkt F (ausgehend von K3)
P_tmp = T_c_mdh(:,:,4) * [-r_3_F_G3; 1];
r_P(6,:) = P_tmp(1:3);

% Punkt H (ausgehend von K6)
P_tmp = T_c_mdh(:,:,7) * [0; -l_const(18); 0; 1];
r_P(8,:) = P_tmp(1:3);