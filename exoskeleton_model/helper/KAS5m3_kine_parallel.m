% Kinematik der Parallelstruktur zur Unterstützung des KAS5
% Berechne nur die tatsächlich verwendete Konfiguration
% 
% Modell m3: Kurbel N7 ist starr mit Körper K2 gekoppelt (über die Achse)
% 
% Version 1 der Berechnung:
% Direkte Berechnung der Winkel des Kurbelmechanismus mit generierter Funktion für allgemeinen Kurbelmechanismus
% 
% 
% Anmerkungen:
% * Die Position aller Punkte wird nicht berechnet, da hierzu ein Aufruf
% der direkten Kinematik der Hauptstruktur notwendig wird

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

function [l_par, w_par] = KAS5m3_kine_parallel(q)

%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [6 1]), ...
  'Gelenkwinkel q gefordert als [6x1] double');
%% Init
% Konstante, bekannte Größen bestimmen
delta = NaN(18,1);

[l_const, w_const] = KAS5_parameter_kinematic();
l12 = l_const(12);
l14 = l_const(14);
l17 = l_const(17);
l18 = l_const(18);

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

%% Kurbelmechanismus Ellenbogen-Oberarm
r_4_G5D = [-l_const(11) + cos(rho(4))*(-l_const(5) + l_const(6)); -sin(rho(4))*(-l_const(5) + l_const(6))];
[tmp_beta_2alt, tmp_gamma_2alt, tmp_delta_2alt] = kine_kurbelmechanismus(r_4_G5D, l_const(14), ...
  l_const(13), l_const(22), pi+delta(7));
% Es gibt zwei Lösungen. Nehme die erste (Schubstange hinten). Die zweite
% Lösung ist zusammen mit oberem Kurbelmechanismus nicht möglich.
delta(3) = tmp_delta_2alt(1);
delta(4) = tmp_gamma_2alt(1) + rho(4) - 3*pi/2 - delta(12);
beta2 = tmp_beta_2alt(1);

%% Berechnungen
% Gl. (22)
delta(16) = pi/2 - delta(4) - delta(12);

% Gl. (3)
delta(11) = delta(10) - delta(4);

%% Kurbelmechanismus Schulter-Oberarm
r_3_D_G3 = [-l_const(6); 0; 0];
tmp_alpha = normalize_angle(pi/2 - delta(11));
[tmp_beta_2alt, tmp_gamma_2alt, tmp_delta_2alt] = kine_kurbelmechanismus(r_3_D_G3(1:2), l_const(21), ...
  l_const(20), l_const(4), tmp_alpha);
% Es gibt zwei Lösungen. 
% Je nachdem, welcher Wert für delta17 eingestellt ist, muss die erste oder
% die zweite Lösung genommen werden.
% Die Lösung entspricht der Konfiguration "Kurbelwange hinten"
% Die andere Lösung ist technisch möglich, aber kein vorgesehener Betriebszustand.

if tmp_alpha < 0
  delta(6) = tmp_delta_2alt(1);
  delta(14) = tmp_beta_2alt(1);
  gamma3 = -tmp_gamma_2alt(1);
else
  delta(6) = tmp_delta_2alt(2);
  delta(14) = tmp_beta_2alt(2);
  gamma3 = -tmp_gamma_2alt(2); % Unterschied zu m4!
end

%% Berechnungen Schulter-Oberarm
% % Gl. (16)
% Warnung: In Tests noch nicht alle Gleichungen konsistent!
% Wahrscheinlich Fehler in Vorzeichen o.ä.
% delta(13) = pi/2 - delta(11) - delta(14);

% Gl. (11)
rho(3) = gamma3 + delta(8) - pi/2;

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

% Gl. (27)
% codegen/KAS5m2_kinematik_parallelstruktur_0FG.mw
% codeexport/KAS5_kinematik_parallel_l16_Fcn_gamma5.m
t4 = cos(rho(6)+pi/2);
t3 = sin(rho(6)+pi/2);
t2 = t4 * l_const(18) + t3 * l_const(17) + sin(-gamma5) * l_const(14);
t1 = -l_const(12) - t3 * l_const(18) + t4 * l_const(17) + cos(-gamma5) * l_const(14);
l_par = sqrt(t1 ^ 2 + t2 ^ 2);
l16 = l_par;

% Gl. (29)
% codegen/KAS5m2_kinematik_parallelstruktur_1FG.mw
% codeexport/KAS5_kinematik_parallel_beta1_Fcn_gamma5_alt1.m
t3 = 0.1e1 / l16;
t2 = cos(rho(6)+pi/2);
t1 = sin(rho(6)+pi/2);
beta1 = atan2(-(t2 * l18 - sin(gamma5) * l14 + t1 * l17) * t3, (t2 * l17 + cos(gamma5) * l14 - t1 * l18 - l12) * t3) - gamma5;


% Gl. (28)
% codegen/KAS5m2_kinematik_parallelstruktur_0FG.mw
% alpha = (3*pi - beta1 - gamma5 - pi - (rho(6)+pi/2) - pi/2);
% Korrektur: Reduktion des Fünfecks auf ein Viereck. TODO: Prüfen.
alpha = 2*pi - beta1 - gamma5 - pi/2 - rho(6);
%% Ergebnisse speichern
w_par = [alpha; beta1; beta2; gamma3; gamma5; delta; rho(3)];
