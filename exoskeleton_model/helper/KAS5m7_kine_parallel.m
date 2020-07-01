% Kinematik der Parallelstruktur zur Unterstützung des KAS5
% Berechne nur die tatsächlich verwendete Konfiguration
% 
% Modell m7: Es werden alle Körper betrachtet (2 für Kompensationsfeder)

% Eingabe:
% q [5x1]
%   Gelenkwinkel, verallgemeinerte Koordinaten von KAS5m5. Definition
%   siehe Dokumentation
% 
% Ausgabe:
% l_par
%   Längen der Parallelstruktur
% w_par_out [24x1]
%   Winkel der Parallelstruktur

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-02
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function [l_par, w_par_out] = KAS5m7_kine_parallel(q)

%% Init
%#codegen
% Coder Information
assert(isa(q,'double') && isreal(q) && all(size(q) == [5 1]), ...
  'Gelenkwinkel q gefordert als [5x1] double');
%% Init
q_m3 = KAS5_convert_q(q, 5, 3);
[l_const, ~] = KAS5_parameter_kinematic();
[l_par, w_par] = KAS5m3_kine_parallel(q_m3);

%% Winkel neu zusammensetzen
alpha = w_par(1);
beta1 = w_par(2);
beta2 = w_par(3);
gamma3 = w_par(4);
gamma5 = w_par(5);
delta = [w_par(6:end-1); NaN(3,1)];
rho3 = w_par(end);

% Neue Winkel im Vergleich zu KAS5m3
delta(19) = q(3); % rho4
delta(20) = atan2(l_const(18), l_const(14));
delta(21) = alpha + delta(20)-pi/2;

%% Ergebnisse speichern
w_par_out = [alpha; beta1; beta2; gamma3; gamma5; delta; rho3];