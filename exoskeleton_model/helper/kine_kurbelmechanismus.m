% Berechne die Kinematik für einen Kurbel-Stange-Mechanismus bestehend aus
% zwei Kurbelarmen und einer Schubstange
% 
% Kurbelarm l1 (Endpunkt B) dreht um A, Kurbelarm l3 (Endpunkt D) dreht um C, 
% Schubstange l2 verbindet die Kurbelarme mit B-D
% 
% Eingabe:
%   r_0_AC
%     Koordinaten des Punktes C. A liegt im Ursprung
%   l1
%     Länge Kurbelarm A-B
%   l2
%     Länge Schubstange B-D
%   l3
%     Länge Kurbelarm C-D
%   alpha
%     Winkel des Kurbelarms l1 gegen die x-Achse des globalen KS
% 
% Ausgabe:
%   beta_2alt
%     Innenwinkel in B (2 Alternativen) 
%   gamma_2alt
%     Innenwinkel in C (2 Alternativen) 
%   delta_2alt
%     Innenwinkel in D (2 Alternativen) 
%   err
%     Fehlerausgabe.
%       0 = Kein Fehler. Berechnung erfolgreich
%       1 = Fehler: Keine Lösung möglich (aus geometrischen Gründen)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [beta_2alt, gamma_2alt, delta_2alt, err] = kine_kurbelmechanismus(r_0_AC, l1, l2, l3, alpha)

%% Compiler-Informationen
%#codegen
assert(isa(r_0_AC,'double') && isreal(r_0_AC) && all(size(r_0_AC) == [2 1]), ...
  'Vektor A-C muss 2x1 double sein');
assert(isa(l1,'double') && isreal(l1) && all(size(l1) == [1 1]), ...
  'Laenge l1 muss 1x1 double sein');
assert(isa(l2,'double') && isreal(l2) && all(size(l2) == [1 1]), ...
  'Laenge l2 muss 1x1 double sein');
assert(isa(l3,'double') && isreal(l3) && all(size(l3) == [1 1]), ...
  'Laenge l3 muss 1x1 double sein');
assert(isa(alpha,'double') && isreal(alpha) && all(size(alpha) == [1 1]), ...
  'Winkel alpha muss 1x1 double sein');


%% Init
% Quelle: codegen/kurbel_mechanismus_kinematik.mw
x_AC = r_0_AC(1);
y_AC = r_0_AC(2);

beta_2alt = NaN(2,1);
gamma_2alt = NaN(2,1);
delta_2alt = NaN(2,1);
%% Prüfe ob Lösung möglich
r_0_AB = l1*[cos(alpha); sin(alpha)];
r_0_BC = -r_0_AB + r_0_AC(1:2);
d_0_BC = norm(r_0_BC);
if (l2 + l3) < d_0_BC + 1e-5
  % Entfernung zu groß, beide Kreise berühren sich nicht
  % Nutze Toleranz von 1e-5, damit keine rundungsbedingten Imaginärteile
  % entstehen
  err = 1;
  return
end
if (l2 + l3*2) > d_0_BC
  % Entfernung zu gering, Kreis des Kurbelarmes ist vollständig innerhalb
  % des Kreises der Schubstange
  % TODO: Kriterium definieren
%   err = 2;
%   return
end

%% Berechne Winkel beta zwischen erster Kurbel und Stange
% Quelle: 
% codegen/kurbel_mechanismus_kinematik.mw
% codeexport/kurbelmechanismus_kinematik_beta_alt_beide_matlab.m

t152 = (l3 ^ 2);
t154 = l2 ^ 2;
t190 = (t152 + t154);
t189 = 8 * t190;
t147 = y_AC ^ 2;
t141 = sin(alpha);
t192 = t141 * y_AC;
t197 = -0.2e1 * l1 * t192 + t147;
t196 = t152 - t154;
t142 = cos(alpha);
t139 = t142 ^ 2;
t150 = (x_AC ^ 2);
t195 = (t150 + t190) * t139;
t148 = t150 ^ 2;
t151 = (t152 ^ 2);
t153 = (t154 ^ 2);
t191 = (t152 * t154);
t176 = 2 * t190 * t150 - t148 - t151 - t153 + 2 * t191;
t194 = l1 * t142;
t172 = t147 ^ 2;
t193 = l1 * t172;
t160 = l1 ^ 2;
t144 = t160 * x_AC;
t161 = l1 * t160;
t187 = 0.4e1 * t161;
t185 = t150 - t190;
t179 = 0.2e1 * x_AC * t194 - t150 - t160 - t197;
t128 = 0.1e1 / t179;
t155 = 0.1e1 / l2;
t183 = t128 * t155 / 0.2e1;
t182 = -0.1e1 / (l1 * t141 - y_AC) * t155 / 0.2e1;
t181 = 4 * t190;
t180 = (-x_AC + t194) * t128;
t177 = t179 + t196;
t149 = x_AC * t150;
t175 = 0.2e1 * t139 * t144 + t144 + t149 + (-3 * t150 + t152) * t194 + (0.2e1 * t160 * t192 - t161 + (-t147 - t154) * l1) * t142 + (-t196 + t197) * x_AC;
t171 = y_AC * t147;
t162 = t160 ^ 2;
t157 = l1 * t162;
t156 = t161 ^ 2;
t138 = t142 * t139;
t137 = t141 ^ 2;
t127 = sqrt(t139 * t156 - t156 - t171 ^ 2 + (-t138 + t142) * t149 * t187 - 0.2e1 * t185 * t172 + ((t190 * t187 - 0.4e1 * t157) * t138 + (-t161 * t181 + 0.4e1 * t157 + 0.4e1 * t193) * t142) * x_AC + ((2 * t152) + 0.2e1 * t154 + (0.4e1 * t139 ^ 2 - 0.2e1) * t150 - 0.2e1 * t195) * t162 + (-0.12e2 * t138 * t161 * x_AC + (0.6e1 * t139 - 0.7e1 + (0.4e1 * t139 - 0.8e1) * t137) * t162 + (0.4e1 * l1 * t149 + ((0.8e1 * t137 + 0.16e2) * t161 - l1 * t181) * x_AC) * t142 + t176) * t147 + (((-0.12e2 * t139 + 0.20e2) * t161 + 0.8e1 * t185 * l1) * t171 + (0.6e1 * t193 + (0.8e1 * t138 - 0.16e2 * t142) * x_AC * t162 + (-0.4e1 * t139 + 0.6e1) * t157 + ((8 * t150) - t189 + 0.4e1 * t195) * t161 + (-t150 * t181 + 2 * t148 + 2 * t151 + 2 * t153 - 4 * t191) * l1) * y_AC) * t141 + ((-0.8e1 * t137 - 0.7e1) * t172 + (-0.8e1 * t149 * y_AC + (y_AC * t189 - 0.16e2 * t171) * x_AC) * t142 * t141 + (-(8 * t150) + (4 * t152) + 0.4e1 * t154 + (-4 * t150 + t189) * t137) * t147 + (0.2e1 * t147 * t185 + 0.5e1 * t172 - t176) * t139 + t176) * t160);
t126 = -t127 + t175;
t125 = t127 + t175;
beta_2alt = [atan2((t126 * t180 + t177) * t182, t126 * t183) - alpha; atan2((t125 * t180 + t177) * t182, t125 * t183) - alpha;];

%% Berechne Winkel delta zwischen zweiter Kurbel und Stange
% Quelle: 
% codegen/kurbel_mechanismus_kinematik.mw
% codeexport/kurbelmechanismus_kinematik_delta_matlab.m

r_0_B_C_x = r_0_BC(1);
r_0_B_C_y = r_0_BC(2);

t198 = alpha + beta_2alt;
t199 = 0.1e1 / l3;
delta_2alt = atan2((l2 * sin(t198) + r_0_B_C_y) * t199, (l2 * cos(t198) + r_0_B_C_x) * t199) - t198;

%% Berechne Winkel gamma von zweiter Kurbel und Koordinatensystem
gamma_2alt = NaN(2,1);
for i = 1:2
  % Quelle: 
  % codegen/kurbel_mechanismus_kinematik.mw
  % codeexport/kurbelmechanismus_kinematik_gamma_matlab.m
  beta = beta_2alt(i);
  t120 = alpha + beta;
  % Benutze atan2 statt atan gegen Vorzeichenproblematik
  gamma_2alt(i) = atan2((-y_AC + l1 * sin(alpha) - l2 * sin(t120)), (-x_AC + l1 * cos(alpha) - l2 * cos(t120)));
end

err = 0;