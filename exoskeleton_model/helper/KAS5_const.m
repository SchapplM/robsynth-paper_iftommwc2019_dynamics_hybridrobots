% Definition von Konstanten für das KAS5
% 
% Ausgabe:
% S
%   Struktur mit verschiedenen konstanten Eigenschaften
%   .WN
%     Namen der einzelnen Hilfswinkel

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function S = KAS5_const(modell)
version = uint8(5);
if nargin == 0
  modell = uint8(2);
end
S = struct('KAS_version', uint8(5), 'KAS_modell', uint8(modell));

% Namen der Winkel
WinkelNamen = cell(24,1);
WinkelNamen(1:5) = {'alpha', 'beta1', 'beta2', 'gamma3', 'gamma5'};
for i = 1:18
  WinkelNamen{5+i} = sprintf('delta%d', i);
end
WinkelNamen{24} = sprintf('rho3');

S.WN = WinkelNamen;

% Namen der Gelenke
S.GN = cell(7,1);
for i = 1:7
  S.GN{i} = sprintf('G%d', i);
end

% Namen der Punkte
S.PN = {'A'; 'B'; 'C'; 'D'; 'E'; 'F'; 'G'; 'H'};

% Anzahl der verallgemeinerten Koordinaten
if modell == 2
  N = 7;
elseif modell == 3
  N = 6;  
elseif modell == 4
  N = 7;  
elseif modell == 5
  N = 5; 
elseif modell == 6
  N = 6;  
else
  error('KAS%d_m%d not implemented yet', version, modell);
end
S.N = N;

% Winkelgrenzen
if modell == 2
  % Keine Grenzen aus mathematischer Sicht
  q_min = ones(7,1)*(-Inf);
  q_max = ones(7,1)*(Inf);
elseif modell == 3
  q_min = [-pi/2; -3*pi/4; 0; 0; 0; -pi];
  q_max = [ pi/4;  pi/6; pi/2; pi/2; pi/2; pi];
elseif modell == 4
  % Gleiche Grenzen wie KAS5m3, zusätzliche Grenze für Schultergelenk (q3)
  q_min = [-pi/2; -pi/4; -pi/4; 0; 0; 0; -pi];
  q_max = [ pi/4; pi/2; pi; pi/2; pi/2; pi/2; pi];
elseif modell == 5
  % wie Modell 3, nur ohne vorletzten Eintrag
  q_min = [-pi/2; -3*pi/4; 0; 0; -pi];
  q_max = [ pi/4;  pi/6; pi/2; pi/2; pi];
elseif modell == 6
  % wie Modell 4, nur ohne vorletzten Eintrag
  q_min = [-pi/2; -pi/4; -pi/4; 0; 0; -pi];
  q_max = [ pi/4; pi/2; pi; pi/2; pi/2; pi];
end
S.q_min = q_min;
S.q_max = q_max;

% Gelenknamen
if modell == 2 || modell == 4
  S.JN = {'rho1', 'rho2', 'rho3', 'rho4', 'rho5', 'rho6', 'rho7'};
elseif modell == 3
  S.JN = {'rho1', 'rho2', 'rho4', 'rho5', 'rho6', 'rho7'};
elseif modell == 5
  S.JN = {'rho1', 'rho2', 'rho4', 'rho5', 'rho7'};
elseif modell == 6
  S.JN = {'rho1', 'rho2', 'rho3', 'rho4', 'rho5', 'rho7'};
else
  error('KAS%d_m%d not implemented yet', version, modell);
end

% Umwandlung der Indizes zwischen den Modellen
% Die folgenden Umwandlungen setzen voraus, dass die Gelenkwinkel zwischen
% den Modellen konsistent sind, also dass immer alle Zwangsbedingungen
% eingehalten werden. Sonst ist die Konvertierung nicht möglich.
S.I_m2_m3 = [1 2 4 5 6 7]; % Wende diese Indizes auf m2-Vektoren an, um m3 zu erhalten
S.I_m3_m5 = [1 2 3 4 6];   % Wende diese Indizes auf m3-Vektoren an, um m5 zu erhalten
S.I_m2_m5 = [1 2 4 5 7];   % Wende diese Indizes auf m2-Vektoren an, um m5 zu erhalten
S.I_m2_m6 = [1 2 3 4 5 7];   % Wende diese Indizes auf m2-Vektoren an, um m5 zu erhalten
% Indizes, um verallgemeinerte Koordinaten dieses Modells (mi) in die eines
% anderen Modells umzuwandeln (m2, m3, m5)
if modell == 2
  S.I_mi_m2 = true(1,5);
  S.I_mi_m3 = S.I_m2_m3;
  S.I_mi_m5 = S.I_m2_m5;
elseif modell == 3
  S.I_mi_m3 = true(1,6);
  S.I_mi_m5 = S.I_m3_m5;
end