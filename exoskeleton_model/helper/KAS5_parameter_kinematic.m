% Kinematische Parameter des KAS5
% 
% Eingabe:
% modellnr
%   Nummer des Modells (z.B. 7 für KAS5m7)
% unconstrained
%   Maker, ob Kinematikparameter für Modell ohne Zwangsbedingungen
%   ausgegeben werden. Bezieht sich nur auf pkin, dass sich auf Ergebnisse
%   der Maple-Toolbox bezieht.

% 
% Ausgabe:
% l
%   Kinematische Längen laut Skizze
% w
%   Konstante Winkel, laut Skizze
% pkin
%   Kinematik-Parameter für aus Maple-Toolbox generierten Code

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [l, w, pkin] = KAS5_parameter_kinematic(modellnr, unconstrained)

if nargin == 1
  unconstrained = false;
end

%% Lade SimMechanics - Solidworks Parameter
% Händisch aus data/KAS5_simm_parameter_kinematic_data.m kopiert
% w(4) und w(6) händisch auf glatte Werte gebracht.
l = [0.10100000000000008; 0.1479788033874222; 0.047999999999999772; 0.022000636309252641; ...
     0.25743477532274756; 0.12999999999999992; 0.027294161844986425; 0.022745134870822012; ...
     0.041412701525616935; 0.044060842421616946; 0.042999999999999892; ...
     0.042999999999999629; 0.18550876399893654; 0.080000000000000016; 0.26579720329431705; ...
     NaN; 0.056335841084723323; 0.071012565588095974; 0.049991166791160169; ...
     0.15815182578775372; 0.035529036450658164; 0.060467922757048584; NaN];
w = [1.6304841265086341; 0.694738276196703; 0.81637025088445614; pi/2; ...
     0; 0; NaN];
w(7) = atan2(l(17), l(18)); % delta20
l(23) = sqrt(l(18)^2+l(17)^2);

if nargout == 3 && nargin == 0
  error('Zur Ausgabe der Kinematikparameter muss die Modellnummer gegeben werden');
end
pkin=NaN(30,1);
if nargout == 3
  delta8 = w(5);
  delta9 = w(1);
  delta10 = w(2);
  delta12 = w(3);
  delta17 = w(4);
  delta18 = w(6);
  delta20 = w(7);
  
  l1 = l(1);
  l2 = l(2);
  l3 = l(3);
  l4 = l(4);
  l5 = l(5);
  l6 = l(6);
  l7 = l(7);
  l8 = l(8);
  l9 = l(9);
  l10 = l(10);
  l11 = l(11);
  l12 = l(12);
  l13 = l(13);
  l14 = l(14);
  l15 = l(15);
  l16 = l(16);
  l17 = l(17);
  l18 = l(18);
  l19 = l(19);
  l20 = l(20);
  l21 = l(21);
  l22 = l(22);
  l23 = l(23);
  
  if modellnr == 5
    pkin=NaN(30,1);
    pkin(1,1) = l(6);%a(10);
    pkin(2,1) = l(22);%a(11);
    pkin(3,1) = l(13);%a(12);
    pkin(4,1) = l(5);%a(4);
    pkin(5,1) = l(11);%a(5);
    pkin(6,1) = l(12);%a(6);
    pkin(7,1) = -l(4);%a(9);
    pkin(8,1) = l(1);%d(1);
    pkin(9,1) = l(2);%d(2);
    pkin(10,1) = l(3);%d(3);
    pkin(11,1) = l(15);%d(7);
    pkin(12,1) = l(3);%d(8);
    pkin(13,1) = delta8;
    pkin(14,1) = delta9;
    pkin(15,1) = l(11);
    pkin(16,1) = l(12);
    pkin(17,1) = l(13);
    pkin(18,1) = l(14);
    pkin(19,1) = l(17);
    pkin(20,1) = l(18);
    pkin(21,1) = l(20);
    pkin(22,1) = l(21);
    pkin(23,1) = l(22);
    pkin(24,1) = l(4);
    pkin(25,1) = l(5);
    pkin(26,1) = l(6);
    pkin(27,1) = delta10;
    pkin(28,1) = delta12;
    pkin(29,1) = delta17;
    pkin(30,1) = delta18;
  elseif modellnr == 6
    pkin=NaN(29,1);
    pkin(1,1) = l(6);%a(10);
    pkin(2,1) = l(22);%a(11);
    pkin(3,1) = l(13);%a(12);
    pkin(4,1) = l(5);%a(4);
    pkin(5,1) = l(11);%a(5);
    pkin(6,1) = l(12);%a(6);
    pkin(7,1) = -l(4);%a(9);
    pkin(8,1) = l(1);%d(1);
    pkin(9,1) = l(2);%d(2);
    pkin(10,1) = l(3);%d(3);
    pkin(11,1) = l(15);%d(7);
    pkin(12,1) = l(3);%d(8);
    pkin(13,1) = delta9;
    pkin(14,1) = l(11);
    pkin(15,1) = l(12);
    pkin(16,1) = l(13);
    pkin(17,1) = l(14);
    pkin(18,1) = l(17);
    pkin(19,1) = l(18);
    pkin(20,1) = l(20);
    pkin(21,1) = l(21);
    pkin(22,1) = l(22);
    pkin(23,1) = l(4);
    pkin(24,1) = l(5);
    pkin(25,1) = l(6);
    pkin(26,1) = delta10;
    pkin(27,1) = delta12;
    pkin(28,1) = delta17;
    pkin(29,1) = delta18;
  elseif modellnr == 7
    if unconstrained
      % Siehe KAS5m7OL_varpar_testfunctions_parameter.m
      pkin = [delta17;           delta20; delta8; delta9; l1; l11; l12; l13; l14; l15; l2; l20; l21; l22; l23; l3; l4; l5; l6;];
%       pkin = [delta17; delta18; delta20; delta8; delta9; l1; l11; l12; l13; l14; l15; l2; l20; l21; l22; l23; l3; l4; l5; l6;];
    else
      % Siehe KAS5m7TE_varpar_testfunctions_parameter.m
      pkin=NaN(24,1);
      pkin(1,1) = delta10;
      pkin(2,1) = delta12;
      pkin(3,1) = delta17;
      pkin(4,1) = delta18;
      pkin(5,1) = delta20;
      pkin(6,1) = delta8;
      pkin(7,1) = delta9;
      pkin(8,1) = l1;
      pkin(9,1) = l11;
      pkin(10,1) = l12;
      pkin(11,1) = l13;
      pkin(12,1) = l14;
      pkin(13,1) = l15;
      pkin(14,1) = l17;
      pkin(15,1) = l18;
      pkin(16,1) = l2;
      pkin(17,1) = l20;
      pkin(18,1) = l21;
      pkin(19,1) = l22;
      pkin(20,1) = l23;
      pkin(21,1) = l3;
      pkin(22,1) = l4;
      pkin(23,1) = l5;
      pkin(24,1) = l6;
    end
  else
    error('Fuer dieses Modell noch nicht implementiert');
  end
end
   
return

%% konstante Längen
l = NaN(22,1);

l(1) = 104.04e-3; % sp, 16.11.2015
l(2) = sqrt(0.084^2 + 0.168^2); % sp, 16.11.2015. Projektion auf falsche Ebene.
l(3) = 60e-3; % sp, 16.11.2015. Grob geschätzt. TODO: Genauer
l(4) = 25e-3; % sp, 16.11.2015
l(5) = 0.258; % sp, 16.11.2015
l(6) = 0.132; % sp, 16.11.2015
l(7) = 24e-3; % sp, 16.11.2015. Grob geschätzt. TODO: Genauer, richtige Projektionslinie
l(8) = 20e-3; % sp, 16.11.2015. Grob geschätzt. TODO: Genauer, richtige Projektionslinie
l(9) = 37e-3; % sp, 16.11.2015. Grob geschätzt. TODO: Genauer, richtige Projektionslinie
l(10) = 43e-3; % sp, 16.11.2015. Grob geschätzt. TODO: Genauer, richtige Projektionslinie
l(11) = 38.5e-3; % sp, 16.11.2015
l(12) = 38.5e-3; % sp, 16.11.2015 (nur geschätzt, nicht abgelesen)
l(13) = 185.3e-3; % sp, 16.11.2015
l(14) = 78.2e-3; % sp, 16.11.2015
l(15) = 0.268; % sp, 16.11.2015
l(16) = NaN; % Diese Länge ist variabel!
l(17) = 54e-3; % sp, 16.11.2015. Grob geschätzt. TODO: Genauer, richtige Projektionsebene
l(18) = 71e-3; % sp, 16.11.2015. Grob geschätzt. TODO: Genauer, richtige Projektionsebene
l(19) = 66e-3; % sp, 16.11.2015. Grob geschätzt. TODO: Werkzeugaufnahme genauer definieren
l(20) = 157.4e-3; % sp, 16.11.2015

% Hilfslängen
l(21) = sqrt(l(7)^2 + l(8)^2);
l(22) = sqrt(l(9)^2 + l(10)^2);

%% Konstante Winkel
% Hilfswinkel
delta10 = atan(l(8) / l(7));
delta12 = atan(l(10) / l(9));

% Gl. (6) in Aufzeichnungen. Dieser Zusammenhang muss immer erfüllt sein!
delta9 = pi - delta10 - delta12;

delta17 = 110*pi/180; % sp, 20.11.2015: Grobe Messung am CAD-Modell (Geodreieck auf Bildschirm). Annahme, dass Zahnrad symmetrisch). TODO: Überprüfen.

delta8 = 0 * pi/180; % sp, 20.11.2015, geschätzt aus CAD-Modell. TODO: richtig messen!

w = NaN(5,1);
w(1) = delta9;
w(2) = delta10;
w(3) = delta12;
w(4) = delta17;

% Winkel delta8 ist bei Modell m3 konstant.
w(5) = delta8;