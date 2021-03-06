% Return Structural Kinematic Parameters of the Robot 
% KAS5m7OL
%
% Output:
% v_mdh [21x1]
%   Vorgänger-Indizes (0=Basis)
% sigma_mdh [21x1]
%   Dregelenk = 0, Schubgelenk = 1
% mu_mdh [21x1]
%   Aktives Gelenk = 1, Passiv = 0
% NL [1x1]
%   Anzahl der Starrkörper (inklusive Basis)
% NKP [1x1]
%   Anzahl der Kinematikparameter im Vektor `pkin`
% NQJ [1x1]
%   Anzahl der Minimalkoordinaten der kinematischen Kette
% pkin_names (1x19) cell
%   Namen aller Kinematik-Parameter im Vektor `pkin`

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:16
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [v_mdh, sigma_mdh, mu_mdh, NL, NKP, NQJ, pkin_names] = KAS5m7OL_structural_kinematic_parameters()

% Aus parameters_mdh_v_matlab.m
t1 = [0; 1; 2; 3; 4; 5; 6; 2; 8; 3; 10; 4; 12; 13; 14; 10; 13; 6; 9; 11; 15;];
v_mdh = uint8(t1);

% Aus parameters_mdh_sigma_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 2; 0; 0; 0; 0; 2; 0; 1; 0; 0; 0; 2; 2; 2;];
sigma_mdh = t1;

% Aus parameters_mdh_mu_matlab.m
t1 = [1; 1; 0; 1; 1; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
mu_mdh = t1;

% Aus Roboterdefinition
% Anzahl der Robotersegmente (inkl Basis)
NL = 16;
% Anzahl der Kinematikparameter
NKP = 19;
% Anzahl der Minimalkoordinaten (für hybride Systeme)
NQJ = 13;
% Namen der Kinematikparameter
pkin_names = {'delta17', 'delta20', 'delta8', 'delta9', 'l1', 'l11', 'l12', 'l13', 'l14', 'l15', 'l2', 'l20', 'l21', 'l22', 'l23', 'l3', 'l4', 'l5', 'l6'};
