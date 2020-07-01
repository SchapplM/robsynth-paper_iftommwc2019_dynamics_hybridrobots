% Return Structural Kinematic Parameters of the Robot 
% KAS5m5OL
%
% Output:
% v_mdh [13x1]
%   Vorgänger-Indizes (0=Basis)
% sigma_mdh [13x1]
%   Dregelenk = 0, Schubgelenk = 1
% mu_mdh [13x1]
%   Aktives Gelenk = 1, Passiv = 0
% NL [1x1]
%   Anzahl der Starrkörper (inklusive Basis)
% NKP [1x1]
%   Anzahl der Kinematikparameter im Vektor `pkin`
% NQJ [1x1]
%   Anzahl der Minimalkoordinaten der kinematischen Kette
% pkin_names (1x12) cell
%   Namen aller Kinematik-Parameter im Vektor `pkin`

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 19:20
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [v_mdh, sigma_mdh, mu_mdh, NL, NKP, NQJ, pkin_names] = KAS5m5OL_structural_kinematic_parameters()

% Aus parameters_mdh_v_matlab.m
t1 = [0; 1; 2; 3; 4; 5; 6; 2; 8; 3; 10; 11; 12;];
v_mdh = uint8(t1);

% Aus parameters_mdh_sigma_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
sigma_mdh = t1;

% Aus parameters_mdh_mu_matlab.m
t1 = [1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1;];
mu_mdh = t1;

% Aus Roboterdefinition
% Anzahl der Robotersegmente (inkl Basis)
NL = 14;
% Anzahl der Kinematikparameter
NKP = 12;
% Anzahl der Minimalkoordinaten (für hybride Systeme)
NQJ = 13;
% Namen der Kinematikparameter
pkin_names = {'a10', 'a11', 'a12', 'a4', 'a5', 'a6', 'a9', 'd1', 'd2', 'd3', 'd7', 'd8'};
