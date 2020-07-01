% Return Structural Kinematic Parameters of the Robot 
% KAS5m3
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
% pkin_names (1x30) cell
%   Namen aller Kinematik-Parameter im Vektor `pkin`

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 17:47
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [v_mdh, sigma_mdh, mu_mdh, NL, NKP, NQJ, pkin_names] = KAS5m3_structural_kinematic_parameters()

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
NKP = 30;
% Anzahl der Minimalkoordinaten (für hybride Systeme)
NQJ = 6;
% Namen der Kinematikparameter
pkin_names = {'a10', 'a11', 'a12', 'a4', 'a5', 'a6', 'a9', 'd1', 'd2', 'd3', 'd7', 'd8', 'delta8s', 'delta9s', 'l11', 'l12', 'l13', 'l14', 'l17', 'l18', 'l20', 'l21', 'l22', 'l4', 'l5', 'l6', 'delta10s', 'delta12s', 'delta17s', 'delta18s'};
