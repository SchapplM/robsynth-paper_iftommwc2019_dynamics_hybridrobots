% Einstellungen für Modultests von KAS5m3 generieren:
% Enthält Definitionen für Parameter und zufällige Konfigurationen
%
% Ausgabe:
% TSS
%   Struktur mit Einstellungen für die Modultests

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-27 17:47
% Revision: bc59515823ab4a8d0fec19bf3bf92c32c39a66b0 (2020-06-27)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
% (C) Institut für Regelungstechnik, Universität Hannover

function TSS = KAS5m3_varpar_testfunctions_parameter()

%% General Definitions
NQJ = 6; % Number of generalized coordinates
NJ = 13; % number of joints (number of rows in Denavit-Hartenberg table)
NL = 14; % number of links (rigid bodies, including the base)
robot_name = 'KAS5m3'; % prefix in all function names and simulink models and blocks

%% Kinematik-Parameter definieren
% These parameters may be overwritten down in this script
a = rand(NJ,1); % Kinematic length of MDH notation
alpha = zeros(NJ,1); % Kinematic angle of MDH notation
d = rand(NJ,1); % Kinematic length of MDH notation
q_offset = zeros(NJ,1); % Kinematic angle of MDH notation
b = zeros(NJ,1); % Kinematic length of MDH notation
beta = zeros(NJ,1); % Kinematic angle of MDH notation

%% Zufällige Roboterkonfigurationen
% Werden in den Testfunktionen benutzt

n = 100;
% Gelenkwinkel und -zeitableitungen
% Aus Datei KAS5m3_kinematic_constraints_matlab.m eingefügt:
q_min = [-pi/2; -3*pi/4; 0; 0; 0; -pi];
q_max = [ pi/4;  pi/6; pi/2; pi/2; pi/2; pi];
Q = repmat(q_min',n,1) + rand(n,NQJ).*repmat(q_max'-q_min',n,1);
QD = (0.5-rand(n, NQJ))*pi;
QD(1:NQJ,:)=eye(NQJ);
QDD = (0.5-rand(n, NQJ))*pi;

% Gravitation
G = (0.5-rand(n, 3))*10;
G(:,~[1,1,1]) = 0;

% Basisposition
RB = (0.5-rand(n, 3))*10;

% Basisorientierung
% Zufällige Zeitableitung der Orientierungsdarstellung und ihrer
% Ableitungen
OB = (0.5-rand(n, 3))*pi;
OB(1,:) = 0;
OBD = (0.5-rand(n, 3))*pi;
OBDD = (0.5-rand(n, 3))*pi;

% Winkelgeschwindigkeit und Beschleunigung aus der Orientierungsdarstellung
% berechnen
VB = NaN(n,6);
AB = NaN(n,6);
VB(:,1:3) = (0.5-rand(n, 3))*10;
AB(:,1:3) = (0.5-rand(n, 3))*10;
for i = 1:n
  % Darstellung umwandeln.
  % TODO: Mit Transformationsmatrizen
  VB(i, 4:6) = eulxyzD2omega(OB(i,:)', OBD(i,:)');
  AB(i, 4:6) = eulxyzDD2omegaD(OB(i,:)', OBD(i,:)', OBDD(i,:)');
  
  % Probe:
  T_basevel = eulxyzjac(OB(i,:)');
  obd_test = T_basevel \ VB(i, 4:6)';
  
  if any(abs(obd_test-OBD(i,:)') > 1e-10)
    error('Orientierung und Basisgeschwindigkeit stimmen nicht überein');
  end
end




%% MDH-Parametereinträge auf Zufallswerte setzen
% Aus robot_matlabtmp_par.m
a1 = a(1);
a2 = a(2);
a3 = a(3);
a4 = a(4);
a5 = a(5);
a6 = a(6);
a7 = a(7);
a8 = a(8);
a9 = a(9);
a10 = a(10);
a11 = a(11);
a12 = a(12);
a13 = a(13);
alpha1 = alpha(1);
alpha2 = alpha(2);
alpha3 = alpha(3);
alpha4 = alpha(4);
alpha5 = alpha(5);
alpha6 = alpha(6);
alpha7 = alpha(7);
alpha8 = alpha(8);
alpha9 = alpha(9);
alpha10 = alpha(10);
alpha11 = alpha(11);
alpha12 = alpha(12);
alpha13 = alpha(13);
d1 = d(1);
d2 = d(2);
d3 = d(3);
d4 = d(4);
d5 = d(5);
d6 = d(6);
d7 = d(7);
d8 = d(8);
d9 = d(9);
d10 = d(10);
d11 = d(11);
d12 = d(12);
d13 = d(13);
qoffset1 = q_offset(1);
qoffset2 = q_offset(2);
qoffset3 = q_offset(3);
qoffset4 = q_offset(4);
qoffset5 = q_offset(5);
qoffset6 = q_offset(6);
qoffset7 = q_offset(7);
qoffset8 = q_offset(8);
qoffset9 = q_offset(9);
qoffset10 = q_offset(10);
qoffset11 = q_offset(11);
qoffset12 = q_offset(12);
qoffset13 = q_offset(13);
b1 = b(1);
b2 = b(2);
b3 = b(3);
b4 = b(4);
b5 = b(5);
b6 = b(6);
b7 = b(7);
b8 = b(8);
b9 = b(9);
b10 = b(10);
b11 = b(11);
b12 = b(12);
b13 = b(13);
beta1 = beta(1);
beta2 = beta(2);
beta3 = beta(3);
beta4 = beta(4);
beta5 = beta(5);
beta6 = beta(6);
beta7 = beta(7);
beta8 = beta(8);
beta9 = beta(9);
beta10 = beta(10);
beta11 = beta(11);
beta12 = beta(12);
beta13 = beta(13);

%% Werte für Kinematikparameter direkt eintragen

% Aus KAS5m3/kinematic_parameter_values.m

% Parameter für MDH-Notation
alpha2 = -pi/2;
alpha3 = pi/2;
alpha7 = pi/2;
alpha8 = pi/2;

a4 = 0.2574;
a5 = 0.0430;
a6 = 0.0430;
a9 = -0.0220;
a10 = 0.1300;
a11 = 0.0605;
a12 =  0.1855;

d1 = 0.1010;
d2 = 0.1480;
d3 = 0.0480;
d7 = 0.2658;
d8 = 0.0480;

% Parameter für kinematische Zwangsbedingungen
delta10s = atan(20e-3 / 24e-3);
delta12s = atan(43e-3 / 37e-3);
delta17s = 110*pi/180;
delta18s = 0 * pi/180;
delta8s = 0;% einziger Unterschied zu KAS5m4
delta9s = pi - delta10s - delta12s;
l11 = 38.5e-3;
l12 = 38.5e-3;
l13 = 185.3e-3;
l14 = 78.2e-3;
l17 = 54e-3;
l18 = 71e-3;
l20 = 157.4e-3;
l21 = 31.2e-3;
l22 = 56.7e-3;
l4 = 25e-3;
l5 = 0.258;
l6 = 0.132;



% Aus KAS5m3/parameter_kin_matlab.m
t1 = [a10; a11; a12; a4; a5; a6; a9; d1; d2; d3; d7; d8; delta8s; delta9s; l11; l12; l13; l14; l17; l18; l20; l21; l22; l4; l5; l6; delta10s; delta12s; delta17s; delta18s;];
pkin = t1;
if isempty(pkin)
  pkin = 0;%Platzhalter-Eingabe
end


%% MDH-Parametereinträge mit vorgegebenen Werten überschreiben
% Aus KAS5m3/parameters_d_matlab.m
t1 = [d1; d2; d3; 0; 0; 0; d7; d8; 0; 0; 0; 0; 0;];
d = t1;

% Aus KAS5m3/parameters_a_matlab.m
t1 = [0; 0; 0; a4; a5; a6; 0; 0; a9; a10; a11; a12; 0;];
a = t1;

% Aus KAS5m3/parameters_theta_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
theta = t1;

% Aus KAS5m3/parameters_b_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
b = t1;

% Aus KAS5m3/parameters_beta_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
beta = t1;

% Aus KAS5m3/parameters_alpha_matlab.m
t1 = pi / 0.2e1;
t2 = [0; -pi / 0.2e1; t1; 0; 0; 0; t1; t1; 0; 0; 0; 0; 0;];
alpha = t2;

% Aus KAS5m3/parameters_qoffset_matlab.m
t2 = pi / 0.2e1;
t1 = [-pi / 0.2e1; t2; 0; 0; 0; t2; 0; t2; 0; 0; pi; 0; pi;];
q_offset = t1;

% Aus KAS5m3/parameters_v_matlab.m
t1 = [0; 1; 2; 3; 4; 5; 6; 2; 8; 3; 10; 11; 12;];
v = uint8(t1);

% Aus KAS5m3/parameters_sigma_matlab.m
t1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];
sigma = t1;

% Aus KAS5m3/parameters_mu_matlab.m
t1 = [1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1;];
mu = t1;

% Aus KAS5m3/kinconstr_index_dependant_joints_matlab.m
t1 = [0; 0; 1; 0; 0; 0; 0; 1; 1; 1; 1; 1; 1;];
Ind_depjoints = t1;
%% Dynamik-Parameter definieren
rSges = rand(NL,3); % All center of mass coordinates in body frames
m = rand(NL,1); % masses of all links (are positive due to rand() function)
Ic_pa = rand(NL,3); % inertia of all links around their center of mass in principal axes
Icges = NaN(NL,6); % inertial of all links around their center of mass in body frame
for i = 1:NL
  R_pa = eulxyz2r(rand(3,1)); % random principal axes
  % inertia tensor in body frame: make sure the eigenvalues are positive and the tensor is positive definite
  Icges(i,:) = inertiamatrix2vector(R_pa*diag(Ic_pa(i,:))*R_pa');
end

% Parameter reduzieren, falls durch Benutzereingabe gefordert.
% Notwendig, damit Dynamikmodell konsistent ist mit den Eingabeparametern
% Das betrifft nur die baryzentrischen Parameter (par1). Die Inertialparameter (par2) werden daraus berechnet.
[m,rSges,Icges]=KAS5m3_dynamics_parameters_modification(pkin,m,rSges,Icges);
% Prüfe, ob die Reduktion konsistent ist. Es dürfen keine zyklischen
% Abhängigkeiten zwischen Masse, Schwerpunkt und Trägheit gesetzt sein.
[m2,rSges2,Icges2]=KAS5m3_dynamics_parameters_modification(pkin,m,rSges,Icges);
if any(abs([m-m2;rSges(:)-rSges2(:);Icges(:)-Icges2(:)])>1e-10)
  error('Bei zweifacher Durchführung der Parameterreduktion ändern sich die Parameter. Nicht konsistent!');
end
% Inertialparameter aus baryzentrischen Parametern berechnen
[mrSges, ... % first moment of all links (mass times center of mass)
 Ifges] = ... % second moment of all links (inertia around body frame origins)
  inertial_parameters_convert_par1_par2(rSges, Icges, m);

%% Set Outputs
TSS = struct('type', 'Test Settings Structure');
% Allgemeine Definitionen
TSS.NQJ = NQJ;
TSS.NJ = NJ;
TSS.NL = NL;
TSS.Ind_depjoints = logical(Ind_depjoints); % Binärindizes der abhängigen Gelenke
% Kinematische Zwangsbedingungen
TSS.NQJ = NQJ;
% Kinematikparameter
TSS.a = a;
TSS.alpha = alpha;
TSS.d = d;
TSS.q_offset = q_offset;
TSS.b = b;
TSS.beta = beta;
TSS.v = v;
TSS.pkin = pkin;
TSS.theta = theta;
TSS.sigma = sigma;
TSS.mu = mu;
TSS.q_min = q_min;
TSS.q_max = q_max;
% Dynamikparameter
TSS.m = m;
TSS.rSges = rSges;
TSS.Icges = Icges;
TSS.mrSges = mrSges;
TSS.Ifges = Ifges;
% Zufällige Konfigurationen für Modultests
TSS.n = n;
TSS.Q = Q;
TSS.QD = QD;
TSS.QDD = QDD;
TSS.RB = RB;
TSS.OB = OB;
TSS.OBD = OBD;
TSS.OBDD = OBDD;
TSS.VB = VB;
TSS.AB = AB;
TSS.G = G;
