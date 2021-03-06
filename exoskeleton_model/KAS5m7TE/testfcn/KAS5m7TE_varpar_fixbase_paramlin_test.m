% Teste parameterlineare Formen der Dynamik
% 
% Quellen:
% [Gautier1990] M. Gautier: Numerical calculation of the base inertial
% parameters of robots, ICRA 1990
% [Sousa2014] Sousa, C. D. and Cortesao, R.: Physical feasibility of robot
% base inertial parameter identification: A linear matrix inequality approach (2014)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
% (C) Institut für Regelungstechnik, Universität Hannover

clc
clear

NQJ = 5;
NJ = 21;
NL = 16;
robot_name = 'KAS5m7TE';

% Prüfe, ob kinematische Zwangsbedingungen vorliegen.
% Dann entfallen alle folgenden Tests (noch nicht implementiert)
thrfac = 1; % Faktor zur Erhöhung der Schwellwerte für Fehlererkennung
if NJ ~= NQJ
  KINCONSTR = true;
  fprintf('Einige Tests entfallen aufgrund der Existenz kinematischer Zwangsbedingungen\n');
  thrfac = 10; % bei ZB sind treten mehr Rechenfehler auf (da aufwändiger)
else
  KINCONSTR = false;
end

%% Parameter
TSS = KAS5m7TE_varpar_testfunctions_parameter();
for f = fields(TSS)'
  eval(sprintf('%s=TSS.%s;',f{1},f{1}));
end

if any(diff(TSS.v) ~= 1)
  fprintf('Einige Tests entfallen aufgrund der Baumstruktur (noch nicht implementiert)\n');
  TREE = 1;
else
  TREE = 0;
end

% Minimalparametervektor
if ~KINCONSTR && ~TREE
  MPV = KAS5m7TE_convert_par2_MPV_fixb(pkin, m, mrSges, Ifges);
end

% Vektor mit Inertialparametern
PV2fixb = NaN(10*(NL-1),1);
for i = 1:(NL-1)
  % different order:
  % Ifges_num_mdh: XX,YY,ZZ,XY,XZ,YZ
  % PV2: XX,XY,XZ,YY,YZ,ZZ
  PV2fixb((1:6) +10*(i-1)) = Ifges(i+1,[1,4,5,2,6,3]);
  PV2fixb((1:3) +10*(i-1)+6) = mrSges(i+1,:);
  PV2fixb(10*i) = m(i+1);
end

% Vektor der Inertialparametern mit Dynamikparametern der Basis (für Newton-Euler-Dynamik)
PV2fixbBase = NaN(10*NL,1);
for i = 1:NL
  % different order:
  % Ifges_num_mdh: XX,YY,ZZ,XY,XZ,YZ
  % PV2: XX,XY,XZ,YY,YZ,ZZ
  PV2fixbBase((1:6) +10*(i-1)) = Ifges(i,[1,4,5,2,6,3]);
  PV2fixbBase((1:3) +10*(i-1)+6) = mrSges(i,:);
  PV2fixbBase(10*i) = m(i);
end

% Liste der Namen der Dynamikparameter. Basis wird hier nicht betrachtet
% (Index "0"), aber bei den Körpern NL mitgezählt.
PV2fixb_Names = cell(10*(NL-1),1);
for i = 1:NL-1
  PV2fixb_Names{10*(i-1)+ 1} = sprintf('XX%d',i);
  PV2fixb_Names{10*(i-1)+ 2} = sprintf('XY%d',i);
  PV2fixb_Names{10*(i-1)+ 3} = sprintf('XZ%d',i);
  PV2fixb_Names{10*(i-1)+ 4} = sprintf('YY%d',i);
  PV2fixb_Names{10*(i-1)+ 5} = sprintf('YZ%d',i);
  PV2fixb_Names{10*(i-1)+ 6} = sprintf('ZZ%d',i);
  PV2fixb_Names{10*(i-1)+ 7} = sprintf('MX%d',i);
  PV2fixb_Names{10*(i-1)+ 8} = sprintf('MY%d',i);
  PV2fixb_Names{10*(i-1)+ 9} = sprintf('MZ%d',i);
  PV2fixb_Names{10*(i-1)+10} = sprintf('M%d', i);
end

%% Test kinetic Energy
for i = 1:n
  q = Q(i,:)';
  qD = QD(i,:)';
  T_func = KAS5m7TE_energykin_fixb_slag_vp1(q, qD, ...
      pkin, m, rSges, Icges);
  % calculate kinetic energy with optimized function
  if ~KINCONSTR && ~TREE
    % calculate kinetic energy with minimal parameter regressor
    t_regmin = KAS5m7TE_energykin_fixb_regmin_slag_vp(q, qD, ...
      pkin);
    T_mpv = t_regmin*MPV;
    if abs(T_mpv-T_func) > thrfac*1e6*eps(max(abs([T_mpv;T_func])))
      error('Kinetic Energy from base parameter vector does not match with direct form.');
    end
  end
  t_reg2 = KAS5m7TE_energykin_fixb_reg2_slag_vp(q, qD, ...
    pkin);
  T_pv2 = t_reg2*PV2fixb;
  delta_abs = T_pv2-T_func;
  delta_rel = delta_abs/T_func;
  if abs(delta_abs) > thrfac*1e6*eps(max(abs([T_pv2;T_func]))) && delta_rel > 1e-5
    error('Kinetic Energy from inertial parameter vector does not match with direct form.');
  end
end
fprintf('Tested kinetic energy for %d random joint angles for %s\n', ...
  n, robot_name);
%% Test Potential Energy
% Die potentielle Energie hat aufgrund des wegstreichens von Parametern
% einen unterschiedlichen statischen Anteil bei der Minimalparameterform
% Wähle die erste Winkelkonfiguration als virtuelle Null-Lage
% (Null-Werte können bei einigen geschlossenen kinematischen Ketten
% eine ungültige Lösung darstellen)
for i = 1:n
  q = Q(i,:)';
  g_base = G(i,:)';

  U_func = KAS5m7TE_energypot_fixb_slag_vp1(q, g_base, ...
    pkin, m, rSges);
  U_func0 = KAS5m7TE_energypot_fixb_slag_vp1(Q(1,:)', g_base, ...
    pkin, m, rSges);
  % calculate potential energy with optimized function
  if ~KINCONSTR && ~TREE
    % calculate potential energy with minimal regressor
    u_regmin = KAS5m7TE_energypot_fixb_regmin_slag_vp(q, g_base, ...
      pkin);
    u_regmin0 = KAS5m7TE_energypot_fixb_regmin_slag_vp(Q(1,:)', g_base, ...
      pkin);
    U_mpv = u_regmin*MPV;
    U_mpv0 = u_regmin0*MPV;
    if abs((U_mpv-U_mpv0)-(U_func-U_func0)) > thrfac*1e6*eps(max(abs([U_mpv;U_mpv0;U_func;U_func0])))
      error('Potential Energy from base parameter vector does not match with direct form.');
    end
  end

  % calculate potential energy with inertial parameters regressor
  u_reg2 = KAS5m7TE_energypot_fixb_reg2_slag_vp(q, g_base, ...
    pkin);
  u_reg20 = KAS5m7TE_energypot_fixb_reg2_slag_vp(Q(1,:)', g_base, ...
    pkin);
  U_pv2 = u_reg2*PV2fixb;
  U_pv20 = u_reg20*PV2fixb;
  if abs((U_pv2-U_pv20)-(U_func-U_func0)) > thrfac*1e6*eps(max(abs([U_pv2;U_pv20;U_func;U_func0])))
    error('Potential Energy from inertial parameter vector does not match with direct form.');
  end
end
fprintf('Tested potential energy for %d random joint angles for %s\n', ...
  n, robot_name);

%% Test Gravitational Load
for i = 1:n
  q = Q(i,:)';
  g_base = G(i,:)';
  taug_func = KAS5m7TE_gravloadJ_floatb_twist_slag_vp1(q, g_base, ...
    pkin, m, rSges);
  % calculate gravitational load with optimized function
  if ~KINCONSTR && ~TREE
    % calculate gravitational load with base parameter regressor
    taug_regmin = KAS5m7TE_gravloadJ_regmin_slag_vp(q, g_base, ...
      pkin);
    taug_mpv = taug_regmin*MPV;
    if any( abs(taug_mpv-taug_func) > thrfac*1e6*eps(max(abs(taug_func))) )
      error('Gravitation joint torque from base parameter vector does not match with direct form.');
    end
  end
  % calculate gravitational load with inertial parameter regressor
  taug_reg2 = KAS5m7TE_gravloadJ_reg2_slag_vp(q, g_base, ...
    pkin);
  taug_pv2 = taug_reg2*PV2fixb;
  delta_abs = taug_pv2-taug_func;
  delta_rel = delta_abs ./ taug_func;
  if any (abs(delta_abs) > thrfac*1e6*eps(max(abs(taug_func))) & delta_rel > 1e-4 )
    error('Gravitation joint torque from inertial parameter vector does not match with direct form.');
  end
end
fprintf('Tested gravitation joint torque for %d random joint angles for %s\n', ...
  n, robot_name);

%% Teste Coriolis-Vektor
for i = 1:n
  q = Q(i,:)';
  qD = QD(i,:)';
  tauc_func = KAS5m7TE_coriolisvecJ_fixb_slag_vp1(q, qD, ...NEW
    pkin, m, rSges, Icges);
  % calculate coriolis torques with optimized function
  if ~KINCONSTR && ~TREE
    % calculate coriolis torques with base parameter regressor
    tauc_regmin = KAS5m7TE_coriolisvecJ_fixb_regmin_slag_vp(q, qD, ...
      pkin);
    tauc_mpv = tauc_regmin*MPV;
    if any (abs(tauc_mpv-tauc_func) > thrfac*1e6*eps(1+max(abs([tauc_mpv;tauc_func]))))
      error('Coriolis joint torque from base parameter vector does not match with direct form.');
    end
  end
  % calculate coriolis torques with inertial parameter regressor
  tauc_reg2 = KAS5m7TE_coriolisvecJ_fixb_reg2_slag_vp(q, qD, ...
    pkin);
  tauc_pv2 = tauc_reg2*PV2fixb;
  delta_abs = tauc_pv2-tauc_func;
  delta_rel = delta_abs ./ max(abs(tauc_func));
  if any( abs(delta_abs) > thrfac*1e6*eps(1+max(abs([tauc_pv2;tauc_func]))) & delta_rel > thrfac*5e-3)
    error('Coriolis joint torque from inertial parameter vector does not match with direct form.');
  end
end
fprintf('Tested Coriolis joint torque for %d random joint angles for %s\n', ...
  n, robot_name);

%% Teste Coriolis-Matrix
for i = 1:n
  q = Q(i,:)';
  qD = QD(i,:)';
  Cq_func = KAS5m7TE_coriolismatJ_fixb_slag_vp2(q, qD, ...
    pkin, m, mrSges, Ifges);
  % calculate Coriolis-Matrix with optimized function
  if ~KINCONSTR && ~TREE
    % calculate Coriolis-Matrix with base parameter regressor
    Cq_regmin = KAS5m7TE_coriolismatJ_fixb_regmin_slag_vp(q, qD, ...
      pkin);
    Cq_mpv_vec = Cq_regmin*MPV;
    Cq_mpv = reshape(Cq_mpv_vec, NQJ, NQJ)';
    if any( abs(Cq_func(:)-Cq_mpv(:)) > thrfac*1e6*eps(1+max(abs(Cq_func(:)))) )
      error('Coriolis matrix from base parameter vector does not match with direct form.');
    end
  end
  % calculate Coriolis-Matrix with inertial parameter regressor
  Cq_reg2 = KAS5m7TE_coriolismatJ_fixb_reg2_slag_vp(q, qD, ...
    pkin);
  Cq_pv2_vec = Cq_reg2*PV2fixb;
  Cq_pv2 = reshape(Cq_pv2_vec, NQJ, NQJ)';
  delta_abs = Cq_func(:)-Cq_pv2(:);
  delta_rel = delta_abs ./ Cq_func(:);
  if any( abs(delta_abs) > thrfac*1e6*eps(1+max(abs(Cq_func(:)))) & delta_rel > 5e-5)
    error('Coriolis matrix from inertial parameter vector does not match with direct form.');
  end
end
fprintf('Tested Coriolis matrix for %d random joint angles for %s\n', ...
  n, robot_name);

%% Teste Massenmatrix
for i = 1:n
  q = Q(i,:)';
  Mq_func = KAS5m7TE_inertiaJ_slag_vp1(q, pkin, m, rSges, Icges);
  % calculate inertia with optimized function
  if ~KINCONSTR && ~TREE
    % calculate inertia with base parameter regressor
    Mq_regmin = KAS5m7TE_inertiaJ_regmin_slag_vp(q, pkin);
    Mq_mpv_vec = Mq_regmin*MPV;
    Mq_mpv = vec2symmat(Mq_mpv_vec);
    if any( abs(Mq_func(:)-Mq_mpv(:)) > thrfac*1e6*eps(max(abs(Mq_func(:)))) )
      error('Joint inertia matrix from base parameter vector does not match with direct form.');
    end
  end
  % calculate inertia with inertial parameter regressor
  Mq_reg2 = KAS5m7TE_inertiaJ_reg2_slag_vp(q, pkin);
  Mq_pv2_vec = Mq_reg2*PV2fixb;
  Mq_pv2 = vec2symmat(Mq_pv2_vec);
  delta_abs = abs(Mq_func(:)-Mq_pv2(:));
  delta_rel = delta_abs ./ Mq_func(:);
  if any( delta_abs > thrfac*1e6*eps(max(abs(Mq_func(:)))) & delta_rel > 1e-5)
    error('Joint inertia matrix from inertial parameter vector does not match with direct form.');
  end
end
fprintf('Tested Joint inertia matrix for %d random joint angles for %s\n', ...
  n, robot_name);

%% Teste Massenmatrix-Zeitableitung
for i = 1:n
  q = Q(i,:)';
  qD = QD(i,:)';
  MqD_func = KAS5m7TE_inertiaDJ_slag_vp1(q, qD, pkin, m, rSges, Icges);
  % calculate inertia time derivative with optimized function
  if ~KINCONSTR && ~TREE
    % calculate inertia time derivative with base parameter regressor
    MqD_regmin = KAS5m7TE_inertiaDJ_regmin_slag_vp(q, qD, pkin);
    MqD_mpv_vec = MqD_regmin*MPV;
    MqD_mpv = vec2symmat(MqD_mpv_vec);
    if any( abs(MqD_func(:)-MqD_mpv(:)) > thrfac*1e6*eps(1+max(abs(MqD_func(:)))) )
      error('Joint inertia matrix time derivative from base parameter vector does not match with direct form.');
    end
  end
  % calculate inertia time derivative with inertial parameter regressor
  MqD_reg2 = KAS5m7TE_inertiaDJ_reg2_slag_vp(q, qD, pkin);
  MqD_pv2_vec = MqD_reg2*PV2fixb;
  MqD_pv2 = vec2symmat(MqD_pv2_vec);
  delta_abs = MqD_func(:)-MqD_pv2(:);
  delta_rel = delta_abs ./ max(abs(MqD_func(:)));
  if any( abs(delta_abs) > thrfac*1e6*eps(1+max(abs(MqD_func(:)))) & delta_rel > thrfac*5e-3)
    error('Joint inertia matrix time derivative from inertial parameter vector does not match with direct form.');
  end
end
fprintf('Tested Joint inertia matrix time derivative for %d random joint angles for %s\n', ...
  n, robot_name);

%% Teste inverse Dynamik
for i = 1:n
  q = Q(i,:)';
  qD = QD(i,:)';
  qDD = QDD(i,:)';
  g_base = G(i,:)';
  tau_func = KAS5m7TE_invdynJ_fixb_slag_vp1(q, qD, qDD, g_base, ...
    pkin, m, rSges, Icges);
  % calculate inverse dynamics with optimized function
  if ~KINCONSTR && ~TREE
    % calculate inverse dynamics with base parameter regressor
    tau_regmin = KAS5m7TE_invdynJ_fixb_regmin_slag_vp(q, qD, qDD, g_base, pkin);
    tau_mpv = tau_regmin*MPV;
    if any( abs(tau_func-tau_mpv) > thrfac*1e6*eps(max(abs(tau_func))) )
      error('Inverse dynamics from base parameter vector does not match with direct form.');
    end
  end
  % calculate inverse dynamics with inertial parameter regressor
  tau_reg2 = KAS5m7TE_invdynJ_fixb_reg2_slag_vp(q, qD, qDD, g_base, pkin);
  tau_pv2 = tau_reg2*PV2fixb;
  delta_abs = tau_func-tau_pv2;
  delta_rel = delta_abs./tau_func;
  if any( abs(delta_abs) > thrfac*1e6*eps(max(abs(tau_func))) & delta_rel > thrfac*1e-4 )
    error('Inverse dynamics from inertial parameter vector does not match with direct form.');
  end
end
fprintf('Tested inverse dynamics for %d random joint angles for %s\n', ...
  n, robot_name);

%% Teste inverse Dynamik aus Newton-Euler
if ~KINCONSTR
  for i = 1:n
    q = Q(i,:)';
    qD = QD(i,:)';
    qDD = QDD(i,:)';
    g_base = G(i,:)';
    tauJ_neweul_func = KAS5m7TE_invdynJ_fixb_snew_vp2(q, qD, qDD, g_base, ...
      pkin, m, mrSges, Ifges);
    % calculate inverse dynamics with inertial parameter regressor
    tauJ_neweul_reg2 = KAS5m7TE_invdynJ_fixb_reg2_snew_vp(q, qD, qDD, g_base, ...
      pkin);
    tauJ_neweul_pv2 = tauJ_neweul_reg2*PV2fixb;
    if any( abs(tauJ_neweul_func-tauJ_neweul_pv2) > thrfac*1e6*eps(max(abs(tauJ_neweul_func))) )
      error('Inverse dynamics from inertial parameter vector from Newton-Euler does not match with direct form.');
    end
  end
  fprintf('Tested inverse dynamics from Newton-Euler for %d random joint angles for %s\n', ...
    n, robot_name);

  % Teste Basis-Reaktionskräfte aus inverser Dynamik aus Newton-Euler
  for i = 1:n
    q = Q(i,:)';
    qD = QD(i,:)';
    qDD = QDD(i,:)';
    g_base = G(i,:)';
    tauB_neweul_func = KAS5m7TE_invdynB_fixb_snew_vp2(q, qD, qDD, g_base, ...
      pkin, m, mrSges, Ifges);
    % calculate base forces with inertial parameter regressor
    tauB_neweul_reg2 = KAS5m7TE_invdynB_fixb_reg2_snew_vp(q, qD, qDD, g_base, ...
      pkin);
    tauB_neweul_pv2 = tauB_neweul_reg2*PV2fixbBase;
    if any( abs(tauB_neweul_func-tauB_neweul_pv2) > thrfac*1e6*eps(max(abs(tauB_neweul_func))) )
      error('Base forces from inertial parameter vector from Newton-Euler does not match with direct form.');
    end
  end
  fprintf('Tested base forces from Newton-Euler for %d random joint angles for %s\n', ...
    n, robot_name);

  % Teste Schnittkräfte/-momente aus Newton-Euler
  for i = 1:n
    q = Q(i,:)';
    qD = QD(i,:)';
    qDD = QDD(i,:)';
    g_base = G(i,:)';
    f_neweul_func = KAS5m7TE_invdynf_fixb_snew_vp2(q, qD, qDD, g_base, ...
      pkin, m, mrSges, Ifges);
    % calculate cutting forces with inertial parameter regressor
    f_neweul_reg2 = KAS5m7TE_invdynf_fixb_reg2_snew_vp(q, qD, qDD, g_base, ...
      pkin);
    f_neweul_pv2 = f_neweul_reg2*PV2fixbBase;
    f_neweul_pv2 = reshape(f_neweul_pv2,[3,16]);
    if any( abs(f_neweul_func(:)-f_neweul_pv2(:)) > thrfac*1e6*eps(max(abs(f_neweul_func(:)))) )
      error('Cutting forces from inertial parameter vector from Newton-Euler does not match with direct form.');
    end
  end
  fprintf('Tested cutting forces inverse dynamics from Newton-Euler for %d random joint angles for %s\n', ...
    n, robot_name);
  for i = 1:n
    q = Q(i,:)';
    qD = QD(i,:)';
    qDD = QDD(i,:)';
    g_base = G(i,:)';
    m_neweul_func = KAS5m7TE_invdynm_fixb_snew_vp2(q, qD, qDD, g_base, ...
      pkin, m, mrSges, Ifges);
    % calculate cutting torques with inertial parameter regressor
    m_neweul_reg2 = KAS5m7TE_invdynm_fixb_reg2_snew_vp(q, qD, qDD, g_base, ...
      pkin);
    m_neweul_pv2 = m_neweul_reg2*PV2fixbBase;
    m_neweul_pv2 = reshape(m_neweul_pv2,[3,16]);
    if any( abs(m_neweul_func(:)-m_neweul_pv2(:)) > thrfac*1e6*eps(max(abs(m_neweul_func(:)))) )
      error('Cutting torques from inertial parameter vector from Newton-Euler does not match with direct form.');
    end
  end
  fprintf('Tested cutting torques inverse dynamics from Newton-Euler for %d random joint angles for %s\n', ...
    n, robot_name);
end

%% Teste Funktionsaufruf mit übergebenem Regressor gegen Regressormatrix
% Die direkte Übergabe des Dynamik-Parametervektors ist effizienter als die
% nachträgliche Multiplikation des Regressors

if ~KINCONSTR && ~TREE
  for i = 1:n
    q = Q(i,:)';
    qD = QD(i,:)';
    qDD = QDD(i,:)';
    g_base = G(i,:)';

    taug_regmin = KAS5m7TE_gravloadJ_regmin_slag_vp(q, g_base, pkin);
    taug_mpv = taug_regmin*MPV;
    taug_func = KAS5m7TE_gravloadJ_floatb_twist_mdp_slag_vp(q, g_base, pkin, MPV);
    if any( abs(taug_func-taug_mpv) > thrfac*1e6*eps(max(abs(taug_func))) )
      error('Grav Load from base parameter regressor does not match with pre-multiplied calculation.');
    end

    tauc_regmin = KAS5m7TE_coriolisvecJ_fixb_regmin_slag_vp(q, qD, pkin);
    tauc_mpv = tauc_regmin*MPV;
    tauc_func = KAS5m7TE_coriolisvecJ_fixb_mdp_slag_vp(q, qD, pkin, MPV);
    if any( abs(tauc_func-tauc_mpv) > thrfac*1e6*eps(max(abs(tauc_func))) )
      error('Coriolis-vector from base parameter regressor does not match with pre-multiplied calculation.');
    end

    Mq_regmin = KAS5m7TE_inertiaJ_regmin_slag_vp(q, pkin);
    Mq_mpv_vec = Mq_regmin*MPV;
    Mq_mpv = vec2symmat(Mq_mpv_vec);
    Mq_func = KAS5m7TE_inertiaJ_mdp_slag_vp(q, pkin, MPV);
    if any( abs(Mq_func(:)-Mq_mpv(:)) > thrfac*1e6*eps(max(abs(Mq_func(:)))) )
      error('Inertia matrix from base parameter regressor does not match with pre-multiplied calculation.');
    end

    tau_regmin = KAS5m7TE_invdynJ_fixb_regmin_slag_vp(q, qD, qDD, g_base, ...
      pkin);
    tau_mpv = tau_regmin*MPV;
    tau_func = KAS5m7TE_invdynJ_fixb_mdp_slag_vp(q, qD, qDD, g_base, pkin, MPV);
    if any( abs(tau_func-tau_mpv) > thrfac*1e6*eps(max(abs(tau_func))) )
      error('Inverse dynamics from base parameter regressor does not match with pre-multiplied calculation.');
    end
  end
end
fprintf('Tested inverse dynamics functions with parameter vector input for %d random joint angles for %s\n', ...
  n, robot_name);

%% Prüfe Trajektorien-Funktionen
if ~KINCONSTR && ~TREE
  % Zeitreihe der Regressor-Matrizen
  RV_Traj = KAS5m7TE_invdynJ_fixb_regmin_slag_vp_traj(Q, QD, QDD, g_base, pkin);
  % Damit Zeitreihe der Momente berechnen
  TAU_vr = KAS5m7TE_invdynJ_fixb_mdp_slag_vr_traj(RV_Traj, MPV);
  % Zeitreihe der Momente über direkten Funktionsaufruf
  TAU_vp = KAS5m7TE_invdynJ_fixb_mdp_slag_vp_traj(Q, QD, QDD, g_base, pkin, MPV);
  % Vergleich der Ergebnisse
  test_vr_vp = TAU_vr - TAU_vp;
  if max(abs(test_vr_vp(:))) > 1e-10
    error('Keine Übereinstimmung zwischen Traj.-Dyn.-Funktionen mit Regressor und MPV als Eingang');
  end
  % Beteiligte Funktionen kompilieren
  matlabfcn2mex({ 'KAS5m7TE_invdynJ_fixb_regmin_slag_vp_traj', ...
                  'KAS5m7TE_invdynJ_fixb_mdp_slag_vr_traj', ...
                  'KAS5m7TE_invdynJ_fixb_mdp_slag_vp_traj'});
end

%% Abbruch bei kinematischen Zwangsbedingungen
% TODO: Test durchführen, wenn Minimalparameterform dafür implementiert
if KINCONSTR || TREE
  return %#ok<*UNRCH>
end

%% Prüfe Minimalparametervektor gegen Matrix mit Inertialparameterzuordnung
% [Sousa2014] Gl. (38)
[K, K_d, P_b, P_d] = KAS5m7TE_PV2_MPV_transformations_fixb(pkin);

MPV_test = K*PV2fixb;
test = MPV - MPV_test;
if any(abs(test(:)) > 1e-10)
  error('Minimalparametervektor stimmt nicht mit MPV aus Inertialparametermatrix überein');
end
% MPV-Transformationsmatrix aus Teilmatrizen aus [Sousa2014] aufbauen und testen
K_test = (P_b' + K_d*P_d'); % [Sousa2014] Gl. (39)
Test = abs(K-K_test);
if any(Test(:)>1e-12)
  error('Test [Sousa2014] Gl. (38) fehlgeschlagen!');
end
fprintf('Transformationen des fixb-Minimalparametervektors erfolgreich getestet\n');

%% Teste Minimalparameterform gegen numerische Berechnung nach [Gautier1990]
% Matrix mit Samples der Regressormatrix erstellen ([Gautier1990], Gl. (8))
% Zufällige Werte, nach [Gautier1990], Kap. 5-1-1
W_g_invdyn = NaN(NQJ*n, length(PV2fixb)); % Informationsmatrix des Inertialparameter-Regressors. Zum Testen des Rangs
W_g_energy = NaN(n, length(PV2fixb)); % Gleiche Matrix, nur mit Energie, statt Dynamik
for i = 1:n
  q = Q(i,:)';
  qD = QD(i,:)';
  qDD = QDD(i,:)';
  g_base = G(i,:)';
  % [Gautier1990] Gl. (7)
  tau_reg2 = KAS5m7TE_invdynJ_fixb_reg2_slag_vp(q, qD, qDD, g_base, ...
    pkin);
  % Füllen der Matrix: [Gautier1990], Kap. 2-3
  % [Gautier1990] Gl. (8) (Füllen mit Dynamik-Regressor)
  W_g_invdyn((i-1)*NQJ+1:i*NQJ,:) = tau_reg2;
  % [Gautier1990] Gl. (6) (Füllen mit Energie-Regressor)
  % Der konstante Anteil der potentiellen Energie muss abgezogen werden
  u_reg2 = KAS5m7TE_energypot_fixb_reg2_slag_vp(q, g_base, pkin);
  u_reg20 = KAS5m7TE_energypot_fixb_reg2_slag_vp(Q(1,:)', g_base, pkin);
  t_reg2 = KAS5m7TE_energykin_fixb_reg2_slag_vp(q, qD, pkin);
  W_g_energy(i,:) = t_reg2 + u_reg2 - u_reg20;
end
% Struktur mit Eingabedaten für Algorithmus generieren
[Ksym, K_dsym, P1sym, P2sym] = KAS5m7TE_PV2_MPV_transformations_fixb(pkin);
plin_num_test_struct = struct('W', W_g_energy, 'PV2', PV2fixb, 'MPV', MPV, ...
  'P1sym', P1sym, 'P2sym', P2sym, 'K_dsym', K_dsym, 'Ksym', Ksym, ...
  'NQ', NQJ, 'PV2_Names', {PV2fixb_Names});
% Aufruf des Test-Algorithmus als Funktion (der Test kann genauso für
% Floating Base durchgeführt werden und wird deshalb ausgelagert. Die
% Übergabeargumente unterscheiden sich nur minimal)
robot_paramlin_test_reg_num(plin_num_test_struct);

% Test genauso für inverse Dynamik durchführen
plin_num_test_struct.W = W_g_invdyn;
robot_paramlin_test_reg_num(plin_num_test_struct);

fprintf('Symbolisch berechnete Minimalparameterform gegen numerische Form geprüft\n');


