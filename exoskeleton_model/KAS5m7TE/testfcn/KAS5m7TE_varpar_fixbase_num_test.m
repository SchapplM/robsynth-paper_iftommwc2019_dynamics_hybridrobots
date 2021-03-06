% Test function for numeric calculations of dynamics
% * Symbolic calculations from maple (..._slag_...)
% * numeric general robotics functions from irt-matlab-toolbox (robot_tree_ ... _nnew...)
% * numeric robot-specific functions from template of this toolbox (KAS5m7TE__...nnew_...)
% with fixed base model

% Quelle: HybrDyn-Toolbox
% Datum: 2020-05-12 08:05
% Revision: 2d0abd6fcc3afe6f578a07ad3d897ec57baa6ba1 (2020-04-13)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (C) Institut für Regelungstechnik, Universität Hannover

clc
clear

NQJ = 5;
NJ = 21;
NL = 16;
robot_name = 'KAS5m7TE';

% Prüfe, ob kinematische Zwangsbedingungen vorliegen.
% Dann entfallen alle folgenden Tests (noch nicht implementiert)
if NJ ~= NQJ
  KINCONSTR = true;
  fprintf('Tests entfallen aufgrund der Existenz kinematischer Zwangsbedingungen\n');
  return
end

%% Parameter
TSS = KAS5m7TE_varpar_testfunctions_parameter();
for f = fields(TSS)'
  eval(sprintf('%s=TSS.%s;',f{1},f{1}));
end
if any(sigma)
  fprintf('Schubgelenke noch nicht implementiert. Überspringe Test des CRBA.\n');
end

MPV = KAS5m7TE_convert_par2_MPV_fixb(pkin, m, mrSges, Ifges);
RotAx_i = repmat([0;0;1]', NQJ, 1); % MDH: Drehachse ist immer die z-Achse (des gedrehten Körpers)

%% Test gegen numerische Funktionen der fixed base Dynamik
for i = 1:n
  qJ = Q(i,:)';
  qJD = QD(i,:)';
  qJDD = QDD(i,:)';
  g_base = G(i,:)';

  % Gelenk-Transformationsmatrizen in Eingabeformat der Modul-Funktion umwandeln
  T = KAS5m7TE_joint_trafo_rotmat_mdh_sym_varpar(qJ, pkin);
  T_stack = NaN(3*NQJ,4);
  for ii = 1:NQJ
      T_stack(3*ii-2:3*ii,:) = T(1:3,:,ii);
  end

  % Gravitation
  taug_sym = KAS5m7TE_gravloadJ_floatb_twist_slag_vp1(qJ, g_base, ...
    pkin, m, rSges);
  taug_num1 = robot_tree_invdyn_floatb_eulxyz_mdh_nnew_vp1(qJ, zeros(NJ,1), zeros(NJ,1), zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)], ...
    alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges, Icges);
  if any( abs(taug_sym-taug_num1(7:end)) > 1e-10)
    error('Fehler in numerisch berechneter Gravitationslast (Methode 1): Stimmt nicht mit symbolisch berechneter überein.');
  end
  taug_num2 = robot_tree_gravload_floatb_eulxyz_mdh_nnew_vp1(qJ, zeros(3,1), g_base, ...
    alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges);
  if any( abs(taug_sym-taug_num2(7:end)) > 1e-10)
    error('Fehler in numerisch berechneter Gravitationslast (Methode 2): Stimmt nicht mit symbolisch berechneter überein.');
  end
  taug_num3 = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1(qJ, zeros(NJ,1), zeros(NJ,1), zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)], ...
    pkin, m, rSges, Icges);
  if any( abs(taug_sym-taug_num3(7:end)) > 1e-10)
    error('Fehler in numerisch berechneter Gravitationslast (Methode 3): Stimmt nicht mit symbolisch berechneter überein.');
  end
  taug_num4 = KAS5m7TE_gravload_floatb_eulxyz_nnew_vp1(qJ, zeros(3,1), g_base, ...
    pkin, m, rSges);
  if any( abs(taug_sym-taug_num4(7:end)) > 1e-10)
    error('Fehler in numerisch berechneter Gravitationslast (Methode 4): Stimmt nicht mit symbolisch berechneter überein.');
  end
  taug_num5 = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp2(qJ, zeros(NJ,1), zeros(NJ,1), zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)], ...
    pkin, m, mrSges, Ifges);
  if any( abs(taug_sym-taug_num5(7:end)) > 1e-10)
    error('Fehler in numerisch berechneter Gravitationslast (Methode 5): Stimmt nicht mit symbolisch berechneter überein.');
  end

  % Coriolis
  tauc_sym = KAS5m7TE_coriolisvecJ_fixb_slag_vp1(qJ, qJD, ...
    pkin, m, rSges, Icges);
  tauc_num1 = robot_tree_invdyn_floatb_eulxyz_mdh_nnew_vp1(qJ, qJD, zeros(NJ,1), zeros(3,1), zeros(6,1), zeros(6,1), ...
    alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges, Icges);
  if any( abs(tauc_sym-tauc_num1(7:end)) > 1e-10)
    error('Fehler in numerisch berechneter Corioliskraft (Methode 1): Stimmt nicht mit symbolisch berechneter überein.');
  end
  tauc_num2 = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1(qJ, qJD, zeros(NJ,1), zeros(3,1), zeros(6,1), zeros(6,1), ...
    pkin, m, rSges, Icges);
  if any( abs(tauc_sym-tauc_num2(7:end)) > 1e-10)
    error('Fehler in numerisch berechneter Corioliskraft (Methode 2): Stimmt nicht mit symbolisch berechneter überein.');
  end
  tauc_num3 = robot_tree_coriolisvec_floatb_eulxyz_mdh_nnew_vp1(qJ, qJD, zeros(3,1), zeros(6,1), ...
    alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges, Icges);
  if any( abs(tauc_sym-tauc_num3(7:end)) > 1e-10)
    error('Fehler in numerisch berechneter Corioliskraft (Methode 3): Stimmt nicht mit symbolisch berechneter überein.');
  end
  tauc_num4 = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp2(qJ, qJD, zeros(NJ,1), zeros(3,1), zeros(6,1), zeros(6,1), ...
    pkin, m, mrSges, Ifges);
  if any( abs(tauc_sym-tauc_num4(7:end)) > 1e-10)
    error('Fehler in numerisch berechneter Corioliskraft (Methode 4): Stimmt nicht mit symbolisch berechneter überein.');
  end

  % Massenmatrix
  Mq_sym = KAS5m7TE_inertiaJ_slag_vp1(qJ, ...
    pkin, m, rSges, Icges);
  Mq_num1 = NaN(NJ,NJ);
  Mq_num2 = NaN(NJ,NJ);
  Mq_num3 = NaN(NJ,NJ);
  INJ = eye(NJ);
  for jj = 1:NJ
    Mq_num1(:,jj) = [zeros(NJ,6), eye(NJ)]*robot_tree_invdyn_floatb_eulxyz_mdh_nnew_vp1  (qJ, zeros(NJ,1), INJ(:,jj), zeros(3,1), zeros(6,1), zeros(6,1) , ...
      alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges, Icges);
    Mq_num2(:,jj) = [zeros(NJ,6), eye(NJ)]*KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1(qJ, zeros(NJ,1), INJ(:,jj), zeros(3,1), zeros(6,1), zeros(6,1) , ...
      pkin, m, rSges, Icges);
    Mq_num3(:,jj) = [zeros(NJ,6), eye(NJ)]*KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp2(qJ, zeros(NJ,1), INJ(:,jj), zeros(3,1), zeros(6,1), zeros(6,1) , ...
      pkin, m, mrSges, Ifges);
  end
  if any( abs(Mq_num1(:)-Mq_sym(:)) > 1e-10)
    error('Fehler in numerisch berechneter Massenmatrix (Methode 1): Stimmt nicht mit symbolisch berechneter überein.');
  end
  if any( abs(Mq_num2(:)-Mq_sym(:)) > 1e-10)
    error('Fehler in numerisch berechneter Massenmatrix (Methode 2): Stimmt nicht mit symbolisch berechneter überein.');
  end
  if any( abs(Mq_num3(:)-Mq_sym(:)) > 1e-10)
    error('Fehler in numerisch berechneter Massenmatrix (Methode 3): Stimmt nicht mit symbolisch berechneter überein.');
  end
  Mq_num4 = KAS5m7TE_inertiaJ_nCRB_vp1(qJ, ...
    pkin, m, rSges, Icges);
  if any( abs(Mq_num4(:)-Mq_sym(:)) > 1e-10) && ~any(sigma)
    error('Fehler in numerisch berechneter Massenmatrix (Methode 4): Stimmt nicht mit symbolisch berechneter überein.');
  end
  Mq_num5 = inertia_nCRB_vp1_m(T_stack, RotAx_i, v, m, rSges, Icges);
  if any( abs(Mq_num5(:)-Mq_sym(:)) > 1e-10) && ~any(sigma)
    error('Fehler in numerisch berechneter Massenmatrix (Methode 5): Stimmt nicht mit symbolisch berechneter überein.');
  end

  % Inverse Dynamik durch symbolisches Newton-Euler Verfahren + Basis Reaktionskräfte
  [~, ~, ~, f_i_i_ges1, n_i_i_ges1] = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1(qJ, qJD, qJDD, zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)], ...
      pkin, m, rSges, Icges);
  [~, ~, ~, f_i_i_ges2, n_i_i_ges2] = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp2(qJ, qJD, qJDD, zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)], ...
      pkin, m, mrSges, Ifges);
  test_fm = [f_i_i_ges1-f_i_i_ges2; n_i_i_ges1-n_i_i_ges2];
  if any(abs(test_fm(:))>1e-10)
    error('Schnittkräfte/-momente stimmen nicht zwischen Parametersatz 1/2');
  end
  tauJ_sym = KAS5m7TE_invdynJ_fixb_snew_vp2(qJ, qJD,  qJDD, g_base, ...
    pkin, m, mrSges, Ifges);
  tauB_sym = KAS5m7TE_invdynB_fixb_snew_vp2(qJ, qJD,  qJDD, g_base, ...
    pkin, m, mrSges, Ifges);
  tau_sym = [tauB_sym; tauJ_sym];
  tau_num1 = robot_tree_invdyn_floatb_eulxyz_mdh_nnew_vp1(qJ, qJD, qJDD, zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)] , ...
    alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges, Icges);
  if any( abs(tau_sym-tau_num1) > 1e-10)
    error('Fehler in numerisch berechneter inversen Dynamik (Methode 1): Stimmt nicht mit symbolisch berechneter aus Newton-Euler Verfahren überein.');
  end
  [tau_num2,~,~,f_neweul_num,m_neweul_num] = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1(qJ, qJD, qJDD, zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)] , ...
    pkin, m, rSges, Icges);
  if any( abs(tau_sym-tau_num2) > 1e-10)
    error('Fehler in numerisch berechneter inversen Dynamik (Methode 2): Stimmt nicht mit symbolisch berechneter aus Newton-Euler Verfahren überein.');
  end
  % Inverse Dynamik für Schnittkräfte und Momente
  f_neweul_sym = KAS5m7TE_invdynf_fixb_snew_vp2(qJ, qJD, qJDD, g_base, ...
    pkin, m, mrSges, Ifges);
  m_neweul_sym = KAS5m7TE_invdynm_fixb_snew_vp2(qJ, qJD, qJDD, g_base, ...
    pkin, m, mrSges, Ifges);
  test_cutwrench = [f_neweul_num;m_neweul_num]-[f_neweul_sym;m_neweul_sym];
  if max(abs(test_cutwrench(:))) > 1e6*eps(1+max(abs([f_neweul_sym(:);m_neweul_sym(:)])))
    error('Fehler zwischen symbolisch und numerisch berechneten Schnittkräften/-momente mit Newton-Euler');
  end
end
fprintf('Numerische Dynamikfunktionen (Fixed Base) gegen symbolische erfolgreich getestet\n');


%% Test gegen numerische Funktionen der Kinematik
for i = 1:n
  qJ = Q(i,:)';
  qJD = QD(i,:)';
  qJDD = QDD(i,:)';
  g_base = G(i,:)';

  % Gestapelte Transformationsmatrizen für alle Körper-KS (einschl. Basis)
  Tc = KAS5m7TE_fkine_fixb_rotmat_mdh_sym_varpar(qJ, pkin);
  Tc_stack = NaN(3*NL,4);
  for ii = 1:NL
      Tc_stack(3*ii-2:3*ii,:) = Tc(1:3,:,ii);
  end
  
  % Test der allgemeinen gegen die angepasste numerische Jacobi-Matrix
  for j = uint8(0:NL-1)
    r_i_i_C = rand(3,1);
    J_gen = robot_tree_jacobig_m(Tc_stack(4:end,:), RotAx_i, v, sigma, j, r_i_i_C);
    J_rob = KAS5m7TE_jacobig_mdh_num(qJ, j, r_i_i_C, pkin);
    test_J = J_gen - J_rob;
    if max(abs(test_J(:))) > 1e-10
      error('Jacobi-Matrix stimmt nicht mit allgemeiner und spezieller numerischer Implementierung');
    end
  end
  
  % Vergleiche Gelenk-Jacobi mit Schnittkraft-Jacobi für Gelenk-Einträge
  for j = uint8(0:NL-1)
    r_i_i_C = rand(3,1);
    J_rob = KAS5m7TE_jacobig_mdh_num(qJ, j, r_i_i_C, pkin);
    Jg_C = robot_tree_jacobig_cutforce_m(Tc_stack, v, j, r_i_i_C);
    jidx = (2:NL)*6 - (sigma'==1)*3; % Prüfe die Spalten aller Gelenkachsen
    test_J = J_rob - Jg_C(:,jidx);
    if max(abs(test_J(:))) > 1e-10
      error('Jacobi-Matrix unterscheidet sich bei robot_tree_jacobig_cutforce_m');
    end
  end
  
  % Teste allgemeine Schnittkraft-Jacobi gegen Schnittkraft-Funktionen
  % Ansatz: Definiere Gravitationskraft des einzigen massebehafteten
  % Segmentes als externe Kraft. Berechne resultierende Schnittkräfte
  % sowohl über inverse Dynamik (Newton-Euler) als auch über Projektion der
  % gedachten externen Kraft
  for j = uint8(0:NL-1)
    % Masse des Roboters belegen um virtuelle Kraft zu erzeugen, die der
    % Gewichtskraft entspricht
    m_test = m;
    m_test((1:NL)~=j+1) = 0;
    mrSges_test = mrSges;
    mrSges_test((1:NL)~=j+1,:) = 0;
    
    % Berechnung der Schnittkräfte durch Projektion
    Jg_C = robot_tree_jacobig_cutforce_m(Tc_stack, v, j, rSges(j+1,:)');
    F_test = [m(j+1)*g_base; zeros(3,1)]; % Gewichtskraft
    W_1 = -Jg_C' * F_test;
    
    % Berechne Gewichtskraft der Testkonfiguration mit Newton-Euler
    f_neweul_sym = KAS5m7TE_invdynf_fixb_snew_vp2(qJ, 0*qJ, 0*qJ, g_base, ...
      pkin, m_test, mrSges_test, 0*Ifges);
    m_neweul_sym = KAS5m7TE_invdynm_fixb_snew_vp2(qJ, 0*qJ, 0*qJ, g_base, ...
      pkin, m_test, mrSges_test, 0*Ifges);
    W_2 = reshape([f_neweul_sym;m_neweul_sym], 6*NL, 1);
    
    % Vergleich
    test_W = W_1 - W_2;
    if max(abs(test_W)) > 1e-10
      error('Schnittkräfte aus Newton-Euler stimmen nicht mit externer Kraft überein');
    end
  end
end
fprintf('Funktionen für Schnittkraft-Jacobi gegen Dynamik validiert\n');

%% Zeitmessung gegen numerische Funktionen: Gravitation (Fixed Base)
% Berechne Gravitationslast mit unterschiedlichen Methoden:
% Symbolisch, numerisch, mit inverser Dynamik und mit Spezialfunktione,
% kompiliert (mex) und normaler Aufruf
matlabfcn2mex({'KAS5m7TE_gravloadJ_floatb_twist_slag_vp1', ...
  'KAS5m7TE_gravloadJ_floatb_twist_slag_vp2', ...
  'KAS5m7TE_gravloadJ_regmin_slag_vp', ...
  'KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1', ...
  'KAS5m7TE_gravload_floatb_eulxyz_nnew_vp1'});
n = 2000;
% Berechne Rechenzeiten. Zeile 1: Normale Funktionen. Zeile 2: Kompilierte Fkt (mex)
t_ber_g = NaN(2,7);
tic;
for i = 1:n
  taug_sym = KAS5m7TE_gravloadJ_floatb_twist_slag_vp1(qJ+i*1e-3, g_base, ...
    pkin, m, rSges);
end
t_ber_g(1,1) = toc/n;
tic;
for i = 1:n
  taug_sym = KAS5m7TE_gravloadJ_floatb_twist_slag_vp1_mex(qJ+i*1e-3, g_base, ...
    pkin, m, rSges);
end
t_ber_g(2,1) = toc/n;
tic;
for i = 1:n
  taug_sym = KAS5m7TE_gravloadJ_floatb_twist_slag_vp2(qJ+i*1e-3, g_base, ...
    pkin, m, mrSges);
end
t_ber_g(1,2) = toc/n;
tic;
for i = 1:n
  taug_sym = KAS5m7TE_gravloadJ_floatb_twist_slag_vp2_mex(qJ+i*1e-3, g_base, ...
    pkin, m, mrSges);
end
t_ber_g(2,2) = toc/n;
tic;
for i = 1:n
  t_ber_g(2,2) = toc/n;
    taug_reg = KAS5m7TE_gravloadJ_regmin_slag_vp(qJ+i*1e-3, g_base, pkin);
    taug_mpv = taug_reg*MPV;
end
t_ber_g(1,3) = toc/n;
tic;
for i = 1:n
  t_ber_g(2,2) = toc/n;
    taug_reg = KAS5m7TE_gravloadJ_regmin_slag_vp_mex(qJ+i*1e-3, g_base, pkin);
    taug_mpv = taug_reg*MPV;
end
t_ber_g(2,3) = toc/n;

tic;
for i = 1:n
  taug_num1 = robot_tree_invdyn_floatb_eulxyz_mdh_nnew_vp1(qJ+i*1e-3, zeros(NJ,1), zeros(NJ,1), zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)], ...
    alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges, Icges);
end
t_ber_g(1,4) = toc/n;
tic;
for i = 1:n
  taug_num2 = robot_tree_gravload_floatb_eulxyz_mdh_nnew_vp1(qJ+i*1e-3, zeros(3,1), g_base, ...
    alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges);
end
t_ber_g(1,5) = toc/n;
tic;
for i = 1:n
  taug_num3 = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1(qJ+i*1e-3, zeros(NJ,1), zeros(NJ,1), zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)], pkin, m, rSges, Icges);
end
t_ber_g(1,6) = toc/n;
tic;
for i = 1:n
  taug_num3 = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1_mex(qJ+i*1e-3, zeros(NJ,1), zeros(NJ,1), zeros(3,1), zeros(6,1), [-g_base;zeros(3,1)], pkin, m, rSges, Icges);
end
t_ber_g(2,6) = toc/n;
tic;
for i = 1:n
  taug_num4 = KAS5m7TE_gravload_floatb_eulxyz_nnew_vp1(qJ+i*1e-3, zeros(3,1), g_base, ...
    pkin, m, rSges);
end
t_ber_g(1,7) = toc/n;
tic;
for i = 1:n
  taug_num4 = KAS5m7TE_gravload_floatb_eulxyz_nnew_vp1_mex(qJ+i*1e-3, zeros(3,1), g_base, ...
    pkin, m, rSges);
end
t_ber_g(2,7) = toc/n;

figure(2);clf;
set(2, 'Name', 'Fixb_ComputationTime_Comparison');
subplot(2,4,sprc2no(2,4,1,1));
bar(t_ber_g(1,:)*1e3);
set(gca, 'xticklabel', {'s1/gl','s2/gl','sm/gl','n/id/allg','n/gl/allg','n/id','n/gl'});
ylabel('t [ms]'); title('gravload');
subplot(2,4,sprc2no(2,4,2,1));
bar(t_ber_g(2,:)*1e3);
set(gca, 'xticklabel', {'s1/gl','s2/gl','sm/gl','n/id/allg','n/gl/allg','n/id','n/gl'});
ylabel('t [ms]'); title('gravload (mex)');

%% Zeitmessung gegen numerische Funktionen: Coriolisvektor (Fixed Base)
% Berechne Gravitationslast mit unterschiedlichen Methoden:
% Symbolisch, numerisch, mit inverser Dynamik und mit Spezialfunktione,
% kompiliert (mex) und normaler Aufruf
matlabfcn2mex({'KAS5m7TE_coriolisvecJ_fixb_slag_vp1', ...
  'KAS5m7TE_coriolisvecJ_fixb_slag_vp2', ...
  'KAS5m7TE_coriolisvecJ_fixb_regmin_slag_vp', ...
  'KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1'});
n = 2000;

t_ber_c = NaN(2,4);
tic;
for i = 1:n
  tauc_sym = KAS5m7TE_coriolisvecJ_fixb_slag_vp1(qJ+i*1e-3, qJD,...
    pkin, m, rSges, Icges);
end
t_ber_c(1,1) = toc/n;
tic;
for i = 1:n
  tauc_sym = KAS5m7TE_coriolisvecJ_fixb_slag_vp1_mex(qJ+i*1e-3, qJD,...
    pkin, m, rSges, Icges);
end
t_ber_c(2,1) = toc/n;
tic;
for i = 1:n
  tauc_sym = KAS5m7TE_coriolisvecJ_fixb_slag_vp2(qJ+i*1e-3, qJD,...
    pkin, m, mrSges, Ifges);
end
t_ber_c(1,2) = toc/n;
tic;
for i = 1:n
  tauc_sym = KAS5m7TE_coriolisvecJ_fixb_slag_vp2_mex(qJ+i*1e-3, qJD,...
    pkin, m, mrSges, Ifges);
end
t_ber_c(2,2) = toc/n;
tic;
for i = 1:n
  tauc_reg = KAS5m7TE_coriolisvecJ_fixb_regmin_slag_vp(qJ+i*1e-3, qJD, ...
    pkin);
  tauc_mpv = tauc_reg*MPV;
end
t_ber_c(1,3) = toc/n;
tic;
for i = 1:n
  tauc_reg = KAS5m7TE_coriolisvecJ_fixb_regmin_slag_vp_mex(qJ+i*1e-3, qJD, ...
    pkin);
  tauc_mpv = tauc_reg*MPV;
end
t_ber_c(2,3) = toc/n;
tic;
for i = 1:n
  tauc_num1 = robot_tree_invdyn_floatb_eulxyz_mdh_nnew_vp1(qJ+i*1e-3, qJD, zeros(NJ,1), zeros(3,1), zeros(6,1),zeros(6,1), ...
    alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges, Icges);
end
t_ber_c(1,4) = toc/n;
tic;
for i = 1:n
  tauc_num2 = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1(qJ+i*1e-3, qJD, zeros(NJ,1), zeros(3,1), zeros(6,1), zeros(6,1), ...
    pkin, m, rSges, Icges);
end
t_ber_c(1,5) = toc/n;
tic;
for i = 1:n
  tauc_num3 = KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1_mex(qJ+i*1e-3, qJD, zeros(NJ,1), zeros(3,1), zeros(6,1), zeros(6,1), ...
    pkin, m, rSges, Icges);
end
t_ber_c(2,5) = toc/n;

subplot(2,4,sprc2no(2,4,1,2));
bar(t_ber_c(1,:)*1e3);
set(gca, 'xticklabel', {'s1/cv','s2/cv','sm/cv', 'n/id/allg','n/id'});
ylabel('t [ms]'); title('coriolisvec');
subplot(2,4,sprc2no(2,4,2,2));
bar(t_ber_c(2,:)*1e3);
set(gca, 'xticklabel', {'s1/cv','s2/cv','sm/cv', 'n/id/allg','n/id'});
ylabel('t [ms]'); title('coriolisvec (mex)');

%% Zeitmessung gegen numerische Funktionen: Massenmatrix (Fixed Base)
% Berechne Gravitationslast mit unterschiedlichen Methoden:
% Symbolisch, numerisch, mit inverser Dynamik und mit Spezialfunktione,
% kompiliert (mex) und normaler Aufruf
matlabfcn2mex({'KAS5m7TE_inertia_floatb_eulxyz_slag_vp1', ...
  'KAS5m7TE_inertia_floatb_eulxyz_slag_vp2', ...
  'KAS5m7TE_inertiaJ_slag_vp1', ...
  'KAS5m7TE_inertiaJ_slag_vp2', ...
  'KAS5m7TE_inertiaJ_regmin_slag_vp', ...
  'KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1', ...
  'KAS5m7TE_inertiaJ_nCRB_vp1'});
n = 200;
t_ber_M = NaN(2,4);
tic;
for i = 1:n
  Mq_sym = KAS5m7TE_inertiaJ_slag_vp1(qJ+1e-3*i, ...
    pkin, m, rSges, Icges);
end
t_ber_M(1,1) = toc/n;
tic;
for i = 1:n
  Mq_sym = KAS5m7TE_inertiaJ_slag_vp1_mex(qJ+1e-3*i, ...
    pkin, m, rSges, Icges);
end
t_ber_M(2,1) = toc/n;
tic;
for i = 1:n
  Mq_sym = KAS5m7TE_inertiaJ_slag_vp2(qJ+1e-3*i, ...
    pkin, m, mrSges, Ifges);
end
t_ber_M(1,2) = toc/n;
tic;
for i = 1:n
  Mq_sym = KAS5m7TE_inertiaJ_slag_vp2_mex(qJ+1e-3*i, ...
    pkin, m, mrSges, Ifges);
end
t_ber_M(2,2) = toc/n;
tic;
for i = 1:n
  Mq_reg = KAS5m7TE_inertiaJ_regmin_slag_vp(qJ, ...
    pkin);
  Mq_mpv_vec = Mq_reg*MPV;
  Mq_mpv = vec2symmat(Mq_mpv_vec);
end
t_ber_M(1,3) = toc/n;
tic;
for i = 1:n
  Mq_reg = KAS5m7TE_inertiaJ_regmin_slag_vp_mex(qJ, ...
    pkin);
  Mq_mpv_vec = Mq_reg*MPV;
  Mq_mpv = vec2symmat(Mq_mpv_vec);
end
t_ber_M(2,3) = toc/n;

tic
for i = 1:n
  Mq_num1 = NaN(NJ,NJ);
  INJ = eye(NJ);
  for jj = 1:NJ
    Mq_num1(:,jj) = [zeros(NJ,6), eye(NJ)]*robot_tree_invdyn_floatb_eulxyz_mdh_nnew_vp1  (qJ+1e-3*i, zeros(NJ,1), INJ(:,jj), zeros(3,1), zeros(6,1), zeros(6,1) , ...
      alpha, a, d, theta, q_offset, b, beta, v, sigma, m, rSges, Icges);
  end
end
t_ber_M(1,6) = toc/n;
tic
for i = 1:50
  Mq_num2 = NaN(NJ,NJ);
  INJ = eye(NJ);
  for jj = 1:NJ
    Mq_num2(:,jj) = [zeros(NJ,6), eye(NJ)]*KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1(qJ+1e-3*i, zeros(NJ,1), INJ(:,jj), zeros(3,1), zeros(6,1), zeros(6,1) , ...
      pkin, m, rSges, Icges);
  end
end
t_ber_M(1,5) = toc/50;
tic
for i = 1:n
  Mq_num2 = NaN(NJ,NJ);
  INJ = eye(NJ);
  for jj = 1:NJ
    Mq_num2(:,jj) = [zeros(NJ,6), eye(NJ)]*KAS5m7TE_invdyn_floatb_eulxyz_nnew_vp1_mex(qJ+1e-3*i, zeros(NJ,1), INJ(:,jj), zeros(3,1), zeros(6,1), zeros(6,1) , ...
      pkin, m, rSges, Icges);
  end
end
t_ber_M(2,5) = toc/n;
tic
for i = 1:50
  Mq_num3 = KAS5m7TE_inertiaJ_nCRB_vp1(qJ, ...
    pkin, m, rSges, Icges);
end
t_ber_M(1,6) = toc/50;

v_test = uint8(ones(30,1));
tic
for i = 1:n
  Mq_num4 = KAS5m7TE_inertiaJ_nCRB_vp1_mex(qJ, ...
    pkin, m, rSges, Icges);
end
t_ber_M(2,6) = toc/n;

subplot(2,4,sprc2no(2,4,1,3));
bar(t_ber_M(1,:)*1e3);
set(gca, 'xticklabel', {'s1/M','s2/M','sm/M','n/id/allg', 'n/id', 'n/CRBA'});
ylabel('t [ms]'); title('inertia');
subplot(2,4,sprc2no(2,4,2,3));
bar(t_ber_M(2,:)*1e3);
set(gca, 'xticklabel', {'s1/M','s2/M','sm/M','n/id/allg', 'n/id', 'n/CRBA'});
ylabel('t [ms]'); title('inertia (mex)');
%% Rechenzeit: Vorwärtsdynamik (Fixed Base)
% Getrennte Betrachtung der inversen Dynamik, da bei numerischer Berechnung
% Coriolis- und Gravitationsmoment zusammen berechnet werden können
t_ber_fdyn = NaN(2,3);
% Alternative 1: Alles symbolisch und kompiliert
t_ber_fdyn(2,1) = t_ber_g(2,1) + t_ber_c(2,2) + t_ber_M(2,1);
% Alternative 2: Nur Coriolisvektor numerisch, alles andere symbolisch und
% kompiliert
t_ber_fdyn(2,2) = t_ber_g(2,1) + t_ber_c(2,5) + t_ber_M(2,1);
% Alternative 3: Gravitation und Coriolis in einem numerischen
% Funktionsaufruf, Massenmatrix symbolisch
t_ber_fdyn(2,3) =  t_ber_c(2,5) + t_ber_M(2,1);

subplot(2,4,sprc2no(2,4,2,4));
bar(t_ber_fdyn(2,:)*1e3);
set(gca, 'xticklabel', {'s1/s1/s1', 's1/n/s1', 'n/s1'});
ylabel('t [ms]'); title('fdyn (mex)');



