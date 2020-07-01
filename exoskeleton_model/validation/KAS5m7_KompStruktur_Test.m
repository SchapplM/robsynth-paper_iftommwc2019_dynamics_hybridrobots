% Teste Kinematik der Kompensationsstruktur des KAS5m7
% TODO: Funktioniert noch nicht für ein Schnitt-KS

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-02
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

clear
clc

%% Initialisierung
exo_example_path_init
S_m3 = KAS5_const(3);
S_m5 = KAS5_const(5);
[a_mdh, d_mdh, alpha_mdh, q_offset_mdh] = KAS5m2_parameter_mdh();
[l_const, w_const] = KAS5_parameter_kinematic();
delta18 = w_const(6);
n = 100;

Q_m5 = (0.5-rand(n, S_m5.N))*pi;
Q_m5(1,:) = 0;
for i = 1:S_m5.N
  Q_m5(Q_m5(:,i)<S_m5.q_min(i),i) = S_m5.q_min(i);
  Q_m5(Q_m5(:,i)>S_m5.q_max(i),i) = S_m5.q_max(i);
end

% KS-Liste mit Entsprechungen zwischen Modellen 5 und 7
I_identKS_5_7 = [ ...
  1 1; ...
  2 2; ...
  3 3; ...
  4 4; ...
  5 5; ...
  6 6; ...
  7 7; ...
  8 8; ...
  9 9; ...
  10 10; ...
  11 11
  ];
% KS-Liste: Koordinatenursprung ist gleich (aber nicht unbedingt die Orientierung)
I_identPkt_7 = [ ...
  16 19; ...
  17 20; ...
  18 21; ...
  ];
%% Bild zeichnen
q0_m5 = zeros(5,1);
q0_m5(3) = 30*pi/180;
q0_m5(4) = 30*pi/180;
q0_m3 = KAS5_convert_q(q0_m5, 5, 3);
[~, ~, r_P, ~] = KAS5m3_kine_parallel_v2(q0_m3);

jv = KAS5m7TE_kinconstr_expl_mdh_num(q0_m5);

T_c_m7 = KAS5m7_fkine_mdh_num(q0_m5);
figure(1);clf;
view(3)
KAS5m2_plot_Hauptstruktur(T_c_m7(:, :, [1:8, 23]));
KAS_plot_KS(T_c_m7, 5, 7)
KAS5m3_plot_Nebenstruktur(T_c_m7, r_P)
ylim([-0.3, 0.3])

Q_m5(1,:) = q0_m5;
%% Prüfe implizite Zwangsbedingungen für KAS5 m7
% Position (und Orientierung) der Schnitt-KS muss gleich sein.
for i = 1:n
  q_m5 = Q_m5(i, :)';
  q_m3 = KAS5_convert_q(q_m5, 5, 3);
  T_c_mdh_m7 = KAS5m7_fkine_mdh_num(q_m5);
  for j = 1:(length(I_identPkt_7)-1)
    j1 = I_identPkt_7(j,1)+1;
    j2 = I_identPkt_7(j,2)+1;
    T_diff = T_c_mdh_m7(:,:,j1) - T_c_mdh_m7(:,:,j2);
    if any(isnan(T_diff(:)))
      error('NaN!');
    end

    if any(abs(T_diff(:)) > 1e-10)
      error('i=%d. Transformationsmatrix %d (m7) stimmt nicht mit %d (m7)!', i, j1-1, j2-1);
      
      % Debug
      [T_c_mdh_m7(:,:,j1), T_c_mdh_m7(:,:,j2), T_c_mdh_m7(:,:,j1)-T_c_mdh_m7(:,:,j2)]
      R_diff = t2r( T_c_mdh_m7(:,:,j2))' * t2r( T_c_mdh_m7(:,:,j1));
      r2rpy(R_diff)
      
      [~, w_par] = KAS5m7_kine_parallel(q_m5)
    end
  end
end

%% Vergleiche KAS5m7 gegen KAS5m5 für Winkelkombinationen
for i = 1:n
  q_m5 = Q_m5(i, :)';
  
  T_c_mdh_m7 = KAS5m7_fkine_mdh_num(q_m5);
  T_c_mdh_m5 = KAS5m5_fkine_mdh_sym(q_m5);
  
  % Vergleiche gegen symbolische Berechnung KAS5m3
  for j = 1:length(I_identKS_5_7)
    Im5 = I_identKS_5_7(j,1)+1; % Umrechnung MDH-Nummer -> Matlab-Nummer
    Im7 = I_identKS_5_7(j,2)+1;
    
    T_diff = T_c_mdh_m5(:,:,Im5) - T_c_mdh_m7(:,:,Im7);
    if any(abs(T_diff(:)) > 1e-9)
      error('j=%d. Transformationsmatrix %d (m7) stimmt nicht mit %d (m5)!', j, Im5, Im7);
    end
  end
end
fprintf('Direkte Kinematik KAS5m5 gegen KAS5m7 für %d Kombinationen getestet.\n', n);

matlabfcn2mex({'KAS5m5_fkine_mdh_sym', 'KAS5m5_fkine_mdh_sym_varpar'});
matlabfcn2mex({'KAS5m7_fkine_mdh_num'});

%% Teste MDH-Parameter symbolisch gegen numerisch
TSS = KAS5m7TE_varpar_testfunctions_parameter();
[~, ~, ~, ~, ~, ~, ~, ~, v_mdh] = KAS5m7_parameter_mdh();
test = TSS.v - v_mdh;
%% Teste symbolische Generierung ohne Zwangsbedingungen
[~, ~, pkin_m7u] = KAS5_parameter_kinematic(7, true);
for i = 1:n
  q_m5 = Q_m5(i, :)';
  
  % Alle Gelenkvariablen
  jv = KAS5m7TE_kinconstr_expl_mdh_num(q_m5);
  % Gelenkvariablen von KSA5m7u
  q_m7u = jv([1:7 9:12 14 15]);

  % Berechnung der Kinematik symbolisch und numerisch
  T_c_mdh_sym = KAS5m7OL_fkine_fixb_rotmat_mdh_sym_varpar(q_m7u, pkin_m7u);
  T_c_mdh_num = KAS5m7_fkine_mdh_num(q_m5);
  
  for jj = 1:22
    T_diff = T_c_mdh_sym(:,:,jj) - T_c_mdh_num(:,:,jj);
    if jj < 16 && any(abs(T_diff(:)) > 1e-10) || jj>=16 && any(abs(T_diff(1:3,4)) > 1e-10)
      error('i=%d. Transformationsmatrix %d stimmt nicht zwischen num und sym!', i, jj-1);
      R_diff = t2r( T_c_mdh_sym(:,:,jj))' * t2r(  T_c_mdh_num(:,:,jj) );
      r2rpy(R_diff)
      [~, w_par] = KAS5m7_kine_parallel(q_m5)
    end
  end
end
fprintf('Direkte Kinematik KAS5m7 numerisch gegen symbolisch (ohne Zwangsbedingungen) für %d Kombinationen getestet.\n', n);

%% Teste symbolische Generierung Mit Zwangsbedingungen
[~, ~, pkin_m7] = KAS5_parameter_kinematic(7);

[~, ~, ~, ~, ~, ~, ~, ~, v_mdh] = KAS5m7_parameter_mdh();
for i = 1:n
  q_m5 = Q_m5(i, :)';

  % Berechnung der Kinematik symbolisch und numerisch
  T_c_mdh_sym = KAS5m7TE_fkine_fixb_rotmat_mdh_sym_varpar(q_m5, pkin_m7);
  T_c_mdh_num = KAS5m7_fkine_mdh_num(q_m5);

  for jj = 1:22
    T_diff = T_c_mdh_sym(:,:,jj) - T_c_mdh_num(:,:,jj);
     % Schnittgelenke interessieren erstmal nicht. Dort nur einfache Prüfung
    if jj < 16 && any(abs(T_diff(:)) > 1e-10) || jj>=16 && any(abs(T_diff(1:3,4)) > 1e-10)
      error('i=%d. Transformationsmatrix %d stimmt nicht zwischen num und sym!', i, jj-1);
      R_diff = t2r( T_c_mdh_sym(:,:,jj))' * t2r(  T_c_mdh_num(:,:,jj) );
      r2rpy(R_diff)
      [~, w_par] = KAS5m7_kine_parallel(q_m5);
      
      figure(2);clf;
      view(3)
      KAS5m2_plot_Hauptstruktur(T_c_mdh_num(:, :, [1:8, 23]));
      trplot(T_c_mdh_sym(:,:,jj), 'frame', sprintf('s%d',jj-1), 'arrow', 'rgb', 'length', 0.15);
      trplot(T_c_mdh_num(:,:,jj), 'frame', sprintf('n%d',jj-1), 'arrow', 'rgb', 'length', 0.15);
      trplot(T_c_mdh_sym(:,:,v_mdh(jj-1)), 'frame', sprintf('s%d',v_mdh(jj-1)), 'arrow', 'rgb', 'length', 0.15);
      trplot(T_c_mdh_num(:,:,v_mdh(jj-1)), 'frame', sprintf('n%d',v_mdh(jj-1)), 'arrow', 'rgb', 'length', 0.15);

      q_m3 = KAS5_convert_q(q_m5, 5, 3);
      [~, ~, r_P, ~] = KAS5m3_kine_parallel_v2(q_m3);
      KAS5m3_plot_Nebenstruktur(T_c_mdh_num, r_P)
      ylim([-0.3, 0.3])
    end
  end
  
  % Berechnung mit alternativer Implementierung
  T_c_mdh_sym2 = KAS5m7DE2_fkine_fixb_rotmat_mdh_sym_varpar(q_m5, pkin_m7);
  for jj = 1:22
    T_diff = T_c_mdh_sym(:,:,jj) - T_c_mdh_sym2(:,:,jj);
     % Schnittgelenke interessieren erstmal nicht. Dort nur einfache Prüfung
    if any(abs(T_diff(:)) > 1e-10)
      error('i=%d. Transformationsmatrix %d stimmt nicht zwischen sym2 und sym!', i, jj-1);
    end
  end
end
fprintf('Direkte Kinematik KAS5m7 numerisch gegen symbolisch (mit Zwangsbedingungen) für %d Kombinationen getestet.\n', n);

%% Unterschiedliche Implementierungen testen
TSSc = KAS5m7TE_varpar_testfunctions_parameter;
g = [0;0;-9.81];
for i = 1:n
  q_m5 = Q_m5(i, :)';
  qD_m5 = rand(5,1);

  taug1_expl = KAS5m7TE_gravloadJ_floatb_twist_slag_vp2(q_m5, g, TSSc.pkin, TSSc.m, TSSc.mrSges);
  taug1_expl2 = KAS5m7DE2_gravloadJ_floatb_twist_slag_vp2(q_m5, g, TSSc.pkin, TSSc.m, TSSc.mrSges);
  test = taug1_expl - taug1_expl2;
  if any( abs(test(:)) > 1e6*eps(1+max(abs(test))) )
    error('Gravitationslast mit alternativer Implementierung des Eliminiations-Ansatzes stimmt nicht.');
  end

  MM1_expl = KAS5m7TE_inertiaJ_slag_vp2(q_m5, TSSc.pkin, TSSc.m, TSSc.mrSges, TSSc.Ifges);
  MM1_expl2 = KAS5m7DE2_inertiaJ_slag_vp2(q_m5, TSSc.pkin, TSSc.m, TSSc.mrSges, TSSc.Ifges);
  test = MM1_expl2 - MM1_expl;
  if any( abs(test(:)) > 1e6*eps(1+max(abs(test))) )
    error('Massenmatrix mit alternativer Implementierung des Eliminiations-Ansatzes stimmt nicht.');
  end

  c1_expl = KAS5m7TE_coriolisvecJ_fixb_slag_vp2(q_m5, qD_m5, TSSc.pkin, TSSc.m, TSSc.mrSges, TSSc.Ifges);
  c1_expl2 = KAS5m7DE2_coriolisvecJ_fixb_slag_vp2(q_m5, qD_m5, TSSc.pkin, TSSc.m, TSSc.mrSges, TSSc.Ifges);
  test = c1_expl - c1_expl2;
  if any( abs(test(:)) > 1e6*eps(1+max(abs(test))) )
    error('Massenmatrix mit alternativer Implementierung des Eliminiations-Ansatzes stimmt nicht.');
  end
end
fprintf('Unterschiedliche Herleitungen gegeneinander getestet.\n');
