% Teste Kinematik der Kompensationsstruktur des KAS5m5

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-02
% (c) Institut für Regelungstechnik, Universität Hannover

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


%% Vergleiche KAS5m3 gegen KAS5m5 für Winkelkombinationen

for i = 1:n

  q_m5 = Q_m5(i, :)';
  q_m3 = KAS5_convert_q(q_m5, 5, 3);
  q_m4 = KAS5_convert_q(q_m5, 5, 4);

  T_c_mdh_m5 = KAS5m5_fkine_mdh_sym(q_m5);
  T_c_mdh_m3 = KAS5m3_fkine_mdh_sym(q_m3);
  % Vergleiche gegen symbolische Berechnung KAS5m3
  for j = 1:size(T_c_mdh_m5,3)
    T_diff = T_c_mdh_m5(:,:,j) - T_c_mdh_m3(:,:,j);
    if any(abs(T_diff(:)) > 1e-10)
      error('i=%d. Transformationsmatrix %d stimmt nicht!', i, j);
      
      % Debug
      [T_c_mdh_m3, T_c_mdh_m5, T_c_mdh_m3-T_c_mdh_m5]
      R_diff = t2r( T_c_mdh_m5(:,:,j))' * t2r( T_c_mdh_m3(:,:,j));
      r2rpy(R_diff)
      
      [~, w_par, ~] = KAS5m3_kine_parallel_v2(q_m4([1 2 4 5 6 7]))
      T_c_mdh_m4num = KAS5_m4_fkine_mdh_num(q_m4, w_par)
    end
  end
  
  % Vergleiche gegen numerische Berechnung KAS5m3
  [~, w_par] = KAS5m3_kine_parallel(q_m3);
  T_c_mdh_m4num = KAS5m4_fkine_mdh_num(q_m4, w_par);
  T_c_diff = T_c_mdh_m4num - T_c_mdh_m5;
  if any(abs(T_c_diff(:)) > 1e-10)
    error('Symbolische Berechnung KAS5m5 stimmt nicht mit numerischer Berechnung KAS5m3 überein');
  end
  

  T_c_mdh_m2 = KAS5m2_fkine_mdh_sym(q_m4);
  r_P_m3 = KAS5m3_kine_parallel_hilfspunkte(l_const, w_par, T_c_mdh_m2);
  
  r_P_m5 = KAS5m5_kine_parallel_hilfspunkte(l_const, T_c_mdh_m5);
  
  r_P_Diff = r_P_m5-r_P_m3;
  if any(abs(r_P_Diff(:)) > 1e-10)
    error('Punkte der Parallelstruktur stimmen nicht zwischen m3 und m5');
  end
  
end
fprintf('Direkte Kinematik KAS5m5 gegen KAS5m3 für %d Kombinationen getestet.\n', n);

matlabfcn2mex({'KAS5m5_fkine_mdh_sym', 'KAS5m5_fkine_mdh_sym_varpar'});

%% Vergleiche KAS5m2 gegen KAS5m5 für Winkelkombinationen

for i = 1:n
  q_m5 = Q_m5(i, :)';
  q_m2 = KAS5_convert_q(q_m5, 5, 2);
  
  T_c_mdh_m5 = KAS5m5_fkine_mdh_sym(q_m5);
  T_c_mdh_m2 = KAS5m2_fkine_mdh_sym(q_m2);
  
  T_c_mdh_m2_test = T_c_mdh_m5(:,:,[1 2 3 4 5 6 7 8 15]);
  
  for j = 1:size(T_c_mdh_m2_test,3)
    T_diff = T_c_mdh_m2(:,:,j) - T_c_mdh_m2_test(:,:,j);
    if any(abs(T_diff(:)) > 1e-10)
      error('i=%d. Transformationsmatrix %d stimmt nicht zwischen KAS5m5 und KAS5m2!', i, j);
    end
  end
end
fprintf('Direkte Kinematik KAS5m5 gegen KAS5m2 für %d Kombinationen getestet.\n', n);

%% Vergleiche KAS5m5 gegen KAS5m4
% Prüfe damit, ob die Konvertierung der Gelenkwinkel mit Zwangsbedingungen
% in KAS5_convert_q richtig funktioniert.
for i = 1:n
  q_m5 = Q_m5(i, :)';
  q_m4 = KAS5_convert_q(q_m5, 5, 4);
  
  T_c_mdh_m5 = KAS5m5_fkine_mdh_sym(q_m5);
  T_c_mdh_m4 = KAS5m2_fkine_mdh_sym(q_m4);
  T_c_mdh_m4_test = T_c_mdh_m5(:,:,[1 2 3 4 5 6 7 8 15]);
  for j = 1:size(T_c_mdh_m4,3)
    T_diff = T_c_mdh_m4(:,:,j) - T_c_mdh_m4_test(:,:,j);
    if any(abs(T_diff(:)) > 1e-10)
      error('i=%d. Transformationsmatrix %d stimmt nicht zwischen KAS5m5 und KAS5_m4!', i, j);
    end
  end
end
