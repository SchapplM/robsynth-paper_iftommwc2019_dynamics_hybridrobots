% Teste Kinematikfunktionen für KAS5m7IC mit denen aus KAS5m7TE
% Rechnet Kinematikterme von der geschlossenen auf die offene Struktur um.

% Quellen:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)
% [ParkChoPlo1999] Park, FC and Choi, Jihyeon and Ploen, SR: Symbolic formulation of closed chain dynamics in independent coordinates
% [UdwadiaKal1992] Udwadia, Firdaus E and Kalaba, Robert E: A new perspective on constrained motion (1992)
% [Docquier2013] Docquier, Nicolas and Poncelet, Antoine and Fisette, Paul: ROBOTRAN: a powerful symbolic gnerator of multibody models (2013)

% Quelle: HybrDyn-Toolbox
% Datum: 2020-06-30 18:50
% Revision: b9e8aa5c608190a7b43c48aaebfd2074f0379b0d (2020-06-30)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

% Tim Job (HiWi-Job bei Moritz Schappler), 2019-05
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover


clear
clc

%% Init

if isempty(fileparts(which('KAS5m7TE_varpar_testfunctions_parameter')))
    fprintf('Test für die impliziten ZB nicht ausgeführt, da die expliziten ZBs noch nicht vorliegen\n');
    return;
end
% Test-Definitionen für drei Fälle aufstellen. Die Kinematikparameter und
% Gelenkkoordinaten können jeweils unterschiedlich sein.
TSSc = KAS5m7TE_varpar_testfunctions_parameter; % mit expl. Zwangsbedingungen
TSSi = KAS5m7IC_varpar_testfunctions_parameter; % mit impl. Zwangsbedingungen
TSSo = KAS5m7OL_varpar_testfunctions_parameter; % ohne Zwangsbedingungen

% Indizes zur Umrechnung von Gelenken von o zu c
% (für den offenen Kreis werden alle Gelenkvariablen ausgegeben, also auch
% die virtuellen Schnittgelenke)
[~, sigma_mdh, mu_mdh] = KAS5m7OL_structural_kinematic_parameters();
posNQJ = KAS5m7IC_positionVector_NQJ();
I_p = (mu_mdh==0);
I_a = (mu_mdh==1);
I_p = I_p(posNQJ==1);
I_a = I_a(posNQJ==1);

NQJ = TSSo.NQJ;
NQJA = sum(I_a==1);
NQJP = sum(I_p==1);

II_oc = find(posNQJ==1);

II_o1 = find(I_a==1);
II_o2 = find(I_p==1);

% Permutationsmatrizen zur Umrechnung von aufgeteilten Koordinaten
% (aktiv/passiv nacheinander) in Koordinaten der Baumstruktur (aktiv/passiv gemischt)
P1 = zeros(NQJA,NQJ);
P2 = zeros(NQJP,NQJ);
for i = 1:NQJA
  P1(i,II_o1(i)) = 1;
end
for i = 1:NQJP
  P2(i,II_o2(i)) = 1;
end
P0 = ([P1;P2])'; % Permutationsmatrix K in [DoThanhKotHeiOrt2009b]

%% Berechnung der Zwangsbedingungs-Projektionsmatrix aus Impliziten ZB
% Nach [Docquier2013]

for iq = 1:TSSc.n

  % Minimalkoordinaten für System mit ZB
  q1 = TSSc.Q(iq,:)';
  qD1 = TSSc.QD(iq,:)';
  qDD1 = TSSc.QDD(iq,:)';

  % Umrechnung von System mit ZB auf offenes System ohne ZB
  W = KAS5m7TE_kinconstr_expl_jacobian_mdh_sym_varpar(q1, TSSc.pkin);
  WD = KAS5m7TE_kinconstr_expl_jacobianD_mdh_sym_varpar(q1, qD1, TSSc.pkin);
  q = KAS5m7TE_kinconstr_expl_mdh_sym_varpar(q1, TSSc.pkin);

  q = q(posNQJ==1);
  W = W(posNQJ==1,:);
  WD = WD(posNQJ==1,:);

  qD = W*qD1;
  qDD = W*qDD1 + WD*qD1;

  % Teste implizite Zwangsbedingungen
  h = KAS5m7IC_kinconstr_impl_mdh_sym_varpar(q, TSSi.pkin);
  h_viol = h(abs(h) > 1e-7); % Toleranz bei Fünfgelenkkette bis zu 1e-7
  if any(abs(h) > 1e-7) && any(abs(abs(h_viol) - 2*pi) > 1e-7) % Für die Orientierungszwangsbedingung ist auch +-2*Pi erlaubt
    error('Implizite Zwangsbedingungen werden nicht erfüllt!');
  end

  % Vergleiche Gelenk-Transformationsmatrizen
  % Keine Prüfung der Orientierung bei den Schließ-KS der kinematischen
  % Ketten. Annahme: Wenn die impl. ZB Null sind, wird das schon richtig
  % sein. Ansonsten wird bei Nicht-Betrachtung der Rotations-ZB ein Fehler
  % aufgeworfen, obwohl die Modellierung richtig ist.
  num_sigma2_end = find((TSSc.sigma(end:-1:1) == 2) == 0, 1)-1; % Anzahl sigma=2 am Ende
  Tc = KAS5m7TE_joint_trafo_rotmat_mdh_sym_varpar(q1, TSSc.pkin);
  Tu = KAS5m7OL_joint_trafo_rotmat_mdh_sym_varpar(q, TSSo.pkin);
  for jj = 1:(TSSc.NJ - 2*num_sigma2_end)
    test = Tc(:,:,jj)\Tu(:,:,jj) - eye(4);
    if any( abs(test(:)) > 1e-7 )
      [Tc(:,:,jj),Tu(:,:,jj)] %#ok<NOPTS>
      r2eulxyz(Tc(1:3,1:3,jj)' * Tu(1:3,1:3,jj))*180/pi %#ok<NOPTS>
      error('Gelenk-Koordinatentransformation %d stimmt nicht. Fehler: abs %e', jj, max(abs(test(:))));
    end
  end
  % Vergleiche KörperKS-Transformationsmatrizen
  Tcc = KAS5m7TE_fkine_fixb_rotmat_mdh_sym_varpar(q1, TSSc.pkin);
  Tcu = KAS5m7OL_fkine_fixb_rotmat_mdh_sym_varpar(q, TSSo.pkin);
  for jj = 1:TSSc.NL % 1=Basis
    test_abs = abs(Tcc(:,:,jj)\Tcu(:,:,jj) - eye(4));
    % betrachte auch relativen Fehler. Bei Fünfgelenkkette teilweise absoluter
    % Fehler eher im Bereich 1e-7
    test_rel = abs(ones(4,4)-Tcc(:,:,jj)./Tcu(:,:,jj));
    if any( test_abs(:) > 1e-8 ) && any(test_rel(:)>1e-6)
      [Tcc(:,:,jj),Tcu(:,:,jj)] %#ok<NOPTS>
      r2eulxyz(Tcc(1:3,1:3,jj)' * Tcu(1:3,1:3,jj))*180/pi %#ok<NOPTS>
      error('Kumulierte Koordinatentransformation %d stimmt nicht. Fehler: abs %e.rel %e', ...
        jj, max(test_abs(:)), max(test_rel(:)));
    end
  end
  % Vergleiche Geschwindigkeit und Jacobi-Matrizen
  for jj = 0:TSSc.NL-1 % 0=Basis
    r_i_i_C = rand(3,1); % Jacobi-Matrix für zufälligen Punkt
    Jgc = KAS5m7TE_jacobig_sym_varpar(q1, uint8(jj), r_i_i_C, TSSc.pkin);
    Jgo = KAS5m7OL_jacobig_sym_varpar(q,  uint8(jj), r_i_i_C, TSSo.pkin);
    % Geschwindigkeit des Punktes mit/ohne ZB
    Vc = Jgc*qD1;
    Vo = Jgo*qD;
    Delta = Vc-Vo;
    Delta_rel = Delta./Vc;
    if any( abs(Delta) > 1e7*eps(1+max(abs([Vc;Vo]))) & abs(Delta_rel) > 1e-4)
      error('Geschwindigkeit/Jacobi-Matrix stimmt nicht. Fehler abs %e, rel %e', ...
        max(abs(Delta)), max(abs(Delta_rel)));
    end
  end
  
  % Dynamik der geschlossenen Kette aus Implizit-Jacobi (Projektion)
  % Quelle: [Docquier2013]
  J1 = KAS5m7IC_kinconstr_impl_act_jacobian_mdh_sym_varpar(q, TSSi.pkin);
  JD1 = KAS5m7IC_kinconstr_impl_act_jacobianD_mdh_sym_varpar(q, qD, TSSi.pkin);

  J2 = KAS5m7IC_kinconstr_impl_pas_jacobian_mdh_sym_varpar(q, TSSi.pkin);
  JD2 = KAS5m7IC_kinconstr_impl_pas_jacobianD_mdh_sym_varpar(q, qD, TSSi.pkin);

  JD = [JD1, JD2]; % [Docquier2013], Gl. 10
  B21= -J2\J1; % [Docquier2013], Text nach Gl. 12
  B21D = (J2\JD2) * (J2\J1) - J2\JD1; % [DoThanhKotHeiOrt2009b], Gl. 20

  % Teste Geschwindigkeit
  % [Docquier2013], Gl. 14
  qD2 = B21*qD1;
  qD_impl = P0 * [qD1; qD2];
  Delta = qD_impl-qD;
  Delta_rel = Delta./qD;
  if any( abs(Delta) > 1e7*eps(1+max(abs([qD_impl;qD]))) & abs(Delta_rel) > 1e-3)
    error('Geschwindigkeiten stimmen nicht mit Herleitung über impliziter und expliziter ZB. Fehler: abs %e, rel %e', ...
      max(abs(abs(Delta))), max(abs(abs(Delta_rel))));
  end

  % Teste Beschleunigung
  % [Docquier2013], Gl. 15
  qDD2 = B21*qDD1 - J2\(JD*[qD1;qD2]);
  qDD_impl = P0 * [qDD1; qDD2];
  Delta = qDD_impl-qDD;
  Delta_rel = Delta ./ max(abs(qDD(:)));
  if any(abs(Delta) > 1e-7 & abs(Delta_rel) > 1e-3)
    error('Beschleunigungen stimmen nicht zwischen Herleitung über impliziter und expliziter ZB. Fehler: abs %e, rel %e', ...
      max(abs(Delta)), max(abs(Delta_rel)));
  end
end

fprintf('Test der Kinematik der Zwangsbedingungen mit impliziter und expliziter Form für %d Kombinationen erfolgreich\n', TSSc.n);
