% Teste Dynamikfunktionen für KAS5m7IC mit denen aus KAS5m7TE
% Rechnet Dynamikterme von der offenen auf die geschlossene Struktur um.

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

TSSc = KAS5m7TE_varpar_testfunctions_parameter; % mit expl. Zwangsbedingungen
TSSi = KAS5m7IC_varpar_testfunctions_parameter; % mit impl. Zwangsbedingungen
TSSo = KAS5m7OL_varpar_testfunctions_parameter; % ohne Zwangsbedingungen

% Dynamikparameter angleichen
TSSc.m = TSSo.m;
TSSc.rSges = TSSo.rSges;
TSSc.Icges = TSSo.Icges;
TSSc.mrSges = TSSo.mrSges;
TSSc.Ifges = TSSo.Ifges;

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
  g = TSSc.G(iq,:)';

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

  % Dynamik der offenen Kette
  tau = KAS5m7OL_invdynJ_fixb_slag_vp2(q, qD, qDD, g, ...
  TSSo.pkin, TSSo.m, TSSo.mrSges, TSSo.Ifges);

  % Dynamik der geschlossenen Kette aus Jacobi (Projektion)
  % [NakamuraGho1989], Gl. 5
  tau1 = W' * tau;

  % Vergleich mit Lösung mit Elimination
  tau1_test = KAS5m7TE_invdynJ_fixb_slag_vp2(q1, qD1, qDD1, g, ...
  TSSc.pkin, TSSc.m, TSSc.mrSges, TSSc.Ifges);

  Delta = tau1-tau1_test;
  Delta_rel = Delta./max(abs(tau1));
  if any( abs(Delta(:)) > 1e8*eps(max(abs(tau1))) & abs(Delta_rel) > 1e-2 ) % lockere Toleranz: 1% Fehler
    error('Dynamik mit ZB-Jacobi stimmt nicht mit Dynamik aus Eliminations-Ansatz. Fehler: abs %e, rel %e', ...
      max(abs(Delta)), max(abs(Delta_rel)));
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

  % Teste inverse Dynamik
  % [Docquier2013], Gl. 12
  tauIC = P1*tau + B21'*P2*tau;

  Delta = tau1-tauIC; % Absoluter Fehler (kann größer sein bei komplizierter Berechnung)
  Delta_rel = Delta./tau1; % Relativer Fehler. Muss immer klein sein
  if any( abs(Delta(:)) > 1e6*eps(1+max(abs(W(:)))) & Delta_rel > 1e-4)
    error('Dynamik aus TE-ZB-Jacobi stimmt nicht mit der aus IC-ZB-Jacobi überein');
  end

  % Test symbolisch berechneter inverse Dynamik über impl. Zwangsbedingungen mit numerischer Projektion
  tauICs = KAS5m7IC_invdynJ_fixb_slag_vp2(q, qD, qDD, g, ...
    TSSi.pkin, TSSo.m, TSSo.mrSges, TSSo.Ifges);

  test = tauIC-tauICs;
  if any( abs(test(:)) > 1e6*eps(max(abs(W))) )
      error('Dynamik symbolischer Berechnung stimmt nicht mit numerischer überein');
  end

  % Test inverse Dynmik Newton-Euler vs Lagrange
  if ~isempty(which('KAS5m7IC_invdynJ_fixb_snew_vp2')) % in einigen Fällen kann die Newton-Euler-Dynamik nicht berechnet werden (z.B. sigma=2)
  tauICs_snew = KAS5m7IC_invdynJ_fixb_snew_vp2(q, qD, qDD, g, ...
    TSSi.pkin, TSSo.m, TSSo.mrSges, TSSo.Ifges);

  test = tauICs_snew-tauICs;
  if any( abs(test(:)) > 1e6*eps(max(abs(W))) )
    error('Dynamik Newton-Euler stimmt nicht mit Lagrange überein');
  end
  end

  % Test Gravitationsmoment
  taug1_expl = KAS5m7TE_gravloadJ_floatb_twist_slag_vp2(q1, g, TSSc.pkin, TSSc.m, TSSc.mrSges);
  taug1_impl = KAS5m7IC_gravloadJ_floatb_twist_slag_vp2(q, g, TSSi.pkin, TSSo.m, TSSo.mrSges);
  Delta = taug1_expl-taug1_impl;
  Delta_rel = Delta./taug1_expl;
  if any(abs(Delta) > 1e-8 & abs(Delta_rel) > 1e-4)
    error('Gravitationsmoment mit ZB-Jacobi stimmt nicht mit Gravitationsmoment aus Eliminations-Ansatz. Fehler: abs %e, rel %e', ...
      max(abs(Delta)), max(abs(Delta_rel)));
  end

  % Test Gravitationsmoment mit Newton-Euler
  if ~isempty(which('KAS5m7IC_invdynJ_fixb_snew_vp2')) % in einigen Fällen kann die Newton-Euler-Dynamik nicht berechnet werden (z.B. sigma=2)
  taug1_snew_impl = KAS5m7IC_gravloadJ_floatb_twist_snew_vp2(q, g, TSSi.pkin, TSSo.m, TSSo.mrSges);

  if any(abs(taug1_snew_impl-taug1_impl) > 1e-10)
    error('Gravitationsmoment mit ZB-Jacobi und Newton-Euler stimmt nicht mit Gravitationsmoment aus Lagrange');
  end
  end

  % Test Massenmatrix
  MM1_impl = KAS5m7IC_inertiaJ_slag_vp2(q, TSSi.pkin, TSSo.m, TSSo.mrSges, TSSo.Ifges);
  MM1_expl = KAS5m7TE_inertiaJ_slag_vp2(q1, TSSc.pkin, TSSc.m, TSSc.mrSges, TSSc.Ifges);

  Delta = MM1_impl(:) - MM1_expl(:);
  Delta_rel = Delta ./ MM1_expl(:);
  if any( abs(Delta) > 1e7*eps(1+max(abs(Delta))) & abs(Delta_rel) > 1e-4)
    error('Massenmatrix mit ZB-Jacobi stimmt nicht mit Massenmatrix aus Eliminations-Ansatz. Fehler abs: %e, rel: %e', ...
      max(abs(Delta)), max(abs(Delta_rel)));
  end

  % Test Coriolisvektor
  tauCC1_offen = KAS5m7OL_coriolisvecJ_fixb_slag_vp2(q, qD, TSSo.pkin, TSSo.m, TSSo.mrSges, TSSo.Ifges);
  tauCC1_expl = KAS5m7TE_coriolisvecJ_fixb_slag_vp2(q1, qD1, TSSc.pkin, TSSc.m, TSSc.mrSges, TSSc.Ifges);

  MM1_offen = KAS5m7OL_inertiaJ_slag_vp2(q, TSSo.pkin, TSSo.m, TSSo.mrSges, TSSo.Ifges);

  tauCC1_impl = W' * tauCC1_offen + W' * MM1_offen * WD * qD1;

  Delta = tauCC1_impl - tauCC1_expl;
  Delta_rel = Delta./tauCC1_expl; % höhere Test-Toleranz als andere Terme, da kompliziertere Berechnung
  if any( abs(Delta(:)) > 1e8*eps(1+max(abs(Delta))) & abs(Delta_rel)>1e-3 )
    error('Coriolisvektor mit ZB-Jacobi stimmt nicht mit Coriolisvektor aus Eliminations-Ansatz. Fehler: abs %e; rel %e', ...
      max(abs(Delta(:))), max(abs(Delta_rel)));
  end
end

fprintf('Test der Zwangsbedingungen mit impliziter und expliziter Form für %d Kombinationen nach [NakamuraGho1989] erfolgreich\n', TSSc.n);

%% Test der Zwangsbedingungen mit Gauß'schem Prinzip des kleinsten Zwangs
% Nach [UdwadiaKal1992]

% Funktioniert noch nicht, da Gelenkmomente an Koppelgelenken null sind
fprintf('Tests der Zwangsbedingungen mit impliziter und expliziter Form nach [UdwadiaKal1992] entfallen, da noch nicht implementiert\n');
return

II = TSSc.Ind_depjoints; % Indizes n2 in n
n = TSSo.NJ; % Anzahl Gelenke
n1 = TSSc.NQJ; % Anzahl unabhängige Gelenke
n2 = sum(II); % Anzahl abhängige Gelenke

for iq = 1:TSSc.n
    g = TSSc.G(iq,:)';

    % Minimalkoordinaten für System mit ZB
    q1 = TSSc.Q(iq,:)';
    qD1 = TSSc.QD(iq,:)';

    % Umrechnung von System mit ZB auf offenes System ohne ZB
    W = KAS5m7TE_kinconstr_expl_jacobian_mdh_sym_varpar(q1, TSSc.pkin);
    WD = KAS5m7TE_kinconstr_expl_jacobianD_mdh_sym_varpar(q1, qD1, TSSc.pkin);
    % Gelenkposition und -geschwindigkeit ohne ZB
    q = KAS5m7TE_kinconstr_expl_mdh_sym_varpar(q1, TSSc.pkin);

    % Korrektur
    q = q(II_oc);
    W = W(II_oc,:);
    WD = WD(II_oc,:);

    qD = W*qD1;

    % Zwangsbedingungen: Bestimme implizite Formulierung nach [Docquier2013]
    % aus expliziter Formulierung (siehe Aufzeichnungen Schappler, 19.12.2017)
    F = W(II,:);
    FD = WD(II,:);
    J = NaN(n2,n); % Aus der Zeitableitung der impliziten ZB nach den Koordinaten des offenen Systems
    JD = NaN(n2,n);
    J(:,1:n1) = -F;
    J(:,n1+1:n1+n2) = eye(n2,n2);
    JD(:,1:n1) = -FD;
    JD(:,n1+1:n1+n2) = zeros(n2,n2);

    % Zwangsbedingungen in Form aus [UdwadiaKal1992], Gl. (3) konvertieren
    % (mit Koeffizientenvergleich der ZB-Gleichung (siehe [Docquier2013]).
    A = J;
    b = -JD*qD;

    % Dynamik ohne ZB ([UdwadiaKal1992], Gl. (1), (2)
    M = KAS5m7OL_inertiaJ_slag_vp1(q, ...
        TSSo.pkin, TSSo.m, TSSo.rSges, TSSo.Icges);
    Q = -KAS5m7OL_invdynJ_fixb_slag_vp1(q, qD, qD*0, g, ...
        TSSo.pkin, TSSo.m, TSSo.rSges, TSSo.Icges);
    a = M \ Q;

    K = sqrtm(M) * pinv( A / sqrtm(M) ); % [UdwadiaKal1992] Text nach Gl. (5a')
    Qc = K*(b - A*(M\Q)); % [UdwadiaKal1992] Gl. (5b)

    % Beschleunigung des offenen Systems mit Kräften aus den Zwangsbedingungen
    qDD2 = M\(Q+Qc);

    % Referenzlösung: Beschleunigung des geschlossenen Systems (mit ZB)
    M1 = KAS5m7TE_inertiaJ_slag_vp2(q1, ...
        TSSc.pkin, TSSc.m, TSSc.mrSges, TSSc.Ifges);
    tau1 = KAS5m7TE_invdynJ_fixb_slag_vp2(q1, qD1, qD1*0, g, ...
        TSSc.pkin, TSSc.m, TSSc.mrSges, TSSc.Ifges);
    qDD1 = M1 \ (-tau1);
    qDD2_test = W*qDD1 + WD*qD1;

    test = qDD2 - qDD2_test;
    if any(abs(test) > 1e-8)
        error('Abgleich nach [UdwadiaKal1992] Fehlgeschlagen');
    end
end

fprintf('Test der Zwangsbedingungen mit impliziter und expliziter Form für %d Kombinationen nach [UdwadiaKal1992] erfolgreich\n', TSSc.n);


