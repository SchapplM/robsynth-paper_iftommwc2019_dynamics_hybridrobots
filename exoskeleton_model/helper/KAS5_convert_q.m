% Konvertiere die verallgemeinerten Koordinaten der unterschiedlichen
% Modelle
% 
% Eingabe:
% q_mi
%   Verallgemeinerte Koordinaten von Modell i
% i
%   Modellnummer der Eingabedaten
% j
%   Modellnummer der Ausgabedaten
% 
% Ausgabe:
% q_mj
%   Verallgemeinerte Koordinaten von Modell j

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-02
% (c) Institut für Regelungstechnik, Universität Hannover

function q_mj = KAS5_convert_q(q_mi, i, j)

assert(isa(q_mi,'double') && isreal(q_mi) && all(size(q_mi) <= [7 1]), ...
  'Gelenkwinkel q_mi gefordert als [Nx1] double');

rho = NaN(7,1); % Winkel der Hauptstruktur. Entspricht q_m2
[~, w_const] = KAS5_parameter_kinematic();
delta18 = w_const(6);
% Eingabe- und Ausgabemodell sind gleich
if i == j
  q_mj = q_mi;
  return
end

if i == 4 && (j == 3 || j == 5)
  rho = q_mi;
  if j == 3 || j == 5
    % Nur Korrekt, wenn rho3 mit den Zwangsbedingungen von KAS5m3
    % übereinstimmt.
    [~, w_par] = KAS5m3_kine_parallel(rho([1 2 4 5 6 7]));
    if abs(w_par(end) - rho(3)) > 1e-10
      error('Umwandlung nicht moeglich, Eingabedaten nicht mit Zwangsbedingungen konsistent');
    end
    if j == 3
      q_mj = rho([1 2 4 5 6 7]);
    elseif j == 5
      q_mj = rho([1 2 4 5 7]);
    end
    return
  end
end
% Eingabemodell ist m6
if i == 6 && (j == 4 || j == 3 || j == 2 || j == 5 )
  % Eingabedaten enthalten weniger Informationen
  rho([1 2 3 4 5 7]) = q_mi;
  % rho6 aus Ellenbogenkopplung
  rho(6) = rho(5) + delta18 - rho(4);
  q_m4 = rho;
  if j == 4
    q_mj = q_m4;
    return;
  end
  if j == 2
    % Gleiche Winkel für m2 und m4
    q_mj = q_m4;
    return;
  end
  if j == 3 || j == 5
    % Nur Korrekt, wenn rho3 mit den Zwangsbedingungen von KAS5m3
    % übereinstimmt.
    [~, w_par] = KAS5m3_kine_parallel(rho([1 2 4 5 6 7]));
    if abs(w_par(end) - rho(3)) > 1e-10
      error('Umwandlung nicht moeglich, Eingabedaten nicht mit Zwangsbedingungen konsistent');
    end
    if j == 3
      q_mj = rho([1 2 4 5 6 7]);
    elseif j == 5
      q_mj = rho([1 2 4 5 7]);
    end
    return
  end
end

% Eingabemodell ist m5
if i == 5 && (j == 2 || j == 3 || j == 4 || j == 6)
  % Eingabedaten enthalten weniger Informationen
  rho([1 2 4 5 7]) = q_mi;
  % rho6 aus Ellenbogenkopplung
  rho(6) = rho(5) + delta18 - rho(4);
  q_m3 = rho([1 2 4 5 6 7]);
  if j == 3
    q_mj = q_m3;
    return;
  end
  % rho3 aus Parallelmechanismus
  [~, w_par] = KAS5m3_kine_parallel(q_m3);
  rho(3) = w_par(end);
  if j == 2 || j == 4
    q_mj = rho;
  end
  if j == 6
    q_mj = rho([1 2 3 4 5 7]);
  end
  return;
end

% Eingabemodell ist m3
if i == 3 && (j == 2 || j == 4)
  rho([1 2 4 5 6 7]) = q_mi;
  % rho3 aus Parallelmechanismus
  [~, w_par] = KAS5m3_kine_parallel(q_mi);
  rho(3) = w_par(end);
  q_mj = rho;
  return;
end

error('Kombination noch nicht implementiert');
