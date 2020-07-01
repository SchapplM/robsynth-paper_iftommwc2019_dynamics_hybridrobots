% Zeichne Strichmodell der Nebenstruktur des KAS5 Modellierung 2 (alle Gelenke)
% (Hebelmechanismus und Kompensationsfeder)
% 
% Input:
% T_c_mdh [4x4x9]
%   homogenious transformation matrices for each body frame
% r_P
%   Koordinaten der Punkte der Nebenstruktur

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function KAS5m3_plot_Nebenstruktur(T_c_mdh, r_P)

hold on;
% X-Z-Ansicht
view(0,0)
%% Text für Namen

Punktnamen = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
for i = 1:8
  % Marker für jeden Punkt
  text(r_P(i,1), r_P(i,2), r_P(i,3)-10e-3, Punktnamen{i});
end

for i = 1:7
  % Marker für jeden Punkt
  text(T_c_mdh(1,4,i+1), T_c_mdh(2,4,i+1), T_c_mdh(3,4,i+1)-10e-3, sprintf('G%d', i));
end
%% Punkte zeichnen


for i = 1:8
  % Marker für jeden Punkt
  plot3(r_P(i,1), r_P(i,2), r_P(i,3),'s','color','m',...
            'MarkerSize',10);
end

%% Zeichne Verbindungen zwischen Punkten
Verbindungsmatrix = [8, 1; ... % H-A
  1, 2; ... % A-B
  2, 3; ... % B-C
  3, 4; ... % C-D
  4, 5; ... % D-E
  5, 6]; % E-F

for i = 1:size(Verbindungsmatrix,1)
  I1 = Verbindungsmatrix(i,1);
  I2 = Verbindungsmatrix(i,2);
  
  plot3(...
    [r_P(I1,1); r_P(I2,1)], ...
    [r_P(I1,2); r_P(I2,2)], ...
    [r_P(I1,3); r_P(I2,3)], ...
    'color', 'b','LineWidth',4);
end
  
%% Zeichne Verbindungen zwischen Gelenken und Punkten
Verbindungsmatrix_GP = [...
  6, 2; ... % G5-B
  4, 6]; ... % G3-F
  
for i = 1:size(Verbindungsmatrix_GP,1)
  I1 = Verbindungsmatrix_GP(i,1);
  I2 = Verbindungsmatrix_GP(i,2);
  
  plot3(...
    [T_c_mdh(1,4,I1); r_P(I2,1)], ...
    [T_c_mdh(2,4,I1); r_P(I2,2)], ...
    [T_c_mdh(3,4,I1); r_P(I2,3)], ...
    'color', 'b','LineWidth',4);
end

