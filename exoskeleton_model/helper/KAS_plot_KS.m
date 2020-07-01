% Zeichne Koordinatensysteme des KAS5_m4
% 
% Input:
% T_c_mdh [4x4xN]
%   Transformationsmatrizen
% version [1x1 uint8]
%   Version des KAS (z.B. 5)
% modell [1x1 uint8]
%   Modellnummer des KAS (z.B. 2)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Universität Hannover


function KAS_plot_KS(T_c_mdh, version, modell)
  
hold on;

%% Plot frames
if version == 5 && modell == 2
  for i = 1:8
      trplot(T_c_mdh(:,:,i), 'frame', sprintf('%d',i-1), 'arrow', 'rgb', 'length', 0.15)
  end
elseif version == 5 && modell == 4
  for i = 1:15
      trplot(T_c_mdh(:,:,i), 'frame', sprintf('%d',i-1), 'arrow', 'rgb', 'length', 0.15)
  end
elseif version == 5 && modell == 7
  for i = 1:23
      trplot(T_c_mdh(:,:,i), 'frame', sprintf('%d',i-1), 'arrow', 'rgb', 'length', 0.15)
  end
elseif version == 6 && modell == 1
  for i = 1:11
      trplot(T_c_mdh(:,:,i), 'frame', sprintf('%d',i-1), 'arrow', 'rgb', 'length', 0.15)
  end
end
end
