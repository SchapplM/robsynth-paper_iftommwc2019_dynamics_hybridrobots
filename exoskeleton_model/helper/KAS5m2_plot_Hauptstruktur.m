% Zeichne Strichmodell der Haupstruktur des KAS5 Modellierung 2 (alle Gelenke)
% 
% Input:
% T_c_mdh [4x4x9]
%   homogenious transformation matrices for each body frame

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function KAS5m2_plot_Hauptstruktur(T_c)

hold on;
for i = 1:8
    if i==1
      % Kugel für erstes KS
        plot3([T_c(1,4,1),T_c(1,4,1)],[T_c(2,4,1),T_c(2,4,1)],[T_c(3,4,1),T_c(3,4,1)],...
            T_c(1,4,1),T_c(2,4,1),T_c(3,4,1),'s','color','k',...
            'LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','k',...
            'MarkerSize',10)
    else
       % Kugel und Linienverbindung für alle kommenden KS
        plot3([T_c(1,4,i-1),T_c(1,4,i)],[T_c(2,4,i-1),...
            T_c(2,4,i)],[T_c(3,4,i-1),T_c(3,4,i)],...
            T_c(1,4,i),T_c(2,4,i),T_c(3,4,i),'o','color',...
            'k','LineWidth',4,'MarkerEdgeColor','r','MarkerFaceColor',...
            'r','MarkerSize',10)
    end
end

% Verbindung zum Endeffektor
plot3([T_c(1,4,8),T_c(1,4,9)],[T_c(2,4,8),T_c(2,4,9)],[T_c(3,4,8),T_c(3,4,9)], ...
  'LineWidth',4,'Color','k')

axis equal
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');