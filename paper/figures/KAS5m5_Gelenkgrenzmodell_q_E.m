% Simulation KAS5m5 mit analytischem Modell und speichern von Bildern für
% den BMBF-Abschlussbericht
% 
% [x] Gelenkgrenzmodell (Hunt-Crossley)
% siehe KAS5_m5_testmodel_02_settings_01.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-12
% (c) Institut fuer Regelungstechnik, Universitaet Hannover

clc
clear

%% Init

KAS_docs_path = fileparts(which('bmbf_3arm_docs_init'));
if isempty(KAS_docs_path)
  error('Das Hauptverzeichnis des 3arm-docs-Repo muss im Matlab-Pfad liegen');
end
export_pfad = fullfile(KAS_docs_path, 'abschlussbericht', 'bilder');
    
%% Simulink-Modell vorbereiten
S = KAS5_const(5);
KAS5_testmodel_01_settings_default

Mensch_F_max = Mensch_F_max*0;
Mensch_F_min = Mensch_F_min*0;
taskspace_trajectory = struct('t', 0, 'xq', [0;0;0;1;0;0;0]', 'xD', zeros(1,6));

usr_time  =2;

% Feder
k_f = 1000;
l_feder0 = 30e-3;


%% Start Modell    
sl_Modellname = 'KAS5_m5_testmodel_01';

KAS5_testmodel_start;    

%% Nachverarbeitung

%% Plot
PlotSettings.KAS_version = KAS_version;
PlotSettings.KAS_modell = KAS_modell;
PlotSettings.plot_q = false;
PlotSettings.plot_q_ges = true;
PlotSettings.plot_qD = false;
PlotSettings.plot_qDD = false;
PlotSettings.plot_tau = false;
PlotSettings.plot_E = true;
PlotSettings.plot_skizze = [false, 2,5];
PlotSettings.I_plot = 1:5:length(sl.t); % spärlich plotten, damit Dateigröße gering ist. Ändert aber scheinbar nichts.
KAS_simulink_plot_results;

%% Bilder Formatieren und speichern: Gelenkwinkel und Energie
CellFigfiles = cell(1,2);
CellFigfiles{1} = fullfile(export_pfad, 'winkel.fig');
CellFigfiles{2} = fullfile(export_pfad, 'energie.fig');
saveas(201, CellFigfiles{1});
saveas(11, CellFigfiles{2});

figure(20);clf;
axhdl = set_fig2subfig(20,CellFigfiles);

axes(axhdl(1));
ch = get(axhdl(1), 'children');
legend(ch)
ylabel('Gelenkwinkel [rad]');
xlabel('t [s]');
title('');
grid on;
leghdl = legend(ch, {'$\rho_1$', '$\rho_2$', '$\rho_4$', '$\rho_5$', '$\rho_7$'}, ...
  'interpreter', 'latex');
set(leghdl, 'Position', [0.38, 0.30, .1, .08])

axes(axhdl(2));
ch = get(axhdl(2), 'children');
leghdl = legend(ch([1,2,3,4,5,6]), {'E_{ges}+E_{Diss}', 'E_{ges}', 'U_{Feder}', 'U_{Grenz}', 'U_{grav}', 'T'}); % 'E_{ges}', 'U_{limit}', 'U_{grav}', 'T'
set(leghdl, 'Position', [0.85, 0.36, .1, .08])
ylabel('Energie [J]');
xlabel('t [s]');
title('');
grid on;

set_sizePositionPlotSubplot_byHandle(20, ...
  15.8, 6, ... 
  axhdl, ...
  0.08, 0.01, 0.02, 0.18, ... % l, r, u, d
  0.1, 0.08); % dx, dy


bmbf_zwischenbericht_format

% Linienstärke erhöhen
ha1 = findobj(gcf,'Type','axes');
for i = 1:length(ha1)   
  ch = get(ha1(i), 'Children');
  for c = ch'
    set(c, 'LineWidth', 1.5);
  end
end

mkdirs(export_pfad);
export_fig(20, fullfile(export_pfad, 'BMBF_Abschlussbericht_KAS5m5_Gelenkgrenzmodell_q_E.fig'));
export_fig(20, fullfile(export_pfad, 'BMBF_Abschlussbericht_KAS5m5_Gelenkgrenzmodell_q_E.eps'));
export_fig(20, fullfile(export_pfad, 'BMBF_Abschlussbericht_KAS5m5_Gelenkgrenzmodell_q_E.pdf'));
export_fig(20, fullfile(export_pfad, 'BMBF_Abschlussbericht_KAS5m5_Gelenkgrenzmodell_q_E.png'));
