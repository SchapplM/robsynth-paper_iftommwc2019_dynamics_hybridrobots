
# Kinematik-Berechnung 3. Arm KAS5
# Beschreibung
# 
# Modell m5: 
# * Kurbel N7 ist starr mit Körper K2 gekoppelt (über die Achse) (wie m3), 
# * Zahnradkopplung zwischen Körpern Z2 und Z3 (Unterschied zu m3)
# 
# Setze die Zwangsbedingung (Zahnradkopplung) in die bereits berechneten Zwangsbedingungen für das KAS5m3 ein
# 
# Berechnung der Kopplung der Zahnräder am Ellenbogen
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-02
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Initialisierung
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
kin_constraints_exist := true: # Für Speicherung
;
with(LinearAlgebra):
with(StringTools): # Für Zeitausgabe
;
with(codegen):
with(CodeGeneration):
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
codegen_act := true:
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Lese Ergebnisse der Kinematik von KAS5m3 aus.
# Siehe KAS5m3_sym_codegen_kinematic_constraints.mw
# kintmp_qs, kintmp_qt, lpar_qs, lpar_qt, kintmp_subsexp
read  sprintf("../codeexport/KAS5m5/tmp/kinematic_constraints_maple_inert.m", robot_name):
kintmp_qs_KAS5m5 := kintmp_qs:
kintmp_qt_KAS5m5 := kintmp_qt:
lpar_qs_KAS5m5 := lpar_qs:
lpar_qt_KAS5m5 := lpar_qt:
kintmp_subsexp_KAS5m5 := kintmp_subsexp:
read  sprintf("../codeexport/KAS5m5/tmp/tree_floatb_definitions", robot_name):
kintmp_s_KAS5m5 := kintmp_s:
kintmp_t_KAS5m5 := kintmp_t:
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
kintmp_s := kintmp_s:
kintmp_t := kintmp_t:
kintmp_qs := kintmp_s:
kintmp_qt := kintmp_t:
kintmp_subsexp := Matrix(2*RowDimension(kintmp_s),2):
for i from 1 to RowDimension(kintmp_s) do
  kintmp_subsexp(2*i-1, 1) := sin(kintmp_s(i,1)):
  kintmp_subsexp(2*i,   1) := cos(kintmp_s(i,1)):
  # Initialisierung der rechten Spalte mit gleichen Werten. Später nur Ersetzung, wenn Vorteilhaft.
  kintmp_subsexp(2*i-1, 2) := kintmp_subsexp(2*i-1, 1):
  kintmp_subsexp(2*i,   2) := kintmp_subsexp(2*i,   1):
end do:
printf("Beginn der Berechnungen. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
st := time():
# Übernehme die Ersetzungsausdrücke von KAS5m5
# 
for i from 1 to RowDimension(kintmp_s_KAS5m5) do
  for j from 1 to RowDimension(kintmp_s) do
    if kintmp_s_KAS5m5(i,1) = kintmp_s(j,1) then
      # printf("KAS5m5 %d = KAS5m7 %d\n", i, j):
      kintmp_qs(j,1) := kintmp_qs_KAS5m5(i,1):
      kintmp_qt(j,1) := kintmp_qt_KAS5m5(i,1):
    end if:
  end do:
end do:
for i from 1 to RowDimension(kintmp_subsexp_KAS5m5) do
  for j from 1 to RowDimension(kintmp_subsexp) do
    if kintmp_subsexp_KAS5m5(i,1) = kintmp_subsexp(j,1) then
      # printf("KAS5m5 %d = KAS5m7 %d\n", i, j):
      kintmp_subsexp(j,2) := kintmp_subsexp_KAS5m5(i,2):
    end if:
  end do:
end do:
# Hierbei werden die Definitionen für kintmp_s und kintmp_t überschrieben, da die Elemente in kintmp_qs und kintmp_qt drin stehen.
# Umgehung des Problems: Definitionen neu laden.
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
kintmp_s := kintmp_s:
kintmp_t := kintmp_t:
interface(rtablesize=100):
# Korrigiere die Bedingungen. Dort wurden die konstanten Winkel auch mit substitutierten Variablen "s" bezeichnet, was eigentlich keinen Sinn ergibt.
# Die mit Suffix "s" gekennzeichneten substituierten Winkel beziehen sich nur auf Zeitableitungen, die mit einem konstanten Ausdruck ersetzt wurden.
# Damit nicht delta8 und delta8s als unterschiedliche Variablen auftauchen, werden diese ersetzt.
subs_kp1 := <delta8s; delta9s; delta10s; delta12s; delta17s; delta18s>:
subs_kp2 := <delta8;  delta9;  delta10;  delta12;  delta17;  delta18>:
for i from 1 to RowDimension(subs_kp1) do
  for j from 1 to RowDimension(kintmp_subsexp) do
    kintmp_subsexp(j,2) := subs({subs_kp1(i,1)=subs_kp2(i,1)}, kintmp_subsexp(j,2));
  end do:
  for j from 1 to RowDimension(kintmp_qs) do
    kintmp_qs(j,1) := subs({subs_kp1(i,1)=subs_kp2(i,1)}, kintmp_qs(j,1));
    kintmp_qt(j,1) := subs({subs_kp1(i,1)=subs_kp2(i,1)}, kintmp_qt(j,1));
  end do:
  lpar_qs_KAS5m5 := subs({subs_kp1(i,1)=subs_kp2(i,1)}, lpar_qs_KAS5m5);
end do:
# Zahnrad Abwälzbedingung (delta19)
# Siehe Aufzeichnungen vom 3.6.2016
kintmp_s(24);
kintmp_subsexp(47,2);
kintmp_subsexp(48,2);
delta19_qs := convert_t_s(theta(4)):
kintmp_qs(24) := delta19_qs:
kintmp_subsexp(47,2) := sin(delta19_qs):
kintmp_subsexp(48,2) := cos(delta19_qs):
# Zahnrad Abwälzbedingung (rho6)
# Siehe Aufzeichnungen vom 3.6.2016
kintmp_s(28);
kintmp_subsexp(55,2);
kintmp_subsexp(56,2);
rho6_qs := convert_t_s(theta(5)) - convert_t_s(theta(4)) + delta18:
kintmp_qs(28) := rho6_qs:
kintmp_subsexp(55,2) := sin(rho6_qs):
kintmp_subsexp(56,2) := cos(rho6_qs):
# Berechne delta7
# Gl. (30)
kintmp_s(12);
kintmp_subsexp(23,2);
kintmp_subsexp(24,2);
delta7_qs := delta17 + delta19_qs: # Geändert ggü. KAS5
;
kintmp_qs(12) := delta7_qs:
kintmp_subsexp(23,2) := sin(delta7_qs):
kintmp_subsexp(24,2) := cos(delta7_qs):

# Dummy-Einträge für Schnitt-Gelenke
# Die Winkel der Schnittgelenke sind egal, daher keine Berechnung.

# delta 14
# 
kintmp_s(19);
kintmp_subsexp(37,2);
kintmp_subsexp(38,2);
kintmp_qs(19) := 0:
# delta 21
# 
kintmp_s(26);
kintmp_subsexp(51,2);
kintmp_subsexp(52,2);
kintmp_qs(26) := 0:

# Setze Federlänge als Variable
kintmp_s(29);
kintmp_qs(29) := lpar_qs_KAS5m5:
# Nachverarbeiten
# 
kintmp_qt := convert_s_t(kintmp_qs):
# Speichern der Ergebnisse
save kin_constraints_exist, kintmp_qs, kintmp_qt, kintmp_subsexp, sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert", robot_name):
save kin_constraints_exist, kintmp_qs, kintmp_qt, kintmp_subsexp, sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
# Exportieren der Zwangsbedingungen
# Matlab-Funktionen für ZB
if codegen_act then
  MatlabExport(kintmp_subsexp, sprintf("../codeexport/%s/tmp/kinematik_parallel_kintmp_subsexp_matlab_opt1.m", robot_name), 1):
end if:
printf("Ausdrücke mit Inert-Arctan exportiert (Matlab). %s. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
# Liste der Parameter speichern
# Liste mit abhängigen konstanten Kinematikparametern erstellen (wichtig für Matlab-Funktionsgenerierung)
read "../helper/proc_list_constant_expressions";
kc_symbols := Matrix(list_constant_expressions( kintmp_subsexp(..,2) )):
save kc_symbols, sprintf("../codeexport/%s/tmp/kinematic_constraints_symbols_list_maple", robot_name):
MatlabExport(Transpose(kc_symbols), sprintf("../codeexport/%s/tmp/kinematic_constraints_symbols_list_matlab.m", robot_name), 2):
printf("Zwangsbedingungen der Parallelstruktur von KAS5m5 nach KAS5m7 angepasst. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):

