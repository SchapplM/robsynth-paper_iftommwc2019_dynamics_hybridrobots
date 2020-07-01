
# Kinematik-Berechnung 3. Arm KAS5 Modell 7
# Beschreibung
# 
# Berechnung der kinematischen Zwangsbedingungen des Systems KAS5m7 in impliziter Form.
# Ansatz: Schließen von Gelenkketten nach [1] und Anwendung der Zahnrad-Zwangsbedingungen.
# 
# Quellen
# [1] Khalil and Bennis: Symbolic calculation of the base inertial parameters of closed-loop robots (1995)
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2018-02
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Initialisierung
restart:
kin_constraints_exist := true: # Für Speicherung
;
with(StringTools): # Für Zeitausgabe
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
codegen_act := true:
codegen_opt := 1: # Geringerer Optimierungsgrad. Sonst zu lange.
codegen_debug := 0: # Zur Code-Generierung auch für Nicht-Inert-Ausdrücke
;
read "../helper/proc_MatlabExport":
read "../transformation/proc_rotx":
read "../transformation/proc_roty":
read "../transformation/proc_rotz":
read "../transformation/proc_trotz":
read "../transformation/proc_transl":
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s":
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
read "../helper/proc_intersect_circle":
with(RealDomain): # Schränkt alle Funktionen auf den reellen Bereich ein. Muss nach Definition von MatlabExport kommen. Sonst geht dieses nicht.
;
read "../robot_codegen_definitions/robot_env_IC":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name_OL):
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name_OL, base_method_name);
read "../robot_codegen_definitions/robot_env_IC":
Trf := Trf:
Trf_c := Trf_c:
# Schleife über B-C-D
# 3-4-12-13-17
T_3_17 := combine( Matrix(Trf(1..4,1..4, 4)) . Matrix(Trf(1..4,1..4,12)) . Matrix(Trf(1..4,1..4,13)) . Matrix(Trf(1..4,1..4,17)) ):
# 3-10-11-20
T_3_20 := combine( Matrix(Trf(1..4,1..4, 10)) . Matrix(Trf(1..4,1..4,11)) . Matrix(Trf(1..4,1..4,20)) ):
h1 := T_3_17(1..3,4) - T_3_20(1..3,4):
# Schleife über 2-F-E
# 
# 2-3-10-16
T_2_16 := combine( Matrix(Trf(1..4,1..4, 3)) . Matrix(Trf(1..4,1..4,10)) . Matrix(Trf(1..4,1..4,16)) ):
# 2-8-9-19
T_2_19 := combine( Matrix(Trf(1..4,1..4, 8)) . Matrix(Trf(1..4,1..4,9)) . Matrix(Trf(1..4,1..4,19)) ):
# Vektorkette
h2 := T_2_16(1..3,4) - T_2_19(1..3,4):
# Schleife über 5-B-A

# 4-5-6-18
T_4_18 := combine( Matrix(Trf(1..4,1..4, 5)) . Matrix(Trf(1..4,1..4,6)) . Matrix(Trf(1..4,1..4,18)) ):
# 4-12-13-14-15-21
T_4_21 := combine( Matrix(Trf(1..4,1..4, 12)) . Matrix(Trf(1..4,1..4,13)) . Matrix(Trf(1..4,1..4,14)) . Matrix(Trf(1..4,1..4,15)) . Matrix(Trf(1..4,1..4,21)) ):
# Vektorkette
h3 := T_4_18(1..3,4) - T_4_21(1..3,4):
# Zahnrad-Kontakt
# 
hz1 := -qJ_t(4) + qJ_t(11):
hz2 := -qJ_t(6) + qJ_t(5) - qJ_t(4) + delta18:
h_user := <hz1; hz2>:
# Zusammenstellen aller Zwangsbedingungen
# 
implconstr_t := <h1([1, 2],1);h2([1, 3],1);h3([1, 2],1); h_user>:
implconstr_s := convert_t_s(implconstr_t):
# Exportiere Code für folgende Skripte
# 
# 
kin_constraints_exist:=true:
save implconstr_t, implconstr_s, kin_constraints_exist, sprintf("../codeexport/%s/tmp/kinematic_constraints_implicit_maple.m", robot_name):
# Exportieren des vollständigen Ausdruckes
if codegen_act then
  MatlabExport(implconstr_s, sprintf("../codeexport/%s/tmp/kinconstr_impl_matlab.m", robot_name), 2):
end if:
# Liste mit abhängigen konstanten Kinematikparametern erstellen (wichtig für Matlab-Funktionsgenerierung)
read "../helper/proc_list_constant_expressions";
kc_symbols := Matrix(list_constant_expressions( implconstr_s )):
save kc_symbols, sprintf("../codeexport/%s/tmp/kinematic_implicit_constraints_symbols_list_maple", robot_name):
MatlabExport(Transpose(kc_symbols), sprintf("../codeexport/%s/tmp/kinematic_implicit_constraints_symbols_list_matlab.m", robot_name), 2):
interface(rtablesize=100):
kc_symbols
;
