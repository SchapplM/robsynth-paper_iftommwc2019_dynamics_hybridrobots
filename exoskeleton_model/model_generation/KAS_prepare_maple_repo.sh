#!/bin/bash
# Kopiere alle benötigten Dateien ins HybrDyn-Repo zur Code-Generierung
# Dieses Skript im Ordner ausführen, in dem es liegt.

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

this_path=$(pwd)
hybrdyn_repo_path=`cat hybrdyn_repo_path`

if [ "$hybrdyn_repo_path" == "" ]; then
  echo "Pfad zum Code-Generierungs-Repo nicht gesetzt!"
  exit 2
fi;

# Definitionen für Modelle KAS5m3 und KAS5m5
defpath=$hybrdyn_repo_path/robot_codegen_definitions
cp $this_path/definitions/robot_env_KAS5m3 $defpath/robot_env_KAS5m3
cp $this_path/definitions/robot_env_KAS5m5 $defpath/robot_env_KAS5m5
cp $this_path/definitions/robot_env_KAS5m5OL $defpath/robot_env_KAS5m5OL

## Unterschiedliche Modellierungen für KAS5m7 kopieren
cp $this_path/definitions/robot_env_KAS5m7 $defpath/robot_env_KAS5m7TE
sed -i "s/KAS5m7/KAS5m7TE/g" $defpath/robot_env_KAS5m7TE
echo "codegen_kinematics_opt := false:" >> $defpath/robot_env_KAS5m7TE
cp $this_path/definitions/robot_env_KAS5m7 $defpath/robot_env_KAS5m7DE1
sed -i "s/KAS5m7/KAS5m7DE1/g" $defpath/robot_env_KAS5m7DE1
echo "codegen_kinematics_opt := false:" >> $defpath/robot_env_KAS5m7DE1
echo "codegen_kinematics_subsorder:=1:" >> $defpath/robot_env_KAS5m7DE1

cp $this_path/definitions/robot_env_KAS5m7 $defpath/robot_env_KAS5m7DE2
sed -i "s/KAS5m7/KAS5m7DE2/g" $defpath/robot_env_KAS5m7DE2
echo "codegen_kinematics_opt := true:" >> $defpath/robot_env_KAS5m7DE2
echo "codegen_kinematics_subsorder:=2:" >> $defpath/robot_env_KAS5m7DE2

cp $this_path/definitions/robot_env_KAS5m7OL $defpath/robot_env_KAS5m7OL
cp $this_path/definitions/robot_env_KAS5m7IC $defpath/robot_env_KAS5m7IC

# Maple-Skripte (Kinematische Zwangsbedingungen)
cp $this_path/constraints/KAS5m3_kinematic_constraints.mpl $hybrdyn_repo_path/robot_codegen_constraints/KAS5m3_kinematic_constraints.mpl
cp $this_path/constraints/KAS5m5_kinematic_constraints.mpl $hybrdyn_repo_path/robot_codegen_constraints/KAS5m5_kinematic_constraints.mpl

cp $this_path/constraints/KAS5m7TE_kinematic_constraints.mpl $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7TE_kinematic_constraints.mpl
cp $this_path/constraints/KAS5m7DE_kinematic_constraints.mpl $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7DE1_kinematic_constraints.mpl
cp $this_path/constraints/KAS5m7DE_kinematic_constraints.mpl $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7DE2_kinematic_constraints.mpl

cp $this_path/constraints/KAS5m7OL_kinematic_constraints.mpl $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7OL_kinematic_constraints.mpl
cp $this_path/constraints/KAS5m7IC_kinematic_constraints_implicit.mpl $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7IC_kinematic_constraints_implicit.mpl

# Werte für Kinematikparameter (für Modultests)
cp $this_path/constraints/KAS5m3_kinematic_parameter_values.m $hybrdyn_repo_path/robot_codegen_constraints
cp $this_path/constraints/KAS5m5_kinematic_parameter_values.m $hybrdyn_repo_path/robot_codegen_constraints
cp $this_path/constraints/KAS5m5_kinematic_parameter_values.m $hybrdyn_repo_path/robot_codegen_constraints/KAS5m5OL_kinematic_parameter_values.m

cp $this_path/constraints/KAS5m7_kinematic_parameter_values.m $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7OL_kinematic_parameter_values.m
cp $this_path/constraints/KAS5m7_kinematic_parameter_values.m $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7IC_kinematic_parameter_values.m
cp $this_path/constraints/KAS5m7_kinematic_parameter_values.m $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7TE_kinematic_parameter_values.m
cp $this_path/constraints/KAS5m7_kinematic_parameter_values.m $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7DE1_kinematic_parameter_values.m
cp $this_path/constraints/KAS5m7_kinematic_parameter_values.m $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7DE2_kinematic_parameter_values.m

# Winkelgrenzen für die Modultests
cp $this_path/constraints/KAS5m3_kinematic_constraints_matlab.m $hybrdyn_repo_path/robot_codegen_constraints
cp $this_path/constraints/KAS5m5_kinematic_constraints_matlab.m $hybrdyn_repo_path/robot_codegen_constraints

cp $this_path/constraints/KAS5m7_kinematic_constraints_matlab.m $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7TE_kinematic_constraints_matlab.m
cp $this_path/constraints/KAS5m7_kinematic_constraints_matlab.m $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7DE1_kinematic_constraints_matlab.m
cp $this_path/constraints/KAS5m7_kinematic_constraints_matlab.m $hybrdyn_repo_path/robot_codegen_constraints/KAS5m7DE2_kinematic_constraints_matlab.m


