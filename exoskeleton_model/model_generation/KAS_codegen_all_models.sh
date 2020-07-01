#!/bin/bash -ex
# Starte die Code-Generierung für alle validierten Roboter dieses Repos
#
# Das Skript muss in dem Verzeichnis ausgeführt werden, in dem es liegt
# Es muss eine Datei hybrdyn_repo_path im selben Ordner angelegt werden, die den Pfad zum HybrDyn-Repo enthält (und nichts anderes)

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-07
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

hybroblib_repo_path=$(pwd)
hybrdyn_repo_path=`cat hybrdyn_repo_path`

cd $hybrdyn_repo_path
echo "### Starte Code-Generierung für KAS5m3 ###"
cp robot_codegen_definitions/robot_env_KAS5m3 robot_codegen_definitions/robot_env
./robot_codegen_start.sh --fixb_only -p --kinematics_only

echo "### Starte Code-Generierung für KAS5m5 ###"
cp robot_codegen_definitions/robot_env_KAS5m5 robot_codegen_definitions/robot_env
./robot_codegen_start.sh --fixb_only -p --kinematics_only

echo "### Starte Code-Generierung für KAS5m5OL ###"
cp robot_codegen_definitions/robot_env_KAS5m5OL robot_codegen_definitions/robot_env
./robot_codegen_start.sh --fixb_only -p --kinematics_only

echo "### Starte Code-Generierung für KAS5m7 ###"
deflist="
robot_env_KAS5m7TE
robot_env_KAS5m7DE1
robot_env_KAS5m7DE2
"

for df in $deflist; do
  echo "Starte Generierung für $df"
  cp robot_codegen_definitions/$df robot_codegen_definitions/robot_env
  ./robot_codegen_start.sh --fixb_only -p
done
cp robot_codegen_definitions/robot_env_KAS5m7OL robot_codegen_definitions/robot_env
cp robot_codegen_definitions/robot_env_KAS5m7IC robot_codegen_definitions/robot_env_IC
./robot_codegen_start.sh --fixb_only --ic -p

