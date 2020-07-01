#!/bin/bash
# Kopiere alle benötigten Dateien ins HybrDyn-Repo zur Code-Generierung
# Dieses Skript im Ordner ausführen, in dem es liegt.

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-06
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

this_path=$(pwd)
hybrdyn_repo_path=`cat hybrdyn_repo_path`

if [ "$hybrdyn_repo_path" == "" ]; then
  echo "you need to place a file named hybrdyn_repo_path at this location; see template file"
fi;

cd $this_path
./KAS_prepare_maple_repo.sh

cd $this_path
./KAS_codegen_all_models.sh

cd $this_path
./KAS_copy_generated_code.sh


