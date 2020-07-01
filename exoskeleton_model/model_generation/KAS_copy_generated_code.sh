#!/bin/bash
# Kopiere generierten Code aus dem HybrDyn-Repo
# Dieses Skript im Ordner ausführen, in dem es liegt.

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-09
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

this_path=$(pwd)
hybrdyn_repo_path=`cat hybrdyn_repo_path`

mkdir -p ../KAS5m3/matlabfcn ../KAS5m3/testfcn
cp -u $hybrdyn_repo_path/codeexport/KAS5m3/matlabfcn/*.* ../KAS5m3/matlabfcn
cp -u $hybrdyn_repo_path/codeexport/KAS5m3/testfcn/*.* ../KAS5m3/testfcn

mkdir -p ../KAS5m5/matlabfcn ../KAS5m5/testfcn
cp -u $hybrdyn_repo_path/codeexport/KAS5m5/matlabfcn/*.* ../KAS5m5/matlabfcn
cp -u $hybrdyn_repo_path/codeexport/KAS5m5/testfcn/*.* ../KAS5m5/testfcn

mkdir -p ../KAS5m5OL/matlabfcn ../KAS5m5OL/testfcn
cp -u $hybrdyn_repo_path/codeexport/KAS5m5OL/matlabfcn/*.* ../KAS5m5OL/matlabfcn
cp -u $hybrdyn_repo_path/codeexport/KAS5m5OL/testfcn/*.* ../KAS5m5OL/testfcn

mdlext="TE DE1 DE2 IC OL"
for m in $mdlext; do
  mkdir -p ../KAS5m7${m}/matlabfcn ../KAS5m7${m}/testfcn
  cp -u $hybrdyn_repo_path/codeexport/KAS5m7${m}/matlabfcn/*.* ../KAS5m7${m}/matlabfcn
  cp -u $hybrdyn_repo_path/codeexport/KAS5m7${m}/testfcn/*.* ../KAS5m7${m}/testfcn
done


