#/bin/bash

cwd=/home/zxy/my_code/gpu_gate_sizing
OPENROAD_EXE=${cwd}/bin/openroad
FILE_PATH=${cwd}/results/NV_NVDLA_partition_m.size
DESIGN_NAME=NV_NVDLA_partition_m
LEF_PATH=${cwd}/benchmark/iccad24/platform/ASAP7/lef/
LIB_PATH=${cwd}/benchmark/iccad24/platform/ASAP7/lib/
DESIGN_PATH=${cwd}/benchmark/iccad24/design/

${OPENROAD_EXE} -python evaluation.py -exit --file_path ${FILE_PATH} \
                --design_name ${DESIGN_NAME} --lefPath ${LEF_PATH} \
                --libPath ${LIB_PATH} --designPath ${DESIGN_PATH} \
                # --equivcell_file_path ${LIB_PATH}/../libcell_id.csv