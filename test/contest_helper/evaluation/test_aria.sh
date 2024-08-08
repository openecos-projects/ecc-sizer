#/bin/bash

cwd=/home/zhaoxueyan/code/gpu_gate_sizing
OPENROAD_EXE=${cwd}/bin/openroad
DESIGN_NAME=ariane136
LEF_PATH=${cwd}/benchmark/iccad24/platform/ASAP7/lef/
LIB_PATH=${cwd}/benchmark/iccad24/platform/ASAP7/lib/
DESIGN_PATH=${cwd}/benchmark/iccad24/design/
# FILE_PATH=${cwd}/test/test2/${DESIGN_NAME}/${DESIGN_NAME}.final.sizes
FILE_PATH=${cwd}/build/${DESIGN_NAME}.size

${OPENROAD_EXE} -python evaluation.py -exit --file_path ${FILE_PATH} \
                --design_name ${DESIGN_NAME} --lefPath ${LEF_PATH} \
                --libPath ${LIB_PATH} --designPath ${DESIGN_PATH} \
                # --equivcell_file_path ${LIB_PATH}/../libcell_id.csv