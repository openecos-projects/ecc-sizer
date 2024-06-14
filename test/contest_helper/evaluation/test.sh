#/bin/bash

OPENROAD_EXE=/home/zxy/my_code/gpu_gate_sizing/bin/openroad
FILE_PATH=/home/zxy/my_code/gpu_gate_sizing/benchmark/iccad24/design/NV_NVDLA_partition_m/NV_NVDLA_partition_m.size
DESIGN_NAME=NV_NVDLA_partition_m
LEF_PATH=/home/zxy/my_code/gpu_gate_sizing/benchmark/iccad24/platform/ASAP7/lef/
LIB_PATH=/home/zxy/my_code/gpu_gate_sizing/benchmark/iccad24/platform/ASAP7/lib/
DESIGN_PATH=/home/zxy/my_code/gpu_gate_sizing/benchmark/iccad24/design/

${OPENROAD_EXE} -python evaluation.py -exit --file_path ${FILE_PATH} \
                --design_name ${DESIGN_NAME} --lefPath ${LEF_PATH} \
                --libPath ${LIB_PATH} --designPath ${DESIGN_PATH}