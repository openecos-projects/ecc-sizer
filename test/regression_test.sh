design_path="/home/zhaoxueyan/code/gpu_gate_sizing/benchmark/iccad24/design/"
directories=$(ls -d ${design_path}*/)
env_file="/home/zhaoxueyan/code/gpu_gate_sizing/.vscode/env_file"
bin_exe=/home/zhaoxueyan/code/gpu_gate_sizing/build/TritonSizer

if [ ! -f cmd_base_file ]; then
  echo "cmd_base_file 文件不存在"
  exit 1
fi

for dir_path in ${directories}; do
  echo ${dir_path}
  design_name=$(basename ${dir_path})
  mkdir ${design_name}
  cp cmd_base_file ${design_name}/cmd_file
  cd ${design_name}
  echo "-top ${design_name}" >> cmd_file
  echo "-def ${dir_path}/${design_name}.def" >> cmd_file
  echo "-sdc ${dir_path}/${design_name}.sdc" >> cmd_file
  #${bin_exe} -env ${env_file} -f cmd_file > $(date +'%m-%d %H:%M').log
  ${bin_exe} -env ${env_file} -f cmd_file > $(date +'%m%d_%H%M').log &
  cd ../
done

