design_name=$1
bin_exe=/app/sizer/sizer
dir_path=/app/2024_ICCAD_Contest_Gate_Sizing_Benchmark/design/${design_name}
cp cmd_base_file ${design_name}.cmd_file
echo "-top ${design_name}" >> ${design_name}.cmd_file
echo "-def ${dir_path}/${design_name}.def" >> ${design_name}.cmd_file
echo "-sdc ${dir_path}/${design_name}.sdc" >> ${design_name}.cmd_file
echo "-outputPath /app/sizer/output" >> ${design_name}.cmd_file
${bin_exe} -env env_file -f ${design_name}.cmd_file
