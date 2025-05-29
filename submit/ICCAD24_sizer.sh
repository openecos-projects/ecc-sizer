
# 获取当前脚本的相关路径信息
script_path=$(realpath "$0")                    # 脚本的绝对路径
script_dir=$(dirname "$(realpath "$0")")        # 脚本所在目录的绝对路径
script_name=$(basename "$0")                    # 脚本文件名

echo "Script path: $script_path"
echo "Script directory: $script_dir"
echo "Script name: $script_name"

# 使用脚本目录来构建相对路径，而不是依赖当前工作目录
sizer_exe="$script_dir/../build/src/Sizer"
design_name=$1
bin_exe=/app/sizer/sizer
dir_path=/app/2024_ICCAD_Contest_Gate_Sizing_Benchmark/design/${design_name}
cp cmd_base_file ${design_name}.cmd_file
echo "-top ${design_name}" >> ${design_name}.cmd_file
echo "-def ${dir_path}/${design_name}.def" >> ${design_name}.cmd_file
echo "-sdc ${dir_path}/${design_name}.sdc" >> ${design_name}.cmd_file
echo "-outputPath /app/sizer/output" >> ${design_name}.cmd_file
${bin_exe} -env env_file -f ${design_name}.cmd_file
