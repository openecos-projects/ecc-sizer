
# Resolve paths related to the current script.
script_path=$(realpath "$0")                    # Absolute path of this script
script_dir=$(dirname "$(realpath "$0")")        # Absolute directory of this script
script_name=$(basename "$0")                    # Script file name

echo "Script path: $script_path"
echo "Script directory: $script_dir"
echo "Script name: $script_name"

# Build paths relative to the script directory instead of relying on cwd.
sizer_exe="$script_dir/../build/src/Sizer"
design_name=$1
bin_exe=/app/sizer/sizer
dir_path=/app/2024_ICCAD_Contest_Gate_Sizing_Benchmark/design/${design_name}
cmd_template="$script_dir/cmd_base_file"
env_template="$script_dir/env_base_file"
cp ${cmd_template} ${design_name}.cmd_file
echo "-top ${design_name}" >> ${design_name}.cmd_file
echo "-def ${dir_path}/${design_name}.def" >> ${design_name}.cmd_file
echo "-sdc ${dir_path}/${design_name}.sdc" >> ${design_name}.cmd_file
echo "-outputPath /app/sizer/output" >> ${design_name}.cmd_file
${bin_exe} -env ${env_template} -f ${design_name}.cmd_file
