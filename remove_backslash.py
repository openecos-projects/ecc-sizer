import os

def remove_backslashes(filepath, output_filepath=None):
    """Removes all backslashes from each line in a file."""
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()

        with open(output_filepath, 'w') as f:
            for line in lines:
                f.write(line.replace('\\', ''))
        print(f"Successfully removed backslashes from: {filepath}")

    except FileNotFoundError:
        print(f"Error: File not found: {filepath}")
    except Exception as e:
        print(f"An error occurred: {e}")

# filepath: /nfs/share/tapout/smic110/retrosoc_asic_flatten/workspace/output/iEDA/result/retrosoc_asic_place.def
file_path = "/nfs/share/tapout/smic110/retrosoc_asic_flatten/workspace/output/iEDA/result/retrosoc_asic_place.def"
output_filepath = "/nfs/share/tapout/smic110/retrosoc_asic_flatten/workspace/output/iEDA/result/retrosoc_asic_place_new.def"
remove_backslashes(file_path, output_filepath)