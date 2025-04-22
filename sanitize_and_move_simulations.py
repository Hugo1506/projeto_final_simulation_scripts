import sys
import shutil
import os
import re

# regex que deteta XXXX: "X.X" 
pattern = r"(\w+):\s*'(\d+\.\d+|\d+)'"

def rename_files_in_directory(directory,yaml_file_path):
    for filename in os.listdir(directory):
        if filename.endswith(".csv"):
            parts = filename.split('_')
            new_filename = f"{parts[0]}_{parts[1]}_{parts[-1]}"
            old_file_path = os.path.join(directory, filename)
            new_file_path = os.path.join(directory, new_filename)
            os.rename(old_file_path, new_file_path)

            update_yaml_file(yaml_file_path, parts)

def update_yaml_file(yaml_file_path, parts):
    if os.path.exists(yaml_file_path):
        with open(yaml_file_path, "r") as file:
            content = file.read()

        new_wind_sim_path = f"{parts[0]}_{parts[1]}"

        updated_content = content.replace("wind_sim_path: \"1ms/wind_at_cell_centers\"", f'wind_sim_path: "{new_wind_sim_path}"')

        with open(yaml_file_path, "w") as file:
            file.write(updated_content)

if os.path.exists(sys.argv[1]):
    path_parts = os.path.normpath(sys.argv[1]).split(os.sep)

    if len(path_parts) >= 2:
        yaml_file_path = os.path.join(sys.argv[1], "params", "preproc_params.yaml")

        if os.path.exists(yaml_file_path):
            with open(yaml_file_path, "r") as file:
                content = file.read()

            updated_content = content.replace("'", "")

            with open(yaml_file_path, "w") as file:
                file.write(updated_content)

        yaml_sim_file_path = os.path.join(sys.argv[1], "params", "gaden_params.yaml")

        if os.path.exists(yaml_sim_file_path):
            with open(yaml_sim_file_path, "r") as file:
                sim_content = file.read()

           

            updated_sim_content = sim_content.replace("pressure: '1.0'", "pressure: 1.0")
            updated_sim_content = updated_sim_content.replace("wind_time_step: '1.0'", "wind_time_step: 1.0")
            updated_sim_content = updated_sim_content.replace("results_min_time: '0.0'", "results_min_time: 0.0")
            updated_sim_content = updated_sim_content.replace("results_min_time: '0.0'", "results_min_time: 0.0")

             # substitui XXXX: "X.X" por XXXX: X.X
            updated_sim_content = re.sub(pattern, r'\1: \2', sim_content)
            
            with open(yaml_sim_file_path, "w") as file:
                file.write(updated_sim_content)
        else:
            print(f"YAML simulation file does not exist at {yaml_sim_file_path}")

        last_two_dirs = os.path.join(path_parts[-2], path_parts[-1]).replace("/","_")

        destination_path = os.path.join("/src/install/test_env/share/test_env/scenarios", last_two_dirs)    
        shutil.copytree(sys.argv[1],destination_path, dirs_exist_ok=True)

        simulation_path = os.path.join(destination_path, "simulations")
        shutil.copytree("/src/install/test_env/share/test_env/scenarios/10x6_central_obstacle/simulations/",simulation_path, dirs_exist_ok=True)

        wind_simulations_path = os.path.join(destination_path, "wind_simulations")
        if os.path.exists(wind_simulations_path):
            yaml_file_path = os.path.join(simulation_path, "sim1.yaml")
            rename_files_in_directory(wind_simulations_path,yaml_file_path)
        else:
            print(f"Wind simulations directory does not exist at {wind_simulations_path}")
else:
    print(f"Path does NOT exist: {sys.argv[1]}")  



