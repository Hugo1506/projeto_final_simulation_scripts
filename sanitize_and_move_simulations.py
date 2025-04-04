import sys
import shutil
import os


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

        last_two_dirs = os.path.join(path_parts[-2], path_parts[-1]).replace("/","_")

        destination_path = os.path.join("/src/install/test_env/share/test_env/scenarios", last_two_dirs)    
        shutil.copytree(sys.argv[1],destination_path, dirs_exist_ok=True)
    