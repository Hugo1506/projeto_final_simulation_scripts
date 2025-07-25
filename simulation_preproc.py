import inotify.calls
import requests
import os
import subprocess
import inotify.adapters
import re
import shutil
import signal
import atexit
import sys

gaden_launch_path = '/src/gaden/test_env/launch'
ros_work_dir = '/src'



x_min, x_max = None, None
y_min, y_max = None, None
z_min, z_max = None, None

# inicia o servidor FastAPI 
'''
fastapi_process = subprocess.Popen(
    ['fastapi', 'run', 'robots_simulation.py'],
    preexec_fn=os.setsid 
)



# funcção para parar o servidor FastAPI quando o script termina
def cleanup():
    try:
        os.killpg(os.getpgid(fastapi_process.pid), signal.SIGKILL)
        print("FastAPI server stopped.")
    except Exception as e:
        print(f"Error stopping FastAPI server: {e}")

# quando o script termina, a função cleanup é chamada para parar o servidor FastAPI
atexit.register(cleanup)
'''

# verifica se a localização da pluma está dentro do espaço de simulação
def extract_min_max(log_file_path):
    with open(log_file_path, 'r', encoding='utf-8') as file:
        log_content = file.read()

    x_pattern = re.compile(r'x\s*:\s*\(([-\d.]+),\s*([-\d.]+)\)')
    y_pattern = re.compile(r'y\s*:\s*\(([-\d.]+),\s*([-\d.]+)\)')
    z_pattern = re.compile(r'z\s*:\s*\(([-\d.]+),\s*([-\d.]+)\)')

    x_matches = x_pattern.findall(log_content)
    y_matches = y_pattern.findall(log_content)
    z_matches = z_pattern.findall(log_content)

    def get_match(matches):
        if len(matches) >= 2:
            return float(matches[1][0]), float(matches[1][1])
        elif len(matches) == 1:
            return float(matches[0][0]), float(matches[0][1])
        else:
            raise ValueError("Coordinate not found in log file.")

    try:
        x_min, x_max = get_match(x_matches)
        y_min, y_max = get_match(y_matches)
        z_min, z_max = get_match(z_matches)
        return x_min, x_max, y_min, y_max, z_min, z_max
    except ValueError as e:
        print(f"Error: {e}")
        return None

# corre um comando e guarda o output num ficheiro de log
def run_and_log(command, log_file):
    with open(log_file, 'w') as log_file:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
        if process.stdout != None:
            for line in process.stdout:
                print(line, end='')  
                log_file.write(line)  

        if process.stderr != None:
            for line in process.stderr:
                print(line, end='', file=sys.stderr)  
                log_file.write(line) 

        process.wait()
        log_file.write(f"Return code: {process.returncode}\n")



# verifica se o ficheiro gaden_sim_no_gui_launch.py existe, se não existir copia o ficheiro para a pasta de launch do gaden e compila os pacotes
if (not os.path.exists(os.path.join(gaden_launch_path,'gaden_sim_no_gui_launch.py')) ):
    with open('/projeto_final_simulation_scripts/gaden_sim_no_gui_launch.py', 'r') as file:
        gaden_sim_no_gui_launch_py_content = file.read()
    
    with open(os.path.join(gaden_launch_path, 'gaden_sim_no_gui_launch.py'), 'w') as file:
        file.write(gaden_sim_no_gui_launch_py_content)

    subprocess.run(['colcon', 'build', '--symlink-install'], cwd=ros_work_dir)   

        
                    
