import requests
import os
import subprocess
import inotify.adapters
import re
import shutil
import signal
import atexit

# vai monitorar todos os eventos que ocorren no directoria /simulation_data e às suas subdiretorias
i = inotify.adapters.InotifyTree('/simulation_data/')

gaden_launch_path = '/src/gaden/test_env/launch'
ros_work_dir = '/src'

log_file_path = '/projeto_final_simulation_scripts/simulation.log'

x_min, x_max = None, None
y_min, y_max = None, None
z_min, z_max = None, None

# inicia o servidor FastAPI 
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


# verifica se a localização da pluma está dentro do espaço de simulação
def extract_min_max(log_file_path,simulation_dir):
    params_path = os.path.join("/simulation_data/",simulation_dir,"params/gaden_params.yaml")

    with open(log_file_path, 'r', encoding='utf-8') as file:
        log_content = file.read()

    with open(params_path, 'r', encoding='utf-8') as file:
        params_content = file.read()

    # regex para extrair os valores de x, y e z do log
    x_pattern = re.compile(r'x\s*:\s*\(([-\d.]+),\s*([-\d.]+)\)')
    y_pattern = re.compile(r'y\s*:\s*\(([-\d.]+),\s*([-\d.]+)\)')
    z_pattern = re.compile(r'z\s*:\s*\(([-\d.]+),\s*([-\d.]+)\)')   

    # procura os valores de x, y e z no log
    x_match = x_pattern.search(log_content)
    y_match = y_pattern.search(log_content)
    z_match = z_pattern.search(log_content)

    # guarda os valores máximos e mínimos de x, y e z 
    if x_match and y_match and z_match:
        x_min, x_max = float(x_match.group(1)), float(x_match.group(2))
        y_min, y_max = float(y_match.group(1)), float(y_match.group(2))
        z_min, z_max = float(z_match.group(1)), float(z_match.group(2))

    else:
        print("No matches found for x, y, or z in the log file.")
    
    # regex para extrair os valores de x, y e z pedidos pelo utilizador
    x_param_pattern = re.compile(r'source_position_x\s*:\s*\'?([-.\d]+)\'?')
    y_param_pattern = re.compile(r'source_position_y\s*:\s*\'?([-.\d]+)\'?')
    z_param_pattern = re.compile(r'source_position_z\s*:\s*\'?([-.\d]+)\'?')

    # procura os valores de x, y e z no ficheiro params
    x_param_match = x_param_pattern.search(params_content)
    y_param_match = y_param_pattern.search(params_content)
    z_param_match = z_param_pattern.search(params_content)

    # guarda os valores de x, y e z
    if x_param_match and y_param_match and z_param_match:
        x_param = float(x_param_match.group(1))
        y_param = float(y_param_match.group(1))
        z_param = float(z_param_match.group(1))

        # se os valores de x, y e z pedidos pelo utilizador estão dentro do espaço de simulação
        # então retorna True, caso contrário retorna False
        if x_param > x_min and x_param < x_max and y_param > y_min and y_param < y_max and z_param > z_min and z_param < z_max:
            return True, x_min, x_max, y_min, y_max, z_min, z_max
        else:
            return False, x_min, x_max, y_min, y_max, z_min, z_max
    else:
        print("No matches found for x, y, or z in the params file.")
        return False

# corre um comando e guarda o output num ficheiro de log
def run_and_log(command, log_file):
    with open(log_file, 'w') as log_file:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        for line in process.stdout:
            print(line, end='')  
            log_file.write(line)  

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





while True:
    try:
        # se o evento for NONE (não existir) continua a execução do loop
        
        for event in i.event_gen():
            if event is None:
                continue
            # extrai a mask do evento
            event_mask = event[1]             

            # se alguma diretoria ou ficheiro for criado então executa o código
            if 'IN_CREATE' in event_mask or 'IN_MODIFY' in event_mask:
                # GET request para obter os dados da próxima simulação 
                response = requests.get('http://webserver:3000/getFirstInQueue')
                if response.status_code == 200:
                    data = response.json()

                    # extrai o user da resposta
                    user = (data.get('simulation')).split('_')[0]
                    # cria a diretoria com base na resposta 
                    simulation_dir = "sim_" +(data.get('simulation')).split('_')[1]

                    # cria a diretoria onde os dados da simulação vão ser guardados
                    simulation_path = os.path.join('/simulation_data',user,simulation_dir)
                    # corre o script que vai tratar e mover os dados da simulação para a diretoria onde ocorre a simulação
                    subprocess.run(['python3', "sanitize_and_move_simulations.py", simulation_path]) 

                    # corre o script do gaden para fazer o pre-processamento dos dados da simulação
                    preprocessing_command = ['ros2', 'launch', 'test_env', 'gaden_preproc_launch.py', f'scenario:={user+"_"+simulation_dir}']
                    run_and_log(preprocessing_command, log_file_path)
                    
                    is_inside_bounds, x_min, x_max, y_min, y_max, z_min, z_max = extract_min_max(log_file_path,(os.path.join(user,simulation_dir)))

                    if not is_inside_bounds:
                        print("está fora do espaço de simulação")
                        print(x_min, x_max, y_min, y_max, z_min, z_max)
                        # pede ao servidor para remover a simulação da queue
                        removeSimulation = requests.post('http://webserver:3000/plumeLocationOutOfBounds', json={
                            'simulation': data.get('simulation'),
                            'x_min': x_min,
                            'x_max': x_max,
                            'y_min': y_min,
                            'y_max': y_max,
                            'z_min': z_min,
                            'z_max': z_max,
                        }) 

                        # remove a simulação 
                        simulation_to_remove = os.path.join('/src/install/test_env/share/test_env/scenarios', f'{user}_{simulation_dir}')
                        if os.path.exists(simulation_to_remove) and os.path.isdir(simulation_to_remove):
                            try:
                                # Remove the directory and all its contents recursively
                                shutil.rmtree(simulation_to_remove)
                                print(f"Successfully removed the simulation: {simulation_to_remove}")
                            except Exception as e:
                                print(f"Error while removing simulation: {e}")
                        else:
                            print(f"The directory does not exist: {simulation_to_remove}")  

                    else:
                        # faz um POST request para atualizar o status da simulação para indicar que já está em simulação
                        updateStatusToInSimulation = requests.post('http://webserver:3000/setStatusToInSimulation', json={
                            'simulation': data.get('simulation'),
                        })
                        
                        # corre o script modificado de simulação do gaden para fazer a simulação mas sem GUI
                        subprocess.run(['ros2', 'launch', 'test_env', 'gaden_sim_no_gui_launch.py', f'scenario:={user+"_"+simulation_dir}'])
                        

                        subprocess.run(['python3', "simulation_visualizer.py", f'{user+"_"+simulation_dir}'])

                        # faz um POST request para atualizar o status da simulação para indicar que já está concluída
                        updateStatusToDone = requests.post('http://webserver:3000/setStatusToDone', json={
                            'simulation': data.get('simulation'),
                        })
            else:
                continue
    except inotify.calls.InotifyError as e:
        print(f"[WARNING] Ignored inotify error: {e}")
        continue
    except Exception as e:
        print(f"[ERROR] Unexpected error in inotify loop: {e}")
        continue
