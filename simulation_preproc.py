import requests
import os
import subprocess
import inotify.adapters

# vai monitorar todos os eventos que ocorren no directoria /simulation_data e às suas subdiretorias
i = inotify.adapters.InotifyTree('/simulation_data/')

gaden_launch_path = '/src/gaden/test_env/launch'
ros_work_dir = '/src'

if (not os.path.exists(os.path.join(gaden_launch_path,'gaden_sim_no_gui_launch.py')) ):
    with open('/projeto_final_simulation_scripts/gaden_sim_no_gui_launch.py', 'r') as file:
        gaden_sim_no_gui_launch_py_content = file.read()
    
    with open(os.path.join(gaden_launch_path, 'gaden_sim_no_gui_launch.py'), 'w') as file:
        file.write(gaden_sim_no_gui_launch_py_content)

    subprocess.run(['colcon', 'build', '--symlink-install'], cwd=ros_work_dir)


    


while True:
    # se o evento for NONE (não existir) continua a execução do loop
    for event in i.event_gen():
        if event is None:
            continue
        # extrai a mask do evento
        event_mask = event[1]  

        # se alguma diretoria ou ficheiro for criado então executa o código
        if 'IN_CREATE' in event_mask :
            # GET request para obter os dados da próxima simulação 
            response = requests.get('http://172.17.0.3:3000/getFirstInQueue')
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
                subprocess.run(['ros2', 'launch', 'test_env', 'gaden_preproc_launch.py', f'scenario:={user+"_"+simulation_dir}'])

                # faz um POST request para atualizar o status da simulação para indicar que já está em simulação
                updateStatusToInSimulation = requests.post('http://172.17.0.3:3000/setStatusToInSimulation', json={
                    'simulation': data.get('simulation'),
                })
                
                # corre o script modificado de simulação do gaden para fazer a simulação mas sem GUI
                subprocess.run(['ros2', 'launch', 'test_env', 'gaden_sim_no_gui_launch.py', f'scenario:={user+"_"+simulation_dir}'])

                # faz um POST request para atualizar o status da simulação para indicar que já está concluída
                updateStatusToDone = requests.post('http://172.17.0.3:3000/setStatusToDone', json={
                    'simulation': data.get('simulation'),
                })
        else:
            continue