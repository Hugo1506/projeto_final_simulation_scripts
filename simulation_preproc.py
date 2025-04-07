import requests
import os
import subprocess

response = requests.get('http://172.17.0.3:3000/getFirstInQueue')

if response.status_code == 200:
    data = response.json()

    user = (data.get('simulation')).split('_')[0]
    simulation_dir = "sim_" +(data.get('simulation')).split('_')[1]


    simulation_path = os.path.join('/simulation_data',user,simulation_dir)
    subprocess.run(['python3', "sanitize_and_move_simulations.py", simulation_path]) 

    subprocess.run(['ros2', 'launch', 'test_env', 'gaden_preproc_launch.py', f'scenario:={user+"_"+simulation_dir}'])

    updateStatusToInSimulation = requests.post('http://172.17.0.3:3000/setStatusToInSimulation', json={
        'simulation': data.get('simulation'),
    })

    updateStatusToDone = requests.post('http://172.17.0.3:3000/setStatusToDone', json={
        'simulation': data.get('simulation'),
    })

else:
    print(f"Failed to get data. Status code: {response.status_code}")