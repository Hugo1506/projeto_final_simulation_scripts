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
else:
    print(f"Failed to get data. Status code: {response.status_code}")