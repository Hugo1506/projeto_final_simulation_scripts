from fastapi import FastAPI
import numpy as np
import cv2
import time
import subprocess
from PIL import Image
import io
import zlib
import yaml
import os
from gadentools.Simulation import Simulation
from gadentools.Utils import Vector3
from gadentools.Utils import block
from fastapi.responses import JSONResponse
import requests
import base64
import sys

app = FastAPI()



@app.get("/set_plume_location")
def set_plume_location(username: str, simulationNumber: str, plumeXlocation: float, plumeYlocation: float, plumeZlocation: float):
    simulation_dir = username + "_sim_" + simulationNumber
    simulation = username + "_" + simulationNumber
    scenario_path = os.path.join("/src/install/test_env/share/test_env/scenarios",simulation_dir)

    gaden_params_file = os.path.join(scenario_path, 'params', 'gaden_params.yaml')

    with open(gaden_params_file, 'r') as file:
        gaden_params = yaml.safe_load(file)

    # faz update com os novos valores de localização do pluma
    gaden_params['gaden_filament_simulator']['ros__parameters']['source_position_x'] = plumeXlocation
    gaden_params['gaden_filament_simulator']['ros__parameters']['source_position_y'] = plumeYlocation
    gaden_params['gaden_filament_simulator']['ros__parameters']['source_position_z'] = plumeZlocation

    with open(gaden_params_file, 'w') as file:
        yaml.dump(gaden_params, file, width=1000)

    # corre o script modificado de simulação do gaden para fazer a simulação mas sem GUI
    subprocess.run(['ros2', 'launch', 'test_env', 'gaden_sim_no_gui_launch.py', f'scenario:={simulation_dir}'])
    

    subprocess.run(['python3', "simulation_visualizer.py", f'{simulation_dir}'])

    # faz um POST request para atualizar o status da simulação para indicar que já está concluída
    updateStatusToDone = requests.post('http://webserver:3000/setStatusToDone', json={
        'simulation': simulation,
    })

    return JSONResponse(content={"message": "Plume location set successfully."})

@app.get("/robot_simulation")
def robot_simulation(username: str, simulationNumber: str, height: float, robotSpeed: float, robotXposition: float, robotYposition: float, finalXposition: float, finalYposition: float):
    vector3Up = Vector3(0, 0, 1)
    initialRobotPosition = Vector3(robotXposition,robotYposition, height)
    finalRobotPosition = Vector3(finalXposition, finalYposition, height)

    direction_vector = np.array([
        finalRobotPosition.x - initialRobotPosition.x,
        finalRobotPosition.y - initialRobotPosition.y
    ])
    direction_vector = direction_vector / np.linalg.norm(direction_vector)

    angle = np.arctan2(direction_vector[1], direction_vector[0])
    print(f"Angle from initial to final robot position: {angle}")

    simulation_dir = username + "_sim_" + simulationNumber
    scenario_path = os.path.join("/src/install/test_env/share/test_env/scenarios",simulation_dir)
    simulation_path = os.path.join(scenario_path,"gas_simulations/sim1")
    ocuppancy_path = os.path.join(scenario_path,"OccupancyGrid3D.csv")

    simulation_name = username + "_" + simulationNumber

    sim = Simulation(simulation_path, \
                    ocuppancy_path)

    imageSizeFactor = 5  
    max_ppm = 7.0

    simulation_data = []

    def vector3_to_dict(v):
        return {"x": v.x, "y": v.y, "z": v.z}

    def capture_simulation_data(robot_position, concentration, wind_speed, iteration):
        frame_data = {
            "robot_position": Vector3(robot_position.x, robot_position.y, robot_position.z),
            "concentration": concentration,
            "wind_speed": wind_speed,
            "iteration": iteration
        }

        simulation_data.append(frame_data)

    def markPreviousPositions(previousPositions, initialRobotPosition, image):
        for pos in previousPositions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        j = int((initialRobotPosition.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
        i = int((initialRobotPosition.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
        image = cv2.circle(image, (i, j), 4,(255,255,255), -1)



    def distance_from_target(robotPosition, finalPosition):
        return np.sqrt((robotPosition.x - finalPosition.x) ** 2 + (robotPosition.y - finalPosition.y) ** 2)

    def surge_cast():
        updateInterval = 0.5 # segundos
        sim.playSimulation(0, updateInterval)
        robotPosition = initialRobotPosition

        last_iteration = -1

        previousRobotPositions = []

        response = requests.get('http://webserver:3000/getRobotSimulationID', params={
            'simulation': simulation_name
        })

        if response.status_code == 200:
            global robotSim_id
            robotSim_id = response.json().get('id')
            if robotSim_id is None:
                robotSim_id = 0
            print(f"Robot simulation ID: {robotSim_id}")


  

        global frames
        frames = []

        while sim.getCurrentIteration() != 0:
            time.sleep(0.01)


        while (True):
            iteration = sim.getCurrentIteration()

            
            if iteration > last_iteration:
                last_iteration = last_iteration + 1
                print(f"Iteration: {iteration} robotPosition: {robotPosition}")
                previousRobotPositions.append(Vector3(robotPosition.x, robotPosition.y, robotPosition.z))

                concentration = sim.getCurrentConcentration(robotPosition)
                print(f"Location: {robotPosition}")
                print(f"Concentration at robot position: {concentration} ppm")

                capture_simulation_data(robotPosition, concentration, sim.getCurrentWind(robotPosition), iteration)


                map = sim.generateConcentrationMap2D(iteration, height, True)
                map_scaled = map * (255.0 / max_ppm)
                formatted_map = np.array(np.clip(map_scaled, 0, 255), dtype=np.uint8)

                base_image = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)
                block(map, base_image)

                newshape = (imageSizeFactor * base_image.shape[1], imageSizeFactor * base_image.shape[0])
                heatmap = cv2.resize(base_image, newshape)

                markPreviousPositions(previousRobotPositions, initialRobotPosition, heatmap)
                capture_frame_for_gif(heatmap)

                robotPosition.x += robotSpeed * np.cos(angle)
                robotPosition.y += robotSpeed * np.sin(angle)
                
                

                distanceFromTarger = distance_from_target(robotPosition, finalRobotPosition)

                if distanceFromTarger < robotSpeed:
                    
                    # Captura o último frame antes de parar a simulação
                    iteration = sim.getCurrentIteration()
                    previousRobotPositions.append(Vector3(robotPosition.x, robotPosition.y, robotPosition.z))
                    concentration = sim.getCurrentConcentration(robotPosition)
                    capture_simulation_data(robotPosition, concentration, sim.getCurrentWind(robotPosition), iteration)
                    map = sim.generateConcentrationMap2D(iteration, height, True)
                    map_scaled = map * (255.0 / max_ppm)
                    formatted_map = np.array(np.clip(map_scaled, 0, 255), dtype=np.uint8)
                    base_image = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)
                    block(map, base_image)
                    newshape = (imageSizeFactor * base_image.shape[1], imageSizeFactor * base_image.shape[0])
                    heatmap = cv2.resize(base_image, newshape)
                    markPreviousPositions(previousRobotPositions, initialRobotPosition, heatmap)
                    capture_frame_for_gif(heatmap)

                    print(f"Robot reached the target position: {robotPosition}")
                    break
                else:
                    print(f"The robot did not reach the end point. Current position: {robotPosition}")
        
        sim.stopPlaying()



    def capture_frame_for_gif(image):
        iteration = sim.getCurrentIteration()
        
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)

        buffered = io.BytesIO()
        pil_img.save(buffered, format="PNG") 
        compressed = zlib.compress(buffered.getvalue())
        img_str = base64.b64encode(compressed).decode('utf-8')

        print(f"Captured frame for GIF in iteration: {iteration}.")


        

        response = requests.post('http://webserver:3000/uploadSimulationResults', json={
                'simulation': simulation_name,
                'type': 'robot',
                'gif': img_str,
                'height': height,
                'iteration': iteration,
                'robotSim_id': robotSim_id + 1 
            })
        global id 
        if response.status_code == 200:
            print("GIF sent successfully.")
            id = response.json().get('id')



    surge_cast()

    simulation_data_serializable = []


    for frame in simulation_data:
        simulation_data_serializable.append({
            "robot_position": vector3_to_dict(frame["robot_position"]),
            "concentration": frame["concentration"],
            "wind_speed": vector3_to_dict(frame["wind_speed"]),
            "iteration": frame["iteration"]
        })



    return JSONResponse(content={"frames": simulation_data_serializable, "robotSim_id": robotSim_id + 1}) 