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
import json
import requests
import base64
import sys
from PBest import PBest
from GBest import GBest
import rclpy
from GBestSubscriber import retrieve_gbest_position

app = FastAPI()



@app.get("/example_simulation")
def example_simulation(username: str, simulationNumber: str, plumeXlocation: float, plumeYlocation: float, plumeZlocation: float, zMin: float, zMax: float):
    try:
        simulation_dir = username + "_sim_" + simulationNumber
        simulation = username + "_" + simulationNumber
        scenario_path = os.path.join("/src/install/test_env/share/test_env/scenarios", simulation_dir)
        gaden_params_file = os.path.join(scenario_path, 'params', 'gaden_params.yaml')
        
        # Update YAML with new plume location
        with open(gaden_params_file, 'r') as file:
            gaden_params = yaml.safe_load(file)

        gaden_params['gaden_filament_simulator']['ros__parameters']['source_position_x'] = plumeXlocation
        gaden_params['gaden_filament_simulator']['ros__parameters']['source_position_y'] = plumeYlocation
        gaden_params['gaden_filament_simulator']['ros__parameters']['source_position_z'] = plumeZlocation

        with open(gaden_params_file, 'w') as file:
            yaml.dump(gaden_params, file, width=1000)

        # Run the simulation first and wait for it to complete
        subprocess.run(
            ['ros2', 'launch', 'test_env', 'gaden_sim_no_gui_launch.py', f'scenario:={simulation_dir}'],
            stdout=sys.stdout,
            stderr=sys.stderr,
            text=True,
            timeout=120 
        )

        # Run visualizer to ensure files are processed
        subprocess.run(['python3', "simulation_visualizer.py", f'{simulation_dir}'])

        # Now generate wind map
        simulation_path = os.path.join(scenario_path, "gas_simulations/sim1")
        ocuppancy_path = os.path.join(scenario_path, "OccupancyGrid3D.csv")
        arrowLength = 10
        spaceBetweenArrows = 5
        imageSizeFactor = 5

        sim = Simulation(simulation_path, ocuppancy_path)
        map = sim.generateWindMap2D(sim.getCurrentIteration(), 0.0, True)

        # Create base image
        base_image = np.full(map.shape, 255, np.uint8)
        block(map, base_image)

        # Resize and convert image
        newshape = (imageSizeFactor * map.shape[1], imageSizeFactor * map.shape[0])
        base_image = cv2.resize(base_image, newshape)
        base_image = cv2.cvtColor(base_image, cv2.COLOR_GRAY2BGR)

        # Draw arrows
        for i in range(0, map.shape[0], spaceBetweenArrows):
            for j in range(0, map.shape[1], spaceBetweenArrows):
                if isinstance(map[i, j], Vector3):
                    offsetX = int(map[i, j].x * arrowLength)
                    offsetY = int(map[i, j].y * arrowLength)
                    start_point = (imageSizeFactor * j, imageSizeFactor * i)
                    end_point = (start_point[0] + offsetY, start_point[1] + offsetX)
                    cv2.arrowedLine(base_image, start_point, end_point, (0, 0, 255), 2)

        # Convert and compress image
        rgb_image = cv2.cvtColor(base_image, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)

        buffered = io.BytesIO()
        pil_img.save(buffered, format="PNG") 
        compressed = zlib.compress(buffered.getvalue())
        img_str = base64.b64encode(compressed).decode('utf-8')

        # Upload results
        response = requests.post('http://webserver:3000/uploadSimulationResults', json={
            'simulation': simulation,
            'type': 'plume_wind',
            'gif': img_str,
            'height': 0.0,
            'iteration': sim.getCurrentIteration(),
            'robotSim_id': -1  # Add this to ensure database insert works
        })

        if response.status_code != 200:
            print(f"Failed to upload: {response.status_code}, {response.text}")
        else: 
            print(f"right: {response.status_code}, {response.text}")

        return JSONResponse(content={"message": "Plume location set successfully."})
    except Exception as e:
        print("Error:", e)
        import traceback
        traceback.print_exc()
        return {"error": str(e)}

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
    subprocess.run(
            ['ros2', 'launch', 'test_env', 'gaden_sim_no_gui_launch.py', f'scenario:={simulation_dir}'],
            stdout=sys.stdout,
            stderr=sys.stderr,
            text=True,
            timeout=120 
    )

    subprocess.run(['python3', "simulation_visualizer.py", f'{simulation_dir}'])

    # faz um POST request para atualizar o status da simulação para indicar que já está concluída
    updateStatusToDone = requests.post('http://webserver:3000/setStatusToDone', json={
        'simulation': simulation,
    })

    return JSONResponse(content={"message": "Plume location set successfully."})

@app.get("/robot_simulation")
def robot_simulation(username: str, simulationNumber: str, height: float, robots):

    if isinstance(robots, str):
        robots = json.loads(robots)

    print(robots)

    robot1Speed = float(robots[0]["robotSpeed"])
    robot1Xposition = float(robots[0]["robotXlocation"])
    robot1Yposition = float(robots[0]["robotYlocation"])
    final1Xposition = float(robots[0]["finalRobotXlocation"])
    final1Yposition = float(robots[0]["finalRobotYlocation"])
    

    vector3Up = Vector3(0, 0, 1)
    initialRobot1Position = Vector3(robot1Xposition,robot1Yposition, height)
    finalRobot1Position = Vector3(final1Xposition, final1Yposition, height)

    direction_vector1 = np.array([
        finalRobot1Position.x - initialRobot1Position.x,
        finalRobot1Position.y - initialRobot1Position.y
    ])
    direction_vector1 = direction_vector1 / np.linalg.norm(direction_vector1)

    angle1 = np.arctan2(direction_vector1[1], direction_vector1[0])
    print(f"angle from initial to final robot position: {angle1}")

    print(f"robots {robots}")

    if len(robots) > 1:
        robot2Speed = float(robots[1]["robotSpeed"])
        robot2Xposition = float(robots[1]["robotXlocation"])
        robot2Yposition = float(robots[1]["robotYlocation"])
        final2Xposition = float(robots[1]["finalRobotXlocation"])
        final2Yposition = float(robots[1]["finalRobotYlocation"])

        initialRobot2Position = Vector3(robot2Xposition,robot2Yposition, height)
        finalRobot2Position = Vector3(final2Xposition, final2Yposition, height)

        direction_vector2 = np.array([
            finalRobot2Position.x - initialRobot2Position.x,
            finalRobot2Position.y - initialRobot2Position.y
        ])
        direction_vector2 = direction_vector2 / np.linalg.norm(direction_vector2)

        angle2 = np.arctan2(direction_vector2[1], direction_vector2[0])
        print(f"angle from initial to final robot position: {angle2}")


    else:
        initialRobot2Position = finalRobot2Position = None
        robot2Speed = robot2Xposition = robot2Yposition = final2Xposition = final2Yposition = angle2 = None
    
    print(f"robot2: speed: {robot2Speed}, X position: {robot2Xposition}, Y position: {robot2Yposition}, final X position: {final2Xposition}, final Y position: {final2Yposition}")

    if len(robots) > 2:
        robot3Speed = float(robots[2]["robotSpeed"])
        robot3Xposition = float(robots[2]["robotXlocation"])
        robot3Yposition = float(robots[2]["robotYlocation"])
        final3Xposition = float(robots[2]["finalRobotXlocation"])
        final3Yposition = float(robots[2]["finalRobotYlocation"])

        initialRobot3Position = Vector3(robot3Xposition,robot3Yposition, height)
        finalRobot3Position = Vector3(final3Xposition, final3Yposition, height)

        direction_vector3 = np.array([
            finalRobot3Position.x - initialRobot3Position.x,
            finalRobot3Position.y - initialRobot3Position.y
        ])
        direction_vector3 = direction_vector3 / np.linalg.norm(direction_vector3)

        angle3 = np.arctan2(direction_vector3[1], direction_vector3[0])
        print(f"angle from initial to final robot position: {angle3}")

    else:
        initialRobot3Position = finalRobot3Position = None
        robot3Speed = robot3Xposition = robot3Yposition = final3Xposition = final3Yposition = angle3 = None
    print(f"robot3: speed: {robot3Speed}, X position: {robot3Xposition}, Y position: {robot3Yposition}, final X position: {final3Xposition}, final Y position: {final3Yposition}")


    if len(robots) > 3:
        robot4Speed = float(robots[3]["robotSpeed"])
        robot4Xposition = float(robots[3]["robotXlocation"])
        robot4Yposition = float(robots[3]["robotYlocation"])
        final4Xposition = float(robots[3]["finalRobotXlocation"])
        final4Yposition = float(robots[3]["finalRobotYlocation"])

        initialRobot4Position = Vector3(robot4Xposition,robot4Yposition, height)
        finalRobot4Position = Vector3(final4Xposition, final4Yposition, height)

        direction_vector4 = np.array([
            finalRobot4Position.x - initialRobot4Position.x,
            finalRobot4Position.y - initialRobot4Position.y
        ])
        direction_vector4 = direction_vector4 / np.linalg.norm(direction_vector4)

        angle4 = np.arctan2(direction_vector4[1], direction_vector4[0])
        print(f"angle from initial to final robot position: {angle4}")

    else:
        initialRobot4Position = finalRobot4Position = None
        robot4Speed = robot4Xposition = robot4Yposition = final4Xposition = final4Yposition = angle4 = None
    print(f"robot4: speed: {robot4Speed}, X position: {robot4Xposition}, Y position: {robot4Yposition}, final X position: {final4Xposition}, final Y position: {final4Yposition}")

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

    def capture_simulation_data(robot_position, concentration, wind_speed, iteration, robot):
        frame_data = {
            "robot_position": Vector3(robot_position.x, robot_position.y, robot_position.z),
            "concentration": concentration,
            "wind_speed": wind_speed,
            "iteration": iteration,
            "robot": robot
        }

        simulation_data.append(frame_data)

    def markPreviousPositions(previous1Positions,previous2Positions,previous3Positions,previous4Positions,
        initialRobot1Position, initialRobot2Position, initialRobot3Position, initialRobot4Position,
        image):

        for pos in previous1Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        for pos in previous2Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        for pos in previous3Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        for pos in previous4Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)
            

        j = int((initialRobot1Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
        i = int((initialRobot1Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
        image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
        cv2.putText(image, "R1", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


        if initialRobot2Position is not None:
            j = int((initialRobot2Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((initialRobot2Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
            cv2.putText(image, "R2", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


            if initialRobot3Position is not None:
                j = int((initialRobot3Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
                i = int((initialRobot3Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
                image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
                cv2.putText(image, "R3", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


                if initialRobot4Position is not None:
                    j = int((initialRobot4Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
                    i = int((initialRobot4Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
                    image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
                    cv2.putText(image, "R4", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)




    def distance_from_target(robot1Position, finalPosition):
        return np.sqrt((robot1Position.x - finalPosition.x) ** 2 + (robot1Position.y - finalPosition.y) ** 2)

    def surge_cast():
        updateInterval = 0.5 # segundos
        sim.playSimulation(0, updateInterval)

        robot1Position = initialRobot1Position
        robot2Position = initialRobot2Position if robot2Speed is not None else None
        robot3Position = initialRobot3Position if robot3Speed is not None else None
        robot4Position = initialRobot4Position if robot4Speed is not None else None

        last_iteration = -1

        previousRobot1Positions = []
        previousRobot2Positions = []
        previousRobot3Positions = []
        previousRobot4Positions = []

        robot1StopFlag = False

        # if the postion doesnt exist the robot has the stop flag set to True if not sets it to false
        robot2StopFlag = robot2Position is None
        robot3StopFlag = robot3Position is None
        robot4StopFlag = robot4Position is None


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
                print(f"Iteration: {iteration} robot1Position: {robot1Position}")

                previousRobot1Positions.append(Vector3(robot1Position.x, robot1Position.y, robot1Position.z))
                if robot2Position is not None:
                    previousRobot2Positions.append(Vector3(robot2Position.x, robot2Position.y, robot2Position.z))
                    if robot3Position is not None:
                        previousRobot3Positions.append(Vector3(robot3Position.x, robot3Position.y, robot3Position.z))                
                        if robot4Position is not None:
                            previousRobot4Positions.append(Vector3(robot4Position.x, robot4Position.y, robot4Position.z))
                


                concentration1 = sim.getCurrentConcentration(robot1Position)
                print(f"Location: {robot1Position}")
                print(f"Concentration at robot1 position: {concentration1} ppm")

                concentration2 = concentration3 = concentration4 = 0

                capture_simulation_data(robot1Position, concentration1, sim.getCurrentWind(robot1Position), iteration, 1)
                if robot2Position is not None:
                    concentration2 = sim.getCurrentConcentration(robot2Position) 
                    capture_simulation_data(robot2Position, concentration2 , sim.getCurrentWind(robot2Position), iteration, 2)
                    if robot3Position is not None:
                        concentration3 = sim.getCurrentConcentration(robot3Position) 
                        capture_simulation_data(robot3Position, concentration3, sim.getCurrentWind(robot3Position), iteration, 3)
                        if robot4Position is not None:
                            concentration4 = sim.getCurrentConcentration(robot4Position) 
                            capture_simulation_data(robot4Position, concentration4, sim.getCurrentWind(robot4Position), iteration, 4)
            


                map = sim.generateConcentrationMap2D(iteration, height, True)
                map_scaled = map * (255.0 / max_ppm)
                formatted_map = np.array(np.clip(map_scaled, 0, 255), dtype=np.uint8)

                base_image = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)
                block(map, base_image)

                newshape = (imageSizeFactor * base_image.shape[1], imageSizeFactor * base_image.shape[0])
                heatmap = cv2.resize(base_image, newshape)

                markPreviousPositions(previousRobot1Positions, previousRobot2Positions,previousRobot3Positions,previousRobot4Positions,
                                        initialRobot1Position, initialRobot2Position, initialRobot3Position, initialRobot4Position,
                                        heatmap)

                capture_frame_for_gif(heatmap)

                if robot1Position is not None and not robot1StopFlag:
                    robot1Position.x += robot1Speed * np.cos(angle1)
                    robot1Position.y += robot1Speed * np.sin(angle1)
                    distanceFromTarger1 = distance_from_target(robot1Position, finalRobot1Position)
                    if distanceFromTarger1 < robot1Speed:
                        robot1StopFlag = True
                        print(f"Robot 1 reached the target position: {robot1Position}")

                if robot2Position is not None and not robot2StopFlag and angle2 is not None:
                    robot2Position.x += robot2Speed * np.cos(angle2)
                    robot2Position.y += robot2Speed * np.sin(angle2)
                    distanceFromTarger2 = distance_from_target(robot2Position, finalRobot2Position)
                    if distanceFromTarger2 < robot2Speed:
                        robot2StopFlag = True
                        print(f"Robot 2 reached the target position: {robot2Position}")

                if robot3Position is not None and not robot3StopFlag and angle3 is not None:
                    robot3Position.x += robot3Speed * np.cos(angle3)
                    robot3Position.y += robot3Speed * np.sin(angle3)
                    distanceFromTarger3 = distance_from_target(robot3Position, finalRobot3Position)
                    if distanceFromTarger3 < robot3Speed:
                        robot3StopFlag = True
                        print(f"Robot 3 reached the target position: {robot3Position}")

                if robot4Position is not None and not robot4StopFlag and angle4 is not None:
                    robot4Position.x += robot4Speed * np.cos(angle4)
                    robot4Position.y += robot4Speed * np.sin(angle4)
                    distanceFromTarger4 = distance_from_target(robot4Position, finalRobot4Position)
                    if distanceFromTarger4 < robot4Speed:
                        robot4StopFlag = True
                        print(f"Robot 4 reached the target position: {robot4Position}")
                

                print(f"flags: robot1StopFlag: {robot1StopFlag}, robot2StopFlag: {robot2StopFlag}, robot3StopFlag: {robot3StopFlag}, robot4StopFlag: {robot4StopFlag}")
                if robot1StopFlag and robot2StopFlag and robot3StopFlag and robot4StopFlag:
                    
                    # capture the final positions and concentrations
                    iteration = sim.getCurrentIteration()
                    previousRobot1Positions.append(Vector3(robot1Position.x, robot1Position.y, robot1Position.z))
                    concentration1 = sim.getCurrentConcentration(robot1Position)
                    capture_simulation_data(robot1Position, concentration1, sim.getCurrentWind(robot1Position), iteration, 1)

                    if robot2Position is not None:
                        previousRobot2Positions.append(Vector3(robot2Position.x, robot2Position.y, robot2Position.z))
                        concentration2 = sim.getCurrentConcentration(robot2Position)
                        capture_simulation_data(robot2Position, concentration2, sim.getCurrentWind(robot2Position), iteration, 2)

                        if robot3Position is not None:
                            previousRobot3Positions.append(Vector3(robot3Position.x, robot3Position.y, robot3Position.z))                
                            concentration3 = sim.getCurrentConcentration(robot3Position)
                            capture_simulation_data(robot3Position, concentration3, sim.getCurrentWind(robot3Position), iteration, 3)

                            if robot4Position is not None:
                                previousRobot4Positions.append(Vector3(robot4Position.x, robot4Position.y, robot4Position.z))
                                concentration4 = sim.getCurrentConcentration(robot4Position)
                                capture_simulation_data(robot4Position, concentration4, sim.getCurrentWind(robot4Position), iteration, 4)

                    map = sim.generateConcentrationMap2D(iteration, height, True)
                    map_scaled = map * (255.0 / max_ppm)
                    formatted_map = np.array(np.clip(map_scaled, 0, 255), dtype=np.uint8)
                    base_image = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)
                    block(map, base_image)
                    newshape = (imageSizeFactor * base_image.shape[1], imageSizeFactor * base_image.shape[0])
                    heatmap = cv2.resize(base_image, newshape)

                    markPreviousPositions(previousRobot1Positions, previousRobot2Positions, previousRobot3Positions, previousRobot4Positions,
                    initialRobot1Position, initialRobot2Position, initialRobot3Position, initialRobot4Position,
                    heatmap)
                    capture_frame_for_gif(heatmap)

                    print(f"Robot reached the target position: {robot1Position}")
                    break
                else:
                    print(f"The robot did not reach the end point. Current position: {robot1Position}")
        
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
            "iteration": frame["iteration"],
            "robot": frame["robot"]
        })


    return JSONResponse(content={"frames": simulation_data_serializable, "robotSim_id": robotSim_id + 1}) 

@app.get("/silkworm_moth_simulation")
def silkworm_moth_simulation(username: str, simulationNumber: str, height: float, robots):
    if isinstance(robots, str):
        robots = json.loads(robots)

    print(robots)

    robot1Iterations = int(robots[0]["iterations"])
    robot1Speed = float(robots[0]["robotSpeed"])
    robot1Xposition = float(robots[0]["robotXlocation"])
    robot1Yposition = float(robots[0]["robotYlocation"])
    angle1 = float(robots[0]["angle"])
    
    initialRobot1Position = Vector3(robot1Xposition,robot1Yposition, height)

    if len(robots) > 1:
        robot2Iterations = int(robots[1]["iterations"])
        robot2Speed = float(robots[1]["robotSpeed"])
        robot2Xposition = float(robots[1]["robotXlocation"])
        robot2Yposition = float(robots[1]["robotYlocation"])
        angle2 = float(robots[1]["angle"])

        initialRobot2Position = Vector3(robot2Xposition,robot2Yposition, height)

    else:
        robot2Iterations = 0 
        initialRobot2Position = None
        robot2Speed = robot2Xposition = robot2Yposition = angle2 = None
    
    print(f"robot2: speed: {robot2Speed}, X position: {robot2Xposition}, Y position: {robot2Yposition}, angle: {angle2}, iterations: {robot2Iterations}")

    if len(robots) > 2:
        robot3Iterations = int(robots[2]["iterations"])
        robot3Speed = float(robots[2]["robotSpeed"])
        robot3Xposition = float(robots[2]["robotXlocation"])
        robot3Yposition = float(robots[2]["robotYlocation"])
        angle3 = float(robots[2]["angle"])

        initialRobot3Position = Vector3(robot3Xposition,robot3Yposition, height)

    else:
        robot3Iterations = 0 
        initialRobot3Position  = None
        robot3Speed = robot3Xposition = robot3Yposition = angle3 = None
    print(f"robot3: speed: {robot3Speed}, X position: {robot3Xposition}, Y position: {robot3Yposition}, angle: {angle3}")


    if len(robots) > 3:
        robot4Iterations = int(robots[3]["iterations"])
        robot4Speed = float(robots[3]["robotSpeed"])
        robot4Xposition = float(robots[3]["robotXlocation"])
        robot4Yposition = float(robots[3]["robotYlocation"])
        angle4 = float(robots[3]["angle"])

        initialRobot4Position = Vector3(robot4Xposition,robot4Yposition, height)

    else:
        robot4Iterations = 0 
        initialRobot4Position = None
        robot4Speed = robot4Xposition = robot4Yposition = angle4 = None
    print(f"robot4: speed: {robot4Speed}, X position: {robot4Xposition}, Y position: {robot4Yposition}, angle: {angle4}")
    
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

    def vector3_to_dict(v):
        return {"x": v.x, "y": v.y, "z": v.z}

    def capture_simulation_data(robot_position, concentration, wind_speed, iteration, robot):
        frame_data = {
            "robot_position": Vector3(robot_position.x, robot_position.y, robot_position.z),
            "concentration": concentration,
            "wind_speed": wind_speed,
            "iteration": iteration,
            "robot": robot
        }

        simulation_data.append(frame_data)


    def markPreviousPositions(previous1Positions,previous2Positions,previous3Positions,previous4Positions,
        initialRobot1Position, initialRobot2Position, initialRobot3Position, initialRobot4Position,
        image):

        for pos in previous1Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        for pos in previous2Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        for pos in previous3Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        for pos in previous4Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)
            

        j = int((initialRobot1Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
        i = int((initialRobot1Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
        image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
        cv2.putText(image, "R1", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


        if initialRobot2Position is not None:
            j = int((initialRobot2Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((initialRobot2Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
            cv2.putText(image, "R2", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


            if initialRobot3Position is not None:
                j = int((initialRobot3Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
                i = int((initialRobot3Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
                image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
                cv2.putText(image, "R3", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


                if initialRobot4Position is not None:
                    j = int((initialRobot4Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
                    i = int((initialRobot4Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
                    image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
                    cv2.putText(image, "R4", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    def normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def main():
        updateInterval = 0.5 # segundos
        sim.playSimulation(0, updateInterval)

        robot1Position = initialRobot1Position
        robot2Position = initialRobot2Position if robot2Speed is not None else None
        robot3Position = initialRobot3Position if robot3Speed is not None else None
        robot4Position = initialRobot4Position if robot4Speed is not None else None

        previousRobot1Positions = []
        previousRobot2Positions = []
        previousRobot3Positions = []
        previousRobot4Positions = []

        robot1StopFlag = False

        # if the postion doesnt exist the robot has the stop flag set to True if not sets it to false
        robot2StopFlag = robot2Position is None
        robot3StopFlag = robot3Position is None
        robot4StopFlag = robot4Position is None


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
        
        last_iteration = -1
        zigzag1_sign = 1 
        zigzag1_period = 3

        zigzag2_sign = 1 
        zigzag2_period = 3

        zigzag3_sign = 1 
        zigzag3_period = 3

        zigzag4_sign = 1 
        zigzag4_period = 3



        while (True):
            iteration = sim.getCurrentIteration()
            if iteration > last_iteration:
                last_iteration = iteration
                print(f"Iteration: {iteration} robot1Position: {robot1Position}")

                previousRobot1Positions.append(Vector3(robot1Position.x, robot1Position.y, robot1Position.z))
                if robot2Position is not None:
                    previousRobot2Positions.append(Vector3(robot2Position.x, robot2Position.y, robot2Position.z))
                    if robot3Position is not None:
                        previousRobot3Positions.append(Vector3(robot3Position.x, robot3Position.y, robot3Position.z))                
                        if robot4Position is not None:
                            previousRobot4Positions.append(Vector3(robot4Position.x, robot4Position.y, robot4Position.z))
                


                concentration1 = sim.getCurrentConcentration(robot1Position)
                print(f"Location: {robot1Position}")
                print(f"Concentration at robot1 position: {concentration1} ppm")

                concentration2 = concentration3 = concentration4 = 0 

                capture_simulation_data(robot1Position, concentration1, sim.getCurrentWind(robot1Position), iteration, 1)
                if robot2Position is not None:
                    concentration2 = sim.getCurrentConcentration(robot2Position)
                    capture_simulation_data(robot2Position, concentration2, sim.getCurrentWind(robot2Position), iteration, 2)
                    if robot3Position is not None:
                        concentration3 = sim.getCurrentConcentration(robot3Position)
                        capture_simulation_data(robot3Position,concentration3, sim.getCurrentWind(robot3Position), iteration, 3)
                        if robot4Position is not None:
                            concentration4 = sim.getCurrentConcentration(robot4Position)
                            capture_simulation_data(robot4Position, concentration4, sim.getCurrentWind(robot4Position), iteration, 4)
            


                map = sim.generateConcentrationMap2D(iteration, height, True)
                map_scaled = map * (255.0 / max_ppm)
                formatted_map = np.array(np.clip(map_scaled, 0, 255), dtype=np.uint8)

                base_image = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)
                block(map, base_image)

                newshape = (imageSizeFactor * base_image.shape[1], imageSizeFactor * base_image.shape[0])
                heatmap = cv2.resize(base_image, newshape)

                markPreviousPositions(previousRobot1Positions, previousRobot2Positions,previousRobot3Positions,previousRobot4Positions,
                                        initialRobot1Position, initialRobot2Position, initialRobot3Position, initialRobot4Position,
                                        heatmap)

                capture_frame_for_gif(heatmap)
            
      
                if robot1Position is not None and not robot1StopFlag:
                    windVector = sim.getCurrentWind(robot1Position)
                    wind_array = np.array([windVector.x, windVector.y, windVector.z])

                    if concentration1 > 0:

                        if np.allclose(wind_array, [0, 0, 0]):
                            robot1Position.x += robot1Speed * updateInterval
                            robot1Position.y += 0
                        else:
                            inverse_wind = wind_array
                            norm = np.linalg.norm(inverse_wind[:2])
                            if norm == 0:
                                robot1Position.x += robot1Speed * updateInterval
                                robot1Position.y += 0
                            else:
                                direction = inverse_wind[:2] / norm
                                robot1Position.x += robot1Speed * updateInterval * direction[0]
                                robot1Position.y += robot1Speed * updateInterval * direction[1]
                    else:
                        zigzag1_angle = normalize_angle(angle1 + zigzag1_sign * (np.pi / 4))

                        if (zigzag1_sign == -1):
                            zigzag1_angle = normalize_angle(zigzag1_angle*2)
                        next1_x = robot1Position.x - robot1Speed * updateInterval * np.cos(zigzag1_angle)
                        next1_y = robot1Position.y + robot1Speed * updateInterval * np.sin(zigzag1_angle)
                        next1_position = Vector3(next1_x, next1_y, robot1Position.z)

                        print(f"Trying to move to ({next1_x}, {next1_y}), Free?: {sim.checkPositionForObstacles(next1_position)}")

                        if not sim.checkPositionForObstacles(next1_position):
                            zigzag1_sign = -zigzag1_sign
                            zigzag1_angle = normalize_angle(angle1 + zigzag1_sign * (np.pi / 4))
                            next1_x = robot1Position.x - robot1Speed * updateInterval * np.cos(zigzag1_angle)
                            next1_y = robot1Position.y + robot1Speed * updateInterval * np.sin(zigzag1_angle)
                            next1_position = Vector3(next1_x, next1_y, robot1Position.z)

                            if not sim.checkPositionForObstacles(next1_position):
                                next1_x = robot1Position.x + robot1Speed * updateInterval * np.cos(zigzag1_angle)
                                next1_y = robot1Position.y + robot1Speed * updateInterval * np.sin(zigzag1_angle)
                                next1_position = Vector3(next1_x, next1_y, robot1Position.z)
                                if not sim.checkPositionForObstacles(next1_position):
                                    next1_x = robot1Position.x + robot1Speed * updateInterval * np.cos(zigzag1_angle)
                                    next1_y = robot1Position.y - robot1Speed * updateInterval * np.sin(zigzag1_angle)
                                    next1_position = Vector3(next1_x, next1_y, robot1Position.z)
                        
                        robot1Position.x = next1_x
                        robot1Position.y = next1_y

                        if (iteration % zigzag1_period) == 0:
                            zigzag1_sign *= -1

                    if iteration >= robot1Iterations:
                        robot1StopFlag = True
                        print(f"Robot 1 stopped at: {robot1Position}")

                if robot2Position and angle2 and robot2Speed is not None  and not robot2StopFlag:
                    windVector = sim.getCurrentWind(robot2Position)
                    wind_array = np.array([windVector.x, windVector.y, windVector.z])

                    if concentration2 > 0:

                        if np.allclose(wind_array, [0, 0, 0]):
                            robot2Position.x += robot2Speed * updateInterval
                            robot2Position.y += 0
                        else:
                            inverse_wind = wind_array
                            norm = np.linalg.norm(inverse_wind[:2])
                            if norm == 0:
                                robot2Position.x += robot2Speed * updateInterval
                                robot2Position.y += 0
                            else:
                                direction = inverse_wind[:2] / norm
                                robot2Position.x += robot2Speed * updateInterval * direction[0]
                                robot2Position.y += robot2Speed * updateInterval * direction[1]
                    else:
                        zigzag2_angle = normalize_angle(angle2 + zigzag2_sign * (np.pi / 4))
                        next2_x = robot2Position.x - robot2Speed * updateInterval * np.cos(zigzag2_angle)
                        next2_y = robot2Position.y + robot2Speed * updateInterval * np.sin(zigzag2_angle)
                        next2_position = Vector3(next2_x, next2_y, robot2Position.z)

                        print(f"Trying to move to ({next2_x}, {next2_y}), Free?: {sim.checkPositionForObstacles(next2_position)}")

                        if not sim.checkPositionForObstacles(next2_position):
                            zigzag2_sign = -zigzag2_sign
                            zigzag2_angle = normalize_angle(angle2 + zigzag2_sign * (np.pi / 4))
                            next2_x = robot2Position.x - robot2Speed * updateInterval * np.cos(zigzag2_angle)
                            next2_y = robot2Position.y + robot2Speed * updateInterval * np.sin(zigzag2_angle)
                            next2_position = Vector3(next2_x, next2_y, robot1Position.z)

                            if not sim.checkPositionForObstacles(next2_position):
                                next2_x = robot2Position.x + robot2Speed * updateInterval * np.cos(zigzag2_angle)
                                next2_y = robot2Position.y + robot2Speed * updateInterval * np.sin(zigzag2_angle)
                                next2_position = Vector3(next2_x, next2_y, robot1Position.z)
                                if not sim.checkPositionForObstacles(next2_position):
                                    next2_x = robot2Position.x + robot2Speed * updateInterval * np.cos(zigzag2_angle)
                                    next2_y = robot2Position.y - robot2Speed * updateInterval * np.sin(zigzag2_angle)
                                    next2_position = Vector3(next2_x, next2_y, robot2Position.z)
                        
                        robot2Position.x = next2_x
                        robot2Position.y = next2_y

                        if (iteration % zigzag2_period) == 0:
                            zigzag2_sign *= -1

                    if iteration >= robot2Iterations:
                        robot2StopFlag = True
                        print(f"Robot 2 reached the target position: {robot2Position}")

                if robot3Position and angle3 and robot3Speed is not None and not robot3StopFlag:
                    windVector = sim.getCurrentWind(robot3Position)
                    wind_array = np.array([windVector.x, windVector.y, windVector.z])

                    if concentration3 > 0:

                        if np.allclose(wind_array, [0, 0, 0]):
                            robot3Position.x += robot3Speed * updateInterval
                            robot3Position.y += 0
                        else:
                            inverse_wind = wind_array
                            norm = np.linalg.norm(inverse_wind[:2])
                            if norm == 0:
                                robot3Position.x += robot3Speed * updateInterval
                                robot3Position.y += 0
                            else:
                                direction = inverse_wind[:2] / norm
                                robot3Position.x += robot3Speed * updateInterval * direction[0]
                                robot3Position.y += robot3Speed * updateInterval * direction[1]
                    else:
                        zigzag3_angle = normalize_angle(angle3 + zigzag3_sign * (np.pi / 4))
                        next3_x = robot3Position.x - robot3Speed * updateInterval * np.cos(zigzag3_angle)
                        next3_y = robot3Position.y + robot3Speed * updateInterval * np.sin(zigzag3_angle)
                        next3_position = Vector3(next3_x, next3_y, robot3Position.z)

                        print(f"Trying to move to ({next3_x}, {next3_y}), Free?: {sim.checkPositionForObstacles(next3_position)}")

                        if not sim.checkPositionForObstacles(next3_position):
                            zigzag3_sign = -zigzag3_sign
                            zigzag3_angle = normalize_angle(angle3 + zigzag3_sign * (np.pi / 4)) 
                            next3_x = robot3Position.x - robot3Speed * updateInterval * np.cos(zigzag3_angle)
                            next3_y = robot3Position.y + robot3Speed * updateInterval * np.sin(zigzag3_angle)
                            next3_position = Vector3(next3_x, next3_y, robot1Position.z)

                            if not sim.checkPositionForObstacles(next3_position):
                                next3_x = robot3Position.x + robot3Speed * updateInterval * np.cos(zigzag3_angle)
                                next3_y = robot3Position.y + robot3Speed * updateInterval * np.sin(zigzag3_angle)
                                next3_position = Vector3(next3_x, next3_y, robot1Position.z)
                                if not sim.checkPositionForObstacles(next3_position):
                                    next3_x = robot3Position.x + robot3Speed * updateInterval * np.cos(zigzag3_angle)
                                    next3_y = robot3Position.y - robot3Speed * updateInterval * np.sin(zigzag3_angle)
                                    next3_position = Vector3(next3_x, next3_y, robot3Position.z)
                        
                        robot3Position.x = next3_x
                        robot3Position.y = next3_y

                        if (iteration % zigzag3_period) == 0:
                            zigzag3_sign *= -1

                    if iteration >= robot3Iterations:
                        robot3StopFlag = True
                        print(f"Robot 3 reached the target position: {robot3Position}")

                if robot4Position and angle4 and robot4Speed is not None and not robot4StopFlag:
                    windVector = sim.getCurrentWind(robot4Position)
                    wind_array = np.array([windVector.x, windVector.y, windVector.z])

                    if concentration4 > 0:

                        if np.allclose(wind_array, [0, 0, 0]):
                            robot4Position.x += robot4Speed * updateInterval
                            robot4Position.y += 0
                        else:
                            inverse_wind = wind_array
                            norm = np.linalg.norm(inverse_wind[:2])
                            if norm == 0:
                                robot4Position.x += robot4Speed * updateInterval
                                robot4Position.y += 0
                            else:
                                direction = inverse_wind[:2] / norm
                                robot4Position.x += robot4Speed * updateInterval * direction[0]
                                robot4Position.y += robot4Speed * updateInterval * direction[1]
                    else:
                        zigzag4_angle = normalize_angle(angle4 + zigzag4_sign * (np.pi / 4))
                        next4_x = robot4Position.x - robot4Speed * updateInterval * np.cos(zigzag4_angle)
                        next4_y = robot4Position.y + robot4Speed * updateInterval * np.sin(zigzag4_angle)
                        next4_position = Vector3(next4_x, next4_y, robot1Position.z)

                        print(f"Trying to move to ({next4_x}, {next4_y}), Free?: {sim.checkPositionForObstacles(next4_position)}")

                        if not sim.checkPositionForObstacles(next4_position):
                            zigzag4_sign = -zigzag4_sign
                            zigzag4_angle = normalize_angle(angle4 + zigzag4_sign * (np.pi / 4))
                            next4_x = robot4Position.x - robot4Speed * updateInterval * np.cos(zigzag4_angle)
                            next4_y = robot4Position.y + robot4Speed * updateInterval * np.sin(zigzag4_angle)
                            next4_position = Vector3(next4_x, next4_y, robot1Position.z)

                            if not sim.checkPositionForObstacles(next4_position):
                                next4_x = robot4Position.x + robot4Speed * updateInterval * np.cos(zigzag4_angle)
                                next4_y = robot4Position.y + robot4Speed * updateInterval * np.sin(zigzag4_angle)
                                next4_position = Vector3(next4_x, next4_y, robot1Position.z)
                                if not sim.checkPositionForObstacles(next4_position):
                                    next4_x = robot4Position.x + robot4Speed * updateInterval * np.cos(zigzag4_angle)
                                    next4_y = robot4Position.y - robot4Speed * updateInterval * np.sin(zigzag4_angle)
                                    next4_position = Vector3(next4_x, next4_y, robot4Position.z)
                        
                        robot4Position.x = next4_x
                        robot4Position.y = next4_y

                        if (iteration % zigzag4_period) == 0:
                            zigzag4_sign *= -1

                    if iteration >= robot4Iterations:
                        robot4StopFlag = True
                        print(f"Robot 4 reached the target position: {robot4Position}")
                
                if robot1StopFlag and robot2StopFlag and robot3StopFlag and robot4StopFlag: 
                    break
            else:
                time.sleep(0.01)

    main()

    simulation_data_serializable = []


    for frame in simulation_data:
        simulation_data_serializable.append({
            "robot_position": vector3_to_dict(frame["robot_position"]),
            "concentration": frame["concentration"],
            "wind_speed": vector3_to_dict(frame["wind_speed"]),
            "iteration": frame["iteration"],
            "robot": frame["robot"]
        })


    return JSONResponse(content={"frames": simulation_data_serializable, "robotSim_id": robotSim_id + 1}) 

@app.get("/pso_simmulation")
def pso(username: str, simulationNumber: str, height: float, robots):
    if isinstance(robots, str):
        robots = json.loads(robots)

    print(robots)

    robot1Iterations = int(robots[0]["iterations"])
    robot1Speed = float(robots[0]["robotSpeed"])
    robot1Xposition = float(robots[0]["robotXlocation"])
    robot1Yposition = float(robots[0]["robotYlocation"])
    
    initialRobot1Position = Vector3(robot1Xposition,robot1Yposition, height)
    pbest1 = PBest(initialRobot1Position,0.0)
    robot1Velocity = Vector3(0.0, 0.0, 0.0)

    if len(robots) > 1:
        robot2Iterations = int(robots[1]["iterations"])
        robot2Speed = float(robots[1]["robotSpeed"])
        robot2Xposition = float(robots[1]["robotXlocation"])
        robot2Yposition = float(robots[1]["robotYlocation"])

        initialRobot2Position = Vector3(robot2Xposition,robot2Yposition, height)

    else:
        robot2Iterations = 0 
        initialRobot2Position = None
        robot2Speed = robot2Xposition = robot2Yposition  = None
    pbest2 = PBest(initialRobot2Position,0.0)
    robot2Velocity = Vector3(0.0, 0.0, 0.0)
    print(f"robot2: speed: {robot2Speed}, X position: {robot2Xposition}, Y position: {robot2Yposition}")

    if len(robots) > 2:
        robot3Iterations = int(robots[2]["iterations"])
        robot3Speed = float(robots[2]["robotSpeed"])
        robot3Xposition = float(robots[2]["robotXlocation"])
        robot3Yposition = float(robots[2]["robotYlocation"])

        initialRobot3Position = Vector3(robot3Xposition,robot3Yposition, height)

    else:
        robot3Iterations = 0 
        initialRobot3Position  = None
        robot3Speed = robot3Xposition = robot3Yposition  = None
    pbest3 = PBest(initialRobot3Position,0.0) 
    robot3Velocity = Vector3(0.0, 0.0, 0.0)
    print(f"robot3: speed: {robot3Speed}, X position: {robot3Xposition}, Y position: {robot3Yposition}")
    

    if len(robots) > 3:
        robot4Iterations = int(robots[3]["iterations"])
        robot4Speed = float(robots[3]["robotSpeed"])
        robot4Xposition = float(robots[3]["robotXlocation"])
        robot4Yposition = float(robots[3]["robotYlocation"])

        initialRobot4Position = Vector3(robot4Xposition,robot4Yposition, height)

    else:
        robot4Iterations = 0 
        initialRobot4Position = None
        robot4Speed = robot4Xposition = robot4Yposition = angle4 = None
    pbest4 = PBest(initialRobot4Position,0.0)
    robot4Velocity = Vector3(0.0, 0.0, 0.0)
    print(f"robot4: speed: {robot4Speed}, X position: {robot4Xposition}, Y position: {robot4Yposition}")
    

    

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
    def calculate_average_position(robot1position, robot2position, robot3position, robot4position):
        valid_positions = []

        if robot1position is not None:
            valid_positions.append(robot1position)
        if robot2position is not None:
            valid_positions.append(robot2position)
        if robot3position is not None:
            valid_positions.append(robot3position)
        if robot4position is not None:
            valid_positions.append(robot4position)

        if not valid_positions:
            return None

        average_position = sum(valid_positions) / len(valid_positions)

        return average_position
    
    average_pointX = calculate_average_position(robot1Xposition, robot2Xposition, robot3Xposition, robot4Xposition)
    average_pointY = calculate_average_position(robot1Yposition, robot2Yposition, robot3Yposition, robot4Yposition)
    
    average_point = Vector3(average_pointX,average_pointY,height)

    rclpy.init()
    gbest = GBest(average_point, 0.0)
    gbest_position = retrieve_gbest_position()
    


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

    def vector3_to_dict(v):
        return {"x": v.x, "y": v.y, "z": v.z}

    def capture_simulation_data(robot_position, concentration, wind_speed, iteration, robot):
        frame_data = {
            "robot_position": Vector3(robot_position.x, robot_position.y, robot_position.z),
            "concentration": concentration,
            "wind_speed": wind_speed,
            "iteration": iteration,
            "robot": robot
        }

        simulation_data.append(frame_data)


    def markPreviousPositions(previous1Positions,previous2Positions,previous3Positions,previous4Positions,
        initialRobot1Position, initialRobot2Position, initialRobot3Position, initialRobot4Position,
        image):

        for pos in previous1Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        for pos in previous2Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        for pos in previous3Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)

        for pos in previous4Positions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            cv2.rectangle(image, (i, j-1), (i, j), (255,255,255), -1)
            

        j = int((initialRobot1Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
        i = int((initialRobot1Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
        image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
        cv2.putText(image, "R1", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


        if initialRobot2Position is not None:
            j = int((initialRobot2Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((initialRobot2Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
            cv2.putText(image, "R2", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


            if initialRobot3Position is not None:
                j = int((initialRobot3Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
                i = int((initialRobot3Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
                image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
                cv2.putText(image, "R3", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


                if initialRobot4Position is not None:
                    j = int((initialRobot4Position.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
                    i = int((initialRobot4Position.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
                    image = cv2.circle(image, (i, j), 4,(255,255,255), -1)
                    cv2.putText(image, "R4", (i - 30, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    def main():
        updateInterval = 0.5 # segundos
        sim.playSimulation(0, updateInterval)

        robot1Position = initialRobot1Position
        robot2Position = initialRobot2Position if robot2Speed is not None else None
        robot3Position = initialRobot3Position if robot3Speed is not None else None
        robot4Position = initialRobot4Position if robot4Speed is not None else None

        previousRobot1Positions = []
        previousRobot2Positions = []
        previousRobot3Positions = []
        previousRobot4Positions = []

        robot1StopFlag = False

        # if the postion doesnt exist the robot has the stop flag set to True if not sets it to false
        robot2StopFlag = robot2Position is None
        robot3StopFlag = robot3Position is None
        robot4StopFlag = robot4Position is None


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
        
        last_iteration = -1
        
        w = 0.7      
        c1 = 1.5   
        c2 = 1.5  


        while (True):
            iteration = sim.getCurrentIteration()
            if iteration > last_iteration:
                last_iteration = iteration
                print(f"Iteration: {iteration} robot1Position: {robot1Position}")

                previousRobot1Positions.append(Vector3(robot1Position.x, robot1Position.y, robot1Position.z))
                if robot2Position is not None:
                    previousRobot2Positions.append(Vector3(robot2Position.x, robot2Position.y, robot2Position.z))
                    if robot3Position is not None:
                        previousRobot3Positions.append(Vector3(robot3Position.x, robot3Position.y, robot3Position.z))                
                        if robot4Position is not None:
                            previousRobot4Positions.append(Vector3(robot4Position.x, robot4Position.y, robot4Position.z))
                


                concentration1 = sim.getCurrentConcentration(robot1Position)
                print(f"Location: {robot1Position}")
                print(f"Concentration at robot1 position: {concentration1} ppm")

                concentration2 = concentration3 = concentration4 = 0 

                capture_simulation_data(robot1Position, concentration1, sim.getCurrentWind(robot1Position), iteration, 1)
                if robot2Position is not None:
                    concentration2 = sim.getCurrentConcentration(robot2Position)
                    capture_simulation_data(robot2Position, concentration2, sim.getCurrentWind(robot2Position), iteration, 2)
                    if robot3Position is not None:
                        concentration3 = sim.getCurrentConcentration(robot3Position)
                        capture_simulation_data(robot3Position,concentration3, sim.getCurrentWind(robot3Position), iteration, 3)
                        if robot4Position is not None:
                            concentration4 = sim.getCurrentConcentration(robot4Position)
                            capture_simulation_data(robot4Position, concentration4, sim.getCurrentWind(robot4Position), iteration, 4)
            


                map = sim.generateConcentrationMap2D(iteration, height, True)
                map_scaled = map * (255.0 / max_ppm)
                formatted_map = np.array(np.clip(map_scaled, 0, 255), dtype=np.uint8)

                base_image = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)
                block(map, base_image)

                newshape = (imageSizeFactor * base_image.shape[1], imageSizeFactor * base_image.shape[0])
                heatmap = cv2.resize(base_image, newshape)

                markPreviousPositions(previousRobot1Positions, previousRobot2Positions,previousRobot3Positions,previousRobot4Positions,
                                        initialRobot1Position, initialRobot2Position, initialRobot3Position, initialRobot4Position,
                                        heatmap)

                capture_frame_for_gif(heatmap)

                if robot1Position is not None and not robot1StopFlag:
                    print("ok")
                    if concentration1 > pbest1.get_concentration():
                        pbest1.update_pbest(robot1Position,concentration1)
                    
                    gbest_position = gbest.position

                    r1 = np.random.rand()
                    r2 = np.random.rand()

                    pbest_diff = Vector3(
                        pbest1.position.x - robot1Position.x,
                        pbest1.position.y - robot1Position.y,
                        0
                    )
                    gbest_diff = Vector3(
                        gbest_position.x - robot1Position.x,
                        gbest_position.y - robot1Position.y,
                        0
                    )
                    # Update velocity
                    robot1Velocity.x = (w * robot1Velocity.x +
                                        c1 * r1 * pbest_diff.x +
                                        c2 * r2 * gbest_diff.x)
                    robot1Velocity.y = (w * robot1Velocity.y +
                                        c1 * r1 * pbest_diff.y +
                                        c2 * r2 * gbest_diff.y)

                    speed = np.sqrt(robot1Velocity.x**2 + robot1Velocity.y**2)
                    if speed > robot1Speed:
                        robot1Velocity.x = (robot1Velocity.x / speed) * robot1Speed
                        robot1Velocity.y = (robot1Velocity.y / speed) * robot1Speed

                    robot1Position.x += robot1Velocity.x * updateInterval
                    robot1Position.y += robot1Velocity.y * updateInterval

                    if iteration > robot1Iterations:
                        robot1StopFlag = True

                    if robot1StopFlag and robot2StopFlag and robot3StopFlag and robot4StopFlag:
                        break

        
    main()
    simulation_data_serializable = []
    rclpy.shutdown()

    for frame in simulation_data:
        simulation_data_serializable.append({
            "robot_position": vector3_to_dict(frame["robot_position"]),
            "concentration": frame["concentration"],
            "wind_speed": vector3_to_dict(frame["wind_speed"]),
            "iteration": frame["iteration"],
            "robot": frame["robot"]
        })


    return JSONResponse(content={"frames": simulation_data_serializable, "robotSim_id": robotSim_id + 1}) 


