from fastapi import FastAPI
import numpy as np
import cv2
import time
from PIL import Image
import io
import zlib
import os
from gadentools.Simulation import Simulation
from gadentools.Utils import Vector3
from gadentools.Utils import block
from fastapi.responses import JSONResponse
import requests
import base64
import sys

app = FastAPI()

@app.get("/")
def robot_simulation(username: str, simulationNumber: str, height: float, robotSpeed: float, robotXposition: float, robotYposition: float):
    vector3Up = Vector3(0, 0, 1)
    initialRobotPosition = Vector3(robotXposition,robotYposition, height)

    simulation_dir = username + "_sim_" + simulationNumber
    scenario_path = os.path.join("/src/install/test_env/share/test_env/scenarios",simulation_dir)
    simulation_path = os.path.join(scenario_path,"gas_simulations/sim1")
    ocuppancy_path = os.path.join(scenario_path,"OccupancyGrid3D.csv")

    simulation_name = username + "_" + simulationNumber

    sim = Simulation(simulation_path, \
                    ocuppancy_path)

    imageSizeFactor = 5  
    max_ppm = 10.0

    simulation_data = []

    def vector3_to_dict(v):
        return {"x": v.x, "y": v.y, "z": v.z}

    def capture_simulation_data(robot_position, concentration, wind_speed):
        frame_data = {
            "robot_position": robot_position,
            "concentration": concentration,
            "wind_speed": wind_speed
        }

        simulation_data.append(frame_data)

    def markPreviousPositions(previousPositions, initialRobotPosition, image):
        for pos in previousPositions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            image = cv2.circle(image, (i, j), 2, (0, 0, 0), -1)

        j = int((initialRobotPosition.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
        i = int((initialRobotPosition.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
        image = cv2.circle(image, (i, j), 4, (255, 0, 0), -1)

        capture_frame_for_gif(image)

    def distanceFromSource(robotPosition):
        return (sim.source_position - robotPosition).magnitude()

    def changeVelocityForObstacles(robotPosition, robotVelocity, deltaTime):
        originalVelocity = robotVelocity

        newRobotPosition = robotPosition + robotVelocity * deltaTime
        if not sim.checkPositionForObstacles(newRobotPosition):
            robotVelocity = originalVelocity.cross(vector3Up).normalized() * originalVelocity.magnitude()
        newRobotPosition = robotPosition + robotVelocity * deltaTime

        if not sim.checkPositionForObstacles(newRobotPosition):
            robotVelocity = -originalVelocity.cross(vector3Up).normalized() * originalVelocity.magnitude()
        newRobotPosition = robotPosition + robotVelocity * deltaTime

        if not sim.checkPositionForObstacles(newRobotPosition):
            robotVelocity = -originalVelocity

        return robotVelocity

    def surge_cast():
        updateInterval = 0.5
        sim.playSimulation(0, updateInterval)
        robotPosition = initialRobotPosition
        hitThreshold = 0.2

        cast_timer = 0
        castDirectionMultiplier = 1
        baseCastLength = 0.2
        currentCastLength = baseCastLength

        previousRobotPositions = []

        simulationTime = 0
        deltaTime = 0.1
        timeLimitSeconds = 100

        global frames
        frames = []

        lastHigh = None
        lastHighConc = 0.0
        found_high_point = False

        low_conc_threshold = 0.01
        time_to_try_return = 5

        while simulationTime < timeLimitSeconds and distanceFromSource(robotPosition) > 0.5:
            iteration = sim.getCurrentIteration()

            if not sim.checkPositionForObstacles(robotPosition):
                print("Something went wrong! The robot is in a wall!")
                break

            previousRobotPositions.append(robotPosition)

            concentration = sim.getCurrentConcentration(robotPosition)
            print(f"Concentration at robot position: {concentration} ppm")

            if concentration > lastHighConc:
                lastHigh = robotPosition
                lastHighConc = concentration
                found_high_point = True
                cast_timer = simulationTime
                print(f"New highest concentration found at {lastHigh} with concentration: {lastHighConc} ppm")

            if found_high_point and (concentration < low_conc_threshold or (simulationTime - cast_timer > time_to_try_return)):
                print(f"Low concentration or timeout. Returning to last high concentration at {lastHigh}")
                robotVelocity = (lastHigh - robotPosition).normalized() * robotSpeed

            else:
                robotVelocity = sim.getCurrentWind(robotPosition).projectOnPlane(vector3Up).cross(vector3Up).normalized() * robotSpeed * castDirectionMultiplier

                if simulationTime - cast_timer > currentCastLength:
                    castDirectionMultiplier *= -1
                    cast_timer = simulationTime
                    currentCastLength += baseCastLength

            robotVelocity = changeVelocityForObstacles(robotPosition, robotVelocity, deltaTime)
            robotPosition += robotVelocity * deltaTime

            map = sim.generateConcentrationMap2D(iteration, height, True)
            map_scaled = map * (255.0 / max_ppm)
            formatted_map = np.array(np.clip(map_scaled, 0, 255), dtype=np.uint8)

            base_image = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)
            block(map, base_image)

            newshape = (imageSizeFactor * base_image.shape[1], imageSizeFactor * base_image.shape[0])
            heatmap = cv2.resize(base_image, newshape)

            markPreviousPositions(previousRobotPositions, initialRobotPosition, heatmap)

            capture_simulation_data(robotPosition, concentration, sim.getCurrentWind(robotPosition))
            simulationTime += deltaTime
            time.sleep(updateInterval)

        if distanceFromSource(robotPosition) <= 0.5:
            print("Success! The robot reached the source.")
        else:
            print(f"Failed to reach the source. Final distance: {distanceFromSource(robotPosition)}")

        save_gif_and_send()
        sim.stopPlaying()



    def capture_frame_for_gif(image):
        iteration = sim.getCurrentIteration()
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)

        frames.append(pil_img)
        print(f"Captured frame for GIF in iteration: {iteration}.")

    def save_gif_and_send():
        gif_filename = "simulation_result.gif"

        gif_io = io.BytesIO()
        frames[0].save(gif_io, format='GIF', save_all=True, append_images=frames[1:], duration=50, loop=0)

        gif_raw = gif_io.getvalue()
        compressed_gif = zlib.compress(gif_raw)
        compressed_gif_base64 = base64.b64encode(compressed_gif).decode('utf-8')

        response = requests.post('http://webserver:3000/uploadSimulationResults', json={
            'simulation': simulation_name,  
            'type': 'robot',
            'gif': compressed_gif_base64,
            'height': height,
        })

        print("GIF sent successfully.")

    surge_cast()

    simulation_data_serializable = []

    for frame in simulation_data:
        simulation_data_serializable.append({
            "robot_position": vector3_to_dict(frame["robot_position"]),
            "concentration": frame["concentration"],
            "wind_speed": vector3_to_dict(frame["wind_speed"]),
        })

    return JSONResponse(content={"frames": simulation_data_serializable})