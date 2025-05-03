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
import requests
import base64
import sys

app = FastAPI()

@app.get("/")
def test():
    vector3Up = Vector3(0, 0, 1)
    
    scenario_path = os.path.join("/src/install/test_env/share/test_env/scenarios","new_sim_266")
    simulation_path = os.path.join(scenario_path,"gas_simulations/sim1")
    ocuppancy_path = os.path.join(scenario_path,"OccupancyGrid3D.csv")

    simulation_name = sys.argv[1].replace("_sim_", "_")

    sim = Simulation(simulation_path, \
                    ocuppancy_path)

    def markPreviousPositions(previousPositions, initialRobotPosition, image):
        for pos in previousPositions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
            image = cv2.circle(image, (i, j), 2, (0, 0, 0), -1)

        j = int((initialRobotPosition.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * image.shape[0])
        i = int((initialRobotPosition.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * image.shape[1])
        image = cv2.circle(image, (i, j), 4, (255, 0, 0), -1)

        return image

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
        initialRobotPosition = Vector3(3.5, 3, 0.5)
        robotPosition = initialRobotPosition
        robotSpeed = 2
        hitThreshold = 0.02

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

        base_image = np.zeros((500, 500, 3), dtype=np.uint8)  

        while simulationTime < timeLimitSeconds and distanceFromSource(robotPosition) > 0.5:
            iteration = sim.getCurrentIteration()

            if iteration >= 100:
                print(f"Stopping simulation at iteration {iteration}")
                break
            if not sim.checkPositionForObstacles(robotPosition):
                print("Something went wrong! The robot is in a wall!")
                break
            previousRobotPositions.append(robotPosition)

            concentration = sim.getCurrentConcentration(robotPosition)
            if concentration > hitThreshold and simulationTime - cast_timer > deltaTime:
                robotVelocity = -sim.getCurrentWind(robotPosition).projectOnPlane(vector3Up).normalized() * robotSpeed
                currentCastLength = baseCastLength
            else:
                robotVelocity = sim.getCurrentWind(robotPosition).projectOnPlane(vector3Up).cross(vector3Up).normalized() * robotSpeed * castDirectionMultiplier
                if simulationTime - cast_timer > currentCastLength:
                    castDirectionMultiplier *= -1
                    cast_timer = simulationTime
                    currentCastLength += baseCastLength

            robotVelocity = changeVelocityForObstacles(robotPosition, robotVelocity, deltaTime)
            robotPosition += robotVelocity * deltaTime

            base_image = markPreviousPositions(previousRobotPositions, initialRobotPosition, base_image)

            capture_frame_for_gif(robotPosition, previousRobotPositions)

            simulationTime += deltaTime
            time.sleep(updateInterval)

        if distanceFromSource(robotPosition) <= 0.5:
            print("Success!")
        
        save_gif_and_send()

        sim.stopPlaying()

    def capture_frame_for_gif(robotPosition, previousRobotPositions):
        """
        Capture the current frame and add it to the list of frames.
        """
        iteration = sim.getCurrentIteration()
        map = sim.generateConcentrationMap2D(iteration, robotPosition.z, True)
        map_scaled = map * (255.0 / 10.0) 
        formatted_map = np.array(np.clip(map_scaled, 0, 255), dtype=np.uint8)

        heatmap = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)

        for pos in previousRobotPositions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * heatmap.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * heatmap.shape[1])
            heatmap = cv2.circle(heatmap, (i, j), 2, (0, 0, 0), -1)

        rgb_image = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)

        frames.append(pil_img)
        print(f"robotPosition: {robotPosition}, concentration: {sim.getCurrentConcentration(robotPosition)}")
        print(f"Captured frame for iteration {iteration}")

    def save_gif_and_send():
        """
        Save the collected frames as a GIF locally and send it via POST request.
        """
        gif_filename = "simulation_result.gif"

        gif_io = io.BytesIO()
        frames[0].save(gif_io, format='GIF', save_all=True, append_images=frames[1:], duration=50, loop=0)

        gif_raw = gif_io.getvalue()
        compressed_gif = zlib.compress(gif_raw)
        compressed_gif_base64 = base64.b64encode(compressed_gif).decode('utf-8')

        response = requests.post('http://172.17.0.3:3000/uploadSimulationResults', json={
            'simulation': "new_266",  
            'type': 'robot',
            'gif': compressed_gif_base64,
            'height': "0.5",
        })

        print("GIF sent successfully.")

    surge_cast()
