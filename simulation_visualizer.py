from __future__ import print_function
from gadentools.Simulation import Simulation
from gadentools.Utils import Vector3
from gadentools.Utils import block
import numpy
import cv2
from PIL import Image
import imageio
import time
import os
from IPython.display import clear_output
import sys
import requests
import base64
import io
import zlib

scenario_path = os.path.join("/src/install/test_env/share/test_env/scenarios",sys.argv[1])
simulation_path = os.path.join(scenario_path,"gas_simulations/sim1")
ocuppancy_path = os.path.join(scenario_path,"OccupancyGrid3D.csv")

simulation_name = sys.argv[1].replace("_sim_", "_")

sim = Simulation(simulation_path, \
                 ocuppancy_path)

arrowLength = 10
spaceBetweenArrows = 5
maxHeight = sim.env_max.z
minHeight = sim.env_min.z              
max_ppm = 10.0                 
imageSizeFactor = 5          
frame_duration_ms = 50  
contour_threshold = 40      
timePerIteration = 1
numberOfIterations = 10



def save_heatmaps(height):
    sim.playSimulation(0, timePerIteration)
    last_iteration = -1
    while sim.getCurrentIteration() != 0:
        time.sleep(0.1)
    while sim.getCurrentIteration() < numberOfIterations:
        clear_output(wait=True)

        
        map = sim.generateConcentrationMap2D(sim.getCurrentIteration(), height, True)
        map_scaled = map * (255.0 / max_ppm)
        formatted_map = numpy.array(numpy.clip(map_scaled, 0, 255), dtype=numpy.uint8)

        heatmap = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)

        block(map, heatmap)

        newshape = (imageSizeFactor * heatmap.shape[1], imageSizeFactor * heatmap.shape[0])
        heatmap = cv2.resize(heatmap, newshape)

        rgb_image = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)

        buffered = io.BytesIO()
        pil_img.save(buffered, format="PNG") 
        compressed = zlib.compress(buffered.getvalue())
        img_str = base64.b64encode(compressed).decode('utf-8')

        
        if sim.getCurrentIteration() > last_iteration:
            last_iteration = sim.getCurrentIteration()

            print(f"{sim.getCurrentIteration()}: Captured heatmap at {height}")
            response = requests.post('http://webserver:3000/uploadSimulationResults', json={
                'simulation': simulation_name,
                'type': 'heatmap',
                'gif': img_str,
                'height': height,
                'iteration': str(sim.getCurrentIteration())
            })

            if response.status_code != 200:
                print(f"Failed to upload: {response.status_code}, {response.text}")

    print("iteration limit reached, stopping simulation")
    sim.stopPlaying()


    
def save_wind_vector_field(height):
    sim.stopPlaying()
    sim.playSimulation(0, timePerIteration)
    last_iteration = -1

    while sim.getCurrentIteration() != 0:
        time.sleep(0.1)
    while sim.getCurrentIteration() < numberOfIterations:
    
        clear_output(wait=True)
        map = sim.generateWindMap2D(sim.getCurrentIteration(), height, True)

        base_image = numpy.full(map.shape, 255, numpy.uint8)
        block(map, base_image)

        newshape = (imageSizeFactor * map.shape[1], imageSizeFactor * map.shape[0])
        base_image = cv2.resize(base_image, newshape)
        base_image = cv2.cvtColor(base_image, cv2.COLOR_GRAY2BGR)


        for i in range(0, map.shape[0], spaceBetweenArrows):
            for j in range(0, map.shape[1], spaceBetweenArrows):
                if isinstance(map[i, j], Vector3):
                    offsetX = int(map[i, j].x * arrowLength)
                    offsetY = int(map[i, j].y * arrowLength)
                    start_point = (imageSizeFactor * j, imageSizeFactor * i)
                    end_point = (start_point[0] + offsetY, start_point[1] + offsetX)
                    cv2.arrowedLine(base_image, start_point, end_point, (0, 0, 255), 2)

        rgb_image = cv2.cvtColor(base_image, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)

        buffered = io.BytesIO()
        pil_img.save(buffered, format="PNG") 
        compressed = zlib.compress(buffered.getvalue())
        img_str = base64.b64encode(compressed).decode('utf-8')
            

        if sim.getCurrentIteration() > last_iteration:
            last_iteration = sim.getCurrentIteration()
            print(f"{sim.getCurrentIteration()}: Captured wind vector at {height:.2f}")

            response = requests.post('http://webserver:3000/uploadSimulationResults', json={
                'simulation': simulation_name,
                'type': 'wind',
                'gif': img_str,
                'height': height,
                'iteration': str(sim.getCurrentIteration())
            })

            if response.status_code != 200:
                print(f"Failed to upload: {response.status_code}, {response.text}")

    print("iteration limit reached, stopping simulation")
    sim.stopPlaying()


def save_contour_map(height: float):
    sim.stopPlaying()
    sim.playSimulation(0, timePerIteration)
    last_iteration = -1

    while sim.getCurrentIteration() != 0:
        time.sleep(0.1)
    while sim.getCurrentIteration() < numberOfIterations:
        clear_output(wait=True)

        
        map = sim.generateConcentrationMap2D(sim.getCurrentIteration(), height, True)
        map = map * (255.0 / max_ppm)
        formatted_map = numpy.array(numpy.clip(map, 0, 255), dtype=numpy.uint8)

        newshape = (imageSizeFactor * formatted_map.shape[1], imageSizeFactor * formatted_map.shape[0])
        formatted_map = cv2.resize(formatted_map, newshape)

        ret, thresh = cv2.threshold(formatted_map, contour_threshold, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        formatted_map = cv2.cvtColor(formatted_map, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(formatted_map, contours, -1, (0, 255, 0), 3)


        rgb_image = cv2.cvtColor(formatted_map, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)

        buffered = io.BytesIO()
        pil_img.save(buffered, format="PNG") 
        compressed = zlib.compress(buffered.getvalue())
        img_str = base64.b64encode(compressed).decode('utf-8')

        if sim.getCurrentIteration() > last_iteration:
            last_iteration = sim.getCurrentIteration()
            print(f"{sim.getCurrentIteration()}: Captured countor at {height } — Gas patches: {len(contours)}")

            response = requests.post('http://webserver:3000/uploadSimulationResults', json={
                'simulation': simulation_name,
                'type': 'contour',
                'gif': img_str,
                'height': height,
                'iteration': str(sim.getCurrentIteration())
            })
            if response.status_code != 200:
                print(f"Failed to upload: {response.status_code}, {response.text}")

    print("iteration limit reached, stopping simulation")
    sim.stopPlaying()


for height in numpy.arange(minHeight, maxHeight, 0.5):
  save_heatmaps(height)
  save_wind_vector_field(height)
  save_contour_map(height)
