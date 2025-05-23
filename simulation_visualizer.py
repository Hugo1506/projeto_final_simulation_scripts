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
timeLimitSeconds = 15       
minHeight = sim.env_min.z              
max_ppm = 10.0                 
imageSizeFactor = 5          
frame_duration_ms = 50  
contour_threshold = 40      

frames = []

def save_heatmap_gifs(height):
    frames.clear()
    sim.playSimulation(0, 0.5)
    time_start = time.time()
    gif_io = io.BytesIO()
    iteration_counter = 0

    while (time.time() - time_start) < timeLimitSeconds:
        clear_output(wait=True)
        iteration = sim.getCurrentIteration()
        
        if iteration == iteration_counter:
            iteration_counter += 1
            map = sim.generateConcentrationMap2D(iteration, height, True)
            map_scaled = map * (255.0 / max_ppm)
            formatted_map = numpy.array(numpy.clip(map_scaled, 0, 255), dtype=numpy.uint8)

            heatmap = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)

            block(map, heatmap)

            newshape = (imageSizeFactor * heatmap.shape[1], imageSizeFactor * heatmap.shape[0])
            heatmap = cv2.resize(heatmap, newshape)

            rgb_image = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb_image)
            frames.append(pil_img)

            print(f"Captured frame for iteration {iteration}")

    sim.stopPlaying()

    gif_io = io.BytesIO()
    frames[0].save(gif_io, format='GIF', save_all=True, append_images=frames[1:], duration=frame_duration_ms, loop=0)

    gif_raw = gif_io.getvalue()

    compressed_gif = zlib.compress(gif_raw)

    compressed_gif_base64 = base64.b64encode(compressed_gif).decode('utf-8')

    response = requests.post('http://webserver:3000/uploadSimulationResults', json={
        'simulation': simulation_name,
        'type': 'heatmap',
        'gif': compressed_gif_base64,
        'height': height,
    })
    
def save_wind_vector_field_gif(height):
    frames.clear()
    sim.playSimulation(0, 0.5)
    time_start = time.time()
    gif_io = io.BytesIO()
    iteration_counter = 0

    while (time.time() - time_start) < timeLimitSeconds:
        iteration = sim.getCurrentIteration()
        if iteration == iteration_counter:
            iteration_counter += 1
            clear_output(wait=True)
            map = sim.generateWindMap2D(iteration, height, True)

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
            frames.append(pil_img)

            print(f"Captured wind vector field at height {height:.2f}")

    sim.stopPlaying()

    gif_io = io.BytesIO()
    frames[0].save(gif_io, format='GIF', save_all=True, append_images=frames[1:], duration=frame_duration_ms, loop=0)

    gif_raw = gif_io.getvalue()
    compressed_gif = zlib.compress(gif_raw)
    compressed_gif_base64 = base64.b64encode(compressed_gif).decode('utf-8')

    response = requests.post('http://webserver:3000/uploadSimulationResults', json={
        'simulation': simulation_name,
        'type': 'wind',
        'gif': compressed_gif_base64,
        'height': height,
    })

def save_contour_map_gif(height: float):
    frames.clear()
    sim.playSimulation(0, 0.5)
    time_start = time.time()
    iteration_counter = 0
    while (time.time() - time_start) < timeLimitSeconds:
        clear_output(wait=True)
        iteration = sim.getCurrentIteration()
        if iteration == iteration_counter:
            iteration_counter += 1
            map = sim.generateConcentrationMap2D(iteration, height, True)
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
            frames.append(pil_img)

            print(f"Iteration {iteration} — Gas patches: {len(contours)}")

    sim.stopPlaying()

    gif_io = io.BytesIO()
    frames[0].save(gif_io, format='GIF', save_all=True, append_images=frames[1:], duration=frame_duration_ms, loop=0)

    gif_raw = gif_io.getvalue()
    compressed_gif = zlib.compress(gif_raw)
    compressed_gif_base64 = base64.b64encode(compressed_gif).decode('utf-8')

    response = requests.post('http://webserver:3000/uploadSimulationResults', json={
        'simulation': simulation_name,
        'type': 'contour',
        'gif': compressed_gif_base64,
        'height': height,
    })


for height in numpy.arange(minHeight, maxHeight, 0.5):
  save_heatmap_gifs(height)
  save_wind_vector_field_gif(height)
  save_contour_map_gif(height)