from fastapi import FastAPI
import numpy
import cv2
import time
from PIL import Image
import io
import zlib
import os
from gadentools.Simulation import Simulation
from gadentools.Utils import Vector3
from IPython.display import clear_output
from gadentools.Utils import block

app = FastAPI()

@app.get("/")
def test():
    vector3Up = Vector3(0,0,1)

    scenario_path = "/src/install/test_env/share/test_env/scenarios/new_sim_263"
    simulation_path = os.path.join(scenario_path,"gas_simulations/sim1")
    ocuppancy_path = os.path.join(scenario_path,"OccupancyGrid3D.csv")

    sim = Simulation(simulation_path, \
                 ocuppancy_path)
    timeLimitSeconds = 100  # simulation time limit in seconds
    updateInterval = 0.5  # how frequently to update the simulation
    robotSpeed = 2
    hitThreshold = 0.2
    baseCastLength = 0.2
    imageSizeFactor = 5
    frame_duration_ms = 50
    contour_threshold = 40
    max_ppm = 10.0

    frames = []  # to store the frames for GIFs

    def markPreviousPositions(previousPositions, initialRobotPosition, image):
        #trail
        for pos in previousPositions:
            j = int( (pos.x-sim.env_min.x) / (sim.env_max.x-sim.env_min.x) * image.shape[0] )
            i = int( (pos.y-sim.env_min.y) / (sim.env_max.y-sim.env_min.y) * image.shape[1] )
            image = cv2.circle(image, (i,j), 2, (0,0,0), -1)

        #starting point
        j = int( (initialRobotPosition.x-sim.env_min.x) / (sim.env_max.x-sim.env_min.x) * image.shape[0] )
        i = int( (initialRobotPosition.y-sim.env_min.y) / (sim.env_max.y-sim.env_min.y) * image.shape[1] )
        image = cv2.circle(image, (i,j), 4, (255, 0,0), -1)

        return image


    def blend(image1:numpy.ndarray, image2:numpy.ndarray, mask:numpy.ndarray):
        mask = mask.astype(float)/255
        return mask * image2 + (1-mask) * image1

    def distanceFromSource(robotPosition):
        return (sim.source_position - robotPosition).magnitude()

    def changeVelocityForObstacles(robotPosition, robotVelocity, deltaTime):
        # if path is blocked, try to go to the right, then to the left, and then back
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


    def selectPoint(x, y):
        if not(initiated):
            return
        im = numpy.copy(base_image)
        j = int( (x-sim.env_min.x) / (sim.env_max.x-sim.env_min.x) * im.shape[0] )
        i = int( (y-sim.env_min.y) / (sim.env_max.y-sim.env_min.y) * im.shape[1] )

        im=cv2.circle(im, (i,j), 4, (0,0,255), -1)

        im = cv2.rotate(im, cv2.ROTATE_90_COUNTERCLOCKWISE)
        global pointToQuery
        pointToQuery = Vector3(x,y,h)   


    def showMapAndPlume(height:float, iteration:int):
        global base_image
        global h
        h = height

        map = sim.generateConcentrationMap2D(iteration, height, True)
        map = map * (255.0/10)
        formatted_map = numpy.array(numpy.clip(map, 0, 255), dtype=numpy.uint8)
        formatted_map = cv2.cvtColor(formatted_map, cv2.COLOR_GRAY2BGR)

        base_image = numpy.full(formatted_map.shape, 255, numpy.uint8)
        block(map, base_image) #draw obstacles in black

        # draw the gas plume
        solid_color=numpy.empty_like(base_image)
        solid_color[:] = (0, 255, 0)
        base_image = blend(base_image, solid_color, formatted_map)

        #resize
        imageSizeFactor = 5
        newshape = (imageSizeFactor* map.shape[1], imageSizeFactor*map.shape[0])
        base_image = cv2.resize(base_image, newshape)

        global initiated
        initiated = True


    def save_simulation_gifs():
        previousRobotPositions = []  # for drawing the robot trail
        cast_timer = 0
        castDirectionMultiplier = 1
        currentCastLength = baseCastLength
        simulationTime = 0
        deltaTime = 0.1
        robotPosition = Vector3(3.5, 3, 0.5)  # initial position
        initialRobotPosition = robotPosition
        while simulationTime < timeLimitSeconds and distanceFromSource(robotPosition) > 0.5:
            if not sim.checkPositionForObstacles(robotPosition):
                print("Something went wrong! The robot is in a wall!")
                break
            previousRobotPositions.append(robotPosition)
            clear_output(wait=True)

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

            # Obstacle handling (same as in the original code)
            robotVelocity = changeVelocityForObstacles(robotPosition, robotVelocity, deltaTime)
            robotPosition += robotVelocity * deltaTime

            # Visualization
            showMapAndPlume(0.5, sim.getCurrentIteration())
            global base_image
            base_image = markPreviousPositions(previousRobotPositions, initialRobotPosition, base_image)
            selectPoint(robotPosition.x, robotPosition.y)

            # Capture frame for GIF
            capture_simulation_frame(robotPosition,previousRobotPositions)

            simulationTime += deltaTime
            time.sleep(updateInterval)

        # Check if simulation succeeded
        if distanceFromSource(robotPosition) <= 0.5:
             print("Success!")
        sim.stopPlaying()

    def capture_simulation_frame(robotPosition,previousRobotPositions):
        """
        Capture the current frame and add it to the list of frames.
        """
        iteration = sim.getCurrentIteration()
        map = sim.generateConcentrationMap2D(iteration, robotPosition.z, True)
        map_scaled = map * (255.0 / max_ppm)
        formatted_map = numpy.array(numpy.clip(map_scaled, 0, 255), dtype=numpy.uint8)

        heatmap = cv2.applyColorMap(formatted_map, cv2.COLORMAP_JET)
        
        # Add the robot trail (mark previous positions)
        for pos in previousRobotPositions:
            j = int((pos.x - sim.env_min.x) / (sim.env_max.x - sim.env_min.x) * heatmap.shape[0])
            i = int((pos.y - sim.env_min.y) / (sim.env_max.y - sim.env_min.y) * heatmap.shape[1])
            heatmap = cv2.circle(heatmap, (i, j), 2, (0, 0, 0), -1)

        # Resize and convert for GIF
        newshape = (imageSizeFactor * heatmap.shape[1], imageSizeFactor * heatmap.shape[0])
        heatmap = cv2.resize(heatmap, newshape)
        rgb_image = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_image)
        
        # Store the captured frame
        frames.append(pil_img)

        print(f"Captured frame for iteration {iteration}")

    def save_gif():
        """
        Save the collected frames as a GIF locally in the current working directory.
        """
        # Define the filename for the GIF
        gif_filename = os.path.join(os.getcwd(), "simulation_result.gif")

        gif_io = io.BytesIO()
        frames[0].save(gif_io, format='GIF', save_all=True, append_images=frames[1:], duration=frame_duration_ms, loop=0)

        gif_raw = gif_io.getvalue()

        # Save the GIF locally
        with open(gif_filename, "wb") as f:
            f.write(gif_raw)
        
        print(f"GIF saved to {gif_filename}")

    # Main simulation loop
    save_simulation_gifs()

    # After the simulation is finished, save the GIF locally
    save_gif()