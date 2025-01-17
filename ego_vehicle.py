import carla, time, pygame # type: ignore
import numpy as np

from modules.utilities import load_alarm_sound, log
from modules.pygame_screen import screen_setup, screen_color, screen_half, check_screen_color

import modules.environment as environment
import modules.controls as controls
import modules.radars as radars


# Contstants #################################################################
EGO_VEHICLE_INITIAL_LOCATION = carla.Location(x=280, y=-207.5, z=0.1) # z=0.1 used to make the car not clip with the ground
EGO_VEHICLE_INITIAL_ROTATION = carla.Rotation(yaw=180)

OBSTACLE_DIRECTION_DEFAULT_COLOR = (0, 255, 0)

######################################################################
#                              MAIN
######################################################################

# !!! Constants are placed on top of the file !!!

# Initial setup #####################################################

pygame.init()
pygame.joystick.init()



joystick = None

# Variables setup ####################################################

# Flag that indicates if the script is ran on the simulator
#   needs to be set manually
simulator = False

reverse = False
automatic_brake_engaged = False
obstacles_enabled = False
running = True
points = np.zeros((600, 800, 4), dtype=np.uint8)
alarm_sound = load_alarm_sound()
spectator_camera = carla.Location(x=-12, y=0, z=7) if simulator else carla.Location(x=0, y=0, z=5)
screen_setup()


obstacle_on_right = False
if obstacles_enabled:
    obstacles = environment.spawn_obstacle_vehicles(
        [(284, (-205 if obstacle_on_right else -211), 0.1)],
        [(None, None, 90)],
        patterns=["vehicle.tesla.model3"]
        # [(286, -202.5, 0.1), (282, -210.5, 0.1)],
        # [(None, None, 90), (None, None, 180)],
        # patterns=["vehicle.mitsubishi.fusorosa", "vehicle.carlamotors.carlacola"]
    )

# Vehicle configuration
vehicle = environment.spawn_vehicle()
vehicle_transform = vehicle.get_transform()
vehicle_transform.location = EGO_VEHICLE_INITIAL_LOCATION
vehicle_transform.rotation = EGO_VEHICLE_INITIAL_ROTATION
vehicle.set_transform(vehicle_transform)

# Radar configuration
left_radar, right_radar = radars.spawn_rear_radars(attach_to=vehicle)
# right_radar.listen(lambda image: radar_callback(image))
# left_radar.listen(lambda image: radar_callback(image))
right_radar.listen(lambda image: radars.right_radar_callback(image, radar_point_color=carla.Color(255, 0, 0)))
left_radar.listen(lambda image: radars.left_radar_callback(image, radar_point_color=carla.Color(0, 0, 255)))

detected_obstacle = [False, False]

class DetectedObstacleRadar:
    def __init__(self):
        self.active = False
        self.distance = 0

    def setActive(self, status):
        self.active = status

    def isActive(self):
        return self.active
    
    def getDistance(self):
        return self.distance
    
    def setDistance(self, distance):
        self.distance = distance

class DetectedObstacle:

    def __init__(self):
        self.right_radar = DetectedObstacleRadar()
        self.left_radar = DetectedObstacleRadar()

    def getObstacleSide(self, mirrored=True, color=OBSTACLE_DIRECTION_DEFAULT_COLOR):
        """
        Returns the side from which the obstacle is coming.
        
        Parameters:
        ---
        mirrored: Boolean (default)
            Indicates whether the returned side is shown mirrored relative to the side from which the obstacle comes.
        """

        side = ""

        # Quando un oggetto è molto spostato da un lato (o destra o sinistra)
        # i radar leggono distanze molto diverse (es. oggetto a destra, radar destro legge 1 metro
        # radar sinistro legge 5 metri). Il delta quindi è grande
        # Quando un oggetto è dietro alla macchina, i radar leggono valori simili, quindi il
        # delta è piccolo
        delta_distance = round(abs(self.right_radar.distance - self.left_radar.distance), 3)
        coming_from_right = None
        obstacle_behind = False

        if self.right_radar.isActive() and not self.left_radar.isActive():
            coming_from_right = True
        elif not self.right_radar.isActive() and self.left_radar.isActive():
            coming_from_right = False
        elif self.right_radar.isActive() and self.left_radar.isActive() and delta_distance < MINIMUM_DELTA_DISTANCE:
            obstacle_behind = True
        else:
            obstacle_behind = False

        if coming_from_right is not None:
            coming_from_right = coming_from_right if not mirrored else not coming_from_right
            side = "right" if coming_from_right else "left"
            screen_half(color, side)
        elif obstacle_behind:
            side = "behind"
            screen_color(color)
        else:
            side = ""
            screen_color((0, 0, 0))

        return side
    
    def print_radar_state(self, dx=True, sx=True, active=True, distances=True):
        if dx:
            dx_active = self.right_radar.active if active else ""
            dx_distance = round(self.right_radar.distance, 2) if distances else ""
            dx_separator = ", " if dx_active is not "" and dx_distance is not "" else ""
            dx_output = f"dx({dx_active}{dx_separator}{dx_distance})"

        if sx:
            sx_active = self.left_radar.active if active else ""
            sx_distance = round(self.left_radar.distance, 2) if distances else ""
            sx_separator = ", " if sx_active is not "" and sx_distance is not "" else ""
            sx_output = f"sx({sx_active}{sx_separator}{sx_distance})"
        
        log(f"[{sx_output} {dx_output}]", "radars")

detected_obstacle = DetectedObstacle()

time.sleep(0.2)

environment.move_spectator_relative_to_vehicle(vehicle, spectator_camera, carla.Rotation(yaw=0))

# Controls configuration
if simulator:
    # Associa il joystick (volante e pedaliera)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

try:
    # Vehicle control
    control = carla.VehicleControl()

    while running:
        pygame.event.pump()
        if reverse:
            side = detected_obstacle.getObstacleSide()

        if simulator:
            controls.apply_control_using_joystick(joystick, vehicle, control)

        controls.apply_control_using_keyboard()

        check_screen_color()
        environment.world.tick()
        pygame.display.flip()
finally:
    pygame.quit()
    right_radar.destroy()
    left_radar.destroy()
    vehicle.destroy()
    if obstacles_enabled:
        for obstacle in obstacles:
            obstacle.destroy()
