import carla, time # type: ignore
import numpy as np

import modules.utilities as utilities
import modules.environment as environment
import modules.controls as controls



######################################################################
#                              MAIN
######################################################################

# !!! Constants are placed on top of the file !!!

# Initial setup #####################################################

controls.pygame.init()
controls.pygame.joystick.init()



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
spectator_camera = carla.Location(x=-12, y=0, z=7) if simulator else carla.Location(x=0, y=0, z=5)
controls.screen_setup()


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



time.sleep(0.2)

environment.move_spectator_relative_to_vehicle(environment.vehicle, spectator_camera, carla.Rotation(yaw=0))

# Controls configuration
if simulator:
    # Associa il joystick (volante e pedaliera)
    joystick = controls.pygame.joystick.Joystick(0)
    joystick.init()

try:
    # Vehicle control
    control = carla.VehicleControl()

    while controls.running:
        controls.pygame.event.pump()
        if reverse:
            side = environment.detected_obstacle.getObstacleSide()

        if simulator:
            controls.apply_control_using_joystick(joystick, environment.vehicle, control)

        controls.apply_control_using_keyboard(controls.pygame.event.get())

        controls.check_screen_color()
        environment.world.tick()
        controls.pygame.display.flip()
finally:
    controls.pygame.quit()
    environment.right_radar.destroy()
    environment.left_radar.destroy()
    environment.vehicle.destroy()
    if obstacles_enabled:
        for obstacle in obstacles:
            obstacle.destroy()
