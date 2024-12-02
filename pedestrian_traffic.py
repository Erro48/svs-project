import carla, time, random
import numpy as np

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.get_world()
map = world.get_map()

def spawn_pedestrians_right_lane(
        x=283,
        y=-213,
        spawn_delay=0,
        start_delay=1):
    time.sleep(spawn_delay)

    blueprint_library = world.get_blueprint_library()
    walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)
    pedestrian = world.spawn_actor(walker_bp, spawn_point)

    pedestrian_transform = pedestrian.get_transform()
    pedestrian_transform.location = carla.Location(x=x, y=y, z=0.5)
    pedestrian_transform.rotation = carla.Rotation(yaw=-91.5)
    pedestrian.set_transform(pedestrian_transform)

    time.sleep(start_delay)
    pedestrian.apply_control(carla.WalkerControl(direction=carla.Vector3D(0, 1, 0), speed=random.uniform(0.9, 1)))

    return pedestrian

def spawn_pedestrian():
    pedestrian_spawn = True
    try:
        while(True):
            if(pedestrian_spawn):
                try:
                    pedestrian = spawn_pedestrians_right_lane()
                    pedestrian_spawn = False
                except RuntimeError as e:
                    print(f"RuntimeError: {e}")
            try:    
                if(pedestrian.get_location().y > -203.5):
                    pedestrian.destroy()
                    pedestrian_spawn = True
            except RuntimeError as e:
                print(f"RuntimeError: {e}")
    finally:
        pedestrian.destroy()

#Run
spawn_pedestrian()