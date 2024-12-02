import carla, time, random
import numpy as np

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.get_world()
map = world.get_map()

# Functions
def spawn_vehicle(vehicle_index=0, spawn_index=0, pattern='vehicle.*'):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle

def spawn_vehicle_right_lane(
        x=285, y=-227.5,
        spawn_delay=0,
        start_delay=1):
    time.sleep(spawn_delay)
    traffic_vehicle = spawn_vehicle()

    traffic_vehicle_transform = traffic_vehicle.get_transform()
    traffic_vehicle_transform.location = carla.Location(x=x, y=y)
    traffic_vehicle_transform.rotation = carla.Rotation(yaw=91.5) # with yaw=90 it's not aligned with the parking slots
    traffic_vehicle.set_transform(traffic_vehicle_transform)

    time.sleep(start_delay)
    traffic_vehicle.apply_control(carla.VehicleControl(throttle=random.uniform(0.7, 1)))

    return traffic_vehicle

def spawn_vehicle_left_lane(
        x=286, y=-192,
        spawn_delay=0,
        start_delay=1):
    time.sleep(spawn_delay)
    traffic_vehicle = spawn_vehicle()

    traffic_vehicle_transform = traffic_vehicle.get_transform()
    traffic_vehicle_transform.location = carla.Location(x=x, y=y, z=0.5)
    traffic_vehicle_transform.rotation = carla.Rotation(yaw=271.5)
    traffic_vehicle.set_transform(traffic_vehicle_transform)

    time.sleep(start_delay)
    traffic_vehicle.apply_control(carla.VehicleControl(throttle=random.uniform(0.7, 1)))

    return traffic_vehicle

def spawn_two_vehicles(time_to_live=4):
    vehicle1_destroy = True
    vehicle2_destroy = True
    try:
        while(True):
            if(vehicle1_destroy):
                vehicle1 = spawn_vehicle_right_lane()
                vehicle1_destroy = False

            if(vehicle2_destroy):
                vehicle2 = spawn_vehicle_left_lane()
                vehicle2_destroy = False

            if(vehicle1.get_transform().location.y > -192):
                vehicle1_destroy = True

            if(vehicle2.get_transform().location.y < -227.5):
                vehicle2_destroy = True

            if(vehicle1_destroy):
                vehicle1.destroy()

            if(vehicle2_destroy):
                vehicle2.destroy()
    finally:
       vehicle1.destroy()
       vehicle2.destroy()

#Run
spawn_two_vehicles()