import carla

def move_spectator_to(transform, distance=1.0, x=0, y=0, z=4, yaw=0, pitch=-30, roll=0):
    back_location = transform.location - transform.get_forward_vector() * distance
    
    back_location.x += x
    back_location.y += y
    back_location.z += z
    transform.rotation.yaw += yaw
    transform.rotation.pitch = pitch
    transform.rotation.roll = roll
    
    spectator_transform = carla.Transform(back_location, transform.rotation)
    
    _spectator.set_transform(spectator_transform)

def move_spectator_relative_to_vehicle(vehicle, location_offset, rotation):
    """
    Moves the spectator, using coordinates relative to a vehicle.

    Parameters
    ---
    vehicle:
        Starting vehicle.
    location_offset: carla.Location
        Offset used to place the camera, relative to the vehicle.
        The camera position is calculated adding the offset to the vehicle position.
    rotation: carla.Rotation
        Absolute rotation.
    """
    global joystick
    
    rear_camera_transform = vehicle.get_transform()
    rear_camera_transform.location = rear_camera_transform.location + location_offset
    rear_camera_transform.rotation = rotation
    move_spectator_to(rear_camera_transform)

def spawn_vehicle(vehicle_index=0, spawn_index=0, pattern='vehicle.*'):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle

def spawn_camera(attach_to=None, transform=carla.Transform(carla.Location(x=1.2, z=1.2), carla.Rotation(pitch=-10)), width=800, height=600):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(width))
    camera_bp.set_attribute('image_size_y', str(height))
    camera = world.spawn_actor(camera_bp, transform, attach_to=attach_to)
    return camera

def spawn_obstacle_vehicles(
    locations,
    rotations,
    patterns=[]
):
    """
    Spawns a set of vehicles, in the given positions.

    Parameters
    ---
    locations: list<carla.Location>
        `carla.Location`s where to place the obstacles.
    rotations: list<carla.Rotation>
        `carla.Rotation`'s to give to the obstacles.
    patterns: list<String>, optional
        List of patterns used to select which vehicle to spawn.
        If you want to specify the pattern for some obstacles, set as `None` the patterns you don't want to specify.
        E.g. given 3 obstacles, you only want to specify the pattern of the 2nd: patterns = [ None, "vehicle.mitsubishi.fusorosa", None ]

    Return
    ---
    obstacles: list<CarlaVehicle>
        List with all the spawned vehicles.
    """
    
    if len(locations) != len(rotations):
        return None

    obstacles = []
    obstacles_number = len(locations)

    if len(patterns) < obstacles_number:            
        patterns = [ None if i >= len(patterns) or patterns[i] is None else patterns[i] for i in range(obstacles_number)]
    
    for i in range(obstacles_number):
        obstacle = spawn_vehicle(pattern="vehicle.*" if patterns[i] == None else patterns[i])
        obstacle_transform = obstacle.get_transform()

        x, y, z = locations[i]
        pitch, roll, yaw = rotations[i]

        # Check if variables are None
        x = x or obstacle_transform.location.x
        y = y or obstacle_transform.location.y
        z = z or obstacle_transform.location.z
        
        pitch = pitch or obstacle_transform.rotation.pitch
        roll = roll or obstacle_transform.rotation.roll
        yaw = yaw or obstacle_transform.rotation.yaw
        
        obstacle_transform.location = carla.Location(x=x, y=y, z=z)
        obstacle_transform.rotation = carla.Rotation(pitch=pitch, roll=roll, yaw=yaw)
        obstacle.set_transform(obstacle_transform)

        obstacles.append(obstacle)

    return obstacles

def destroy_actors():
    for actor in world.get_actors().filter("vehicle.*"):
        actor.destroy()


_client = carla.Client('localhost', 2000)
_client.set_timeout(20.0)

world = _client.get_world()
if(world.get_map().name != "Carla/Maps/Town04"):
    world = _client.load_world("Town04")

_spectator = world.get_spectator()