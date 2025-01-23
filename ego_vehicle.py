import carla, time, pygame, math, random, cv2
import numpy as np
import paho.mqtt.client as mqtt

# Contstants #################################################################
EGO_VEHICLE_INITIAL_LOCATION = carla.Location(x=280, y=-207.5, z=0.1) # z=0.1 used to make the car not clip with the ground
EGO_VEHICLE_INITIAL_ROTATION = carla.Rotation(yaw=180)

MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_TOPIC = "svs-rcta"
MQTT_MESSAGE = "warning: vehicle nearby"

RADARS_DISTANCE = 4 # distanza a cui il radar vede
MINIMUM_DELTA_DISTANCE = 2 # minimo delta tra i due radar

OBSTACLE_DIRECTION_DEFAULT_COLOR = (0, 255, 0)

# Functions ##############################################################

def log(message, event="vehicle"):
    print(f"[{event}] {message}")

def move_spectator_to(transform, distance=1.0, x=0, y=0, z=4, yaw=0, pitch=-30, roll=0):
    back_location = transform.location - transform.get_forward_vector() * distance
    
    back_location.x += x
    back_location.y += y
    back_location.z += z
    transform.rotation.yaw += yaw
    transform.rotation.pitch = pitch
    transform.rotation.roll = roll
    
    spectator_transform = carla.Transform(back_location, transform.rotation)
    
    spectator.set_transform(spectator_transform)

def spawn_vehicle(vehicle_index=0, spawn_index=0, pattern='vehicle.*'):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(pattern)[vehicle_index]
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle

def destroy_actors():
    for actor in world.get_actors().filter("vehicle.*"):
        actor.destroy()

# def draw_on_screen(world, transform, content='O', color=carla.Color(0, 255, 0), life_time=20):
#     world.debug.draw_string(transform.location, content, color=color, life_time=life_time)

def spawn_camera(attach_to=None, transform=carla.Transform(carla.Location(x=1.2, z=1.2), carla.Rotation(pitch=-10)), width=800, height=600):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(width))
    camera_bp.set_attribute('image_size_y', str(height))
    camera = world.spawn_actor(camera_bp, transform, attach_to=attach_to)
    return camera

def spawn_radar(attach_to=None,
                transform=carla.Transform(carla.Location(x=2.0, z=1.5), carla.Rotation(pitch=5)),
                width=800, height=600,
                horizontal_fov=30,
                vertical_fov=20,
                range=100,
                sensor_tick=0,
                points_per_second=1500):
    
    radar_bp = world.get_blueprint_library().find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', str(horizontal_fov))
    radar_bp.set_attribute('vertical_fov', str(vertical_fov))
    radar_bp.set_attribute('range', str(range))
    radar_bp.set_attribute('sensor_tick', str(sensor_tick))
    radar_bp.set_attribute('points_per_second', str(points_per_second))
    radar = world.spawn_actor(radar_bp, transform, attach_to=attach_to)
    return radar

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    # print(f"Connected with result code {reason_code}")
    log(f"Connected with result code {reason_code}", "MQTT")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_TOPIC + "/#")

def on_publish(client, userdata, mid, reason_code, properties):
    # reason_code and properties will only be present in MQTTv5. It's always unset in MQTTv3
    try:
        userdata.remove(mid)
    except KeyError:
        print("on_publish() is called with a mid not present in unacked_publish")
        print("This is due to an unavoidable race-condition:")
        print("* publish() return the mid of the message sent.")
        print("* mid from publish() is added to unacked_publish by the main thread")
        print("* on_publish() is called by the loop_start thread")
        print("While unlikely (because on_publish() will be called after a network round-trip),")
        print(" this is a race-condition that COULD happen")
        print("")
        print("The best solution to avoid race-condition is using the msg_info from publish()")
        print("We could also try using a list of acknowledged mid rather than removing from pending list,")
        print("but remember that mid could be re-used !")

def mqtt_publish(message, topic=MQTT_TOPIC, qos=1, publish_interval=1):
    global last_message_time
    global unacked_publish
    
    current_time = time.time()
    tdelta = current_time - last_message_time
    if tdelta > publish_interval:
        last_message_time = time.time()
        msg_info = mqttc.publish(topic, message, qos=qos)
        unacked_publish.add(msg_info.mid)

publish_interval = 1

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

def spawn_rear_radars(attach_to, horizontal_fov=120, range=40):
    """
    Spawns two radars and set them in the rear of the given vehicle.
    """

    x = -1.8
    y = 0.75
    z = 0.5
    pitch = 3
    yaw = 90 + horizontal_fov // 2
    right_radar_location = carla.Location(x=x, y=y, z=z)
    right_radar_rotation = carla.Rotation(pitch=pitch, yaw=yaw)
    left_radar_location = carla.Location(x=x, y=-y, z=z)
    left_radar_rotation = carla.Rotation(pitch=pitch, yaw=-yaw)
    
    right_radar = spawn_radar(attach_to=attach_to,
                          transform=carla.Transform(right_radar_location, right_radar_rotation),
                          horizontal_fov=horizontal_fov,
                          range=range)
    left_radar = spawn_radar(attach_to=attach_to,
                         transform=carla.Transform(left_radar_location, left_radar_rotation),
                         horizontal_fov=horizontal_fov,
                         range=range)
    return left_radar, right_radar

from datetime import datetime, timedelta

def draw_radar_points(
    location,
    size=0.075,
    life_time=0.06,
    persistent_lines=False,
    color=carla.Color(255, 255, 255)
):
    """
    Draws a set of point on the view, based on the locations given.
    """
    
    global world
    world.debug.draw_point(
        location=location,
        size=size,
        life_time=life_time,
        persistent_lines=persistent_lines,
        color=color
    )

TTC_THRESHOLD = 0.4 # second
def hirst_graham_distance_algorithm(v_rel, vF):
    return TTC_THRESHOLD * v_rel + 0.4905 * vF

def compute_velocity_from_vector(velocity_vector):
    v_x = velocity_vector.x
    v_y = velocity_vector.y
    v_z = velocity_vector.z
    return math.sqrt(v_x**2 + v_y**2 + v_z**2)

def get_radar_points_colors(velocity, velocity_range=0.8):
    """
    Returns the radar colors based on the velocities.
    """

    def clamp(min_v, max_v, value):
        return max(min_v, min(value, max_v))

    norm_velocity = velocity / velocity_range # range [-1, 1]
    r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
    g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
    b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)

    return norm_velocity, r, g, b

def automatic_brake(vehicle):
    """
    Stops the vehicle if it is moving.
    """

    global automatic_brake_engaged
    if compute_velocity_from_vector(vehicle.get_velocity()) > 0:
        automatic_brake_engaged = True
        control = vehicle.get_control()
        control.throttle = 0
        control.brake = 1
        vehicle.apply_control(control)

# def radar_callback(radar_data, draw_radar=True, radar_point_color=carla.Color(2, 0, 255)):
#     global reverse
#     global automatic_brake_engaged
#     global last_message_time
#     global screen_color_start_time

#     if not reverse:
#         return
    
#     # deactivate automatic brake
#     if automatic_brake_engaged and compute_velocity_from_vector(vehicle.get_velocity()) == 0:
#         print("Deactivate auto brake")
#         automatic_brake_engaged = False

#     current_rot = radar_data.transform.rotation
#     for detect in radar_data:
#         azi = math.degrees(detect.azimuth)
#         alt = math.degrees(detect.altitude)
#         # The 0.25 adjusts a bit the distance so the dots can
#         # be properly seen
#         fw_vec = carla.Vector3D(x=detect.depth - 0.25)
#         carla.Transform(
#             carla.Location(),
#             carla.Rotation(
#                 pitch=current_rot.pitch + alt,
#                 yaw=current_rot.yaw + azi,
#                 roll=current_rot.roll)).transform(fw_vec)

#         def compute_ttc(distance, delta_v):
#             if delta_v == 0: return None
#             return abs(distance / delta_v)

#         norm_velocity, r, g, b = get_radar_points_colors(detect.velocity)
#         ttc = compute_ttc(detect.depth, detect.velocity)
#         if norm_velocity < 0 and ttc != None and ttc < TTC_THRESHOLD: # and detect.depth < 5:
#             mqtt_publish(MQTT_MESSAGE, publish_interval=publish_interval)
#             try:
#                 alarm_sound.play()
#             except Exception as e:
#                 log(f"Error during alarm sound play", "sound")
#                 # print(f"Errore durante la riproduzione del suono: {e}")

#             if screen_color_start_time == None:
#                 screen_color_start_time = pygame.time.get_ticks()
#                 screen_color((255, 0, 0))

#             collision_distance = hirst_graham_distance_algorithm(-detect.velocity, compute_velocity_from_vector(vehicle.get_velocity()))

#             world.debug.draw_point(
#                     radar_data.transform.location + fw_vec,
#                     size=0.075,
#                     life_time=0.06,
#                     persistent_lines=False,
#                     color=carla.Color(r, g, b))
            
#             if detect.depth < collision_distance:
#                 automatic_brake(vehicle)
        
#         # keeps notifying if the obstacle is nearer than 1 meter
#         if detect.depth < 1:
#             mqtt_publish(MQTT_MESSAGE, publish_interval=publish_interval)

#         # draw radars
#         if draw_radar:
#             world.debug.draw_point(radar_data.transform.location,
#                                 size=0.075,
#                                     life_time=0.06,
#                                     persistent_lines=False,
#                                     color=radar_point_color)

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

def load_alarm_sound():
    pygame.mixer.init()
    alarm_sound = None
    try:
        alarm_sound = pygame.mixer.Sound("alarm_sound.mp3")
    except Exception as e:
        # print(f"Errore durante il caricamento del file audio: {e}")
        log(f"Errore durante il caricamento del file audio: {e}", "sound")
    return alarm_sound

def screen_color(color):
    screen.fill(color)
    pygame.display.flip()

def screen_half(color, side):
    width, height = screen.get_size()
    if side == "right":
        rect = pygame.Rect(width // 2, 0, width // 2, height)
    elif side == "left":
        rect = pygame.Rect(0, 0, width // 2, height)
    else:
        raise ValueError("side must be 'left' or 'right'")
    
    screen.fill((0, 0, 0))  # Fill the entire screen with black before drawing the rectangle
    pygame.draw.rect(screen, color, rect)
    pygame.display.flip()

def check_screen_color():
    global screen_color_start_time

    if screen_color_start_time is not None:
        elapsed_time = pygame.time.get_ticks() - screen_color_start_time
        if elapsed_time > 2000:
            screen_color((0, 0, 0))
            screen_color_start_time = None

# === Controls =======================================

def toggle_reverse_gear():
    global reverse
    reverse = not reverse
    control = carla.VehicleControl(reverse=reverse)

    reverse_gear_status = ("" if reverse else "dis") + "engaged"
    log(f"Reverse gear {reverse_gear_status}")

    return control

def apply_control_using_joystick(joystick, vehicle, control):
    global automatic_brake_engaged
    global reverse

    def normalize(value, deadzone=0.05):
        """
        Normalize both steering wheel and pedals controls.
        """

        if abs(value) < deadzone:
            return 0.0
        return value
    
    # Leggi input dal volante (es. asse 0 per sterzo)
    steering = joystick.get_axis(0)  # Sterzo

    K2 = 1.6  # 1.6
    throttleCmd = K2 + (2.05 * math.log10(
        -0.7 * joystick.get_axis(3) + 1.4) - 1.2) / 0.92
    if throttleCmd <= 0:
        throttleCmd = 0
    elif throttleCmd > 1:
        throttleCmd = 1

    brakeCmd = 1.6 + (2.05 * math.log10(
        -0.7 * joystick.get_axis(4) + 1.4) - 1.2) / 0.92
    if brakeCmd <= 0:
        brakeCmd = 0
    elif brakeCmd > 1:
        brakeCmd = 1
    
    # Normalizza e inverti valori, se necessario
    control.steer = normalize(steering)
    control.brake = 1 - brakeCmd      # Invertito per alcuni dispositivi
    if not automatic_brake_engaged:
        control.throttle = 1 - throttleCmd  # Invertito per alcuni dispositivi
    else:
        control.throttle = 0

    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 5:
                control = toggle_reverse_gear()

    vehicle.apply_control(control)

def apply_control_using_keyboard():
    global reverse
    global running

    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            key = event.key
            log(f"{pygame.key.name(key)}", "key pressed")
            if key == pygame.QUIT or key == pygame.K_ESCAPE or key == pygame.K_q:
                running = False
            elif key == pygame.K_r:
                control = toggle_reverse_gear()
                vehicle.apply_control(control)
            elif key == pygame.K_w:
                log("Going " + ("backward" if reverse else "foreward"))
                control = carla.VehicleControl(throttle=0.4, reverse=reverse)
                vehicle.apply_control(control)
            elif key == pygame.K_s:
                control = carla.VehicleControl(throttle=0, brake=1)
                vehicle.apply_control(control)
            elif key == pygame.K_c:
                destroy_actors()
                log(f"Actors on map destroyed. Relunch the script...", "system")

def common_radar_function(radar_data, draw_radar=True, radar_point_color=carla.Color(2, 0, 255)):
    """
    Function used by a radar to detect objects.
    """

    global reverse
    global automatic_brake_engaged
    global last_message_time
    global screen_color_start_time

    distance_sum = None
    velocity_sum = None

    if not reverse:
        return distance_sum, velocity_sum
    
    # deactivate automatic brake
    if automatic_brake_engaged and compute_velocity_from_vector(vehicle.get_velocity()) == 0:
        print("Deactivate auto brake")
        automatic_brake_engaged = False

    current_rot = radar_data.transform.rotation
    for detect in radar_data:
        azi = math.degrees(detect.azimuth)
        alt = math.degrees(detect.altitude)
        # The 0.25 adjusts a bit the distance so the dots can
        # be properly seen
        fw_vec = carla.Vector3D(x=detect.depth - 0.25)
        carla.Transform(
            carla.Location(),
            carla.Rotation(
                pitch=current_rot.pitch + alt,
                yaw=current_rot.yaw + azi,
                roll=current_rot.roll)).transform(fw_vec)

        def compute_ttc(distance, delta_v):
            if delta_v == 0: return None
            return abs(distance / delta_v)
        
        norm_velocity, r, g, b = get_radar_points_colors(detect.velocity)
        ttc = compute_ttc(detect.depth, detect.velocity)

        # norm_velocity < 0 => obstacle is approaching
        # if ttc != None and ttc < TTC_THRESHOLD:
        # if detect.depth <= RADARS_DISTANCE:
        if ttc != None and ttc < TTC_THRESHOLD: # and detect.depth < 5:
            mqtt_publish(MQTT_MESSAGE, publish_interval=publish_interval)
            try:
                alarm_sound.play()
            except Exception as e:
                log(f"Error during alarm sound play", "sound")
                # print(f"Errore durante la riproduzione del suono: {e}")

            if screen_color_start_time == None:
                screen_color_start_time = pygame.time.get_ticks()
                screen_color((255, 0, 0))

            collision_distance = hirst_graham_distance_algorithm(-detect.velocity, compute_velocity_from_vector(vehicle.get_velocity()))

            world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(r, g, b))
            
            if detect.depth < collision_distance:
                automatic_brake(vehicle)\
            
            if distance_sum is None: distance_sum = 0
            if velocity_sum is None: velocity_sum = 0

            distance_sum = distance_sum + detect.depth
            velocity_sum = velocity_sum + detect.velocity

        # keeps notifying if the obstacle is nearer than 1 meter
        if detect.depth < 1:
            mqtt_publish(MQTT_MESSAGE, publish_interval=publish_interval)

        # draw radars
        if draw_radar:
            world.debug.draw_point(radar_data.transform.location,
                                size=0.075,
                                    life_time=0.06,
                                    persistent_lines=False,
                                    color=radar_point_color)
    
    avg_distance = distance_sum / len(radar_data) if distance_sum is not None else None
    avg_velocity = velocity_sum / len(radar_data) if velocity_sum is not None else None
    return avg_distance, avg_velocity

def left_radar_callback(radar_data, draw_radar=True, radar_point_color=carla.Color(2, 0, 255)):
    global detected_obstacle

    distance, _ = common_radar_function(radar_data, draw_radar, radar_point_color)

    if distance is None:
        detected_obstacle.left_radar.setActive(False)
        return

    if distance < RADARS_DISTANCE:
        detected_obstacle.left_radar.setActive(True)
        detected_obstacle.left_radar.setDistance(distance)
    else:
        detected_obstacle.left_radar.setActive(False)

def right_radar_callback(radar_data, draw_radar=True, radar_point_color=carla.Color(2, 0, 255)):
    global detected_obstacle

    distance, _ = common_radar_function(radar_data, draw_radar, radar_point_color)

    if distance is None:
        detected_obstacle.right_radar.setActive(False)
        return

    if distance < RADARS_DISTANCE:
        detected_obstacle.right_radar.setActive(True)
        detected_obstacle.right_radar.setDistance(distance)
    else:
        detected_obstacle.right_radar.setActive(False)

######################################################################
#                              MAIN
######################################################################

# !!! Constants are placed on top of the file !!!

# Initial setup #####################################################

pygame.init()
pygame.joystick.init()

client = carla.Client('localhost', 2000)
client.set_timeout(20.0)

world = client.get_world()
if(world.get_map().name != "Carla/Maps/Town04"):
    world = client.load_world("Town04")
spectator = world.get_spectator()

joystick = None

# Variables setup ####################################################

# Flag that indicates if the script is ran on the simulator
#   needs to be set manually
simulator = False
screen = pygame.display.set_mode((200, 100))
reverse = False
automatic_brake_engaged = False
obstacles_enabled = False
screen_color_start_time = None
running = True
points = np.zeros((600, 800, 4), dtype=np.uint8)
alarm_sound = load_alarm_sound()
spectator_camera = carla.Location(x=-12, y=0, z=7) if simulator else carla.Location(x=0, y=0, z=5)

# MQTT setup
unacked_publish = set()
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_publish = on_publish
mqttc.user_data_set(unacked_publish)

mqttc.on_connect = on_connect

mqttc.connect(MQTT_BROKER, MQTT_PORT, 60)
mqttc.loop_start()

last_message_time = time.time()

obstacle_on_right = False
if obstacles_enabled:
    obstacles = spawn_obstacle_vehicles(
        [(284, (-205 if obstacle_on_right else -211), 0.1)],
        [(None, None, 90)],
        patterns=["vehicle.tesla.model3"]
        # [(286, -202.5, 0.1), (282, -210.5, 0.1)],
        # [(None, None, 90), (None, None, 180)],
        # patterns=["vehicle.mitsubishi.fusorosa", "vehicle.carlamotors.carlacola"]
    )

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

# Vehicle configuration
vehicle = spawn_vehicle()
vehicle_transform = vehicle.get_transform()
vehicle_transform.location = EGO_VEHICLE_INITIAL_LOCATION
vehicle_transform.rotation = EGO_VEHICLE_INITIAL_ROTATION
vehicle.set_transform(vehicle_transform)

# Radar configuration
left_radar, right_radar = spawn_rear_radars(attach_to=vehicle)
# right_radar.listen(lambda image: radar_callback(image))
# left_radar.listen(lambda image: radar_callback(image))
right_radar.listen(lambda image: right_radar_callback(image))
left_radar.listen(lambda image: left_radar_callback(image))
time.sleep(0.2)

move_spectator_relative_to_vehicle(vehicle, spectator_camera, carla.Rotation(yaw=0))

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
            apply_control_using_joystick(joystick, vehicle, control)

        apply_control_using_keyboard()

        check_screen_color()
        world.tick()
        pygame.display.flip()
finally:
    pygame.quit()
    right_radar.destroy()
    left_radar.destroy()
    vehicle.destroy()
    if obstacles_enabled:
        for obstacle in obstacles:
            obstacle.destroy()
