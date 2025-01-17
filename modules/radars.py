from modules import controls, environment
import math, carla, pygame # type: ignore
import modules.mqtt as mqtt
import modules.utilities as utilities
import os

FILENAME = os.path.splitext(os.path.basename(__file__))[0]
RADARS_DISTANCE = 4 # distanza a cui il radar vede
MINIMUM_DELTA_DISTANCE = 2 # minimo delta tra i due radar

def spawn_radar(world,
                attach_to=None,
                transform=carla.Transform(carla.Location(x=2.0, z=1.5), carla.Rotation(pitch=5)),
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

def spawn_rear_radars(attach_to, world, horizontal_fov=120, range=40):
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
    
    right_radar = spawn_radar(world, attach_to=attach_to,
                          transform=carla.Transform(right_radar_location, right_radar_rotation),
                          horizontal_fov=horizontal_fov,
                          range=range)
    
    left_radar = spawn_radar(world, attach_to=attach_to,
                         transform=carla.Transform(left_radar_location, left_radar_rotation),
                         horizontal_fov=horizontal_fov,
                         range=range)
    
    return left_radar, right_radar

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
#             mqtt_publish(MQTT_MESSAGE)
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
#             mqtt_publish(MQTT_MESSAGE)

#         # draw radars
#         if draw_radar:
#             world.debug.draw_point(radar_data.transform.location,
#                                 size=0.075,
#                                     life_time=0.06,
#                                     persistent_lines=False,
#                                     color=radar_point_color)

def common_radar_function(radar_data,
                          draw_radar=True,
                          radar_point_color=carla.Color(2, 0, 255)):
    """
    Function used by a radar to detect objects.
    """

    distance_sum = None
    velocity_sum = None

    if not controls.reverse:
        return distance_sum, velocity_sum
    
    # deactivate automatic brake
    if controls.automatic_brake_engaged and utilities.compute_velocity_from_vector(environment.vehicle.get_velocity()) == 0:
        utilities.log("Deactivate auto brake", FILENAME)
        controls.automatic_brake_engaged = False

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
        if ttc != None and ttc < utilities.TTC_THRESHOLD:
        # if detect.depth <= RADARS_DISTANCE:
            mqtt.mqtt_publish(mqtt.MQTT_MESSAGE)
            try:
                environment.alarm_sound.play()
            except Exception as e:
                utilities.log(f"Error during alarm sound play", "sound")
                # print(f"Errore durante la riproduzione del suono: {e}")

            if controls.screen_color_start_time == None:
                controls.screen_color_start_time = pygame.time.get_ticks()
                controls.screen_color((255, 0, 0))

            collision_distance = utilities.hirst_graham_distance_algorithm(-detect.velocity, utilities.compute_velocity_from_vector(environment.vehicle.get_velocity()))

            environment.world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(r, g, b))
            
            if distance_sum is None: distance_sum = 0
            if velocity_sum is None: velocity_sum = 0

            distance_sum = distance_sum + detect.depth
            velocity_sum = velocity_sum + detect.velocity

            if detect.depth < collision_distance:
                controls.automatic_brake(environment.vehicle)

        # keeps notifying if the obstacle is nearer than 1 meter
        if detect.depth < 1:
            mqtt.mqtt_publish(mqtt.MQTT_MESSAGE)

        # draw radars
        if draw_radar:
            environment.world.debug.draw_point(radar_data.transform.location,
                                size=0.075,
                                    life_time=0.06,
                                    persistent_lines=False,
                                    color=radar_point_color)
    
    avg_distance = distance_sum / len(radar_data) if distance_sum is not None else None
    avg_velocity = velocity_sum / len(radar_data) if velocity_sum is not None else None
    return avg_distance, avg_velocity

def left_radar_callback(radar_data,
                         detected_obstacle,
                         draw_radar=True,
                         radar_point_color=carla.Color(2, 0, 255)):

    distance, _ = common_radar_function(radar_data, draw_radar, radar_point_color)

    if distance is None:
        detected_obstacle.left_radar.setActive(False)
        return

    if distance < RADARS_DISTANCE:
        detected_obstacle.left_radar.setActive(True)
        detected_obstacle.left_radar.setDistance(distance)
    else:
        detected_obstacle.left_radar.setActive(False)

def right_radar_callback(radar_data,
                         detected_obstacle,
                         draw_radar=True,
                         radar_point_color=carla.Color(2, 0, 255)):

    distance, _ = common_radar_function(radar_data, draw_radar, radar_point_color)

    if distance is None:
        detected_obstacle.right_radar.setActive(False)
        return

    if distance < RADARS_DISTANCE:
        detected_obstacle.right_radar.setActive(True)
        detected_obstacle.right_radar.setDistance(distance)
    else:
        detected_obstacle.right_radar.setActive(False)
