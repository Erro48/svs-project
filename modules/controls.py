import math
import os, carla, pygame # type: ignore
from modules.utilities import compute_velocity_from_vector, log
FILENAME = os.path.splitext(os.path.basename(__file__))[0]

def toggle_reverse_gear():
    global reverse
    reverse = not reverse
    control = carla.VehicleControl(reverse=reverse)

    reverse_gear_status = ("" if reverse else "dis") + "engaged"
    log(f"Reverse gear {reverse_gear_status}", FILENAME)

    return control

def apply_control_using_keyboard():
    global reverse
    global running
    global vehicle

    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            key = event.key
            log(f"{pygame.key.name(key)}", FILENAME)
            if key == pygame.QUIT or key == pygame.K_ESCAPE or key == pygame.K_q:
                running = False
            elif key == pygame.K_r:
                control = toggle_reverse_gear()
                vehicle.apply_control(control)
            elif key == pygame.K_w:
                log("Going " + ("backward" if reverse else "foreward"), FILENAME)
                control = carla.VehicleControl(throttle=0.4, reverse=reverse)
                vehicle.apply_control(control)
            elif key == pygame.K_s:
                control = carla.VehicleControl(throttle=0, brake=1)
                vehicle.apply_control(control)
            elif key == pygame.K_c:
                destroy_actors()
                log(f"Actors on map destroyed. Relunch the script...", FILENAME)

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