import math, os, carla, pygame # type: ignore
from modules import environment, utilities
# from modules.utilities import compute_velocity_from_vector, log


FILENAME = os.path.splitext(os.path.basename(__file__))[0]
DEFAULT_COLOR = (0, 0, 0)
screen_color_start_time = None
_screen = None

reverse = False
running = True
automatic_brake_engaged = False

def toggle_reverse_gear():
    global reverse
    reverse = not reverse
    control = carla.VehicleControl(reverse=reverse)

    reverse_gear_status = ("" if reverse else "dis") + "engaged"
    utilities.log(f"Reverse gear {reverse_gear_status}", FILENAME)

    return control

def apply_control_using_keyboard(pygame_events):
    global running, reverse

    for event in pygame_events: #pygame.event.get():
        if event.type == pygame.KEYDOWN:
            key = event.key
            utilities.log(f"{pygame.key.name(key)}", FILENAME)
            if key == pygame.QUIT or key == pygame.K_ESCAPE or key == pygame.K_q:
                running = False
            elif key == pygame.K_r:
                control = toggle_reverse_gear()
                environment.vehicle.apply_control(control)
            elif key == pygame.K_w:
                utilities.log("Going " + ("backward" if reverse else "foreward"), FILENAME)
                control = carla.VehicleControl(throttle=0.4, reverse=reverse)
                environment.vehicle.apply_control(control)
            elif key == pygame.K_s:
                control = carla.VehicleControl(throttle=0, brake=1)
                environment.vehicle.apply_control(control)
            elif key == pygame.K_c:
                environment.destroy_actors()
                utilities.log(f"Actors on map destroyed. Relunch the script...", FILENAME)

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
    if utilities.compute_velocity_from_vector(vehicle.get_velocity()) > 0:
        automatic_brake_engaged = True
        control = vehicle.get_control()
        control.throttle = 0
        control.brake = 1
        vehicle.apply_control(control)

def screen_setup(width=200, height=100):
    global _screen
    _screen = pygame.display.set_mode((width, height))

def screen_color(color):
    """
    Fills the screen with the given color.
    """

    _screen.fill(color)
    pygame.display.flip()

def screen_half(color, side):
    """
    Fill one side of the screen with the given color.

    Parameters
    ---
    color: (number, number, number)
        Color to use to paint the half screen.
    side: string
        Side of the screen to paint. Possible values are "left" and "right"
    """

    width, height = _screen.get_size()
    if side == "right":
        rect = pygame.Rect(width // 2, 0, width // 2, height)
    elif side == "left":
        rect = pygame.Rect(0, 0, width // 2, height)
    else:
        raise ValueError("side must be 'left' or 'right'")
    
    _screen.fill(DEFAULT_COLOR)  # Fill the entire screen with black before drawing the rectangle
    pygame.draw.rect(_screen, color, rect)
    pygame.display.flip()

def check_screen_color():
    global screen_color_start_time

    if screen_color_start_time is not None:
        elapsed_time = pygame.time.get_ticks() - screen_color_start_time
        if elapsed_time > 2000:
            screen_color(DEFAULT_COLOR)
            screen_color_start_time = None

