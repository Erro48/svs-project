import math, pygame # type: ignore


TTC_THRESHOLD = 0.4 # second

def log(message, source="vehicle"):
    """
    Prints on console the given message, preceded by the source.
    The print format is: [source] message
    """
    
    print(f"[{source}] {message}")

def hirst_graham_distance_algorithm(v_rel, vF):
    return TTC_THRESHOLD * v_rel + 0.4905 * vF

def compute_velocity_from_vector(velocity_vector):
    v_x = velocity_vector.x
    v_y = velocity_vector.y
    v_z = velocity_vector.z
    return math.sqrt(v_x**2 + v_y**2 + v_z**2)

def load_alarm_sound():
    pygame.mixer.init()
    alarm_sound = None
    try:
        alarm_sound = pygame.mixer.Sound("alarm_sound.mp3")
    except Exception as e:
        # print(f"Errore durante il caricamento del file audio: {e}")
        log(f"Errore durante il caricamento del file audio: {e}", "sound")
    return alarm_sound