import os
from modules import controls, radars, utilities

FILENAME = os.path.splitext(os.path.basename(__file__))[0]
OBSTACLE_DIRECTION_DEFAULT_COLOR = (0, 255, 0)

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
        elif self.right_radar.isActive() and self.left_radar.isActive() and delta_distance < radars.MINIMUM_DELTA_DISTANCE:
            obstacle_behind = True
        else:
            obstacle_behind = False

        if coming_from_right is not None:
            coming_from_right = coming_from_right if not mirrored else not coming_from_right
            side = "right" if coming_from_right else "left"
            controls.screen_half(color, side)
        elif obstacle_behind:
            side = "behind"
            controls.screen_color(color)
        else:
            side = ""
            controls.screen_color((0, 0, 0))

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
        
        utilities.log(f"[{sx_output} {dx_output}]", FILENAME)
