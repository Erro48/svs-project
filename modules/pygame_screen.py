import pygame

DEFAULT_COLOR = (0, 0, 0)
screen_color_start_time = None
_screen = None

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

