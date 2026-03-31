"""
config.py - esibot_vision global constants
"""

# -- Sign classes (GTSRB subset -> YOLOv8 label index) ---------------------
SIGN_CLASSES = {
    0: "speed_30",
    1: "speed_50",
    2: "speed_70",
    3: "speed_80",
    4: "stop",
    5: "dir_straight",
    6: "dir_right",
    7: "dir_left",
}

# GTSRB class_id -> local label index
GTSRB_TO_LOCAL = {
    1: 0,  # speed_30
    2: 1,  # speed_50
    4: 2,  # speed_70
    5: 3,  # speed_80
    14: 4,  # stop
    35: 5,  # dir_straight
    38: 6,  # dir_right
    39: 7,  # dir_left
}

NUM_SIGN_CLASSES = len(SIGN_CLASSES)

# -- BGR annotation colors -------------------------------------------------
COLOR_CYAN = (255, 255, 0)
COLOR_GRAY = (160, 160, 160)
COLOR_YELLOW = (0, 255, 255)
COLOR_GREEN = (0, 255, 0)
COLOR_ORANGE = (0, 165, 255)
COLOR_RED = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLUE = (255, 0, 0)
COLOR_PURPLE = (255, 0, 255)

# -- Obstacle proximity labels ----------------------------------------------
PROXIMITY_VERY_CLOSE = "VERY_CLOSE"
PROXIMITY_CLOSE = "CLOSE"
PROXIMITY_DETECTED = "DETECTED"

# -- Colors per sign class -------------------------------------------------
SIGN_COLORS = {
    "speed_30": COLOR_ORANGE,
    "speed_50": COLOR_ORANGE,
    "speed_70": COLOR_ORANGE,
    "speed_80": COLOR_ORANGE,
    "stop": COLOR_RED,
    "dir_straight": COLOR_BLUE,
    "dir_right": COLOR_BLUE,
    "dir_left": COLOR_BLUE,
}
