DEFAULT_CONFIG = {
    # CONFIGS
    "MAP_FILE": '/home/user/catkin_ws/src/anki_description/world/sample1.world',
    "PARTICLES": 500,
    "ROTATION_SPEED": 90,  # deg/s = 1.57rad/s
    "TRANSLATION_CHOICES": [2.5, 5, 7.5],
    "ROTATION_CHOICES": [90, -90],
    "GAUSSIAN_PARTICLES_RATE": 0.55,  # Around bests
    "KEEP_BEST_PARTICLES_RATE": 0.4,   # Keep bests (rest  is uniform random)
    "WEIGHT_CUTOFF": 0.1,
    "MANUAL": False,

    # ENV CONSTANTS
    "VECTOR_LENGTH": 0.045,
    "SENSOR_MAX_DISTANCE": 0.4,  # ~40cm

    # MATH
    "PI": 3.1415926535897,
}