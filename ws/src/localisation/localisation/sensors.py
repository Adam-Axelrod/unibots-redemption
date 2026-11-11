
# ws/src/localisation/localisation/sensors.py

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


# Load the sensor geometry from the installed localisation package
def load_sensor_config():
    config_path = (
        Path(get_package_share_directory('localisation'))
        / 'config'
        / 'sensors.yaml'
    )

    with config_path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


# Return the LiDAR mounting as (x, y, z) metres and (roll, pitch, yaw) radians
def lidar_mount(config):
    lidar = config['lidar']
    translation = tuple(float(v) for v in lidar['translation_m'])
    rotation = tuple(math.radians(float(v)) for v in lidar['rotation_rpy_deg'])
    return translation, rotation
