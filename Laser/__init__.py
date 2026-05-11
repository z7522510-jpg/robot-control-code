from .laser import Laser, connect_laser
from .laser_api import LaserApi

LaserController = Laser

__all__ = ["Laser", "LaserApi", "LaserController", "connect_laser"]
