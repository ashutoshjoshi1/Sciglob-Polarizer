import math
import datetime
import numpy as np
from astral import LocationInfo
from astral.sun import azimuth, elevation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def compute_sun_vector(lat, lon):
    """
    Returns a unit vector pointing toward the sun based on current UTC time.
    """
    now = datetime.datetime.now(datetime.timezone.utc)
    city = LocationInfo(latitude=lat, longitude=lon)
    az = azimuth(city.observer, now)
    el = elevation(city.observer, now)
    az_r = math.radians(az)
    el_r = math.radians(el)
    x = math.cos(el_r) * math.sin(az_r)
    y = math.cos(el_r) * math.cos(az_r)
    z = math.sin(el_r)
    return x, y, z

def draw_device_orientation(ax, roll, pitch, yaw, lat, lon):
    """
    Draws a cube at the origin rotated by (roll,pitch,yaw) and plots sun vector.
    """
    ax.cla()
    ax.set_title("3D Orientation with Sun")
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_xlim([-3, 3]); ax.set_ylim([-3, 3]); ax.set_zlim([-3, 3])
    s = 0.2
    cube = np.array([[-s, -s, -s], [ s, -s, -s], [ s,  s, -s], [-s,  s, -s],
                     [-s, -s,  s], [ s, -s,  s], [ s,  s,  s], [-s,  s,  s]])
    rx = np.array([[1, 0, 0],
                   [0, math.cos(math.radians(roll)), -math.sin(math.radians(roll))],
                   [0, math.sin(math.radians(roll)),  math.cos(math.radians(roll))]])
    ry = np.array([[ math.cos(math.radians(pitch)), 0, math.sin(math.radians(pitch))],
                   [ 0, 1, 0],
                   [-math.sin(math.radians(pitch)), 0, math.cos(math.radians(pitch))]])
    rz = np.array([[math.cos(math.radians(yaw)), -math.sin(math.radians(yaw)), 0],
                   [math.sin(math.radians(yaw)),  math.cos(math.radians(yaw)), 0],
                   [0, 0, 1]])
    rc = (rz @ (ry @ (rx @ cube.T))).T
    edges = [
        [rc[0], rc[1], rc[2], rc[3]],
        [rc[4], rc[5], rc[6], rc[7]],
        [rc[0], rc[1], rc[5], rc[4]],
        [rc[2], rc[3], rc[7], rc[6]],
        [rc[1], rc[2], rc[6], rc[5]],
        [rc[4], rc[7], rc[3], rc[0]],
    ]
    for e in edges:
        ax.add_collection3d(Poly3DCollection([e], facecolors='#8f8f8f', linewidths=1, edgecolors='k', alpha=0.2))
    sx, sy, sz = compute_sun_vector(lat, lon)
    ax.scatter([sx], [sy], [sz], marker='o', s=50)