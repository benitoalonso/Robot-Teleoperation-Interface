# Import des paquets requis
from scipy.spatial.transform import Rotation as R
import numpy as np
import rosbag
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import image as mpimg
import math

SIM = True
BAG_FILE = "test-traj.bag"
# rosbag record /mobile_manip/t265/odom/sample /odometry/filtered /amcl_pose /mobile_manip/ground_truth/state /scan

def wraptopi(angle):
    xwrap=np.remainder(angle, 2*np.pi)
    if np.abs(xwrap)>np.pi:
        xwrap -= 2*np.pi * np.sign(xwrap)
    return xwrap

# Fonction pour le calcul de l'orientation à partir d'un quaternion
def get_heading_from_quaternion(q):
    r = R.from_quat([q.x, q.y, q.z, q.w])
    angles = r.as_euler('xyz', degrees=False)
    return angles[2]

# Boucle pour lire l'ensemble du ROSBAG et extraire seulement les positions x/y
timestamp_scan, scan_ranges, scan_angle_min, scan_angle_increment = [], [], [], []
timestamp_t265, x_t265, y_t265, theta_t265 = [], [], [], []
timestamp_uwb, x_uwb, y_uwb, theta_uwb = [], [], [], []
with rosbag.Bag(BAG_FILE, 'r') as in_bag:
    for topic, msg, t in in_bag.read_messages():
        if SIM:
            if topic=='/mobile_manip/ground_truth/state':
                timestamp_uwb.append(msg.header.stamp.secs*1000000+msg.header.stamp.nsecs)
                x_uwb.append(msg.pose.pose.position.x)
                y_uwb.append(msg.pose.pose.position.y)
                theta_uwb.append(get_heading_from_quaternion(msg.pose.pose.orientation))
        else:
            if topic=='/odometry/filtered':
                timestamp_uwb.append(msg.header.stamp.secs*1000000+msg.header.stamp.nsecs)
                x_uwb.append(msg.pose.pose.position.x)
                y_uwb.append(msg.pose.pose.position.y)
                theta_uwb.append(get_heading_from_quaternion(msg.pose.pose.orientation))
        if topic=='/scan':
            timestamp_scan.append(msg.header.stamp.secs*1000000+msg.header.stamp.nsecs)
            scan_ranges.append(msg.ranges)
            scan_angle_min.append(msg.angle_min)
            scan_angle_increment.append(msg.angle_increment)

map_res = 0.02
if SIM:
    x_start = 23.7
    y_start = 22.6
else:
    x_start = 23.7 + 4.6
    y_start = 22.6 - 4.2  

fig, ax = plt.subplots()
scatter = ax.scatter([], [], color='r', s=5)
robot_pos = ax.scatter([], [], color='g')
frame_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, color='black')
arrow = ax.annotate('', xy=(0,0), arrowprops=dict(arrowstyle="->", color='b'))

def update(frame):
    i = frame

    if SIM:
        index = i * math.floor(in_bag.get_message_count('/mobile_manip/ground_truth/state')/in_bag.get_message_count('/scan'))
    else:
        index = i * math.floor(in_bag.get_message_count('/odometry/filtered')/in_bag.get_message_count('/scan'))

    cap = wraptopi(theta_uwb[index])
    x_pos = x_start + x_uwb[index]
    y_pos = y_start + y_uwb[index]

    obstacles_xy = []
    for j, r in enumerate(scan_ranges[i]):
        angle = cap - scan_angle_min[i] + j * scan_angle_increment[i]      - math.pi/2 # Ajuster cet offset
        obstacles_xy.append([(x_pos + r * np.cos(angle))/map_res, (y_pos + r * np.sin(angle))/map_res])

    scatter.set_offsets(np.asarray(obstacles_xy))
    robot_pos.set_offsets(np.asarray([[x_pos/map_res, y_pos/map_res]]))
    frame_text.set_text('Frame: {} / {}'.format(frame + 1, len(scan_ranges)))

    arr_len = 100
    arrow.xy = (x_pos/map_res + arr_len*np.cos(cap), y_pos/map_res + arr_len*np.sin(cap))
    arrow.set_position((x_pos/map_res, y_pos/map_res))

ani = FuncAnimation(fig, update, frames=len(scan_ranges), interval=1)
ax.grid(True)

plt.imshow(mpimg.imread("a2230_map_closed_fliped.png"))
plt.xlim(1250, 1950)
plt.ylim(850, 1600)
plt.show()