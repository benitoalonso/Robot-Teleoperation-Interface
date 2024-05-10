# Import des paquets requis
from scipy.spatial.transform import Rotation as R
import rosbag
from matplotlib import pyplot as plt
from matplotlib import image as mpimg

SIM = True
BAG_FILE = "test-traj.bag"
# rosbag record /mobile_manip/t265/odom/sample /odometry/filtered /amcl_pose /mobile_manip/ground_truth/state /scan

topics = []
if SIM:
    topics = ["/mobile_manip/ground_truth/state",
              "/mobile_manip/t265/odom/sample",
              "/amcl_pose"]
else:
    topics = ["/odometry/filtered",
              "/mobile_manip/t265/odom/sample",
              "/amcl_pose"]    

# Fonction pour le calcul de l'orientation à partir d'un quaternion
def get_heading_from_quaternion(q):
    r = R.from_quat([q.x, q.y, q.z, q.w])
    angles = r.as_euler('xyz', degrees=False)
    return angles[2]

# Boucle pour lire l'ensemble du ROSBAG et extraire seulement les positions x/y
x_t265, y_t265, theta_t265 = [], [], []
x_gt, y_gt, theta_gt  = [], [], []
heading = []
x_amcl, y_amcl, theta_amcl = [], [], []
x_uwb, y_uwb, theta_uwb  = [], [], []
i_gt, i_t265, i_amcl, i_uwb = 0, 0, 0, 0
x_axis_gt, x_axis_t265, x_axis_amcl, x_axis_uwb = [], [], [], []
len_gt, len_t265, len_amcl, len_uwb = 0, 0, 0, 0
with rosbag.Bag(BAG_FILE, 'r') as in_bag:
    if SIM:
        len_gt = in_bag.get_message_count(topics[0])
    else:
        len_uwb = in_bag.get_message_count(topics[0])
    len_t265 = in_bag.get_message_count(topics[1])
    len_amcl = in_bag.get_message_count(topics[2])

    max_len = max(len_gt, len_t265, len_amcl, len_uwb)
    for topic, msg, t in in_bag.read_messages():
        if SIM:
            if topic==topics[0]:
                x_gt.append(msg.pose.pose.position.x)
                y_gt.append(msg.pose.pose.position.y)
                theta_gt.append(get_heading_from_quaternion(msg.pose.pose.orientation))
                x_axis_gt.append(i_gt)
                i_gt += round(max_len/len_gt)
        else:
            if topic==topics[0]:
                x_uwb.append(msg.pose.pose.position.x)
                y_uwb.append(msg.pose.pose.position.y)
                theta_uwb.append(get_heading_from_quaternion(msg.pose.pose.orientation))
                x_axis_uwb.append(i_uwb)
                i_uwb += round(max_len/len_uwb)
        if topic==topics[1]:
            x_t265.append(msg.pose.pose.position.x)
            y_t265.append(msg.pose.pose.position.y)
            theta_t265.append(get_heading_from_quaternion(msg.pose.pose.orientation))
            x_axis_t265.append(i_t265)
            i_t265 += round(max_len/len_t265)
        if topic==topics[2]:
            x_amcl.append(msg.pose.pose.position.x)
            y_amcl.append(msg.pose.pose.position.y)
            theta_amcl.append(get_heading_from_quaternion(msg.pose.pose.orientation))
            x_axis_amcl.append(i_amcl)
            i_amcl += round(max_len/len_amcl)

map_res = 0.02
if SIM:
    offset_x = 3.9              # offset x=3.9 pour etre collé au mur
    offset_y = 0.0
else:
    # conversion (0,0) map<->uwb
    # offset (x,y) utilise la seconde valeur du uwb ([0]=(0,0) donc inutilisable)
    offset_x = 4.6 + x_uwb[1]
    offset_y = -4.2 + y_uwb[1]
# conversion (0,0) map<->world 
# y=0 au robot 5, offset 0.916m entre chaque robot
x_start = 23.7 + offset_x   
y_start = 22.6 + offset_y   

if SIM:
    for i in range(1, len(x_gt)):
        x_gt[i] = (x_gt[i] - x_gt[0] + x_start) / map_res
        y_gt[i] = (y_gt[i] - y_gt[0] + y_start) / map_res
    x_gt[0], y_gt[0] = x_start/map_res, y_start/map_res
else:
    for i in range(1, len(x_uwb)):
        x_uwb[i] = (x_uwb[i] - x_uwb[0] + x_start) / map_res
        y_uwb[i] = (y_uwb[i] - y_uwb[0] + y_start) / map_res
    x_uwb[0], y_uwb[0] = x_start/map_res, y_start/map_res

for i in range(1, len(x_t265)):
    x_t265[i] = (x_t265[i] - x_t265[0] + x_start) / map_res
    y_t265[i] = (y_t265[i] - y_t265[0] + y_start) / map_res
x_t265[0], y_t265[0] = x_start/map_res, y_start/map_res

for i in range(1, len(x_amcl)):
    x_amcl[i] = (x_amcl[i] - x_amcl[0] + x_start) / map_res
    y_amcl[i] = (y_amcl[i] - y_amcl[0] + y_start) / map_res
x_amcl[0], y_amcl[0] = x_start/map_res, y_start/map_res

plt.imshow(mpimg.imread("a2230_map_closed_fliped.png"))
if SIM:
    plt.plot(x_gt, y_gt, label='gt', color='r')
    plt.scatter(27.6/map_res, 22.6/map_res, label='origin', color='black', s=20)
else:
    plt.plot(x_uwb, y_uwb, label='uwb', color='r')
    plt.scatter(28.3/map_res, 18.1/map_res, label='origin', color='black', s=20)
plt.plot(x_t265, y_t265, label='t265', color='b')
plt.plot(x_amcl, y_amcl, label='amcl', color='orange')
plt.xlim(1350, 1950)
plt.ylim(850, 1600)
plt.title('(X,Y)')
plt.legend()
plt.show()