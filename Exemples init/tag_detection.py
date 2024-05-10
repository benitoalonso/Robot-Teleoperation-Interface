import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tkinter as tk
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS
rospy.init_node('dingo_controller', anonymous=True)
rate = rospy.Rate(50)

# Apriltag callback
tag_array = AprilTagDetectionArray()
def tag_callback(msg):
    global tag_array
    tag_array  = msg

def get_heading_from_quaternion(q):
    if(np.sqrt(q.x**2+q.y**2+q.z**2+q.w**2) == 0):
        print('Quaternion norm 0 : ', q.x, q.y, q.z, q.w)
        return 0
    else:
        r = R.from_quat([q.x, q.y, q.z, q.w])
        angles = r.as_euler('xyz', degrees=False)
        return angles[2]

tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)

root = tk.Tk()

def refresh():  
    if (tag_array.detections == []):
        x_value.set(0.00)
        y_value.set(0.00)
        z_value.set(0.00)
        orientation_value.set(0.00)
    else: # Affiche le premier tag de la liste detections
        x_value.set('{:2f}'.format(tag_array.detections[0].pose.pose.pose.position.x))
        y_value.set('{:2f}'.format(tag_array.detections[0].pose.pose.pose.position.y))
        z_value.set('{:2f}'.format(tag_array.detections[0].pose.pose.pose.position.z))
        orientation_value.set('{:2f}'.format(get_heading_from_quaternion(tag_array.detections[0].pose.pose.pose.orientation)))
    
    root.after(10, refresh)

title_label = tk.Label(root, text="Tag detection")
title_label.grid(row=0, column=0, columnspan=3)

x_label = tk.Label(root, text="x:")
x_label.grid(row=1, column=0)
y_label = tk.Label(root, text="y:")
y_label.grid(row=2, column=0)
z_label = tk.Label(root, text="z:")
z_label.grid(row=3, column=0)
orientation_label = tk.Label(root, text="Orientation:")
orientation_label.grid(row=4, column=0)

x_value = tk.DoubleVar()
y_value = tk.DoubleVar()
z_value = tk.DoubleVar()
orientation_value = tk.DoubleVar()

x_value_label = tk.Label(root, textvariable=x_value)
x_value_label.grid(row=1, column=1)
y_value_label = tk.Label(root, textvariable=y_value)
y_value_label.grid(row=2, column=1)
z_value_label = tk.Label(root, textvariable=z_value)
z_value_label.grid(row=3, column=1)
orientation_value_label = tk.Label(root, textvariable=orientation_value)
orientation_value_label.grid(row=4, column=1)

refresh()
root.mainloop()