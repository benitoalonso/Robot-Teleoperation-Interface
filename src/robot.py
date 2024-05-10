# Insérez ici le numéro de votre équipe
VOTRE_NUMERO_EQUIPE = 2

# Import des paquets requis et configuration du ROS Master
from os import environ
import rospy
from jackal_msgs.msg import Drive
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from apriltag_ros.msg import AprilTagDetectionArray
from mobile_manip.srv import ReachName, GetValues, ReachValues

import multiprocessing as mp
import time
from scipy.spatial.transform import Rotation as R
import sys

from pid import PID
from lab_utils.global_values import *
from lab_utils.orientation_utils import *

TAG_DIMENSION = 11

debugg_prints = False


class ROBOT:
    # Multiprocessing components
    _positions = mp.Array("d", 3)                       # DISTANCE EN METRES ET ANGLE EN DEGRES (inertiel)
    _velocities = mp.Array("d", 2)                      # v, w
    _target_pos = mp.Array("d", 1000)                    # Targets to be reached
    _running = mp.Value("i", 1)                         # Process running flag
    _process = None 
    _is_moving = mp.Value("i", 1)                       # Robot moving flag
    _control_mode = mp.Value("i", 1)                    # Mode flag (0 = automatic, 1 = manual, 2 = turning on itself)
    _is_ready_tag = mp.Value("i", 1)                    # Ready to reach tag flag
    
    _tag_detected = mp.Array("d", TAG_DIMENSION * 5)    # Detected tags coordinates
    _ismsgtagnotnull = mp.Value("i", 1)                        # Tag message #0 if null, 1 if not null
    
    _detection_history = mp.Array('i', 5)  # Holds the last 5 detection statuses (1 or 0)
    _history_index = mp.Value('i', 0)  # Index to track the current position in the circular buffer
    _detection_id_history = mp.Array('i', 5)  # Holds the last 5 detection IDs

    sizeofblocktag = 12
    _tag_detectedd = mp.Array("d", sizeofblocktag * 5)    # Detected tags coordinates
    
    _tag_corrected = mp.Array("d", 6 * 5)               # Corrected tags coordinates
    _nbTag = mp.Value("i", 1)                           # Number of detected tags
    _nbTarget = mp.Value("i", 1)                        # Number of targets to reach
    _sim = mp.Value("i", 1)                             # Simulation flag
    
    cam_msg = Image()
    map_conversion = [1,1]
    # laser_msg = LaserScan()
        
    def __init__(self, sim, robotID=0):
        if sim == 1:
            self._sim.value = 1
            pass
        else:
            self._sim.value = 0
            environ['ROS_MASTER_URI'] = "http://cpr-ets05-0{}.local:11311/".format(robotID)
            environ['ROS_IP'] = "192.168.0.81" # adresse IP de votre station de travail
            print("ROS_MASTER_URI = ", environ['ROS_MASTER_URI'])
        import rospy

        # Créer et démarrer un nouveau noeud
        rospy.init_node('dingo_controller', anonymous=True)
        self.rate = rospy.Rate(50)
        
        # ROS subscribers et publishers
        if self._sim.value == 1: #CAS SIMULATION
            self.cmd_drive_pub = rospy.Publisher('/mobile_manip/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
            self.cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, self.cam_callback)
            self.ground_truth_sub = rospy.Subscriber('/mobile_manip/ground_truth/state', Odometry, self.ground_truth_callback)
            self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
            self.map_conversion = [MAP_TO_SIM[0], MAP_TO_SIM[1]]
            
        elif self._sim.value == 0: #CAS REEL
            self.cmd_drive_pub = rospy.Publisher('/mobile_manip/base/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
            self.cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, self.cam_callback)
            self.position_odomuwb_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.position_odomuwb_callback)
            self.tag_sub = rospy.Subscriber('/mobile_manip/tag_detections', AprilTagDetectionArray, self.tag_callback)
            self.map_conversion = (SIM_TO_REAL[0] + MAP_TO_SIM[0],
                                   SIM_TO_REAL[1] + MAP_TO_SIM[1]
                                  )  

        else:
            raise ValueError("Robot is not selected correctly.")
                                      
        self.pid_controller = PID()
        self._velocities[:] = [0.0, 0.0]
        self._running.value = 0
        self._is_moving.value = 0
        self._nbTag.value = 0
        self._nbTarget.value = 0
        self._is_ready_tag.value = 0


        self._ismsgtagnotnull.value = 0
        self._detection_history[:] = [0, 0, 0, 0, 0]
        self._detection_id_history[:] = [0, 0, 0, 0, 0]
        self._history_index.value = 0


    # Apriltag callback
    def tag_callback(self, msg):
        self._ismsgtagnotnull.value = 0

        #if len(msg.detections) > 0:
        #    self._ismsgtagnotnull.value = 1

        detection_present = 1 if len(msg.detections) > 0 else 0
        # Update the circular buffer
        with self._detection_history.get_lock():  # Ensure thread-safe updates
            self._detection_history[self._history_index.value] = detection_present

        for det in msg.detections:
            self._ismsgtagnotnull.value = 1

            tag_id = det.id[0] 
            #print("tag_id", tag_id)
            found = False

            with self._detection_id_history.get_lock():
                self._detection_id_history[self._history_index.value] = tag_id

            # Assuming 10 slots now: 1 for tag ID, and 9 for data (x, y, z, orx, ory, orz, orw, x_corr, y_corr)
            for i in range(0, len(self._tag_detectedd), self.sizeofblocktag):
                # Check if this slot in the array is for the current tag ID
                if self._tag_detectedd[i] == tag_id:
                    # Update existing tag data
                    self.update_tag_data(i + 1, det)  # Pass the starting index for data
                    found = True
                    break
            
            if not found:
                # Find the next available slot based on self._nbTag.value
                next_index = int(self._nbTag.value) * self.sizeofblocktag
                if next_index < len(self._tag_detectedd) - self.sizeofblocktag:  # Ensure there's enough space
                    self._tag_detectedd[next_index] = tag_id
                    self.update_tag_data(next_index + 1, det)
                    self._nbTag.value += 1  # Increment count of detected tags

        if detection_present == 0:
            self._detection_id_history[self._history_index.value] = -1

        #print('tag id history', self._detection_id_history[:])
        self._history_index.value = (self._history_index.value + 1) % 5  # Move to the next index, wrapping around if necessary
        #print('is msg tag not null ', self._ismsgtagnotnull.value)

    def update_tag_data(self, start_index, det):
        # Extract and calculate the tag data similar to the previous examples
        x_value, y_value, z_value = det.pose.pose.pose.position.x, det.pose.pose.pose.position.y, det.pose.pose.pose.position.z
        quaternion_value = det.pose.pose.pose.orientation
        orrx = quaternion_value.x
        orry = quaternion_value.y
        orrz = quaternion_value.z
        orrw = quaternion_value.w

        tagx_inertial, tagy_inertial, tagz_inertial , tagx_robot, tagy_robot, tagz_robot = self.tag_cam2inertialandrobot(x_value, y_value, z_value, orrx, orry, orrz, orrw)
        
        tagx_inertial = round(tagx_inertial, 2)
        tagy_inertial = round(tagy_inertial, 2)
        tagz_inertial = round(tagz_inertial, 2)

        tagx_robot = round(tagx_robot, 2)
        tagy_robot = round(tagy_robot, 2)
        tagz_robot = round(tagz_robot, 2)

        ##---------------------------
        # Define the threshold for correction
        base_correction = 0.50  # Example value, adjust based on your needs

        # Dynamic threshold adjustment
        additional_per_unit = 0.1

        # Adjust the threshold based on the distance
        correction_x = max(base_correction, base_correction + additional_per_unit * max(0, tagx_robot - 1.5))
        correction_y = max(base_correction, base_correction + additional_per_unit * max(0, tagy_robot - 1.5))

        # Adjust tagx_inertial based on the robot's current x position
        tagx_inertial_corr = tagx_inertial
        tagy_inertial_corr = tagy_inertial
        
        if self.positions[0] > tagx_inertial:
            tagx_inertial_corr += correction_x
        elif self.positions[0] < tagx_inertial:
            tagx_inertial_corr -= correction_x

        if self.positions[1] > tagy_inertial:
            tagy_inertial_corr += correction_y
        elif self.positions[1] < tagy_inertial:
            tagy_inertial_corr -= correction_y
        ##---------------------------


        # Update the array with this tag's data, starting from start_index
        self._tag_detectedd[start_index] = tagx_inertial
        self._tag_detectedd[start_index + 1] = tagy_inertial
        self._tag_detectedd[start_index + 2] = tagz_inertial
        self._tag_detectedd[start_index + 3] = 0.0
        self._tag_detectedd[start_index + 4] = 0.0
        self._tag_detectedd[start_index + 5] = 0.0
        self._tag_detectedd[start_index + 6] = 0.0
        self._tag_detectedd[start_index + 7] = tagx_inertial_corr
        self._tag_detectedd[start_index + 8] = tagy_inertial_corr
        self._tag_detectedd[start_index + 9] = tagx_robot
        self._tag_detectedd[start_index + 10] = tagy_robot


    def tag_cam2inertialandrobot(self, x, y, z, orrx, orry, orrz, orrw):
        position_tagintag = np.array([0, 0, 0, 1]) 
        
                ## -- tag2cam -- ##
        euler_mat = get_euleurmatrix_from_quaternion(orrx, orry, orrz, orrw)
        position_tagincam = np.array([x, y, z]) 

        T_tag2cam = np.eye(4)  # Start with an identity matrix
        T_tag2cam[:3, :3] = euler_mat  # Insert the rotation matrix
        T_tag2cam[:3, 3] = position_tagincam  # Insert the translation vector
        
                ## -- cam2robot -- ##
        if self.sim:
            position_caminrobot = np.array([0.16 , 0.04 , 0.463])
        else:
            position_caminrobot = np.array([0.0 , 0.05 , -0.137])
        
        T_cam2robot = np.eye(4)  
        rotation_cam2robot = np.array([[0.0, 0.0, 1.0],
                                      [-1.0, 0.0, 0.0],
                                      [0.0, -1.0, 0.0]])
        T_cam2robot[:3, 3] = position_caminrobot  # Insert the translation vector
        T_cam2robot[:3, :3] = rotation_cam2robot
        
                ## -- robot2intertial -- ##
        x_I, y_I, theta_degrees = self.positions
        #print("robot theta_degress", theta_degrees)  # For debugging, print
        
        theta_radians = np.radians(theta_degrees)  # Convert degrees to radians for trigonometric functions

        # Create the rotation matrix part for rotation about the z-axis
        R_z = np.array([[np.cos(theta_radians), -np.sin(theta_radians), 0],
                        [np.sin(theta_radians), np.cos(theta_radians), 0],
                        [0, 0, 1]])
        
        # Constructing the 4x4 transformation matrix T_robot2inertial
        T_robot2inertial = np.eye(4)  # Start with an identity matrix
        T_robot2inertial[:3, :3] = R_z  # Insert the rotation matrix
        T_robot2inertial[:3, 3] = [x_I, y_I, 0.015053]  # Insert the translation vector #TODO: Check if z should be 0.0 or the robot's z position !!!!
        
        
        # Apply the transformations
        position_in_cam = np.dot(T_tag2cam, position_tagintag)
        position_in_robot = np.dot(T_cam2robot, position_in_cam)
        position_in_inertial = np.dot(T_robot2inertial, position_in_robot)
        
        return position_in_inertial[0], position_in_inertial[1], position_in_inertial[2], position_in_robot[0], position_in_robot[1], position_in_robot[2] 
        # return the x, y, z coordinates in the inertial frame ANDD the x, y, z coordinates in the robot frame



    # Camera subscriber callback
    def cam_callback(self, msg):
        self.cam_msg = msg

    # Odometry subscriber callback
    def position_odomuwb_callback(self, msg):
        self._positions[:] = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              get_heading_from_quaternion(msg.pose.pose.orientation)]
    
    # Ground truth subscriber callback
    def ground_truth_callback(self, msg):
        self._positions[:] = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              get_heading_from_quaternion(msg.pose.pose.orientation)]

    # Start the robot process
    def start_robot(self):
        timer_start = time.perf_counter()
        max_time = 5
        self._running.value = 1
        while time.perf_counter() - timer_start < max_time and self._process is None:
            self._process = mp.Process(
                target = self.loop_process,
                args = (self._positions, 
                        self._velocities, 
                        self._target_pos, 
                        self._nbTarget, 
                        self._running, 
                        self._is_moving, 
                        self._control_mode,
                        self._is_ready_tag
                        )
            )
            self._process.start()
    
    # Update robot velocities to reach target coordinates if not in manual mode
    def update_velocities(self, pt):            # pt is the target coordinates [x, y] in the INERTIAL frame
        is_arrived = False

        while self._control_mode.value == 0 and self._is_moving.value == 1:
            angle_error = get_angle(self.positions[0], self.positions[1], pt[0], pt[1]) - self.positions[2]
            

            while abs(angle_error) > ANGLE_THRESHOLD and self._is_moving.value == 1:
                self._velocities[1] = self.pid_controller.get_w_input_pid(self.positions[:], pt)
                angle_error = get_angle(self.positions[0], self.positions[1], pt[0], pt[1]) - self.positions[2]   
                #print("angle_error : ", angle_error)        

            self._velocities[:] = [0.0, 0.0]
            
            prev_vel = self.pid_controller.get_v_input_pid(self.positions[:], pt)

            while not is_arrived and self._is_moving.value == 1:
                current_vel = self.pid_controller.get_v_input_pid(self.positions[:], pt)
                self._velocities[0] = current_vel
                distance_error = get_distance(self.positions[0], self.positions[1], pt[0], pt[1])
                #print("distance_error : ", distance_error)

                if distance_error < DISTANCE_THRESHOLD:
                    self._velocities[:] = [0.0, 0.0]
                    is_arrived = True
                    break
                elif current_vel - prev_vel < SPEED_TRESHOLD:   
                    self._velocities[:] = [0.0, 0.0]
                    is_arrived = True
                    break

                prev_vel = current_vel
                    
            self._velocities[:] = [0.0, 0.0]
            break
    
    # Reach the tag coordinates
    def reach_tag(self, effector_goal):
        # xr yr zr rotx roty rotz wall
        effector_goal = [float(coord) for coord in effector_goal]

        # Proceed to call the service if coordinates are valid
        rospy.wait_for_service('/mobile_manip/reach_cartesian')
        try:
            reach_pose = rospy.ServiceProxy('/mobile_manip/reach_cartesian', ReachValues)
            response = reach_pose(effector_goal)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    # Send the velocities commands to the robot
    def control_robot(self):
        vel_left  = (self._velocities[0] - self._velocities[1] * INTERWHEEL_DISTANCE / 2.0) / LEFT_WHEEL_RADIUS
        vel_right = (self._velocities[0] + self._velocities[1] * INTERWHEEL_DISTANCE / 2.0) / RIGHT_WHEEL_RADIUS

        cmd_drive_msg = Drive()
        cmd_drive_msg.drivers[0] = vel_left
        cmd_drive_msg.drivers[1] = vel_right
        self.cmd_drive_pub.publish(cmd_drive_msg)
    
    # Turn the robot on itself to a desired angle (degrees)
    def turn_on_itself(self, target):
        self._is_moving.value = 1
        initial = self.positions[2]
        
        if (abs(target - initial) < 0.1):
            target = 360
            i = 1
        elif abs(initial) <= abs(target):
            i = 1
            target = target - initial
        else:
            i = -1
            target = initial - target

        
        degrees_turned = 0
        previous = initial
        
        if debugg_prints :
            print("turn_on_itself target : ", target)
        
        while self._is_moving.value == 1:
            current = self.positions[2]
            difference = current - previous
            
            if debugg_prints :
                print("difference : ", difference)
            
            if difference > 180:
                difference -= 360
            elif difference < -180:
                difference += 360

            degrees_turned += difference
            
            if debugg_prints :
                print("degrees_turned : ", degrees_turned)

            if abs(degrees_turned) < target:
                self._velocities[:] = [0.0, i * MAX_ANGULAR_VELOCITY]
            else:
                self._velocities[:] = [0.0, 0.0]
                break
            previous = current
            
            
    # Move the robot arm to the vertical position
    def arm_vertical(self):
        state = 'vertical'

        rospy.wait_for_service('/mobile_manip/reach_name')
        reach_vertical_pose = rospy.ServiceProxy('/mobile_manip/reach_name', ReachName)
        
        try:
            reach_vertical_pose(state)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    
    def arm_retracted(self):
        state = 'retract'

        rospy.wait_for_service('/mobile_manip/reach_name')
        reach_vertical_pose = rospy.ServiceProxy('/mobile_manip/reach_name', ReachName)
        
        try:
            reach_vertical_pose(state)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    
    
    
    # Stop the movement of the robot 
    def stop_robot(self):
        if self._process is not None:
            self._is_moving.value = 0
            self._velocities[:] = [0.0, 0.0] 
            self._nbTarget.value = 0
            self.control_robot()

    # Main loop of the robot process
    def loop_process(
            self,
            position,
            velocity,
            target,
            nbTarget,
            running,
            moving,
            control_mode,       # 0 = automatic, 1 = manual, 2 = turning on itself
            ready_tag
            ):
        
        while running.value == 1:
            if control_mode.value == 0:
                i = 0
                self._is_moving.value = 1
                while i < 2*nbTarget.value and moving.value == 1:  # 2*nbTarget.value because with i=i+2 we still need to reach the end of the array
                    print("target : ", target[i], target[i+1])
                    self.update_velocities([target[i], target[i+1]])
                    i += 2
                    print(" target n: ", int(i/2), " ok, pos robot", round(position[0], 2), round(position[1], 2))
                
                self._nbTarget.value = 0
                self._is_moving.value = 0
                    
            elif control_mode.value == 1:
                self._velocities[:] = velocity
                
            else:
                #TODO 3avril a corriger ici pour le turn on itself ???
                self.turn_on_itself(nbTarget.value)
                self._nbTarget.value = 0
                self._is_moving.value = 0
                if ready_tag.value == 1:
                    self._is_ready_tag.value = 2
                
    # Send all the targets to the robot    
    def send_path(self, path):
        """
        i = 0
        for pt in path:
            self._target_pos[i] = pt.x * MAP_RES - self.map_conversion[0]
            self._target_pos[i+1] = pt.y * MAP_RES - self.map_conversion[1]
            i += 2
            self._nbTarget.value += 1
        print("path sent")
        """

        #modifie car le path est directement en coordonnées inertielles cf. fonction update_pathh
        i = 0
        print("len path in robot send_path", len(path))
        for pt in path:
            # Assuming pt is a tuple of (x, y)
            self._target_pos[i] = pt[0]  # Set x
            self._target_pos[i+1] = pt[1]  # Set y
            i += 2  # Move to the next position for the next point
            self._nbTarget.value += 1  # Increment the number of targets by 1 for each point
        print("number of targets : ", self._nbTarget.value)
        print("path sent")

           
    @property
    def positions(self):
        return self._positions
    
    @property
    def is_moving(self):
        return self._is_moving
    
    @property
    def velocities(self):
        return self._velocities
    
    @property
    def target_pos(self):
        return self._target_pos
    
    @property
    def is_manual(self):
        return self._control_mode
    
    @property
    def sim(self):
        return self._sim
    
    @property
    def nbTag(self):
        return self._nbTag
    
    @property
    def nbTarget(self):
        return self._nbTarget
    
    @property
    def running(self):
        return self._running
    
    @property
    def tag_detected(self):
        return self._tag_detected
    
    @property
    def tag_corrected(self):
        return self._tag_corrected
    
    @property
    def is_ready_tag(self):
        return self._is_ready_tag
    
    
    