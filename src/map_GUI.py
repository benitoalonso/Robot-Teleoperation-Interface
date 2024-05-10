# roslaunch mobile_manip gen3_lite_dingo_labsim.launch

# ssh mecbot@cpr-ets05-04.local
# A1330dingo
# roslaunch mobile_manip gen3_lite_dingo_amcl_uwb.launch
# ./launch-a2230.sh gen3_lite_dingo_amcl_uwb.launch 

# Interface
from map_image import MAP_IMG
from PIL import Image
from cv_bridge import CvBridge
from robot import ROBOT


import matplotlib.pyplot as plt

import multiprocessing as mp

import PySimpleGUI as sg
from datetime import datetime as dt
import os
import time
import base64

from lab_utils.rrt import RRTPlanner
from lab_utils.astart import AStarPlanner
from lab_utils.plan_utils import *
from lab_utils.image_utils import *
from lab_utils.global_values import *


from PIL import Image, ImageDraw

mode_list = ["Simulation", 
             "Real Robot"]

debugg_prints = False

class UI:
    # Class objects 
    window = sg.Window
    control_started = False
    sim = None
    start_pressed = False           
    manual_mode = False
    scanning_mode = False
    map_data = None
    mapID = None
    path_pos = [0.0, 0.0, 0.0]
    robot = None
    bridge = CvBridge()
    tag_dict = dict()
    tag_id = list()
    tag_to_get = None
    start_time = dt.now()
    
    original_map_img = None
    original_width_img = 0
    original_height_img = 0
    
    withelements_map_img = None
    
    current_selection_tag = None

    #TO HANDLE THE SUPPR AND DELETING IN THE GRAPH // INSTEAD OF ERASING ALL
    robotpoint_id = None
    robotcap_id = None
    endpoint_id = None
    pathpoints_id = []
    tagpoints_id = []
    tagids_id = []


    # Fonts
    font_family = "SegoeUI"
    title_font = "%sSemiBold 20"%font_family
    text_font = "%sSemiLight 16"%font_family
    input_font = "%s 12"%font_family
    error_font = "%s 12 bold"%font_family

    # UI components
    comp = ["mode_selection",
            "robot_selection",
            "start_btn",
            "map_interact",
            "position",
            "test_timer",
            "quit_btn",
            #"scan_btn",#
            "robot_left_btn",
            "robot_right_btn",
            "robot_forward_btn",
            "robot_backward_btn",
            "control_mode_btn",
            "camera",
            "go_tag_btn",
            "go_arm_btn",
            "tag_selection",
            "tag_detected"
            ]

    #open_disable = ["start_btn", "scan_btn", "robot_left_btn", "robot_right_btn", "robot_forward_btn", "robot_backward_btn", "control_mode_btn", "go_tag_btn","go_arm_btn"]
    open_disable = ["start_btn", "robot_left_btn", "robot_right_btn", "robot_forward_btn", "robot_backward_btn", "control_mode_btn", "go_tag_btn","go_arm_btn"]
    open_enable = ["mode_selection", "robot_selection", "quit_btn"]   
    
    startbnt_disable = ["start_btn"]

    robot_enable = ["start_btn"]
    manual_enable = ["robot_left_btn", "robot_right_btn", "robot_forward_btn", "robot_backward_btn"]
    
    #start_enable = ["scan_btn", "control_mode_btn"]
    start_enable = [ "control_mode_btn"]
    start_disable = ["mode_selection", "robot_selection"]
    
    tag_enable = ["go_tag_btn"]
    arm_enable = ["go_arm_btn"]
    #scan_enable = ["scan_btn"]
    
    # UI Setup
    def __init__(self):
        
        # Create the window
        self.tag_id.append("No tag detected")
        map_name = "a2230_map_closed_fliped.png"
        folder = "Images"
        self.image_folder = os.path.join(os.getcwd(), folder)
        file_path = os.path.join(self.image_folder, map_name)
        self.map_data = MAP_IMG(file_path) 

        file_path = os.path.join(self.image_folder, "blackimg.png")
        img_camera = Image.open(file_path)
        self.cam_data = img_to_bytes(resize_img(img_camera, scale=1.25))

        file_path_mapp = os.path.join(self.image_folder, "a2230_map_closed_fliped.png")
        self.original_map_img = Image.open(file_path_mapp)
        self.original_width_img = self.original_map_img.size[0]
        self.original_height_img = self.original_map_img.size[1]
        print("original map size: ", self.original_width_img, "x", self.original_height_img)
        
        ### NEW FOR MAP PNG WITH ALL THE ELEMENTS
        self.withelements_map_img = Image.open(file_path_mapp)
        self.endpoint = []  # Initialize an empty list to track the endpoint
        self.pathpoints = []  # Initialize an empty list to track the path points
        self.pathpoints_inertial = []  # Initialize an empty list to track the path points in inertial
        
        
        self.robotpoint = []  # Initialize an empty list to track the robot point
        self.previous_robotpoint = []  # Tracks the previous robot point

        self.tagpoints = {}  # Use a dictionary to track current tag points by their IDs
        self.previous_tagpoints = {}  # Tracks previous positions of tags
        
        self.window = sg.Window("Robot Navigation", self.create_layout(), finalize=True)
        
        self.mapID = self.window['map_interact'].draw_image(data=self.map_data.map_avec_robot, location=(0, 0))

        
        self.disable(self.open_disable)
        self.enable(self.open_enable)
        
        # Main loop to check the different components
        while True:
            event, values = self.window.read(timeout=100)

            if self.robot is not None:    
                self.update_data()
                self.robot.control_robot()
        
            if event == sg.WIN_CLOSED or event == 'quit_btn':    
                self.close()
                break         
            
            # APPLICATION LANCEE UNE FOIS BOUTON START APPUYE
            if not self.control_started:
                
                if values["mode_selection"] == "Simulation":
                    self.sim = True
                elif values["mode_selection"] == "Real Robot":
                    self.sim = False
                    print("Real Robot selecteddd")

                if (not self.sim and values["robot_selection"]) or self.sim:
                    self.enable(self.robot_enable)
            
            #si on a appuye sur start
            if self.control_started:
                
                """
                if event == "scan_btn":    
                    self.scanning_mode = True
                    self.robot._control_mode.value = 2
                    self.robot._nbTarget.value = 0 #TO ASKK ben
                """
                
                if event == "control_mode_btn":
                    self.manual_mode = not self.manual_mode
                    self.robot._is_moving.value = 0
                    self.robot._velocities[:] = [0.0, 0.0]
                    self.robot.control_robot()

                    if self.manual_mode:
                        self.enable(self.manual_enable)
                        self.robot._control_mode.value = 1

                    else:
                        self.disable(self.manual_enable)
                        self.robot._control_mode.value = 0
                    
                    self.window.Element("control_mode_btn").Update(('Manual Mode', 'Auto Mode')[self.manual_mode])
                
                if self.scanning_mode:
                    continue

                elif self.manual_mode:
                    self.window['control_mode_status'].update(f"Map not activated", text_color='black')

                    self.robot._control_mode.value = 1
                    self.robot._nbTarget.value = 0
                    self.robot._velocities[:] = [0.0, 0.0]

                    if event == "robot_forward_btn":
                        self.robot._velocities[0] = MAX_LINEAR_VELOCITY*0.8
                        
                    elif event == "robot_backward_btn":
                        self.robot._velocities[0] = -MAX_LINEAR_VELOCITY*0.8
                        
                    elif event == "robot_right_btn":
                        self.robot._velocities[1] = -MAX_ANGULAR_VELOCITY
                        
                    elif event == "robot_left_btn":
                        self.robot._velocities[1] = MAX_ANGULAR_VELOCITY

                    
                    if event == 'map_interact':
                        self.window['control_mode_status'].update(f"Map not activated in manual mode", text_color='black')

                else:
                    self.window['control_mode_status'].update(f"Map activated", text_color='black')

                    if debugg_prints:
                        print("else mode auto pour map interract")
    
                    self.robot._control_mode.value = 0
                    if event == 'map_interact' and self.sim == True: #**********************************************

                        #if self.sim == False:
                        #    self.window['control_mode_status'].update(f"Map not activated in real world")   #**********************************************************************************
                        #    print('breaking as map interract in real world is not allowed')
                            
                        
                        if debugg_prints:
                            print("map interractedd, robot is moving: ", self.robot.is_moving.value, "ok if 0")
                        
                        #if self.robot.is_moving.value == 0: #TO ASKK ben desactive pour debug 17h 2avril benito
                        #if 1 == 1 :
                        # Detect the point clicked on the map and transform it according to scaling
                        x, y = values[event]
                        target = [ int(int(x)/ MAP_SCALE), int(int(y) / MAP_SCALE)] #semble ok pour la position en pixel sur la MAP
                        
                        if debugg_prints:
                            print("map interract x: ", x, " y: ", y)
                            print("map interract target: ", target)
                        
                        #self.send_path(target)
                        self.onclick(target)
                


                if event == 'tag_selection': 
                
                    # Extract the numeric part from the selected tag (assuming format "Tag X")
                    tag_number = values['tag_selection'].split()[-1]  # Splits the string and takes the last part
                    self.current_selection_tag = tag_number
                    print("Tag selectionne:", self.current_selection_tag)

                    self.window['tag_selected'].update(f"Tag Selected : {self.current_selection_tag} | ")

                if self.current_selection_tag != None and self.robot.is_moving.value == 0 and not(self.manual_mode) :
                    #print("enabling go tag btn")
                    self.enable(self.tag_enable)
                
                if self.manual_mode: 
                    self.disable(self.tag_enable)
                
                #DESACIVER LE GO TAG ET ET GO ARM EN REEL CAR LE BRAS NE FONCTIONNE PAS **********************************************************************************
                if self.sim == False:
                    self.disable(self.arm_enable)
                    self.disable(self.tag_enable)

                if event == "go_tag_btn":
                    print("Go to tag pressed, armvertical, isradytag =", self.robot.is_ready_tag.value)
                    #self.robot.arm_vertical()
                    
                    #if self.sim:
                    #    self.robot.arm_retracted()
                    
                    if self.manual_mode :
                        self.window['control_mode_status'].update(f"Go in auto mode please", text_color='red')

                    else:
                        self.robot._is_ready_tag.value = 1
                        print("now isreadytag =", self.robot.is_ready_tag.value)
                        
                        tag_array = self.robot._tag_detectedd
                        number_of_tags = self.robot._nbTag.value

                        for i in range(number_of_tags):
                            start_index = i * self.robot.sizeofblocktag
                            tag_id = int(tag_array[start_index])
                            x_corr = tag_array[start_index + 8]
                            y_corr = tag_array[start_index + 9]
                            
                            if tag_id == int(self.current_selection_tag):
                                print(f"x_tagtogo-inertial: {x_corr}, y_tagtogo-inertial: {y_corr}")
                                break
                        
                        x_tagtogo_map, y_tagtogo_map = self.convert_inertial_to_map(x_corr, y_corr)

                        target = [int(x_tagtogo_map), int(y_tagtogo_map)]    
                        print("tagtogo-map: ", target)

                        #self.send_path(target)
                        self.onclick(target)

                
                if self.current_selection_tag != None and self.robot.is_moving.value == 0:
                    
                    tag_array = self.robot._tag_detectedd
                    number_of_tags = self.robot._nbTag.value

                    x_tag = None
                    y_tag = None

                    for i in range(number_of_tags):
                        start_index = i * self.robot.sizeofblocktag
                        tag_id = int(tag_array[start_index])
                        x_rob = tag_array[start_index + 10]
                        y_rob = tag_array[start_index + 11]
                        
                        if tag_id == int(self.current_selection_tag):
                            #print(f"x_tag2robb: {x_rob}, y_tag2robb: {y_rob}")
                            
                            x_tag = x_rob
                            y_tag = y_rob

                        with self.robot._detection_id_history.get_lock():  # Ensure thread-safe read
                            tagidseen = any(tag_id == int(self.current_selection_tag) for tag_id in self.robot._detection_id_history)

                        if tagidseen == False:
                                x_tag = None
                                y_tag = None
                            

                    if x_tag is not None and y_tag is not None:
                        
                        x_color = 'green' if 0.4 < x_rob < 0.8 else 'red'
                        y_color = 'green' if -0.4 < y_rob < 0.4 else 'red'
                    

                        # Updating the position status with appropriate colors
                        self.window['x_rob_prefix'].update(f"x_rob 0.4 <", text_color='black')
                        self.window['x_rob_value'].update(f"{x_rob}", text_color=x_color)
                        self.window['x_rob_suffix'].update(f"< 0.8", text_color='black')
                        
                        self.window['y_rob_prefix'].update(f"| y_rob  -0.4 <", text_color='black')
                        self.window['y_rob_value'].update(f"{y_rob}", text_color=y_color)
                        self.window['y_rob_suffix'].update(f"< 0.4", text_color='black')
                        
                        if 0.4 < x_tag < 0.8 and abs(y_tag) < 0.4:
                            self.window['go_arm_status'].update(f"Tag {self.current_selection_tag} in reach", text_color='black')
                            self.enable(self.arm_enable)
                                

                            if event == "go_arm_btn":
                                x_arm = max(x_tag - 0.3, 0.1)
                                if self.sim:
                                    self.robot.arm_vertical()
                                    self.robot.reach_tag([x_arm, y_tag, 0.6, -90, 0.0, -90.0, 0.0])
                    
                        else:
                            self.disable(self.arm_enable)
                            
                            if x_tag <= 0.4:
                                self.window['go_arm_status'].update(f"Too close to the tag")
                                #print('Too close to the tag, back up')
                            elif x_tag > 0.8:
                                self.window['go_arm_status'].update(f"Move closer to the tag")
                                #print('Move closer to the tag')
                            
                            if y_tag > 0.3:
                                self.window['go_arm_status'].update(f"Move to the left")
                                #print('Move to the left position')
                            elif y_tag < -0.3:
                                self.window['go_arm_status'].update(f"Move to the right")
                                #print('Move to the right position')
                        
                    else:
                        self.window['x_rob_value'].update(f"0.00", text_color='red')
                        self.window['y_rob_value'].update(f"0.00", text_color='red')
                        self.window['go_arm_status'].update(f"Tag selected not seen")
                            
                    self.robot._is_ready_tag.value = 0
                
            if event == "start_btn":
                if not self.control_started:
                    # Start robot and map process
                    self.robot = ROBOT(self.sim, values["robot_selection"])
                    self.robot.start_robot()
                    
                    print("sim?:",self.sim)
                    
                    self.robot._velocities[:] = [0.0, 0.0]
                    self.robot.control_robot()

                    #if self.sim:
                    #    print("bras retracte au depart")
                    #    self.robot.arm_retracted()
                    
                    # Disable other control - enable requiered buttons 
                    self.enable(self.start_enable)
                    self.disable(self.start_disable)
                    
                    # get the clock working
                    self.start_time = dt.now() 
                    
                    self.control_started = True                    
                    
                if self.start_pressed:
                    self.stop()
                else:
                    self.robot._is_moving.value = 0
                    self.robot._nbTarget.value = 0
                
                    if self.manual_mode:
                        self.enable(self.manual_mode)


                self.disable(self.startbnt_disable)
                #self.window.Element("start_btn").Update(('Stop', 'Resume')[self.start_pressed], button_color=('white', ('red', 'green')[self.start_pressed]))    
                self.start_pressed = not self.start_pressed
    
    def onclick(self, target):
        #print("target: ", target)
        
        print("on clicked")

        targetx = target[0]
        targety = target[1]
        
        pixel_value = self.original_map_img.getpixel((targetx, targety))
        is_white = all(value >= 245 for value in pixel_value[:3])
        radius = 8
        print("pixel value: ", pixel_value, "is white: ", is_white)
 
        if (is_white) : 
            #print("ok pixel is white end point valid")

            ### ENDPOINT TO MAPPPP
            
            if self.endpoint_id:
                self.window['map_interact'].delete_figure(self.endpoint_id)
                self.endpoint_id = None

            # Draw the new point
            self.endpoint_id = self.window['map_interact'].draw_point((targetx*MAP_SCALE, targety*MAP_SCALE), size=8, color='red')
            
            # Update the points list
            self.endpoint = [(targetx, targety)]

            ### PATH PLANNING
            end_point_forpath = Point(targetx, targety)

            mat_map = self.image_to_matrixx(self.original_map_img)
            map = BMPMap(width=mat_map.shape[1], height=mat_map.shape[0], mat=mat_map)
            
            Planner = AStarPlanner(map=map, step_size=8, heuristic_dist='Euclidean') #CHANGE PLANNER IF NEEDED ASTAR OR RRT
            #Planner = RRTPlanner(map=map, epsilon=0.05, stepSize=20)

            robotposeinmap = self.convert_robot_pos()
            print("robotposeinmap: ", robotposeinmap)
            robotpoint = Point(int(robotposeinmap[0]), int(robotposeinmap[1]))
            
            self.update_pathh(Planner, end_point_forpath, robotpoint, map)
    
    #___________________________________________________________________________________________________________________________________________________________________________
    
    def update_pathh(self,astarPlanner, end_point_forpath, robotpoint, map):
        print("Path planning in progress...")

        flagok = 0
        try: 
            path = astarPlanner.plan(start=robotpoint, target=end_point_forpath)
            print("Path found successfully")
            flagok = 1
        except:
            print("Error in path planning")
            return None
        
        if flagok == 1:
            last_calculated_path = astarPlanner.finalPath
            
            # If a path exists, remove it
            if self.pathpoints:
                self.pathpoints.clear()  # Clear the pathpoints list after removal
                self.pathpoints_inertial.clear()  # Clear the pathpoints_inertial list after removal


            if self.pathpoints_id:
                for line_id in self.pathpoints_id:
                    self.window['map_interact'].delete_figure(line_id)
                self.pathpoints_id.clear()
            
            # Draw the new path
            for i in range(len(last_calculated_path)-1):
                pt1 = last_calculated_path[i].tuple()
                pt2 = last_calculated_path[i+1].tuple()
                
                # Convert pt1 and pt2 to tuples with integer elements
                pt1_int = tuple(int(coordinate) for coordinate in pt1)
                pt2_int = tuple(int(coordinate) for coordinate in pt2)

                line_id =  self.window['map_interact'].draw_line((pt1_int[0]*MAP_SCALE, pt1_int[1]*MAP_SCALE), (pt2_int[0]*MAP_SCALE, pt2_int[1]*MAP_SCALE), color='green', width=2)
                self.pathpoints_id.append(line_id)

                # Update the path points list with both points
                self.pathpoints.append(pt1_int)
                #self.pathpoints.append(pt2)

                if(i == len(last_calculated_path)-2): #TODO 3 avril a verif pour le dernier point a ajouter..
                    self.pathpoints.append(pt2_int)
                    
            # Remove the first point from the path points list
            self.pathpoints.pop(0)
             
            print("len(pathpoints)",len(self.pathpoints), "pathpoints: ", self.pathpoints) 
            #need to store also value of path in inertial
            for point in self.pathpoints:
                x_pos, y_pos = self.convert_map_to_inertial(point[0], point[1])
                #just keep 2 decimal
                x_pos = round(x_pos, 2)
                y_pos = round(y_pos, 2)
                self.pathpoints_inertial.append((x_pos, y_pos))
            
            print("len(pathpoints_inertial)",len(self.pathpoints_inertial),"pathpoints_inertial: ", self.pathpoints_inertial)    
            
            #for debug plot the path-----------------------------------------
            #self.visualize_path_inertial()
            
            ### finally we can send the path to the robot
            self.robot.send_path(self.pathpoints_inertial)

    
    def visualize_path_inertial(self):
        fig, ax = plt.subplots()
        
        # Extract x and y coordinates from pathpoints_inertial for plotting
        xs, ys = zip(*self.pathpoints_inertial)
        
        ax.plot(xs, ys, '-o', color='red', markersize=5, linewidth=2, label='Inertial Path')
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.legend()
        
        # Specify the path where you want to save the plot image
        plot_save_path = os.path.join(self.image_folder, "path_inertial_plot.png")
        
        # Save the figure
        plt.savefig(plot_save_path, bbox_inches='tight')
        plt.close(fig)  # Close the plot to free up memory
        
        print(f"Inertial path plot saved to {plot_save_path}")        
                

    def image_to_matrixx(self, original_img):
        img_array = np.array(original_img)
        map_img = 1 - np.floor_divide(img_array[:,:,1], 255)
        mat_map = map_img
        return mat_map
    
    
    # Update the data on the interface
    def update_data(self):
        if self.window is not None and not self.window.was_closed():
            self.update_time()
            self.update_map()
            self.update_camera()
            self.update_tag()
            
            self.update_pointsformap()
            #self.update_map_png()
            
            self.window.Element('position').update("%.2f, %.2f, %.2f"%(self.robot.positions[0], self.robot.positions[1], self.robot.positions[2]))
            

    def stop(self): 
        if self.robot is not None:
            self.robot.stop_robot()
        self.disable(self.manual_enable)
    
    # Stop the robot and map process and close the window
    def close(self):
        self.stop()
        self.control_started = False
        
        if self.robot is not None:
            if self.robot._process is not None:
                self.robot._is_moving.value = 0
                self.robot._running.value = 0
                self.robot._process.join()
        if self.map_data._process is not None:
            self.map_data._running.value = 0
            self.map_data._process.join()
        self.window.close()

    def update_pointsformap(self):
        
        #update the robot position
        robotposeinmap = self.convert_robot_pos()
        self.previous_robotpoint = self.robotpoint  # Store the current point as previous
        self.robotpoint = (robotposeinmap[0], robotposeinmap[1], robotposeinmap[2]) #x y cap
        
        
        #update the tags position
        tag_array = self.robot._tag_detectedd
        number_of_tags = self.robot._nbTag.value
        #print("number of tags: ", number_of_tags)
        new_tagpoints = {}
        
        for i in range(number_of_tags):
            start_index = i * self.robot.sizeofblocktag
            tag_id = int(tag_array[start_index])
            x = tag_array[start_index + 1]
            y = tag_array[start_index + 2]
            
            tag_canvas_x, tag_canvas_y = self.convert_inertial_to_map(x, y)
            new_tagpoints[tag_id] = (tag_canvas_x, tag_canvas_y)

            #print("new tag point id: ", tag_id, " x: ", tag_canvas_x, " y: ", tag_canvas_y)
        
        # Prepare to update map with current tagpoints
        # Instead of replacing self.tagpoints with new_tagpoints, update it with new or updated tag points
        self.previous_tagpoints = self.tagpoints.copy()  # Create a copy of the current tagpoints before updating
        self.tagpoints.update(new_tagpoints)  # Update existing dictionary with new or updated entries
        
        #print("tagpoints: ", self.tagpoints)

        
            

    def update_map_png(self):
        
        update_required_robot = False
        if self.robotpoint:
            # Determine if the update is necessary based on position change threshold
            update_required_robot = True
            if self.previous_robotpoint:
                # Calculate the difference in positions
                dx = abs(self.robotpoint[0] - self.previous_robotpoint[0])
                dy = abs(self.robotpoint[1] - self.previous_robotpoint[1])
                
                #print("robott dx: ", dx, " dy: ", dy) #TODO 
                # Check if the change is within the threshold
                #if dx <= 1 and dy <= 1:
                #    update_required = False
            
            if update_required_robot:
                
                robotmap_x, robotmap_y , robotmap_cap = self.robotpoint
                
                radius = 12
        
                self.robotpoint_id = self.window['map_interact'].DrawRectangle(( (robotmap_x - radius)*MAP_SCALE, (robotmap_y - radius)*MAP_SCALE), 
                                                      ((robotmap_x + radius)*MAP_SCALE, (robotmap_y + radius)*MAP_SCALE), 
                                                      fill_color='blue', 
                                                      line_color='black')


                # Calculate the end point of the cap line
                arrow_length = 45
                end_x = robotmap_x + arrow_length * math.cos(math.radians(self.robot.positions[2]))
                end_y = robotmap_y + arrow_length * math.sin(math.radians(self.robot.positions[2]))
                
                self.robotcap_id = self.window['map_interact'].DrawLine(((robotmap_x)*MAP_SCALE, (robotmap_y)*MAP_SCALE), 
                                        (end_x*MAP_SCALE, end_y*MAP_SCALE), 
                                        color='blue', 
                                        width=2)

        
        # Tags Update
        update_required_tags = False

        for tag_id, tag_pos in self.tagpoints.items():                   # Update the tags on the map
            previous_pos = self.previous_tagpoints.get(tag_id)
            #print("previous_pos: ", previous_pos)
            
            if previous_pos:
                # Calculate the difference in positions for this tag
                dx = abs(tag_pos[0] - previous_pos[0])
                dy = abs(tag_pos[1] - previous_pos[1])
                
                # Check if the change is within the threshold
                #if dx <= 20 and dy <= 20:
                #    continue  # Skip update for this tag; movement is within threshold
                
                update_required_tags = True
            
            if update_required_tags:
                
                radius = 8
                
                tagidd = self.window['map_interact'].DrawCircle((tag_pos[0]*MAP_SCALE, tag_pos[1]*MAP_SCALE), radius=4, fill_color='green', line_color='black')
                self.tagpoints_id.append(tagidd)
                
                tagidd = self.window['map_interact'].DrawText(str(tag_id), (tag_pos[0]*MAP_SCALE + radius, tag_pos[1]*MAP_SCALE), color='green', font='Arial 10')
                self.tagids_id.append(tagidd)


        
    # Update the map on the interface
    def update_map(self):
        if self.window is not None and not self.window.was_closed():
            #plutot gestion avec des id pour les elements a effacer + voir dans updatepath et onclick pour le path et endpoint qui eux sont gérés à part
             
            if self.robotpoint_id is not None:
                self.window['map_interact'].delete_figure(self.robotpoint_id)
                self.robotpoint_id = None
            if self.robotcap_id is not None:
                self.window['map_interact'].delete_figure(self.robotcap_id)
                self.robotcap_id = None

            for tagidd in self.tagpoints_id:
                self.window['map_interact'].delete_figure(tagidd)
            self.tagpoints_id.clear()

            for tagidd in self.tagids_id:
                self.window['map_interact'].delete_figure(tagidd)
            self.tagids_id.clear()
                
            self.update_map_png()
            #print("update map")       
    
    # Update the tags detected
    def update_tag(self):
        tag_array   = self.robot._tag_detectedd
        number_of_tags  = self.robot._nbTag.value

        tag_map = [0.0, 0.0, 0.0, 0.0]

        self.tag_id = list()

        #go through the list of tags detected
        temp = "Tags (Inertial):\n"
        for i in range(number_of_tags):
            start_index = i * self.robot.sizeofblocktag  # Calculate the start index for this tag's data
            tag_id = int(tag_array[start_index])
            self.tag_id.append("Tag " + str(tag_id))

            x = tag_array[start_index+1]
            y = tag_array[start_index + 2]
            z = tag_array[start_index + 3]
            x_corr = tag_array[start_index + 8]
            y_corr = tag_array[start_index + 9]
            x_rob = tag_array[start_index + 10]
            y_rob = tag_array[start_index + 11]

            # Format the display string for this tag's data
            #formatted_positions = f"ID {tag_id}: x={x:.2f}, y={y:.2f}, z={z:.2f}, x_cor={x_corr:.2f}, y_cor={y_corr:.2f}, x_rob={x_rob:.2f}, y_rob={y_rob:.2f}\n"
            formatted_positions = f"ID {tag_id}: x={x:.2f}, y={y:.2f}, z={z:.2f}, x_rob={x_rob:.2f}, y_rob={y_rob:.2f}\n"
            temp += formatted_positions

        if self.window is not None and not self.window.was_closed():
            self.window['tag_selection'].update(values=self.tag_id)
            self.window['tag_detected'].update(temp)
    
    # Update the time on the interface
    def update_time(self):
        elapsed = dt.now() - self.start_time
        sec = elapsed.seconds 
        hours = sec//3600
        minutes = (sec//60) - (hours*60)
        seconds = sec - (hours*3600 + minutes*60)
        if self.window is not None and not self.window.was_closed():
            try:
                self.window['test_timer'].update("%02i:%02i:%02i"%(hours, minutes, seconds))
                self.window['actual_mode'].update(('Auto Mode', 'Manual Mode', 'Turning Mode')[self.manual_mode]) #add for better UI experience #TO ASKKK ben
                
            except Exception as e:
                print("Error updating timer:", e)
                
            
    # Update the camera on the interface
    def update_camera(self):
        if self.robot._running.value == 1:
            if (self.robot.cam_msg.height == 0):
                pass
            else:
                cv_image = self.bridge.imgmsg_to_cv2(self.robot.cam_msg, "rgb8")
                image = Image.fromarray(cv_image)
                #self.cam_data = img_to_bytes(image)
                self.cam_data = img_to_bytes(resize_img(image, scale=0.5)) #reduire la taille sinon trop laggy
                
                if self.window is not None and not self.window.was_closed():
                    self.window.Element('camera').update(data=self.cam_data)
    
    # Disable the components of the interface
    def disable(self, components):
        if self.window is not None and not self.window.was_closed():
            for item in components:
                self.window[item].update(disabled=True)
                
        if debugg_prints:        
            print("Disable map gui, components: ", components)
        
    # Enable the components of the interface
    def enable(self, components):
        try :
            for item in components:
                self.window[item].update(disabled=False)
        except Exception as e:
            print("Error enabling components:", e)

    # Create the layout of the interface
    def create_layout(self):
        # Convenience functions for creating nice layouts
        def center_line(line):
            return [sg.Column(line, justification = 'center')]
        
        def encode_image(image):
            image_path = os.path.join(self.image_folder, image)
            return base64.b64encode(open(image_path, 'rb').read())
        
        sg.theme('LightBlue3')
        default_color = sg.theme_background_color()
        
        title = center_line([[sg.Text("Robot Navigation", font=self.title_font)]])
                
        interactive_map = center_line([[sg.Graph(canvas_size=(2048*MAP_SCALE, 2112*MAP_SCALE), 
                                                 graph_bottom_left=(0, 2112*MAP_SCALE), 
                                                 graph_top_right=(2048*MAP_SCALE, 0), 
                                                 enable_events=True,                               
                                                 key='map_interact')]])
        
        robot_selection = center_line([[sg.Text("Select Operating Mode", font = self.text_font),
                                        sg.Combo(mode_list, default_value="", size=(18,1), enable_events=True, font = self.input_font,         key='mode_selection'),
                                        sg.Text("Enter Robot ID", font = self.text_font),
                                        sg.Input(font = self.input_font, size=(18,1), enable_events=True,                                      key='robot_selection'),
                                        sg.Button("Start",size=(10,1), font=self.input_font, button_color="green",                                           key='start_btn'),
                                        sg.Button("Quit",size=(18,1), font=self.input_font, button_color="red",                                key='quit_btn')
                                    ]])                  
                
        test_monitor = center_line([[sg.Text("Elapsed Time :", font=self.text_font),
                                      sg.Text("00:00:00", font="Consolas 12 bold",                                                             key='test_timer'),
                                      sg.Text("No Mode", font=self.text_font,                                                                  key = 'actual_mode')]])
                        
                        
        data_monitor = center_line([[sg.Text("Position (x, y, theta) :", font=self.text_font),
                                      sg.Text("%.2f, %.2f, %.2f"%(00.00, 00.00, 00.00), font=self.text_font,                                  key='position')]])
                        
        
        up_down = [center_line([[sg.RealtimeButton('', image_data=encode_image("Up_btn.png"), button_color= default_color,                     key='robot_forward_btn')]]),
                   center_line([[sg.RealtimeButton('', image_data=encode_image("Down_btn.png"),button_color= default_color,                    key='robot_backward_btn')]])
                   ]
        
        right = [[sg.RealtimeButton('', image_data=encode_image("Right_btn.png"), button_color= default_color,                                 key='robot_right_btn')]] 
        left = [[sg.RealtimeButton('', image_data=encode_image("Left_btn.png"), button_color= default_color,                                   key='robot_left_btn')]]

        control_mode = center_line([[sg.Button("Manual Mode",size=(10,1), font=self.input_font,                                                key='control_mode_btn'),
                                     sg.Text("", font="Consolas 12 bold", text_color='black',                                      key='control_mode_status')]])
        
        test_controls = [#[sg.Button("Start",size=(10,1), font=self.input_font, button_color="green",                                           key='start_btn')]#,
                         #[sg.Button("Scan",size=(10,1), font=self.input_font,                                                                  key='scan_btn')]
                         ]
        
        robot_btn = center_line([[sg.Column(left), sg.Column(up_down), sg.Column(right), sg.Column(test_controls)]])         
        
        robot_controls = [interactive_map, control_mode, robot_btn, data_monitor]            
        
        camera = center_line([[sg.Image(data=self.cam_data,                                                                                     key='camera')
                               
                               ]])
        
        tag_selection = center_line([[sg.Text("Select Tag", font = self.text_font),
                        sg.Combo(self.tag_id, default_value="", size=(18,1), enable_events=True, font = self.input_font,                        key='tag_selection'),
                          sg.Text("Tag Selected : None | ", font="Consolas 12 bold", text_color='black',                                             key='tag_selected'),
                          sg.Text("Tag not in reach", font="Consolas 12 bold",   text_color='black',                                            key='go_arm_status')
                        ]])
        
        tag_btn = center_line([[sg.Button("Go to Tag",size=(10,1), font=self.input_font,                                                        key='go_tag_btn'),
                                sg.Button("Go Arm",size=(10,1), font=self.input_font,                                                           key='go_arm_btn'),
                                
                                sg.Text("", font="Consolas 12 bold",   text_color='black',                                                        key='x_rob_prefix'),
                                sg.Text("", font="Consolas 12 bold",   text_color='black',                                                        key='x_rob_value'),
                                sg.Text("", font="Consolas 12 bold",   text_color='black',                                                        key='x_rob_suffix'),
                                
                                sg.Text("", font="Consolas 12 bold",   text_color='black',                                                        key='y_rob_prefix'),
                                sg.Text("", font="Consolas 12 bold",   text_color='black',                                                        key='y_rob_value'),
                                sg.Text("", font="Consolas 12 bold",   text_color='black',                                                        key='y_rob_suffix')]])
        
        tag_display = center_line([[sg.Text("Tag detected (x, y, z, x_rob, y_rob ) :", font=self.text_font)],
                                   [sg.Text("No tag detected", font="Consolas 12 bold", text_color='black',                                     key='tag_detected')]])
        
        right_side = [camera, tag_selection, tag_btn, tag_display]
        
        middle_elements = center_line([[sg.Column(robot_controls), sg.Column(right_side)]])
                                  
        layout = [title,
                  robot_selection,      
                  test_monitor,
                  middle_elements
                  ]

        return layout
    
    def convert_robot_pos(self) : 

        robot_x, robot_y, robot_theta = self.robot.positions[0], self.robot.positions[1], self.robot.positions[2]
        # print("robot_x: ", robot_x)
        # print("robot_y: ", robot_y)
        # print("robot_theta: ", robot_theta)

        # Offset pour ajuster les coordonnées de la map 2D à la map ROS
        if self.sim == True:
            offset_x = 0.0
            offset_y = 0.0
        else:
            offset_x = 4.6 
            offset_y = -4.2 

        x_start = 23.7 + offset_x   
        y_start = 22.6 + offset_y   

        x_pos = robot_x + x_start
        y_pos = robot_y + y_start

        # Convertir les coordonnées du robot pour les adapter à la taille de l'image
        canvas_robot_x = (x_pos/ MAP_RES) 
        canvas_robot_y = (y_pos / MAP_RES) 

        # print("canvas_robot_x: ", canvas_robot_x)
        # print("canvas_robot_y: ", canvas_robot_y)
        # print("cap: ", cap)
    
        return int(canvas_robot_x), int(canvas_robot_y), int(robot_theta) #TODO 3avril pas besoin du cap.. a enlever

    def convert_map_to_inertial(self, x_map, y_map) :

        x_pos = (x_map * MAP_RES) - 23.7
        y_pos = (y_map * MAP_RES) - 22.6

        return x_pos, y_pos
    
    
    def convert_inertial_to_map(self, x_pos, y_pos) :
        
        if self.sim == True:
            offset_x = 0.0
            offset_y = 0.0
        else:
            offset_x =  4.6 
            offset_y = -4.2 

        tag_x = 23.7 + offset_x
        tag_y = 22.6 + offset_y
        
        tag_x_final = x_pos + tag_x
        tag_y_final = y_pos + tag_y

        tag_canvas_x = (tag_x_final / MAP_RES)
        tag_canvas_y = (tag_y_final / MAP_RES)
        
        return tag_canvas_x, tag_canvas_y

    
UI = UI()
