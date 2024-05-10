import matplotlib.pyplot as plt

# Initialize lists to hold the robot's positions
robot_positions_x = []
robot_positions_y = []
# Initialize variables to hold the goal point
goal_x = None
goal_y = None

# Open the output file and process each line
with open('mov.txt', 'r') as file:
    for line in file:
        # Check if the line contains the goal point and robot's position
        if "goalpt:" in line and "posrob:" in line:
            try:
                # Extracting goal point
                goal_part = line.split('goalpt:')[1].split(',')[0:2]
                goal_x, goal_y = float(goal_part[0]), float(goal_part[1])
                
                # Extracting robot's position
                pos_part = line.split('posrob:')[1].split(',')[0:2]
                x_pos, y_pos = float(pos_part[0]), float(pos_part[1])
                
                robot_positions_x.append(x_pos)
                robot_positions_y.append(y_pos)
            except ValueError as e:
                print(f"Error parsing line: {line}. Error: {e}")

# Ensure we have the goal point before plotting
if goal_x is not None and goal_y is not None:
    goal_point = (goal_x, goal_y)
    # Plotting
    plt.figure(figsize=(8, 6))
    plt.plot(robot_positions_x, robot_positions_y, label='Robot Path', marker='o')
    plt.scatter(goal_point[0], goal_point[1], color='red', s=100, label='Goal Point', marker='x')
    plt.title('Robot Trajectory and Goal Point')
    plt.xlabel('X Position (meters)')
    plt.ylabel('Y Position (meters)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # Ensure equal aspect ratio for x and y axes
    plt.show()
else:
    print("Goal point not found in the file.")
