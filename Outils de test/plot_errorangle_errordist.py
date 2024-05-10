import matplotlib.pyplot as plt
import numpy as np
 
# Initialize lists to hold the error values
error_angles = []
error_distances = []
 
# Open the output file and process each line
with open('mov.txt', 'r') as file:
    for line in file:
        # Check if the line contains an error angle
        if "error angle:" in line:
            try:
                # Extract and append the error angle value to the list
                error_angle = float(line.split(":")[1].strip())
                error_angles.append(error_angle)
            except ValueError:
                pass  # If conversion fails, skip this line
 
        # Check if the line contains an error distance
        elif "error dist:" in line:
            try:
                # Extract and append the error distance value to the list
                error_distance = float(line.split(":")[1].strip())
                error_distances.append(error_distance)
            except ValueError:
                pass  # If conversion fails, skip this line
 
# Convert lists to numpy arrays for plotting
error_angles = np.array(error_angles)
error_distances = np.array(error_distances)
 
# Plotting
fig, axs = plt.subplots(2, 1, figsize=(10, 8))
 
# Plot error angle
axs[0].plot(error_angles, label='Error Angle')
axs[0].set_title('Error Angle Over Time')
axs[0].set_ylabel('Angle Error (degrees)')
axs[0].legend()
 
# Plot error distance
axs[1].plot(error_distances, label='Error Distance')
axs[1].set_title('Error Distance Over Time')
axs[1].set_ylabel('Distance Error (meters)')
axs[1].set_xlabel('Time (arbitrary units)')
axs[1].legend()
 
plt.tight_layout()
plt.show()