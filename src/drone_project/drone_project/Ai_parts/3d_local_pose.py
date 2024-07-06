import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# Specify the CSV file within the directory
file_path = '/home/naor/Desktop/naor/study/Ai_gps_ros2/src/drone_project/drone_project/log_flight/flight_data1.csv'

# Check if the file exists
if not os.path.isfile(file_path):
    print(f"File not found: {file_path}")
else:
    # Load the flight data from the CSV file
    flight_data = pd.read_csv(file_path)

    # Extract GPS coordinates
    x_gps = flight_data['x_gps']
    y_gps = flight_data['y_gps']
    z_gps = flight_data['z_gps']

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the GPS coordinates
    ax.plot(x_gps, y_gps, z_gps, marker='o')

    # Set labels
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('3D GPS Flight Path')

    # Show plot
    plt.show()
