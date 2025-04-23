import rospy
import rosbag
from std_msgs.msg import Float64MultiArray
import numpy as np

# Open the bag file
bag = rosbag.Bag('/home/rsl/catkin_ws/cobot_data/matrix_recording.bag')

with open('/home/rsl/catkin_ws/cobot_data/matrix.txt', 'a') as f:
# Iterate through the messages
    for topic, msg, t in bag.read_messages(topics=['/matrix_topic']):
        # The data in msg.data is a list with 72 elements (36 elements for each 6x6 matrix)
        matrix1 = np.array(msg.data[:36]).reshape(6, 6)  # First 6x6 matrix
        matrix2 = np.array(msg.data[36:]).reshape(6, 6)  # Second 6x6 matrix

        # Print matrices (or process them)
        print("S Matrix:\n", file=f)
        
        for i in range(6):
            print("[", end='', file=f)
            for j in range(6):
                print(f'\t{matrix1[i][j]:.7f}', end='', file=f)
            print("\t]\n", file=f)
        print("\n\n", file=f)

        print("L Matrix:\n", file=f)
        
        for i in range(6):
            print("[", end='', file=f)
            for j in range(6):
                print(f'\t{matrix2[i][j]:.7f}', end='', file=f)
            print("\t]\n", file=f)
        print("\n\n", file=f)

bag.close()
