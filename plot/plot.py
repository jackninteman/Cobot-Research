# Use rosbag to record data with infinite buffer
# rosbag record -b 0 -a

import rosbag
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({'font.size': 25})
plt.get_backend()

bag = rosbag.Bag('../../../2023-02-04-16-11-15.bag')

# Initialize some empty lists for storing the data
tt = []
x = []
y = []
z = []
tt_traj = []
x_traj = []
y_traj = []
z_traj = []

for topic,msg,t in bag.read_messages(topics=['/pose']):
    tt.append(t.to_sec())
    x.append(msg.position.x)
    y.append(msg.position.y)
    z.append(msg.position.z)
for topic2,msg2,t2 in bag.read_messages(topics=['/traj']):
    tt_traj.append(t2.to_sec())
    x_traj.append(msg2.x)
    y_traj.append(msg2.y)
    z_traj.append(msg2.z)
bag.close()
#print tt
plt.figure()
plt.plot(tt, x, linewidth=5)
plt.plot(tt, y, linewidth=5)
plt.plot(tt, z, linewidth=5)
# plt.plot(tt, [0.0]*len(tt), '--', label='y_des', linewidth=3)
# plt.plot(tt, [0.0]*len(tt), '--', linewidth=3)
# plt.plot(tt, [0.4]*len(tt), '--', linewidth=3)
plt.plot(tt_traj,x_traj, '--', linewidth=2)
plt.plot(tt_traj,y_traj, '--', linewidth=2)
plt.plot(tt_traj,z_traj, '--', linewidth=2)
# plt.legend(bbox_to_anchor=(0.6, 0.6), loc='upper left', borderaxespad=0)
# plt.legend()
plt.xlabel('time (seconds)')
plt.ylabel('Position (meters)')
plt.yticks(np.arange(-0.75, 1, step=0.25))
# plt.xlim([1656609745, 1656609770])
plt.xlim([15, 35])
# print(tt)
plt.show()