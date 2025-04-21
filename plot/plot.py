# Use rosbag to record data with infinite buffer
# rosbag record -b 0 -a

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math
plt.rcParams.update({'font.size': 25})
plt.get_backend()

bag = rosbag.Bag('/home/rsl/catkin_ws/continuous_limiting.bag')

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
    ttz = [tt_ - tt[0] for tt_ in tt]
    x.append(msg.position.x)
    y.append(msg.position.y)
    z.append(msg.position.z)
for topic2,msg2,t2 in bag.read_messages(topics=['/traj']):
    tt_traj.append(t2.to_sec())
    tt_trajz = [tt_traj_ - tt_traj[0] for tt_traj_ in tt_traj]
    x_traj.append(msg2.x)
    y_traj.append(msg2.y)
    z_traj.append(msg2.z)
bag.close()

tmin = 1
tmax = 15

plt.figure(1)
plt.plot(ttz, x, 'r', linewidth=5, label='x actual')
plt.plot(tt_trajz, x_traj, 'b--', linewidth=2, label='x trajectory')
plt.title('X Position Trajectory Control (error limit = 20mm, continuous movement)')
plt.legend(loc='best', fontsize='x-small')
plt.xlabel('Time (seconds)')
plt.ylabel('Position (meters)')
ymin = min(min(x), min(x_traj)) - 0.03
ymax = max(max(x), max(x_traj)) + 0.03
plt.xticks(np.arange(math.floor(ttz[0]), math.ceil(ttz[-1]), step=1), fontsize='x-small')
plt.yticks(np.arange(ymin, ymax, step=0.05), fontsize='x-small')
plt.xlim(tmin, tmax)
plt.ylim(ymin, ymax)
plt.grid(1)

plt.figure(2)
plt.plot(ttz, y, 'r', linewidth=5, label='y actual')
plt.plot(tt_trajz, y_traj, 'b--', linewidth=2, label='y trajectory')
plt.title('Y Position Trajectory Control (error limit = 20mm, continuous movement)')
plt.legend(loc='best', fontsize='x-small')
plt.xlabel('Time (seconds)')
plt.ylabel('Position (meters)')
ymin = min(min(y), min(y_traj)) - 0.03
ymax = max(max(y), max(y_traj)) + 0.03
plt.xticks(np.arange(math.floor(ttz[0]), math.ceil(ttz[-1]), step=1), fontsize='x-small')
plt.yticks(np.arange(ymin, ymax, step=0.05), fontsize='x-small')
plt.xlim(tmin, tmax)
plt.ylim(ymin, ymax)
plt.grid(1)

plt.figure(3)
plt.plot(ttz, z, 'r', linewidth=5, label='z actual')
plt.plot(tt_trajz, z_traj, 'b--', linewidth=2, label='z trajectory')
plt.title('Z Position Trajectory Control (error limit = 20mm, continuous movement)')
plt.legend(loc='best', fontsize='x-small')
plt.xlabel('Time (seconds)')
plt.ylabel('Position (meters)')
ymin = min(min(z), min(z_traj)) - 0.03
ymax = max(max(z), max(z_traj)) + 0.03
plt.xticks(np.arange(math.floor(ttz[0]), math.ceil(ttz[-1]), step=1), fontsize='x-small')
plt.yticks(np.arange(ymin, ymax, step=0.05), fontsize='x-small')
plt.xlim(tmin, tmax)
plt.ylim(ymin, ymax)
plt.grid(1)

plt.show()