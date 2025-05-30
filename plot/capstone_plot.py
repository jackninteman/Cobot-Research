# Use rosbag to record data with infinite buffer
# rosbag record -b 0 -a

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math
plt.rcParams.update({'font.size': 25})
plt.get_backend()

file = 'sim_line_ee.bag'
bag = rosbag.Bag(f'/home/rsl/catkin_ws/capstone_figures/data/{file}')
print(f'/home/rsl/catkin_ws/capstone_figures/data/{file}')

# Initialize some empty lists for storing the data
f_ext = []
t_f_ext = []
pos_c = []
t_pos_c = []
pos_d = []
t_pos_d = []
rot_c = []
t_rot_c = []
rot_d = []
t_rot_d = []
k_switch = []
t_k_switch = []

ttmp = []
for topic,msg,t in bag.read_messages(topics=['/f_ext']):
    ttmp.append(t.to_sec())
    tmp = np.array([msg.x, msg.y, msg.z])
    f_ext.append(tmp)
t_f_ext = [tt - ttmp[0] for tt in ttmp]
ttmp = []
for topic,msg,t in bag.read_messages(topics=['/cur_pos_ee']):
    ttmp.append(t.to_sec())
    tmp = np.array([msg.x, msg.y, msg.z])
    pos_c.append(tmp)
t_pos_c = [tt - ttmp[0] for tt in ttmp]
ttmp = []
for topic,msg,t in bag.read_messages(topics=['/des_pos_ee']):
    ttmp.append(t.to_sec())
    tmp = np.array([msg.x, msg.y, msg.z])
    pos_d.append(tmp)
t_pos_d = [tt - ttmp[0] for tt in ttmp]
ttmp = []
for topic,msg,t in bag.read_messages(topics=['/cur_rot']):
    ttmp.append(t.to_sec())
    tmp = np.array([msg.x, msg.y, msg.z])
    rot_c.append(tmp)
t_rot_c = [tt - ttmp[0] for tt in ttmp]
ttmp = []
for topic,msg,t in bag.read_messages(topics=['/des_rot']):
    ttmp.append(t.to_sec())
    tmp = np.array([msg.x, msg.y, msg.z])
    rot_d.append(tmp)
t_rot_d = [tt - ttmp[0] for tt in ttmp]
ttmp = []
for topic,msg,t in bag.read_messages(topics=['/k_switch']):
    ttmp.append(t.to_sec())
    tmp = msg.data
    k_switch = np.array(tmp).reshape((6,6))
t_k_switch = [tt - ttmp[0] for tt in ttmp]
ttmp = []
bag.close()

tmin = 1
tmax = 15

axes_upper = ['X', 'Y', 'Z']
axes_lower = ['x', 'y', 'z']
m_to_mm = 1000

time1 = t_f_ext
data1 = f_ext
ytick = 0.5
for i in range(3):
    plt.figure(i)
    plt.plot(time1, [d[i] for d in data1], linewidth=1, label=fr'F_${{{axes_lower[i]}}}$')
    plt.title(f'External Force Applied at the End Effector ($\hat{axes_upper[i]}_{{ee}}$ Axis)')
    plt.legend(loc='best', fontsize='x-small')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Force (Newtons)')
    max_val = max([d[i] for d in data1])
    min_val = min([d[i] for d in data1])
    ymin = min_val + (min_val / 10.0)
    ymax = max_val + (max_val / 10.0)
    plt.xticks(np.arange(math.floor(t_f_ext[0]), math.ceil(t_f_ext[-1]), step=1), fontsize='x-small')
    plt.yticks(np.arange(ymin, ymax, ytick), fontsize='x-small')
    plt.xlim(tmin, tmax)
    plt.ylim(ymin, ymax)
    plt.grid(1)

time1 = t_pos_c
data1 = pos_c
time2 = t_pos_d
data2 = pos_d
y_step = 3
K = 500
for i in range(3):
    plt.figure(i+3)
    plt.plot(time1, [d[i]*m_to_mm for d in data1], linewidth=1, label=fr'{axes_lower[i]}_${{ee}}$')
    plt.plot(time2, [d[i]*m_to_mm for d in data2], '--', linewidth=1, label=fr'{axes_lower[i]}_${{ee_d}}$')
    plt.title(fr'$\hat{axes_upper[i]}_{{ee}}$ Axis Trajectory Control ($K_{{d}}$ = {K})')
    plt.legend(loc='best', fontsize='x-small')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Position (mm)')
    max_val = max(max([d[i]*m_to_mm for d in data1]), max([d[i] for d in data2]))
    min_val = min(min([d[i]*m_to_mm for d in data2]), min([d[i] for d in data2]))
    ymin = -10 * abs(max(abs(min_val), abs(max_val)))
    y_min_round = round(ymin / y_step) * y_step
    ymax = 10 * abs(max(abs(min_val), abs(max_val)))
    y_max_round = round(ymax / y_step) * y_step
    plt.xticks(np.arange(math.floor(t_f_ext[0]), math.ceil(t_f_ext[-1]), step=1), fontsize='x-small')
    plt.yticks(np.arange(y_min_round, y_max_round, step=y_step), fontsize='x-small')
    plt.xlim(tmin, tmax)
    plt.ylim(y_min_round, y_max_round)
    plt.grid(1)

plt.show(block=False)

input("Press any key to close all figures...")
plt.close('all')