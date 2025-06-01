# Use rosbag to record data with infinite buffer
# rosbag record -b 0 -a

import rosbag
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import argrelextrema
plt.rcParams.update({'font.size': 12})
plt.rcParams["figure.figsize"] = (6.4*1.5, 4.8*1.5)
plt.get_backend()
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

file = 'real_spline_kd600-60.bag'
bag = rosbag.Bag(f'/home/rsl/catkin_ws/capstone_figures/data/real/{file}')

# Initialize some empty lists for storing the data
f_ext, t_f_ext = [], [] # done
pos_c_ee, t_pos_c_ee = [], [] # done
pos_c, t_pos_c = [], [] # done
pos_d_ee, t_pos_d_ee = [], [] # done
pos_d, t_pos_d = [], [] # done
rot_c, t_rot_c = [], []
rot_d, t_rot_d = [], []
k_switch, t_k_switch = [], [] # done

plot_f_ext = True
plot_pos_ee = True
plot_pos = True
plot_rot = True
plot_k_switch = False

ttmp = []
for topic,msg,t in bag.read_messages(topics=['/pose']):
    ttmp.append(t.to_sec())
    tmp = np.array([msg.position.x, msg.position.y, msg.position.z])
    pos_c.append(tmp)
    tmp = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    rot_c.append(tmp)
t_pos_c = [tt - ttmp[0] for tt in ttmp]
t_rot_c = t_pos_c
ttmp = []
for topic,msg,t in bag.read_messages(topics=['/traj']):
    ttmp.append(t.to_sec())
    tmp = np.array([msg.position.x, msg.position.y, msg.position.z])
    pos_d.append(tmp)
    tmp = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    rot_d.append(tmp)
t_pos_d = [tt - ttmp[0] for tt in ttmp]
t_rot_d = t_pos_d
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
    pos_c_ee.append(tmp)
t_pos_c_ee = [tt - ttmp[0] for tt in ttmp]
ttmp = []
for topic,msg,t in bag.read_messages(topics=['/des_pos_ee']):
    ttmp.append(t.to_sec())
    tmp = np.array([msg.x, msg.y, msg.z])
    pos_d_ee.append(tmp)
t_pos_d_ee = [tt - ttmp[0] for tt in ttmp]
ttmp = []
# for topic,msg,t in bag.read_messages(topics=['/cur_rot']):
#     ttmp.append(t.to_sec())
#     tmp = np.array([msg.x, msg.y, msg.z])
#     rot_c.append(tmp)
# t_rot_c = [tt - ttmp[0] for tt in ttmp]
# ttmp = []
# for topic,msg,t in bag.read_messages(topics=['/des_rot']):
#     ttmp.append(t.to_sec())
#     tmp = np.array([msg.x, msg.y, msg.z])
#     rot_d.append(tmp)
# t_rot_d = [tt - ttmp[0] for tt in ttmp]
# ttmp = []
for topic,msg,t in bag.read_messages(topics=['/k_switch']):
    ttmp.append(t.to_sec())
    tmp = msg.data
    tmp = np.array([tmp[0], tmp[7], tmp[14], tmp[21], tmp[28], tmp[35]])
    k_switch.append(tmp)
t_k_switch = [tt - ttmp[0] for tt in ttmp]
ttmp = []
bag.close()

if file == 'sim_line_kd300-30.bag':
    tmin = 14.2
    tmax = 30
    mode = 'Line'
    min_gap = 2.5
    Kp = 300
    Kr = 30
elif file == 'sim_line_kd600-60.bag':
    tmin = 5.6
    tmax = 19
    mode = 'Line'
    min_gap = 2.5
    Kp = 600
    Kr = 60
elif file == 'sim_plane_kd300-30.bag':
    tmin = 43
    tmax = 62
    mode = 'Plane'
    min_gap = 2.5
    Kp = 300
    Kr = 30
elif file == 'sim_plane_kd600-60.bag':
    tmin = 8.3
    tmax = 37
    mode = 'Plane'
    min_gap = 4.0
    Kp = 600
    Kr = 60
elif file == 'sim_spline_kd300-30.bag':
    tmin = 9.1
    tmax = 27
    mode = 'Spline'
    min_gap = 2.5
    Kp = 300
    Kr = 30
elif file == 'sim_spline_kd600-60.bag':
    tmin = 0.5
    tmax = 30
    mode = 'Spline'
    min_gap = 4.0
    Kp = 600
    Kr = 60
elif file == 'real_line_kd300-30.bag':
    tmin = 5.1
    tmax = 15
    mode = 'Line'
    min_gap = 2.5
    Kp = 300
    Kr = 30
elif file == 'real_line_kd600-60.bag':
    tmin = 4.6
    tmax = 15
    mode = 'Line'
    min_gap = 2.5
    Kp = 600
    Kr = 60
elif file == 'real_plane_kd300-30.bag':
    tmin = 5
    tmax = 21
    mode = 'Plane'
    min_gap = 2.5
    Kp = 300
    Kr = 30
elif file == 'real_plane_kd600-60.bag':
    tmin = 15
    tmax = 25
    min_gap = 2.5
    mode = 'Plane'
    Kp = 600
    Kr = 60
elif file == 'real_spline_kd300-30.bag':
    tmin = 17.1
    tmax = 32
    min_gap = 2.5
    mode = 'Spline'
    Kp = 300
    Kr = 30
elif file == 'real_spline_kd600-60.bag':
    tmin = 3.51
    tmax = 18
    min_gap = 2.5
    mode = 'Spline'
    Kp = 600
    Kr = 60
else:
    tmin = -np.inf
    tmax = np.inf
    min_gap = 2.5
    mode = 'None'
    Kp = 300
    Kr = 30

axes_upper = ['X', 'Y', 'Z']
axes_lower = ['x', 'y', 'z']
m_to_mm = 1000
fig_count = 0
linewidth = 1.5

#-----------------------------------------------------------------------
# 3D plots with 2D projections
#-----------------------------------------------------------------------
tc = np.asarray(t_pos_c)
xc = np.asarray([d[0] for d in pos_c])
yc = np.asarray([d[1] for d in pos_c])
zc = np.asarray([d[2] for d in pos_c])
td = np.asarray(t_pos_d)
xd = np.asarray([d[0] for d in pos_d])
yd = np.asarray([d[1] for d in pos_d])
zd = np.asarray([d[2] for d in pos_d])

# Find local maxima and minima
max_idx = argrelextrema(yc, np.greater)[0]
min_idx = argrelextrema(yc, np.less)[0]
merged_idx = np.sort(np.concatenate((min_idx, max_idx)))

# Plotting for reference
plt.figure(fig_count)
fig_count += 1

plt.plot(tc, yc, label='y(t)')
plt.plot(tc[max_idx], yc[max_idx], 'ro', label='Maxima')
plt.plot(tc[min_idx], yc[min_idx], 'go', label='Minima')
plt.legend()
plt.xlabel('Time')
plt.ylabel('Position y')
plt.title('Local Extrema of y(t)')
plt.grid(True)

filtered_idx = []

for i in merged_idx:
    if not filtered_idx:
        filtered_idx.append(i)
    else:
        # Check time difference from last accepted extremum
        if tc[i] - tc[filtered_idx[-1]] >= min_gap:
            filtered_idx.append(i)

# Form desired segments
start_idx = 0
for i in range(len(tc)):
    if tc[i] >= tmin:
        start_idx = i
        break
first_merge = 0
for i in range(len(filtered_idx)):
    if tc[filtered_idx[i]] >= tmin:
        first_merge = i
        break

idx_segs = []
idx_segs.append((start_idx, filtered_idx[first_merge]))
for i in range(first_merge, len(filtered_idx)-1):
    if tc[filtered_idx[i]] > tmin and tc[filtered_idx[i+1]] < tmax:
        idx_segs.append((filtered_idx[i], filtered_idx[i+1]))
print(idx_segs)

for seg in idx_segs:
    print(f'({tc[seg[0]]}, {tc[seg[1]]})')




idx_start = idx_segs[0][0]
idx_end = idx_segs[-1][1]

if plot_pos:
    #-------------------------------------------------------------------
    # Make individual plots for all segments
    #-------------------------------------------------------------------
    for seg in idx_segs:
        fig = plt.figure(fig_count)
        fig_count += 1
        ax = fig.add_subplot(111, projection='3d')

        # Main 3D trajectory
        ax.plot(xc[seg[0]:seg[1]], yc[seg[0]:seg[1]], zc[seg[0]:seg[1]], label='3D Position', linewidth=linewidth)
        ax.plot(xd[seg[0]:seg[1]], yd[seg[0]:seg[1]], zd[seg[0]:seg[1]], '--', label='3D Trajectory', linewidth=linewidth)

        # Projection on XY plane (z = min(z))
        ax.plot(xc[seg[0]:seg[1]], yc[seg[0]:seg[1]], min(zc[seg[0]:seg[1]]) * np.ones_like(zc[seg[0]:seg[1]]), label='XY Position', linewidth=linewidth)
        ax.plot(xd[seg[0]:seg[1]], yd[seg[0]:seg[1]], min(zd[seg[0]:seg[1]]) * np.ones_like(zd[seg[0]:seg[1]]), '--', label='XY Trajectory', linewidth=linewidth)

        # Projection on XZ plane (y = min(y))
        ax.plot(xc[seg[0]:seg[1]], min(yc[seg[0]:seg[1]]) * np.ones_like(yc[seg[0]:seg[1]]), zc[seg[0]:seg[1]], label='XZ Position', linewidth=linewidth)
        ax.plot(xd[seg[0]:seg[1]], min(yd[seg[0]:seg[1]]) * np.ones_like(yd[seg[0]:seg[1]]), zd[seg[0]:seg[1]], '--', label='XZ Trajectory', linewidth=linewidth)

        # Projection on YZ plane (x = min(x))
        ax.plot(min(xc[seg[0]:seg[1]]) * np.ones_like(xc[seg[0]:seg[1]]), yc[seg[0]:seg[1]], zc[seg[0]:seg[1]], label='YZ Position', linewidth=linewidth)
        ax.plot(min(xd[seg[0]:seg[1]]) * np.ones_like(xd[seg[0]:seg[1]]), yd[seg[0]:seg[1]], zd[seg[0]:seg[1]], '--', label='YZ Trajectory', linewidth=linewidth)

        # Axis labels
        offset = 0.25
        xmin = ax.get_xlim3d()[0]
        xmax = ax.get_xlim3d()[1]
        xoff = (xmax - xmin) * offset
        ymin = ax.get_ylim3d()[0]
        ymax = ax.get_ylim3d()[1]
        yoff = (ymax - ymin) * offset
        zmin = ax.get_zlim3d()[0]
        zmax = ax.get_zlim3d()[1]
        zoff = (zmax - zmin) * offset
        ax.text(x=xmin + ((xmax - xmin) / 2), y=ymax + yoff, z=zmin - zoff, s=fr'$\hat{{X}}_{{g}}$ [m]')
        ax.text(x=xmax + xoff, y=ymin + ((ymax - ymin) / 2), z=zmin - zoff, s=fr'$\hat{{Y}}_{{g}}$ [m]')
        ax.text(x=xmax + xoff, y=ymin - yoff, z=zmin + ((zmax-zmin) / 2), s=fr'$\hat{{Z}}_{{g}}$ [m]')
        ax.set_title(fr"3D Trajectory Control ({mode} Mode, Global Frame, $t \in [{round(tc[seg[0]], 2)}, {round(tc[seg[1]], 2)}]$ s, $K_{{d}}$ = {Kp})")
        ax.legend(loc='center left', bbox_to_anchor=(0.85, 0.5), fontsize='small')
        ax.view_init(elev=35, azim=45)

    #-------------------------------------------------------------------
    # Make a plot for all time segments
    #-------------------------------------------------------------------
    fig = plt.figure(fig_count)
    fig_count += 1
    ax = fig.add_subplot(111, projection='3d')

    # Main 3D trajectory
    ax.plot(xc[idx_start:idx_end], yc[idx_start:idx_end], zc[idx_start:idx_end], label='3D Position', linewidth=linewidth)
    ax.plot(xd[idx_start:idx_end], yd[idx_start:idx_end], zd[idx_start:idx_end], '--', label='3D Trajectory', linewidth=linewidth)

    # Projection on XY plane (z = min(z))
    ax.plot(xc[idx_start:idx_end], yc[idx_start:idx_end], min(zc[idx_start:idx_end]) * np.ones_like(zc[idx_start:idx_end]), label='XY Position', linewidth=linewidth)
    ax.plot(xd[idx_start:idx_end], yd[idx_start:idx_end], min(zd[idx_start:idx_end]) * np.ones_like(zd[idx_start:idx_end]), '--', label='XY Trajectory', linewidth=linewidth)

    # Projection on XZ plane (y = min(y))
    ax.plot(xc[idx_start:idx_end], min(yc[idx_start:idx_end]) * np.ones_like(yc[idx_start:idx_end]), zc[idx_start:idx_end], label='XZ Position', linewidth=linewidth)
    ax.plot(xd[idx_start:idx_end], min(yd[idx_start:idx_end]) * np.ones_like(yd[idx_start:idx_end]), zd[idx_start:idx_end], '--', label='XZ Trajectory', linewidth=linewidth)

    # Projection on YZ plane (x = min(x))
    ax.plot(min(xc[idx_start:idx_end]) * np.ones_like(xc[idx_start:idx_end]), yc[idx_start:idx_end], zc[idx_start:idx_end], label='YZ Position', linewidth=linewidth)
    ax.plot(min(xd[idx_start:idx_end]) * np.ones_like(xd[idx_start:idx_end]), yd[idx_start:idx_end], zd[idx_start:idx_end], '--', label='YZ Trajectory', linewidth=linewidth)

    # Axis labels
    offset = 0.25
    xmin = ax.get_xlim3d()[0]
    xmax = ax.get_xlim3d()[1]
    xoff = (xmax - xmin) * offset
    ymin = ax.get_ylim3d()[0]
    ymax = ax.get_ylim3d()[1]
    yoff = (ymax - ymin) * offset
    zmin = ax.get_zlim3d()[0]
    zmax = ax.get_zlim3d()[1]
    zoff = (zmax - zmin) * offset
    ax.text(x=xmin + ((xmax - xmin) / 2), y=ymax + yoff, z=zmin - zoff, s=fr'$\hat{{X}}_{{g}}$ [m]')
    ax.text(x=xmax + xoff, y=ymin + ((ymax - ymin) / 2), z=zmin - zoff, s=fr'$\hat{{Y}}_{{g}}$ [m]')
    ax.text(x=xmax + xoff, y=ymin - yoff, z=zmin + ((zmax-zmin) / 2), s=fr'$\hat{{Z}}_{{g}}$ [m]')
    ax.set_title(fr"3D Trajectory Control ({mode} Mode, Global Frame, $t \in [{round(tc[idx_segs[0][0]], 2)}, {round(tc[idx_segs[-1][1]], 2)}]$ s, $K_{{d}}$ = {Kp})")
    ax.legend(loc='center left', bbox_to_anchor=(0.85, 0.5), fontsize='small')
    ax.view_init(elev=35, azim=45)

#-----------------------------------------------------------------------
# External force plots
#-----------------------------------------------------------------------
if plot_f_ext:
    time1 = np.asarray(t_f_ext)
    data1 = np.asarray(f_ext)
    time1_trimmed = time1[idx_start:idx_end]
    data1_trimmed = data1[idx_start:idx_end]

    plt.figure(fig_count)
    fig_count += 1
    
    for i in range(3):
        ax = plt.subplot(3, 1, i+1)

        plt.plot(time1_trimmed, data1_trimmed[:,i], linewidth=linewidth)#, label=fr'$F_{{{axes_lower[i]}_ee}}$')
        if i == 0:
            plt.title(f'External Force Applied at the End Effector')
        # plt.legend(loc='best')
        if i == 2:
            plt.xlabel('Time [s]')
        plt.ylabel(fr'$F_{{{axes_lower[i]}}}$ [N]')

        max_val = np.max(data1_trimmed[:,i])
        min_val = np.min(data1_trimmed[:,i])
        for i in range(len(idx_segs)):
            if i >= len(colors) - 1:
                break
            seg = idx_segs[i]
            ax.axvspan(tc[seg[0]], tc[seg[1]], color=colors[i+1], alpha=0.2)
        plt.grid(1)

#-----------------------------------------------------------------------
# EE position plots to show error
#-----------------------------------------------------------------------
if plot_pos_ee:
    time1 = np.asarray(t_pos_c_ee)
    data1 = np.asarray(pos_c_ee)
    time2 = np.asarray(t_pos_d_ee)
    data2 = np.asarray(pos_d_ee)
    time1_trimmed = time1[idx_start:idx_end]
    data1_trimmed = data1[idx_start:idx_end]
    time2_trimmed = time2[idx_start:idx_end]
    data2_trimmed = data2[idx_start:idx_end]
    y_tick = 2

    plt.figure(fig_count)
    fig_count += 1
    
    for i in range(3): 
        ax = plt.subplot(3, 1, i+1)

        plt.plot(time1_trimmed, data1_trimmed[:,i]*m_to_mm, linewidth=linewidth, label=fr'${axes_lower[i]}_{{ee}}$')
        plt.plot(time2_trimmed, data2_trimmed[:,i]*m_to_mm, '--', linewidth=linewidth, label=fr'${axes_lower[i]}_{{ee_d}}$')
        if i == 0:
            plt.title(fr'Position Trajectory Control ({mode} Mode, $K_{{d}}$ = {Kp})')
        plt.legend(loc='center left', bbox_to_anchor=(0.95, 0.75))
        if i == 2:
            plt.xlabel('Time [s]')
        plt.ylabel(fr'$\hat{axes_upper[i]}_{{ee}}$ [m]')

        max_val = max(np.max(data1_trimmed[:,i]), np.max(data2_trimmed[:,i]))*m_to_mm
        min_val = min(np.min(data1_trimmed[:,i]), np.min(data2_trimmed[:,i]))*m_to_mm
        ymin = -3 * abs(max(abs(min_val), abs(max_val)))
        y_min_round = round(ymin / y_tick) * y_tick
        ymax = 3 * abs(max(abs(min_val), abs(max_val)))
        y_max_round = round(ymax / y_tick) * y_tick

        # plt.yticks(np.arange(y_min_round, y_max_round, step=y_tick))
        plt.ylim(y_min_round, y_max_round)
        ax.yaxis.set_major_locator(MultipleLocator(abs(round(ymax / 3))))
        for i in range(len(idx_segs)):
            if i >= len(colors) - 1:
                break
            seg = idx_segs[i]
            ax.axvspan(tc[seg[0]], tc[seg[1]], color=colors[i+1], alpha=0.2)
        plt.grid(1)

#-----------------------------------------------------------------------
# Make plots for the orientation control
#-----------------------------------------------------------------------
if plot_rot:
    time1 = np.asarray(t_rot_c)
    data1 = np.asarray(rot_c)
    time2 = np.asarray(t_rot_d)
    data2 = np.asarray(rot_d)
    time1_trimmed = time1[idx_start:idx_end]
    data1_trimmed = data1[idx_start:idx_end]
    time2_trimmed = time2[idx_start:idx_end]
    data2_trimmed = data2[idx_start:idx_end]

    plt.figure(fig_count)
    fig_count += 1

    labels = ['x', 'y', 'z', 'w']
    
    for i, comp in enumerate(labels):
        # print(i)
        ax = plt.subplot(4, 1, i+1)

        plt.plot(time1_trimmed, data1_trimmed[:, i], label=fr'$q_{{{comp}}}$')
        plt.plot(time2_trimmed, data2_trimmed[:, i], '--', label=fr'$q_{{{comp}_d}}$')

        mean = np.mean(data2_trimmed[:,i])
        max_dist = np.max(abs(data1_trimmed[:,i] - mean))
        ymin = mean - 2 * max_dist
        ymax = mean + 2 * max_dist


        plt.ylabel(fr'$q_{{{comp}}}$')
        if i == 0:
            plt.title(f"Quaternion Trajectory Control ({mode} Mode, Global Frame, $K_{{d}}$ = {Kr})")
        if i == 3:
            plt.xlabel("Time [s]")
        plt.legend(loc='center left', bbox_to_anchor=(0.95, 0.75))
        plt.grid(1)
        plt.ylim(ymin, ymax)
        ax.yaxis.set_major_locator(MultipleLocator(abs((ymax - mean) / 2)))
        for i in range(len(idx_segs)):
            if i >= len(colors) - 1:
                break
            seg = idx_segs[i]
            ax.axvspan(tc[seg[0]], tc[seg[1]], color=colors[i+1], alpha=0.2)

#-----------------------------------------------------------------------
# Make a plot for the switching matrix
#-----------------------------------------------------------------------
if plot_k_switch:
    time1 = np.asarray(t_k_switch)
    data1 = np.asarray(k_switch)
    time1_trimmed = time1[idx_start:idx_end]
    data1_trimmed = data1[idx_start:idx_end]

    plt.figure(fig_count, figsize=(8, 4))
    fig_count += 1

    spacing = 0.01
    labels = [fr'$x$', fr'$y$', fr'$z$', fr'$\alpha$', fr'$\beta$', fr'$\gamma$']

    for i in range(6):
        offset = i * spacing
        plt.plot(time1_trimmed, data1_trimmed[:,i] + offset, linewidth=linewidth, label=labels[i])

    plt.yticks([0, 1], ['0 = Stiffness\nDisabled', '1 = Stiffness\nEnabled'])
    plt.xlabel('Time [s]')
    plt.title(fr'$K_{{switch}}$ Values vs Time')
    plt.legend(loc='center left', bbox_to_anchor=(0.95, 0.75))

plt.show(block=False)

input("Press any key to close all figures...")
plt.close('all')