import rosbag
import matplotlib.pyplot as plt
plt.get_backend()

bag = rosbag.Bag('../../../result9.bag')

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
plt.plot(tt, x, label='x')
plt.plot(tt, y, label='y')
plt.plot(tt, z, label='z')
plt.plot(tt, [0.3]*len(tt), '--', label='y_des', linewidth=2)
plt.plot(tt, [0.5]*len(tt), '--', label='z_des', linewidth=2)
plt.plot(tt_traj,x_traj, '--', label='x_traj', linewidth=2)
plt.plot(tt_traj,y_traj, '--', label='y_traj', linewidth=2)
plt.plot(tt_traj,z_traj, '--', label='z_traj', linewidth=2)
plt.legend()
plt.xlabel('time (seconds)')
plt.ylabel('Position (meters)')
plt.show()