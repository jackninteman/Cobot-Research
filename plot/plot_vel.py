import rosbag
import matplotlib.pyplot as plt
plt.get_backend()

bag = rosbag.Bag('../../../2022-02-13-18-25-13.bag')

# Initialize some empty lists for storing the data
tt = []
x = []
y = []
z = []
tt_traj = []
x_traj = []
y_traj = []
z_traj = []

for topic,msg,t in bag.read_messages(topics=['/twist']):
    tt.append(t.to_sec())
    x.append(msg.linear.x)
    y.append(msg.linear.y)
    z.append(msg.linear.z)

bag.close()
#print tt
plt.figure()
plt.plot(tt, x, label='x')
plt.plot(tt, y, label='y')
plt.plot(tt, z, label='z')
plt.legend()
plt.xlabel('time (seconds)')
plt.ylabel('Velocity (meter/sec)')
plt.show()