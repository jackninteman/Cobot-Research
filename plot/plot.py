import rosbag
import matplotlib.pyplot as plt
plt.get_backend()

bag = rosbag.Bag('../../../result4.bag')

# Initialize some empty lists for storing the data
tt = []
x = []
y = []
z = []

for topic,msg,t in bag.read_messages(topics=['/pose']):
    tt.append(t.to_sec())
    x.append(msg.position.x)
    y.append(msg.position.y)
    z.append(msg.position.z)
bag.close()
#print tt
plt.figure()
plt.plot(tt, x, label='x')
plt.plot(tt, y, label='y')
plt.plot(tt, z, label='z')
plt.legend()
plt.xlabel('time (seconds)')
plt.ylabel('Position (meters)')
plt.show()