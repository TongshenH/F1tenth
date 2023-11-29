import os
import csv

import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy import interpolate

path = os.getcwd()
os.chdir(path)

flag = "test"  # flag = "save" to save the keypoints
fp = ("/home/tongshen/Projects/Scaled_auto_vehicle/catkin_ws/"
      "src/f110_ros/f110_simulator/trajectory/desired_traj2.csv")

trajectory = np.loadtxt(fp, skiprows=1, delimiter=',')
X, Y = trajectory[:, 0], trajectory[:, 1]
start_index = 0
end_index = len(X)
# Normalize the initial global frame
# traj = trajectory[start_index:end_index, :2] - np.array([X[start_index], Y[start_index]])
traj = trajectory[start_index:end_index, :2]
start_point = []

# split the trajectory
for index, (x, y) in enumerate(traj):
    if np.linalg.norm([x, y]) < 1:
        start_point.append(index)
start_point = np.array(start_point)
clustering = DBSCAN(eps=0.5, min_samples=1).fit(start_point.reshape(-1, 1))
label = clustering.labels_
num_cluster = len(set(label)) - (1 if -1 in label else 0)
sp = []  # seperate start point list
lp = []  # same start point list
index = 0
for i, label in enumerate(clustering.labels_):
    if label != index:
        index = label
        sp.append(lp)
        lp = []
    lp.append(start_point[i])
sp.append(lp)
process_start_index = [int(sum(p) / len(p)) for p in sp]
process_start_index[0] = 0

circles = []
circle_num = len(process_start_index) - 1
for i in range(circle_num):
    circle_trajectory = traj[process_start_index[i]:process_start_index[i + 1]]
    circles.append(circle_trajectory)

# interpolate the trajectory and fuse the trajectory
circle = circles[-2]
x_data = circle[:, 0]
y_data = circle[:, 1]
tck, u = interpolate.splprep([x_data, y_data], s=0.0)
x_i, y_i = interpolate.splev(np.linspace(0, 1, 200), tck)
x_i = x_i.reshape(len(x_i), 1)
y_i = y_i.reshape(len(y_i), 1)
keypoints = np.hstack((x_i, y_i))
# Saving data
fields = ['x_position', 'y_position']
with open('/home/tongshen/Projects/Scaled_auto_vehicle/catkin_ws/src/f110_ros/f110_simulator/trajectory'
          '/key_points.csv', 'w') as f:
    write = csv.writer(f)
    write.writerow(fields)
    write.writerows(keypoints)

plt.figure()
plt.scatter(traj[:, 0], traj[:, 1], s=10)
plt.plot(x_i, y_i, color="r")
plt.xlabel("x axis /m")
plt.ylabel("y axis /m")
plt.title('Processed Trajectory (Long-hall)')
plt.legend(['Measured waypoints', "Designed Trajectory"])
plt.grid()
plt.savefig("/home/tongshen/Projects/Scaled_auto_vehicle/catkin_ws/"
            "src/f110_ros/f110_simulator/trajectory/processed_path.svg")
plt.show()