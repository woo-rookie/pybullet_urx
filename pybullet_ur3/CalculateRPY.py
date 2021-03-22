import numpy as np
import math
# socket and connect
# addr = (UR_HOST_IP, 30003)
# client = socket(AF_INET, SOCK_STREAM)
# client.connect(addr)
#
# # get robot initial position
# data = bytes(client.recv(1060))
# x, y, z = struct.unpack('!ddd', data[444:468])
# rx, ry, rz = struct.unpack('!ddd', data[468:492])
rx, ry, rz = 2.049, -2.360, -0.184
#rodrigues tranform
theta = math.sqrt(rx*rx + ry*ry + rz*rz)
r = np.array([rx, ry, rz]).reshape((3, 1))/theta
print("r: ",r)
print("~~~~~~~")
rrx = r[0]
rry = r[1]
rrz = r[2]
r1 = np.array([[0, -rrz, rry], [rrz, 0, -rrx], [-rry, rrx, 0]])

R = np.cos(theta) * np.eye(3) + (1-np.cos(theta)) * np.dot(r, r.T) + np.sin(theta)*r1
