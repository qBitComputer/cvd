import numpy as np
import matplotlib.pyplot as plt
import math

# Tekenrobot model, computation and plotting
# Job Meijer - 22 May 2024


print("Starting...");

# Parameters
l1 = 3;
l2 = 5;
r1 = 3;
r2 = 5;

# Known coordinates
x4 = 1;
y4 = 0;
x5 = 4;
y5 = 0;

# Assuming servo angles are known
alpha = 90;
beta = 30;

# Point L(x1,y1)
x1 = l1*math.cos((math.pi*alpha)/180) + x4;
y1 = l1*math.sin((math.pi*alpha)/180) + y4;

# Point R(x2,y2)
x2 = r1*math.cos((math.pi*beta)/180) + x5;
y2 = r1*math.sin((math.pi*beta)/180) + y5;

# Computations needed to calculate S(x3,y3)
distLR = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));                  # Distance between L and R
angleLR = math.atan2(y2-y1, x2-x1);                                     # Angle between L and R, relative to x-axis
anglePhi = math.acos((distLR*distLR + l2*l2 - r2*r2)/(2*distLR*l2));    # Angle of l2, relative to distLR

# Point S(x3,y3)
x3 = l2*math.cos(angleLR + anglePhi) + x1;
y3 = l2*math.sin(angleLR + anglePhi) + y1;

# compute l3 and r3
distBLS = math.sqrt((x3-x4)*(x3-x4) + (y3-y4)*(y3-y4));                 # length l3
distBRS = math.sqrt((x3-x5)*(x3-x5) + (y3-y5)*(y3-y5));                 # length r3

# Define lines l1, l2, r1, r2
l1 = np.array([[x4, y4],[x1, y1]]);
l2 = np.array([[x1, y1],[x3, y3]]);
l3 = np.array([[x4, y4],[x3, y3]]);
r1 = np.array([[x5, y5],[x2, y2]]);
r2 = np.array([[x2, y2],[x3, y3]]);
r3 = np.array([[x5, y5],[x3, y3]]);


# Plotting
markerSize = 25;

plt.figure()

# lines l1, l2, r1, r2
plt.plot(l1[:,0], l1[:,1], color='black', label='l1')
plt.plot(l2[:,0], l2[:,1], color='black', label='l2')
plt.plot(l3[:,0], l3[:,1], color='black', linestyle='--', label='l3')
plt.plot(r1[:,0], r1[:,1], color='black', label='r1')
plt.plot(r2[:,0], r2[:,1], color='black', label='r2')
plt.plot(r3[:,0], r3[:,1], color='black', linestyle='--', label='r3')

# points BL, BR, L, R, S
plt.plot(x4, y4, '.', color='black', markersize=markerSize, label='BL')
plt.plot(x5, y5, '.', color='blue', markersize=markerSize, label='BR')
plt.plot(x1, y1, '.', color='red', markersize=markerSize, label='L')
plt.plot(x2, y2, '.', color='orange', markersize=markerSize, label='R')
plt.plot(x3, y3, '.', color='green', markersize=markerSize, label='S')




# Make plot fancy
plt.grid(True)
plt.xlabel('x')
plt.ylabel('y')
#plt.xlim([0, 5])
#plt.ylim([0, 10])
plt.title('Tekenrobot')
plt.legend()

print("Done!");
plt.show()


