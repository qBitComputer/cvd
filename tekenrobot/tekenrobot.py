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
alpha = 135;
beta = 45;

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
l3 = math.sqrt((x3-x4)*(x3-x4) + (y3-y4)*(y3-y4));                      # length l3
r3 = math.sqrt((x3-x5)*(x3-x5) + (y3-y5)*(y3-y5));                      # length r3

# Define lines l1, l2, r1, r2
lineL1 = np.array([[x4, y4],[x1, y1]]);
lineL2 = np.array([[x1, y1],[x3, y3]]);
lineL3 = np.array([[x4, y4],[x3, y3]]);
lineR1 = np.array([[x5, y5],[x2, y2]]);
lineR2 = np.array([[x2, y2],[x3, y3]]);
lineR3 = np.array([[x5, y5],[x3, y3]]);


# Is it possible to compute L(x1,y1) and R(x2,y2) from S(x3,y3)?
angleL3 = math.atan2(y3-y4, x3-x4);                                     # angle between l3 and x-axis
print(angleL3*180/math.pi);

angleR3 = math.atan2(y3-y5,x3-x5);
print(angleR3*180/math.pi);                                             # angle between r3 and x-axis

angleL = math.acos((l1*l1 + l3*l3 - l2*l2)/(2*l1*l3));                  # angle between l1 and l3
print(angleL*180/math.pi);

angleR = math.acos((r1*r1 + r3*r3 - r2*r2)/(2*r1*r3));                  # angle between r1 and r3   
print(angleR*180/math.pi);

compAngleAlpha = angleL + angleL3;                                      # Computed angle alpha, based on point S(x3,y3) and l1, l2
compAngleBeta = angleR3 - angleR ;                                      # Computed angle beta, based on point S(x3,y3) and r1, r2
print(compAngleAlpha*180/math.pi);
print(compAngleBeta*180/math.pi);

# Plotting
markerSize = 25;

plt.figure()

# lines l1, l2, r1, r2
plt.plot(lineL1[:,0], lineL1[:,1], color='black', label='l1')
plt.plot(lineL2[:,0], lineL2[:,1], color='black', label='l2')
plt.plot(lineL3[:,0], lineL3[:,1], color='black', linestyle='--', label='l3')
plt.plot(lineR1[:,0], lineR1[:,1], color='black', label='r1')
plt.plot(lineR2[:,0], lineR2[:,1], color='black', label='r2')
plt.plot(lineR3[:,0], lineR3[:,1], color='black', linestyle='--', label='r3')

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


