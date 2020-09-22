# Before calling this script, issue the following command on the terminal:
# $ prepare_nis.sh

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import chi2

nis_radar = []
nis_lidar = []

with open('/Users/philipp/vm_share/nis_car1_radar.txt', 'r') as fid:
    while True:
        s = fid.readline()
        if s == '':
            break
        value = float(s.rstrip('\n'))
        nis_radar.append(value)
        
t = list(range(len(nis_radar)))

# Radar has 3 degrees of freedom
df = 3
x = np.linspace(min(nis_radar), max(nis_radar), 100)
y = chi2.pdf(x, df)

fig = plt.figure(1)
plt.clf()
plt.plot(t, nis_radar)
plt.grid(True)
plt.xlabel('t')
plt.ylabel('NIS Radar')
plt.show()

fig = plt.figure(2)
plt.clf()
plt.hist(nis_radar, bins=50)
plt.plot(x, y*100)
plt.grid(True)
plt.xlabel('NIS Radar value')
plt.ylabel('Frequency')
plt.show()




with open('/Users/philipp/vm_share/nis_car1_lidar.txt', 'r') as fid:
    while True:
        s = fid.readline()
        if s == '':
            break
        value = float(s.rstrip('\n'))
        nis_lidar.append(value)
        
t = list(range(len(nis_lidar)))

# Lidar has 3 degrees of freedom
df = 2
x = np.linspace(min(nis_lidar), max(nis_lidar), 100)
y = chi2.pdf(x, df)

fig = plt.figure(3)
plt.clf()
plt.plot(t, nis_lidar)
plt.grid(True)
plt.xlabel('t')
plt.ylabel('NIS Lidar')
plt.show()

fig = plt.figure(4)
plt.clf()
plt.hist(nis_lidar, bins=50)
plt.plot(x, y*100)
plt.grid(True)
plt.xlabel('NIS Lidar value')
plt.ylabel('Frequency')
plt.show()
