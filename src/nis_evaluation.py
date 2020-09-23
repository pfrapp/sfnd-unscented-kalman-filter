# Before calling this script, issue the following command on the terminal:
# $ prepare_nis.sh

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import chi2

nis_radar = []
nis_lidar = []

car_idx = 3

with open('/Users/philipp/vm_share/nis_car{}_radar.txt'.format(car_idx), 'r') as fid:
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

# Trace over time
if False:
    fig = plt.figure(1)
    plt.clf()
    plt.plot(t, nis_radar)
    plt.grid(True)
    plt.xlabel('t')
    plt.ylabel('NIS Radar')
    plt.show()

# Histogram
num_bins = round(max(nis_radar) - min(nis_radar))*2
fig = plt.figure(2)
plt.clf()
plt.hist(nis_radar, bins=num_bins, density=True, label='Histogram')
plt.plot(x, y, label='Chi-square distribution (3 degreees of freedom)')
plt.grid(True)
plt.xlabel('NIS Radar value')
plt.ylabel('Frequency')
plt.legend()
plt.title('Car {} NIS evaluation'.format(car_idx))
plt.show()




with open('/Users/philipp/vm_share/nis_car{}_lidar.txt'.format(car_idx), 'r') as fid:
    while True:
        s = fid.readline()
        if s == '':
            break
        value = float(s.rstrip('\n'))
        nis_lidar.append(value)
        
t = list(range(len(nis_lidar)))

# Lidar has 2 degrees of freedom
df = 2
x = np.linspace(min(nis_lidar), max(nis_lidar), 100)
y = chi2.pdf(x, df)

# Trace over time
if False:
    fig = plt.figure(3)
    plt.clf()
    plt.plot(t, nis_lidar)
    plt.grid(True)
    plt.xlabel('t')
    plt.ylabel('NIS Lidar')
    plt.show()

# Histogram
num_bins = round(max(nis_lidar) - min(nis_lidar))*2
fig = plt.figure(4)
plt.clf()
plt.hist(nis_lidar, bins=num_bins, density=True, label='Histogram')
plt.plot(x, y, label='Chi-square distribution (2 degreees of freedom)')
plt.grid(True)
plt.xlabel('NIS Lidar value')
plt.ylabel('Frequency')
plt.legend()
plt.title('Car {} NIS evaluation'.format(car_idx))
plt.show()
