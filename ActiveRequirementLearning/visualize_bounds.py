import os, json
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')

### load the data
exp_name = 'survivor_problem'
data_root_dir = '/home/jliu3103/project/q_itags/Output_proof_ICRA/'
prefixs = ['0.010000', '0.100000', '0.200000', '0.300000', '0.400000', '0.500000', '0.600000', '0.700000', '0.800000',
           '0.900000', '1.000000']
exp_ids = [1, 2, 3, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18, 20, 21, 22, 62, 24, 25, 26, 27, 28, 30, 31,
           32, 33, 34, 35, 37, 38, 39, 40, 41, 42, 43, 44, 45, 47, 48, 49, 50, 51, 52, 54, 55, 58, 59, 60]
print(exp_ids[18])
json_files = []
for prefix in prefixs:
    filenames_per_alpha = [prefix + exp_name + str(idx) + '.json' for idx in exp_ids]
    jsons_per_alpha = [data_root_dir + filename for filename in filenames_per_alpha]
    json_files.append(jsons_per_alpha)

all_datapoints = []
for files in json_files:
    datapoint_per_alpha = []
    for file in files:
        with open(file) as f:
            datapoint = json.load(f)
            quality = datapoint['allocation_quality']
            datapoint_per_alpha.append(quality)

    all_datapoints.append(np.array(datapoint_per_alpha))

## all_datapoints ia matrix
## with shape (N, M)
## where N is the number of alpha values
## M is the number of problems
all_datapoints = np.array(all_datapoints)
print(all_datapoints[:, 18])
N, M = all_datapoints.shape
QROOT = np.max(all_datapoints, axis=0) # shape: number of problems
QNULL = np.min(all_datapoints, axis=0) # shape: number of problems
for idx in range(M):
    dp = all_datapoints[:, idx]
    dp = (QROOT[idx] - dp) / (QROOT[idx] - QNULL[idx])
    all_datapoints[:, idx] = dp

x = 0.01 * np.arange(100)
y = x / (1-x)
y[50:] = 1.0
plt.plot(x, y, 'orange', linewidth=2)
print(all_datapoints[:, 18])
alphas = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
# print('max', QROOT, 'min', QNULL)
for row in range(N):
    alpha = alphas[row]
    alloc_qualities = all_datapoints[row, :]
    plt.scatter(alpha * np.ones(M), alloc_qualities, c='#029386')

plt.ylim([-0.02, 1.02])
plt.xlim([-0.02, 1.02])
plt.xticks(ticks=0.1*np.arange(11), labels=['0.0', '0.1', '0.2', '0.3', '0.4', '0.5', '0.6', '0.7', '0.8', '0.9', '1.0'])
plt.savefig("images/bounds.png")
# plt.show()