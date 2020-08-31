import matplotlib.pyplot as plt
import numpy as np
import dill as pickle
import os
import codecs
import csv

"""directory = 'results\\py_objs'

all_paths = []
all_path_names = []
for filename in os.listdir(directory):
    if filename.endswith(".list"):
        with open(directory+"\\"+filename, 'rb') as l_file:
            l = pickle.load(l_file)
            all_paths.append(l)
            all_path_names.append(filename[:-6])


plt.figure()
for path in all_paths:
    plt.scatter(path[0], path[1], s=1)
plt.show()
"""
# 2 = crash, 1 = goal
o_keys = []
o_data = []
o_path = []
with open('results\\tangent_s1_1.csv', newline='') as csvfile:
    csvReader = csv.reader(csvfile, delimiter=',')
    i = 0
    for i, row in enumerate(csvReader):
        if i == 0:
            for j, entry in enumerate(row):
                if "iteration" in entry:
                    # no more keys
                    break
                else:
                    o_keys.append(entry)
        else:
            o_data.append(row[:-1])
            with open('results\\py_objs\\' + row[-1]+'.po', 'rb') as pyf:
                path = pickle.load(pyf)
                o_path.append(path)

print(len(o_data), len(o_path))
o_data = np.array(o_data).astype(np.float)

plt.figure()
for i, path in enumerate(o_path):
    plt.scatter(path[0], path[1], s=1, label=o_keys[0]+"-"+str(o_data[i][0]))
    if o_data[i][2] == 2:
        plt.scatter(path[0][-1], path[1][-1], marker="x", s=22)
plt.legend()
plt.show()

plt.figure()
plt.plot(o_data[:,0], o_data[:,3], label="length")
plt.plot(o_data[:,0], (o_data[:,4]*100), label="smoothness")
plt.legend()
plt.xlabel(o_keys[0])
plt.show()

plt.figure()
plt.plot(o_data[:,0], o_data[:,3], label="length")
plt.plot(o_data[:,0], (o_data[:,4]*100), label="smoothness")
plt.legend()
plt.xlabel(o_keys[0])
plt.show()