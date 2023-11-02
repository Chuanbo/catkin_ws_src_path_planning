#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt


# 保存瞬时空闲率的平均值和最大值随时间变化的文件名称 
timeidleness_filename = "results/grid_4_timeidleness.txt"

timeidleness_file = open(timeidleness_filename, 'r')

time_list = []
avg_list = []
max_list = []

for line in timeidleness_file:
    line = line.rstrip()  # remove end-of-line
    if len(line) == 0:
        continue  # 跳过空行 
    parts = line.split(' ')
    that_time = float(parts[0])
    idleness_avg = float(parts[1])
    idleness_max = float(parts[2])
    # 将数据加入列表 
    time_list.append(that_time)
    avg_list.append(idleness_avg)
    max_list.append(idleness_max)

timeidleness_file.close()


x = np.array( time_list )
y1 = np.array( avg_list )
y2 = np.array( max_list )

plt.scatter(x, y1, color='red', label='average')
plt.scatter(x, y2, color='blue', label='maximum')
plt.xlabel('Time')
plt.ylabel('Idleness')
plt.title('The instantaneous idleness along time')
plt.grid(True)
plt.legend()

plt.show()
