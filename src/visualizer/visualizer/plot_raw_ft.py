import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

file_path = '/home/battery/Downloads/takes/ft_002.csv'
data = pd.read_csv(file_path)

time_stamps = np.array(data['Time Elapsed (s)'])
FX = np.array(data['Fx (N)'])
FY = np.array(data['Fy (N)'])
FZ = np.array(data['Fz (N)'])

plt.figure(figsize=(10, 6))

plt.plot(time_stamps, FX, label='FX', )
plt.plot(time_stamps, FY, label='FY', )
plt.plot(time_stamps, FZ, label='FZ', )

plt.title(file_path.split('/')[-1])
plt.xlabel('Time (s)')
plt.ylabel('Force (N)')
plt.legend()

plt.grid(True)
plt.show()