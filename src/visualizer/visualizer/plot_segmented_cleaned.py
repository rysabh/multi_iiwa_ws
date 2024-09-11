import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

file_path = '/home/battery/Downloads/ft_002_edge_3_step_3_force_state_cleaned(1).csv'
data = pd.read_csv(file_path, skiprows=2)

time_stamps = np.array(data['Time'])
FX = np.array(data['chisel_FX'])
FY = np.array(data['chisel_FY'])
FZ = np.array(data['chisel_FZ'])

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