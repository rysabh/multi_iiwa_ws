import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_and_plot(file_paths, skiprows=None, col_names=None):
    fig, axs = plt.subplots(len(file_paths), 1, figsize=(10, 12))
    
    if len(file_paths) == 1:
        axs = [axs]

    # Initialize variables to store the first plot's range
    first_plot_min_time, first_plot_max_time = None, None
    first_plot_min_force, first_plot_max_force = None, None

    for i, file_path in enumerate(file_paths):
        data = pd.read_csv(file_path, skiprows=skiprows[i] if skiprows[i] is not None else None)
        data = data.replace([np.inf, -np.inf], np.nan).dropna(subset=[col_names[i]['time'], col_names[i]['fx'], col_names[i]['fy'], col_names[i]['fz']])

        time_stamps = np.array(data[col_names[i]['time']])
        FX = np.array(data[col_names[i]['fx']])
        FY = np.array(data[col_names[i]['fy']])
        FZ = np.array(data[col_names[i]['fz']])

        if i == 0:  # Only set range based on the first plot
            first_plot_min_time = time_stamps.min()
            first_plot_max_time = time_stamps.max()
            first_plot_min_force = min(FX.min(), FY.min(), FZ.min())
            first_plot_max_force = max(FX.max(), FY.max(), FZ.max())

        axs[i].plot(time_stamps, FX, label='FX')
        axs[i].plot(time_stamps, FY, label='FY')
        axs[i].plot(time_stamps, FZ, label='FZ')
        
        axs[i].set_title(file_path.split('/')[-1])
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('Force (N)')
        axs[i].legend()
        axs[i].grid(True)

    # Set axis limits for all plots based on the first plot's data
    for ax in axs:
        ax.set_xlim(first_plot_min_time, first_plot_max_time)
        ax.set_ylim(first_plot_min_force, first_plot_max_force)

    plt.tight_layout()
    plt.show()

file_paths = [
    '/home/battery/Downloads/segmented_edge_3/ft_187_edge_3_step_3_force_state_cleaned.csv',
    '/home/battery/Downloads/takes/ft_187.csv'
]

skiprows = [2, None]

col_names = [
    {'time': 'Time', 'fx': 'chisel_FX', 'fy': 'chisel_FY', 'fz': 'chisel_FZ'},
    {'time': 'Time Elapsed (s)', 'fx': 'Fx (N)', 'fy': 'Fy (N)', 'fz': 'Fz (N)'}
]

load_and_plot(file_paths, skiprows, col_names)
