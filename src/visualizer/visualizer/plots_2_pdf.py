import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.backends.backend_pdf import PdfPages

def load_and_plot(file_path1, file_path2, pdf):
    fig, axs = plt.subplots(2, 1, figsize=(10, 12))

    files_data = [
        (file_path1, 2, {'time': 'Time', 'fx': 'chisel_FX', 'fy': 'chisel_FY', 'fz': 'chisel_FZ'}),
        (file_path2, None, {'time': 'Time Elapsed (s)', 'fx': 'Fx (N)', 'fy': 'Fy (N)', 'fz': 'Fz (N)'})
    ]
    first_plot_min_time, first_plot_max_time = None, None
    first_plot_min_force, first_plot_max_force = None, None

    for i, (file_path, skiprows, col_names) in enumerate(files_data):
        data = pd.read_csv(file_path, skiprows=skiprows if skiprows is not None else None)
        data = data.replace([np.inf, -np.inf], np.nan).dropna(subset=[col_names['time'], col_names['fx'], col_names['fy'], col_names['fz']])

        time_stamps = np.array(data[col_names['time']])
        FX = np.array(data[col_names['fx']])
        FY = np.array(data[col_names['fy']])
        FZ = np.array(data[col_names['fz']])

        if i == 0:  # Set the range based on the first plot
            first_plot_min_time = time_stamps.min()
            first_plot_max_time = time_stamps.max()
            first_plot_min_force = min(FX.min(), FY.min(), FZ.min())
            first_plot_max_force = max(FX.max(), FY.max(), FZ.max())

        axs[i].plot(time_stamps, FX, label='FX')
        axs[i].plot(time_stamps, FY, label='FY')
        axs[i].plot(time_stamps, FZ, label='FZ')

        axs[i].set_title(os.path.basename(file_path))
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('Force (N)')
        axs[i].legend()
        axs[i].grid(True)

    for ax in axs:
        ax.set_xlim(first_plot_min_time, first_plot_max_time)
        ax.set_ylim(first_plot_min_force, first_plot_max_force)

    plt.tight_layout()
    pdf.savefig(fig)
    plt.close()

def find_and_plot(dir_edge, dir_takes, output_dir):
    pdf_path = os.path.join(output_dir, 'edge_3_plots.pdf')
    with PdfPages(pdf_path) as pdf:
        edge_files = sorted([f for f in os.listdir(dir_edge) if f.endswith('.csv')])
        processed_files = 0

        for edge_file in edge_files:
            file_number = edge_file.split('_')[1]  # Extract the file number accurately
            matching_take_file = f"ft_{file_number}.csv"
            take_file_path = os.path.join(dir_takes, matching_take_file)
            edge_file_path = os.path.join(dir_edge, edge_file)
            
            if os.path.exists(take_file_path):
                load_and_plot(edge_file_path, take_file_path, pdf)
                processed_files += 1
            else:
                print(f"Expected take file does not exist: {take_file_path}")

        print(f"Total processed files: {processed_files}")

# Specify the directories
dir_edge = '/home/battery/Downloads/edge_3_csvs'
dir_takes = '/home/battery/Downloads/takes'
output_dir = '/home/battery/Desktop/plots'

# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)

find_and_plot(dir_edge, dir_takes, output_dir)
