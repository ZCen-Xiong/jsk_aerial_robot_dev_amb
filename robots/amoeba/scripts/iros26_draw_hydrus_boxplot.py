# this file has been hardcoded, use after check
import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse
from utils import calculate_rmse
legend_alpha = 0.5


def quat2euler(qw, qx, qy, qz):
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    pitch = np.arcsin(2 * (qw * qy - qz * qx))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

    roll = np.where(np.abs(roll) > 5/180*np.pi, roll/3, roll)
    pitch = np.where(np.abs(pitch) > 5/180*np.pi, pitch/3, pitch)

    return roll, pitch, yaw

def load_and_process_data(file_path):
    """Extract and process data from a single CSV file"""
    # Load the data from csv file
    data = pd.read_csv(file_path)

    # ======= xyz =========
    data_xyz = data[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/position/x",
            "/beetle1/uav/cog/odom/pose/pose/position/y",
            "/beetle1/uav/cog/odom/pose/pose/position/z",
        ]
    ]

    try:
        data_xyz_ref = data[
            [
                "__time",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_xyz_ref = pd.DataFrame()
        data_xyz_ref["__time"] = data_xyz["__time"]
        data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x"] = -0.095
        data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y"] = -0.015
        data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z"] = 0.6

    data_xyz = data_xyz.dropna()
    data_xyz_ref = data_xyz_ref.dropna()

    return {
        'xyz': data_xyz,
        'xyz_ref': data_xyz_ref,
    }

def compute_error_series(processed_data):
    """Compute time-aligned error series between recorded data and reference for xyz and rpy.

    Returns dict of numpy arrays: 'x','y','z','roll','pitch','yaw' (errors, not absolute).
    """
    data_xyz = processed_data['xyz']
    data_xyz_ref = processed_data['xyz_ref']
    # positions
    t = np.array(data_xyz['__time'])
    t_ref = np.array(data_xyz_ref['__time'])

    x = np.interp(t_ref, t, np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x']))
    y = np.interp(t_ref, t, np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y']))
    z = np.interp(t_ref, t, np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z']))

    x_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x'])
    y_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y'])
    z_ref = np.array(data_xyz_ref['/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z'])

    err_x = x - x_ref
    err_y = y - y_ref
    err_z = z - z_ref

    return {
        'x': err_x,
        'y': err_y,
        'z': err_z,
    }

def main(file_paths):
    """Main function to plot multiple CSV files"""

    # plt.style.use(["science", "grid"])
    # plt.rcParams.update({"font.size": 11})
        # color the boxes by axis (cycle)
    colors_box = ['#E74C3C','#09D45A', "#0F66D0", '#E7E700']
    # Hard-coded pos for PD 
    x_label_pos = np.array([1, 2]) *10


    # Collect per-file results
    all_errors = []  # list of dicts per file
    rmse_vals_pos = {'x': [], 'y': [], 'z': []}

    for i, file_path in enumerate(file_paths):
        print(f"\nProcessing file {i+1}: {file_path}")
        processed_data = load_and_process_data(file_path)

        # compute error series (time-aligned)
        errs = compute_error_series(processed_data)
        all_errors.append(errs)

        # compute scalar RMSE per axis (use time arrays from processed data)
        # positions
        t_traj = np.array(processed_data['xyz']['__time'])
        t_ref = np.array(processed_data['xyz_ref']['__time'])
        x_ref = np.array(processed_data['xyz_ref']['/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x'])
        y_ref = np.array(processed_data['xyz_ref']['/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y'])
        z_ref = np.array(processed_data['xyz_ref']['/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z'])
        # pd.DataFrame({'y_ref': y_ref}).to_csv('y_error_ext75.csv', index=False)
        t_ref_start = t_ref[0]
        rmse_vals_pos['x'].append(calculate_rmse(t_traj -t_ref_start, np.array(processed_data['xyz']['/beetle1/uav/cog/odom/pose/pose/position/x']),
                                                 t_ref -t_ref_start, x_ref))
        rmse_vals_pos['y'].append(calculate_rmse(t_traj -t_ref_start, np.array(processed_data['xyz']['/beetle1/uav/cog/odom/pose/pose/position/y']),
                                                 t_ref -t_ref_start, y_ref))
        rmse_vals_pos['z'].append(calculate_rmse(t_traj -t_ref_start, np.array(processed_data['xyz']['/beetle1/uav/cog/odom/pose/pose/position/z']),
                                                 t_ref -t_ref_start, z_ref))


    # --- Build figure with two subplots ---
    plt.figure(figsize=(14, 6))

    # Left: XYZ RMSE curve (per-extension) + boxplots of absolute errors (9 boxes)
    # Prepare boxplot data and positions: for each file, three boxes for x,y,z
    box_data = []
    positions = []
    offsets = [-3, -1, 1, 3]
    controller_list =  ['PD', 'IMP', 'ADM','HYB']
    for idx, errs in enumerate(all_errors):
        base = x_label_pos[0]       
        box_data.append(np.abs(errs['y']))
        positions.append(base + offsets[idx])
        base = x_label_pos[1]
        box_data.append(np.abs(errs['z']))
        positions.append(base + offsets[idx])

    # draw boxplots (smaller width)
    # hide fliers (outlier markers) so the boxplot only shows boxes/whiskers/medians
    bp = plt.boxplot(box_data, positions=positions, widths=2.0, patch_artist=True, 
                     manage_ticks=False, showfliers=True)
    # False, whis=5

    for i, patch in enumerate(bp['boxes']):
        # box sequence is actually: 0 2 4 6, 1 3 5 7, (x then y)
        # need map them to 0,1,2,3 for coloring
        color_idx = i // 2
        patch.set(facecolor=colors_box[color_idx], alpha=0.6)

    # Create legend handles manually
    from matplotlib.patches import Patch
    legend_handles = [Patch(facecolor=colors_box[i], alpha=0.6, label=controller_list[i]) 
                      for i in range(len(controller_list))]

    plt.xlabel('Axis', fontsize=12)
    # plt.xticks(x_label_pos, ['PD', 'IMP', 'ADM','HYB'])
    plt.xticks(x_label_pos, ['x', 'y'])
    plt.ylabel('Position error (m)', fontsize=12)
    plt.legend(handles=legend_handles, fontsize=12, framealpha=legend_alpha)  # Set legend font size
    # plt.title('XYZ absolute-error distributions', fontsize=14, fontweight='bold')

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot multiple trajectories on the same figure. Please use plotjuggler to generate the csv files."
    )
    parser.add_argument("file_paths", nargs='+', help="The file names of the trajectories (space separated)")
    # parser.add_argument("--type", type=int, default=0, help="The type of the trajectory")

    args = parser.parse_args()

    main(args.file_paths)
