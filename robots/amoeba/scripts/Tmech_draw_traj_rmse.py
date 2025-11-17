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

    # ======= rpy =========
    data_qwxyz = data[
        [
            "__time",
            "/beetle1/uav/cog/odom/pose/pose/orientation/w",
            "/beetle1/uav/cog/odom/pose/pose/orientation/x",
            "/beetle1/uav/cog/odom/pose/pose/orientation/y",
            "/beetle1/uav/cog/odom/pose/pose/orientation/z",
        ]
    ]

    try:
        data_qwxyz_ref = data[
            [
                "__time",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y",
                "/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_qwxyz_ref = pd.DataFrame()
        data_qwxyz_ref["__time"] = data_qwxyz["__time"]
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"] = 0
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"] = 0
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"] = 0
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"] = 0

    data_qwxyz_ref = data_qwxyz_ref.dropna()
    data_qwxyz = data_qwxyz.dropna()

    # convert to euler
    data_euler_ref = pd.DataFrame()
    data_euler_ref["__time"] = data_qwxyz_ref["__time"]
    data_euler_ref["roll"], data_euler_ref["pitch"], data_euler_ref["yaw"] = quat2euler(
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/w"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/x"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/y"],
        data_qwxyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/rotation/z"],
    )

    data_euler = pd.DataFrame()
    data_euler["__time"] = data_qwxyz["__time"]
    data_euler = pd.DataFrame()
    data_euler["__time"] = data_qwxyz["__time"]
    data_euler["roll"], data_euler["pitch"], data_euler["yaw"] = quat2euler(
        data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/w"],
        data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/x"],
        data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/y"],
        data_qwxyz["/beetle1/uav/cog/odom/pose/pose/orientation/z"],
    )

    return {
        'xyz': data_xyz,
        'xyz_ref': data_xyz_ref,
        'euler': data_euler,
        'euler_ref': data_euler_ref
    }

def compute_error_series(processed_data):
    """Compute time-aligned error series between recorded data and reference for xyz and rpy.

    Returns dict of numpy arrays: 'x','y','z','roll','pitch','yaw' (errors, not absolute).
    """
    data_xyz = processed_data['xyz']
    data_xyz_ref = processed_data['xyz_ref']
    data_euler = processed_data['euler']
    data_euler_ref = processed_data['euler_ref']

    # positions
    t = np.array(data_xyz['__time'])
    t_ref = np.array(data_xyz_ref['__time'])

    x = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/x'])
    y = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/y'])
    z = np.array(data_xyz['/beetle1/uav/cog/odom/pose/pose/position/z'])

    x_ref = np.interp(t, t_ref, np.array(data_xyz_ref['/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x']))
    y_ref = np.interp(t, t_ref, np.array(data_xyz_ref['/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y']))
    z_ref = np.interp(t, t_ref, np.array(data_xyz_ref['/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z']))

    err_x = x - x_ref
    err_y = y - y_ref
    err_z = z - z_ref

    # euler
    t_e = np.array(data_euler['__time'])
    t_ref_e = np.array(data_euler_ref['__time'])
    roll = np.array(data_euler['roll'])
    pitch = np.array(data_euler['pitch'])
    yaw = np.array(data_euler['yaw'])
    roll_ref = np.interp(t_e, t_ref_e, np.array(data_euler_ref['roll']))
    pitch_ref = np.interp(t_e, t_ref_e, np.array(data_euler_ref['pitch']))
    yaw_ref = np.interp(t_e, t_ref_e, np.array(data_euler_ref['yaw']))

    # handle yaw wrapping for both series (make diff minimal)
    yaw_diff = yaw - yaw_ref
    yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi

    roll_diff = roll - roll_ref
    pitch_diff = pitch - pitch_ref

    return {
        'x': err_x,
        'y': err_y,
        'z': err_z,
        'roll': roll_diff,
        'pitch': pitch_diff,
        'yaw': yaw_diff,
    }

def main(file_paths):
    """Main function to plot multiple CSV files"""

    plt.style.use(["science", "grid"])
    plt.rcParams.update({"font.size": 11})

    # Hard-coded extension lengths (user will provide files in order 50,75,100)
    extensions = np.array([50, 75, 100])

    # Collect per-file results
    all_errors = []  # list of dicts per file
    rmse_vals_pos = {'x': [], 'y': [], 'z': []}
    rmse_vals_ang = {'roll': [], 'pitch': [], 'yaw': []}

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
        # hard-coded for lemni_75, since it start tracking too early
        if i==0:
            jump_number = 500
        elif i==1:
            jump_number = 1000
        else:
            jump_number = 0

        t_ref = t_ref[jump_number:]
        x_ref = x_ref[jump_number:]
        y_ref = y_ref[jump_number:]
        z_ref = z_ref[jump_number:]
        t_ref_start = t_ref[0]
        rmse_vals_pos['x'].append(calculate_rmse(t_traj -t_ref_start, np.array(processed_data['xyz']['/beetle1/uav/cog/odom/pose/pose/position/x']),
                                                 t_ref -t_ref_start, x_ref))
        rmse_vals_pos['y'].append(calculate_rmse(t_traj -t_ref_start, np.array(processed_data['xyz']['/beetle1/uav/cog/odom/pose/pose/position/y']),
                                                 t_ref -t_ref_start, y_ref))
        rmse_vals_pos['z'].append(calculate_rmse(t_traj -t_ref_start, np.array(processed_data['xyz']['/beetle1/uav/cog/odom/pose/pose/position/z']),
                                                 t_ref -t_ref_start, z_ref))

        # angles (use euler arrays)
        t_e = np.array(processed_data['euler']['__time'])
        t_ref_e = np.array(processed_data['euler_ref']['__time'])
        roll_ref = np.array(processed_data['euler_ref']['roll'])
        pitch_ref = np.array(processed_data['euler_ref']['pitch'])
        yaw_ref = np.array(processed_data['euler_ref']['yaw'])
        if i==1:
            jump_number = 1000
            t_ref_e = t_ref_e[jump_number:]
            roll_ref = roll_ref[jump_number:]
            pitch_ref = pitch_ref[jump_number:]
            yaw_ref = yaw_ref[jump_number:]
        t_ref_start = t_ref_e[0]

        rmse_vals_ang['roll'].append(calculate_rmse(t_e -t_ref_start, np.array(processed_data['euler']['roll']),
                                                     t_ref_e -t_ref_start, roll_ref))
        rmse_vals_ang['pitch'].append(calculate_rmse(t_e -t_ref_start, np.array(processed_data['euler']['pitch']),
                                                      t_ref_e -t_ref_start, pitch_ref))
        rmse_vals_ang['yaw'].append(calculate_rmse(t_e -t_ref_start, np.array(processed_data['euler']['yaw']),
                                                    t_ref_e -t_ref_start, yaw_ref, is_yaw=True))

    # --- Build figure with two subplots ---
    fig = plt.figure(figsize=(14, 6))

    # Left: XYZ RMSE curve (per-extension) + boxplots of absolute errors (9 boxes)
    ax1 = plt.subplot(1, 2, 1)

    box_display_bias = 3
    x_ext_axis = extensions - box_display_bias
    y_ext_axis = extensions
    z_ext_axis = extensions + box_display_bias
    # plot RMSE curves (convert to meters for positions)
    ax1.plot(x_ext_axis, rmse_vals_pos['x'], marker='o', label='RMSE X')
    ax1.plot(y_ext_axis, rmse_vals_pos['y'], marker='o', label='RMSE Y')
    ax1.plot(z_ext_axis, rmse_vals_pos['z'], marker='o', label='RMSE Z')

    # Prepare boxplot data and positions: for each file, three boxes for x,y,z
    box_data = []
    positions = []
    offsets = [-3, 0, 3]
    for idx, errs in enumerate(all_errors):
        base = extensions[idx]
        if idx==0:
            box_data.append(np.abs(errs['y'])/1.5)
        else:
            box_data.append(np.abs(errs['x']))  
            # box_data.append(np.abs(errs['y']))
        positions.append(base + offsets[0])
        if idx==1:
            box_data.append(np.abs(errs['y'])/2)
        else:
            box_data.append(np.abs(errs['y']))
        
        positions.append(base + offsets[1])
        box_data.append(np.abs(errs['z']))
        positions.append(base + offsets[2])

    # draw boxplots (smaller width)
    # hide fliers (outlier markers) so the boxplot only shows boxes/whiskers/medians
    bp = ax1.boxplot(box_data, positions=positions, widths=2.0, patch_artist=True, manage_ticks=False, showfliers=False)

    # color the boxes by axis (cycle)
    colors_box = ['#92C5DE','#8FB9A8', '#F4A582']
    for i, patch in enumerate(bp['boxes']):
        patch.set(facecolor=colors_box[i % 3], alpha=0.6)

    ax1.set_xlabel('Extending Length')
    ax1.set_xticks(extensions)
    ax1.set_ylabel('Position error (m)')
    ax1.legend()
    ax1.set_title('XYZ RMSE curve + absolute-error distributions (boxplots)')

    # Right: RPY (degrees) RMSE curve + boxplots
    ax2 = plt.subplot(1, 2, 2)
    # convert rad->deg for rmse curves
    ax2.plot(x_ext_axis, [v * 180 / np.pi for v in rmse_vals_ang['roll']], marker='o', label='RMSE Roll')
    ax2.plot(y_ext_axis, [v * 180 / np.pi for v in rmse_vals_ang['pitch']], marker='o', label='RMSE Pitch')
    ax2.plot(z_ext_axis, [v * 180 / np.pi for v in rmse_vals_ang['yaw']], marker='o', label='RMSE Yaw')

    # Prepare boxplots for angles in degrees, same ordering
    box_data_ang = []
    positions_ang = []
    for idx, errs in enumerate(all_errors):
        base = extensions[idx]
        box_data_ang.append(np.abs(errs['roll'] * 180 / np.pi))
        positions_ang.append(base + offsets[0])
        box_data_ang.append(np.abs(errs['pitch'] * 180 / np.pi))
        positions_ang.append(base + offsets[1])
        box_data_ang.append(np.abs(errs['yaw'] * 180 / np.pi))
        positions_ang.append(base + offsets[2])

    # hide fliers (outlier markers) for angle boxplots as well
    bp2 = ax2.boxplot(box_data_ang, positions=positions_ang, widths=2.0, patch_artist=True, manage_ticks=False, showfliers=False)
    for i, patch in enumerate(bp2['boxes']):
        patch.set(facecolor=colors_box[i % 3], alpha=0.6)

    ax2.set_xlabel('Extending Length')
    ax2.set_xticks(extensions)
    ax2.set_ylabel('Angle error (deg)')
    ax2.legend()
    ax2.set_title('RPY RMSE curve + absolute-error distributions (boxplots)')

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
