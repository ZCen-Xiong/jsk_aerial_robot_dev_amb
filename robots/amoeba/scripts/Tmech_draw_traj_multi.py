# this file has been hardcoded, use after check
import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
from utils import calculate_rmse
import argparse

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

def plot_trajectory_on_figure(fig, processed_data, color, label_suffix="", plot_ref=True, global_t_bias=None):
    """Plot trajectory data on existing figure"""
    data_xyz = processed_data['xyz']
    data_xyz_ref = processed_data['xyz_ref']
    data_euler = processed_data['euler']
    data_euler_ref = processed_data['euler_ref']
    
    # Use individual file's reference start as time bias for alignment
    t_ref_times = np.array(data_xyz_ref["__time"])
    t_ref_start_epoch = t_ref_times[0]  # This file's ref start time
    
    # Calculate time shift: how much earlier/later this file's ref starts compared to global reference
    if global_t_bias is None:
        time_shift = 0  # First file is the reference
        t_bias = t_ref_start_epoch
    else:
        time_shift = t_ref_start_epoch - global_t_bias  # Positive if this file starts later
        t_bias = t_ref_start_epoch  # Use this file's own ref start as local bias
    
    color_ref = "#05233D"
    label_size = 14

    # -------------------------------- X Position
    plt.figure(fig.number)
    plt.subplot(3, 2, 1)
    
    if plot_ref:
        t_ref = t_ref_times - t_bias + time_shift
        x_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/x"])
        plt.plot(t_ref, x_ref, label="ref", linestyle="--", color=color_ref)
        
        # ref_traj duration shaded area (only plot once)
        t_ref_start = t_ref[0]
        t_ref_end = t_ref[-1]
        # plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

    # Shift trajectory data by the same amount
    t_traj_times = np.array(data_xyz["__time"])
    t = t_traj_times - t_bias + time_shift
    x = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/x"])
    plt.plot(t, x, label=f"real{label_suffix}", color=color)

    if plot_ref:
        plt.legend(framealpha=legend_alpha)
        plt.ylabel("X (m)", fontsize=label_size)
    
    # calculate RMSE (use original alignment for accuracy)
    if plot_ref:
        t_traj_aligned = t_traj_times - t_ref_start_epoch
        t_ref_aligned = t_ref_times - t_ref_start_epoch
        rmse_x = calculate_rmse(t_traj_aligned, x, t_ref_aligned, x_ref)
        print(f"RMSE X{label_suffix} (m): {rmse_x}")

    # -------------------------------- Roll
    plt.subplot(3, 2, 2)
    
    if plot_ref:
        t_ref = np.array(data_euler_ref["__time"]) - t_bias + time_shift
        roll_ref = np.array(data_euler_ref["roll"])
        plt.plot(t_ref, roll_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)
        # plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

    t = np.array(data_euler["__time"]) - t_bias + time_shift
    roll = np.array(data_euler["roll"])
    plt.plot(t, roll * 180 / np.pi, label=f"real{label_suffix}", color=color)
    
    if plot_ref:
        plt.ylabel("Roll (deg)", fontsize=label_size)
        # calculate RMSE
        t_traj_aligned = np.array(data_euler["__time"]) - t_ref_start_epoch
        t_ref_aligned = np.array(data_euler_ref["__time"]) - t_ref_start_epoch
        rmse_roll = calculate_rmse(t_traj_aligned, roll, t_ref_aligned, roll_ref)
        print(f"RMSE Roll{label_suffix} (rad): {rmse_roll}")
        print(f"RMSE Roll{label_suffix} (deg): {rmse_roll * 180 / np.pi}")

    # -------------------------------- Y Position
    plt.subplot(3, 2, 3)
    
    if plot_ref:
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias + time_shift
        y_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/y"])
        plt.plot(t_ref, y_ref, label="ref", linestyle="--", color=color_ref)
        # plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

    t = np.array(data_xyz["__time"]) - t_bias + time_shift
    y = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/y"])
    plt.plot(t, y, label=f"real{label_suffix}", color=color)
    
    if plot_ref:
        plt.ylabel("Y (m)", fontsize=label_size)
        # calculate RMSE
        t_traj_aligned = np.array(data_xyz["__time"]) - t_ref_start_epoch
        t_ref_aligned = np.array(data_xyz_ref["__time"]) - t_ref_start_epoch
        rmse_y = calculate_rmse(t_traj_aligned, y, t_ref_aligned, y_ref)
        print(f"RMSE Y{label_suffix} (m): {rmse_y}")

    # -------------------------------- Pitch
    plt.subplot(3, 2, 4)
    
    if plot_ref:
        t_ref = np.array(data_euler_ref["__time"]) - t_bias + time_shift
        pitch_ref = np.array(data_euler_ref["pitch"])
        plt.plot(t_ref, pitch_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)
        # plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

    t = np.array(data_euler["__time"]) - t_bias + time_shift
    pitch = np.array(data_euler["pitch"])
    plt.plot(t, pitch * 180 / np.pi, label=f"real{label_suffix}", color=color)
    
    if plot_ref:
        plt.ylabel("Pitch (deg)", fontsize=label_size)
        # calculate RMSE
        t_traj_aligned = np.array(data_euler["__time"]) - t_ref_start_epoch
        t_ref_aligned = np.array(data_euler_ref["__time"]) - t_ref_start_epoch
        rmse_pitch = calculate_rmse(t_traj_aligned, pitch, t_ref_aligned, pitch_ref)
        print(f"RMSE Pitch{label_suffix} (rad): {rmse_pitch}")
        print(f"RMSE Pitch{label_suffix} (deg): {rmse_pitch * 180 / np.pi}")

    # -------------------------------- Z Position
    plt.subplot(3, 2, 5)
    
    if plot_ref:
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias + time_shift
        z_ref = np.array(data_xyz_ref["/beetle1/set_ref_traj/points[0]/transforms[0]/translation/z"])
        plt.plot(t_ref, z_ref, label="ref", linestyle="--", color=color_ref)
        # plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

    t = np.array(data_xyz["__time"]) - t_bias + time_shift
    z = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/z"])
    plt.plot(t, z, label=f"real{label_suffix}", color=color)
    
    if plot_ref:
        plt.ylabel("Z (m)", fontsize=label_size)
        # calculate RMSE
        t_traj_aligned = np.array(data_xyz["__time"]) - t_ref_start_epoch
        t_ref_aligned = np.array(data_xyz_ref["__time"]) - t_ref_start_epoch
        rmse_z = calculate_rmse(t_traj_aligned, z, t_ref_aligned, z_ref)
        print(f"RMSE Z{label_suffix} (m): {rmse_z}")

    # -------------------------------- Yaw
    plt.subplot(3, 2, 6)
    
    if plot_ref:
        t_ref = np.array(data_euler_ref["__time"]) - t_bias + time_shift
        yaw_ref = np.array(data_euler_ref["yaw"])
        # if yaw_ref has a jump, we need to fix it
        for i in range(1, len(yaw_ref)):
            if yaw_ref[i] - yaw_ref[i - 1] > np.pi:
                yaw_ref[i:] -= 2 * np.pi
            elif yaw_ref[i] - yaw_ref[i - 1] < -np.pi:
                yaw_ref[i:] += 2 * np.pi
        plt.plot(t_ref, yaw_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)
        # plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

    t = np.array(data_euler["__time"]) - t_bias + time_shift
    yaw = np.array(data_euler["yaw"])
    # if yaw has a jump, we need to fix it
    for i in range(1, len(yaw)):
        if yaw[i] - yaw[i - 1] > np.pi:
            yaw[i:] -= 2 * np.pi
        elif yaw[i] - yaw[i - 1] < -np.pi:
            yaw[i:] += 2 * np.pi
    plt.plot(t, yaw * 180 / np.pi, label=f"real{label_suffix}", color=color)
    
    if plot_ref:
        plt.ylabel("Yaw (deg)", fontsize=label_size)
        # calculate RMSE
        t_traj_aligned = np.array(data_euler["__time"]) - t_ref_start_epoch
        t_ref_aligned = np.array(data_euler_ref["__time"]) - t_ref_start_epoch
        rmse_yaw = calculate_rmse(t_traj_aligned, yaw, t_ref_aligned, yaw_ref, is_yaw=True)
        print(f"RMSE Yaw{label_suffix} (rad): {rmse_yaw}")
        print(f"RMSE Yaw{label_suffix} (deg): {rmse_yaw * 180 / np.pi}")

    # Add legends and x-axis limits to all subplots
    for i in range(1, 7):
        plt.subplot(3, 2, i)
        plt.legend(framealpha=legend_alpha)
        plt.xlim(left=-5)  # Set x-axis to start from 0

def main(file_paths, type_plot):
    """Main function to plot multiple CSV files"""
    if type_plot == 0:
        plt.style.use(["science", "grid"])
        plt.rcParams.update({"font.size": 11})
        
        fig = plt.figure(figsize=(7, 7))
        
        # Define colors for different files
        colors = ["#FF0000D1","#0D16C6", "#0DD10DD1", "#FF8C00D1", "#DC143C"]
        
        # Process and plot each file
        global_t_bias = None  # Will be set from first file's reference start
        
        for i, file_path in enumerate(file_paths):
            print(f"\nProcessing file {i+1}: {file_path}")
            
            # Load and process data
            processed_data = load_and_process_data(file_path)
            
            # Set global time bias from first file's reference start
            global_t_bias = processed_data['xyz_ref']["__time"].iloc[0]
            #  only for lemni_75 real, cuz i wrongly set a fixed pos for 8.03 seconds in real experiment
            if i ==1:
                global_t_bias = global_t_bias + 8.030
            print(f"Global time bias set to: {global_t_bias}")
            # Plot on the same figure
            color = colors[i % len(colors)]
            label_suffix = f" {i+1}" if len(file_paths) > 1 else ""
            plot_ref = (i == 0)  # Only plot reference for first file
            
            plot_trajectory_on_figure(fig, processed_data, color, label_suffix, plot_ref, global_t_bias)
        
        plt.tight_layout()
        fig.subplots_adjust(hspace=0.2)
        plt.show()
    
    else:
        print("Invalid type")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot multiple trajectories on the same figure. Please use plotjuggler to generate the csv files."
    )
    parser.add_argument("file_paths", nargs='+', help="The file names of the trajectories (space separated)")
    parser.add_argument("--type", type=int, default=0, help="The type of the trajectory")

    args = parser.parse_args()

    main(args.file_paths, args.type)
