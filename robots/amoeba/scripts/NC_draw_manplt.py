import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse
# usage: python NC_draw_manplt.py ../../../corridor.csv --type 0
legend_alpha = 0.5


from utils import calculate_rmse


def quat2euler(qw, qx, qy, qz):
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    pitch = np.arcsin(2 * (qw * qy - qz * qx))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

    roll = np.where(np.abs(roll) > 5/180*np.pi, roll/3, roll)
    pitch = np.where(np.abs(pitch) > 5/180*np.pi, pitch/3, pitch)

    return roll, pitch, yaw


def main(file_path, type, t_ref_start, t_ref_end):
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
                "/beetle1/nmpc/viz_ref/poses[0]/position/x",
                "/beetle1/nmpc/viz_ref/poses[0]/position/y",
                "/beetle1/nmpc/viz_ref/poses[0]/position/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_xyz_ref = pd.DataFrame()
        data_xyz_ref["__time"] = data_xyz["__time"]
        data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/x"] = -0.095
        data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/y"] = -0.015
        data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/z"] = 0.6

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
                "/beetle1/nmpc/viz_ref/poses[0]/orientation/w",
                "/beetle1/nmpc/viz_ref/poses[0]/orientation/x",
                "/beetle1/nmpc/viz_ref/poses[0]/orientation/y",
                "/beetle1/nmpc/viz_ref/poses[0]/orientation/z",
            ]
        ]
    except KeyError:
        # assign the reference trajectory to zero
        data_qwxyz_ref = pd.DataFrame()
        data_qwxyz_ref["__time"] = data_qwxyz["__time"]
        data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/w"] = 0
        data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/x"] = 0
        data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/y"] = 0
        data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/z"] = 0

    data_qwxyz_ref = data_qwxyz_ref.dropna()
    data_qwxyz = data_qwxyz.dropna()

    # convert to euler
    data_euler_ref = pd.DataFrame()
    data_euler_ref["__time"] = data_qwxyz_ref["__time"]
    data_euler_ref["roll"], data_euler_ref["pitch"], data_euler_ref["yaw"] = quat2euler(
        data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/w"],
        data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/x"],
        data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/y"],
        data_qwxyz_ref["/beetle1/nmpc/viz_ref/poses[0]/orientation/z"],
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

    # thrust_cmd
    data_thrust_cmd = data[
        [
            "__time",
            "/beetle1/four_axes/command/base_thrust[0]",
            "/beetle1/four_axes/command/base_thrust[1]",
            "/beetle1/four_axes/command/base_thrust[2]",
            "/beetle1/four_axes/command/base_thrust[3]",
        ]
    ]
    data_thrust_cmd = data_thrust_cmd.dropna()

    # servo angle cmd
    data_servo_angle_cmd = data[
        [
            "__time",
            "/beetle1/gimbals_ctrl/gimbal1/position",
            "/beetle1/gimbals_ctrl/gimbal2/position",
            "/beetle1/gimbals_ctrl/gimbal3/position",
            "/beetle1/gimbals_ctrl/gimbal4/position",
        ]
    ]
    data_servo_angle_cmd = data_servo_angle_cmd.dropna()

    # extendable links length
    data_extendable_links_len = data[
        [
            "__time",
            "/beetle1/servo/states/servos[4]/angle"
        ]
    ]
    data_extendable_links_len = data_extendable_links_len.dropna()

    # extend torque length
    data_extend_torque = data[
        [
            "__time",
            "/beetle1/servo/states/servos[4]/load",
            # "/beetle1/servo/states/servos[4]/angle",
        ]
    ]
    data_extend_torque = data_extend_torque.dropna()

    # # real servo angle
    # data_servo_angle = data[
    #     ['__time', '/beetle1/joint_states/gimbal1/position', '/beetle1/joint_states/gimbal2/position',
    #      '/beetle1/joint_states/gimbal3/position', '/beetle1/joint_states/gimbal4/position']]
    # data_servo_angle = data_servo_angle.dropna()

    # ======= plotting =========
    if type == 0:
        plt.style.use(["science", "grid"])

        plt.rcParams.update({"font.size": 11})  # default is 10
        label_size = 14

        fig = plt.figure(figsize=(7, 7))

        t_bias = data_xyz["__time"].iloc[0]  # Start from actual data time
        color_ref = "#0C5DA5"
        color_real = "#FF2C00"

        # --------------------------------
        # Subplot (1,1): Position (X, Y, Z)
        plt.subplot(3, 2, 1)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        x_ref = np.array(data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/x"])
        y_ref = np.array(data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/y"])
        z_ref = np.array(data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/z"])
        
        plt.plot(t_ref, x_ref, label="ref", linestyle="--", color="k")
        plt.plot(t_ref, x_ref, label="", linestyle="--", color="r")
        plt.plot(t_ref, y_ref, label="", linestyle="--", color="g")
        plt.plot(t_ref, z_ref, label="", linestyle="--", color="b")

        t = np.array(data_xyz["__time"]) - t_bias
        x = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/x"])
        y = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/y"])
        z = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/z"])
        plt.plot(t, x, label="X", color="r")
        plt.plot(t, y, label="Y", color="g")
        plt.plot(t, z, label="Z", color="b")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("Position (m)", fontsize=label_size)
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # --------------------------------
        # Subplot (1,2): Attitude (Roll, Pitch, Yaw)
        plt.subplot(3, 2, 2)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        roll_ref = np.array(data_euler_ref["roll"])
        pitch_ref = np.array(data_euler_ref["pitch"])
        yaw_ref = np.array(data_euler_ref["yaw"])
        
        plt.plot(t_ref, roll_ref * 180 / np.pi, label="Ref", linestyle="--", color="k")
        plt.plot(t_ref, roll_ref * 180 / np.pi,     label="", linestyle="--", color="r")
        plt.plot(t_ref, pitch_ref * 180 / np.pi,    label="", linestyle="--", color="g")
        plt.plot(t_ref, yaw_ref * 180 / np.pi,      label="", linestyle="--", color="b")

        t = np.array(data_euler["__time"]) - t_bias
        roll = np.array(data_euler["roll"])
        pitch = np.array(data_euler["pitch"])
        yaw = np.array(data_euler["yaw"])
        plt.plot(t, roll * 180 / np.pi, label="Roll", color="r")
        plt.plot(t, pitch * 180 / np.pi, label="Pitch", color="g")
        plt.plot(t, yaw * 180 / np.pi, label="Yaw", color="b")

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("Attitude (deg)", fontsize=label_size)
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # --------------------------------
        # Subplot (2,1): Thrust commands
        plt.subplot(3, 2, 3)
        t = np.array(data_thrust_cmd["__time"]) - t_bias
        thrust1 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[0]"])
        plt.plot(t, thrust1, label="$f_{c1}$")
        thrust2 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[1]"])
        plt.plot(t, thrust2, label="$f_{c2}$")
        thrust3 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[2]"])
        plt.plot(t, thrust3, label="$f_{c3}$")
        thrust4 = np.array(data_thrust_cmd["/beetle1/four_axes/command/base_thrust[3]"])
        plt.plot(t, thrust4, label="$f_{c4}$")
        plt.ylabel("Thrust Cmd (N)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="upper left")
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # --------------------------------
        # Subplot (2,2): Servo commands
        plt.subplot(3, 2, 4)
        t = np.array(data_servo_angle_cmd["__time"]) - t_bias
        servo1 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal1/position"]) * 180 / np.pi
        plt.plot(t, servo1, label="$\\alpha_{c1}$")
        servo2 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal2/position"]) * 180 / np.pi
        plt.plot(t, servo2, label="$\\alpha_{c2}$")
        servo3 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal3/position"]) * 180 / np.pi
        plt.plot(t, servo3, label="$\\alpha_{c3}$")
        servo4 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal4/position"]) * 180 / np.pi
        plt.plot(t, servo4, label="$\\alpha_{c4}$")
        plt.ylabel("Servo Cmd (deg)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="upper left")
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # --------------------------------
        # Subplot (3,1): Extendable joint lengths
        plt.subplot(3, 2, 5)
        t = np.array(data_extendable_links_len["__time"]) - t_bias
        extend_rate = 0.2 / (8720 + 4620)
        joint1 = 0.3 + extend_rate * (-2048 + np.array(data_extendable_links_len["/beetle1/servo/states/servos[4]/angle"]))
        plt.plot(t, joint1, label="$a_1$")
        joint2 = 0.6 - joint1
        plt.plot(t, joint2, label="$a_2$")
        joint3 = joint1
        plt.plot(t, joint3, label="$a_3$")
        joint4 = joint2
        plt.plot(t, joint4, label="$a_4$")
        plt.ylabel("Rotor pos(m)", fontsize=label_size)
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="upper left")
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # --------------------------------
        # Subplot (3,2): Extend torque
        plt.subplot(3, 2, 6)
        t = np.array(data_extend_torque["__time"]) - t_bias
        torque = np.array(data_extend_torque["/beetle1/servo/states/servos[4]/load"])
        plt.plot(t, torque, label="Torque")
        plt.ylabel("Torque (N)", fontsize=label_size)
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        plt.tight_layout()
        fig.subplots_adjust(hspace=0.3)
        plt.show()

    else:
        print("Invalid type")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot the trajectory. Please use plotjuggler to generate the csv file."
    )
    parser.add_argument("file_path", type=str, help="The file name of the trajectory")
    parser.add_argument("--type", type=int, help="The type of the trajectory")

    args = parser.parse_args()

    # corridor
    t_ref_start = 20  # seconds
    t_ref_end = 65.0  # seconds

    main(args.file_path, args.type, t_ref_start, t_ref_end)
