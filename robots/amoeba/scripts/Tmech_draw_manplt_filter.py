import pandas as pd
import numpy as np
import scienceplots
import matplotlib.pyplot as plt
import argparse

legend_alpha = 0.5


def calculate_rmse(t, x, t_ref, x_ref, is_yaw=False):
    x_ref_interp = np.interp(t, t_ref, x_ref)
    if is_yaw:
        # calculate the RMSE for yaw
        error = np.minimum(np.abs(x - x_ref_interp), 2 * np.pi - np.abs(x - x_ref_interp))
    else:
        error = x - x_ref_interp

    rmse_x = np.sqrt(np.mean(error**2))
    return rmse_x


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
            "/beetle1/joint_states/extendable_joint1/position",
            "/beetle1/joint_states/extendable_joint2/position",
            "/beetle1/joint_states/extendable_joint3/position",
            "/beetle1/joint_states/extendable_joint4/position",
        ]
    ]
    data_extendable_links_len = data_extendable_links_len.dropna()

    # extend torque length
    data_extend_torque = data[
        [
            "__time",
            # "/beetle1/servo/states/servos[4]/load",
            "/beetle1/servo/states/servos[4]/angle",
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
        plt.subplot(5, 2, 1)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        x_ref = np.array(data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/x"])
        plt.plot(t_ref, x_ref, label="ref", linestyle="--", color=color_ref)

        t = np.array(data_xyz["__time"]) - t_bias
        x = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/x"])
        plt.plot(t, x, label="real", color=color_real)

        plt.legend(framealpha=legend_alpha)
        plt.ylabel("X (m)", fontsize=label_size)
        # t_ref_start = t_ref[0]
        # t_ref_end = t_ref[-1]
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # calculate RMSE
        rmse_x = calculate_rmse(t, x, t_ref, x_ref)
        print(f"RMSE X (m): {rmse_x}")

        # --------------------------------
        plt.subplot(5, 2, 2)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        roll_ref = np.array(data_euler_ref["roll"])
        plt.plot(t_ref, roll_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)

        t = np.array(data_euler["__time"]) - t_bias
        roll = np.array(data_euler["roll"])
        plt.plot(t, roll * 180 / np.pi, label="real", color=color_real)
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        plt.ylabel("Roll (deg)", fontsize=label_size)

        # calculate RMSE
        rmse_roll = calculate_rmse(t, roll, t_ref, roll_ref)
        print(f"RMSE Roll (rad): {rmse_roll}")
        print(f"RMSE Roll (deg): {rmse_roll * 180 / np.pi}")

        # --------------------------------
        plt.subplot(5, 2, 3)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        y_ref = np.array(data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/y"])
        plt.plot(t_ref, y_ref, label="ref", linestyle="--", color=color_ref)

        t = np.array(data_xyz["__time"]) - t_bias
        y = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/y"])
        plt.plot(t, y, label="Y", color=color_real)
        plt.ylabel("Y (m)", fontsize=label_size)
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # calculate RMSE
        rmse_y = calculate_rmse(t, y, t_ref, y_ref)
        print(f"RMSE Y (m): {rmse_y}")

        # --------------------------------
        plt.subplot(5, 2, 4)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        pitch_ref = np.array(data_euler_ref["pitch"])
        plt.plot(t_ref, pitch_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)

        t = np.array(data_euler["__time"]) - t_bias
        pitch = np.array(data_euler["pitch"])
        plt.plot(t, pitch * 180 / np.pi, label="real", color=color_real)
        plt.ylabel("Pitch (deg)", fontsize=label_size)
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # calculate RMSE
        rmse_pitch = calculate_rmse(t, pitch, t_ref, pitch_ref)
        print(f"RMSE Pitch (rad): {rmse_pitch}")
        print(f"RMSE Pitch (deg): {rmse_pitch * 180 / np.pi}")

        # --------------------------------
        plt.subplot(5, 2, 5)
        t_ref = np.array(data_xyz_ref["__time"]) - t_bias
        z_ref = np.array(data_xyz_ref["/beetle1/nmpc/viz_ref/poses[0]/position/z"])
        plt.plot(t_ref, z_ref, label="ref", linestyle="--", color=color_ref)

        t = np.array(data_xyz["__time"]) - t_bias
        z = np.array(data_xyz["/beetle1/uav/cog/odom/pose/pose/position/z"])

        plt.plot(t, z, label="Z", color=color_real)
        plt.ylabel("Z (m)", fontsize=label_size)
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # calculate RMSE
        rmse_z = calculate_rmse(t, z, t_ref, z_ref)
        print(f"RMSE Z (m): {rmse_z}")

        # --------------------------------
        plt.subplot(5, 2, 6)
        t_ref = np.array(data_euler_ref["__time"]) - t_bias
        yaw_ref = np.array(data_euler_ref["yaw"])
        # if yaw_ref has a jump, we need to fix it
        for i in range(1, len(yaw_ref)):
            if yaw_ref[i] - yaw_ref[i - 1] > np.pi:
                yaw_ref[i:] -= 2 * np.pi
            elif yaw_ref[i] - yaw_ref[i - 1] < -np.pi:
                yaw_ref[i:] += 2 * np.pi
        plt.plot(t_ref, yaw_ref * 180 / np.pi, label="ref", linestyle="--", color=color_ref)

        t = np.array(data_euler["__time"]) - t_bias
        yaw = np.array(data_euler["yaw"])
        # if yaw has a jump, we need to fix it
        for i in range(1, len(yaw)):
            if yaw[i] - yaw[i - 1] > np.pi:
                yaw[i:] -= 2 * np.pi
            elif yaw[i] - yaw[i - 1] < -np.pi:
                yaw[i:] += 2 * np.pi
        plt.plot(t, yaw * 180 / np.pi, label="real", color=color_real)
        plt.ylabel("Yaw (deg)", fontsize=label_size)
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # calculate RMSE
        rmse_yaw = calculate_rmse(t, yaw, t_ref, yaw_ref, is_yaw=True)
        print(f"RMSE Yaw (rad): {rmse_yaw}")
        print(f"RMSE Yaw (deg): {rmse_yaw * 180 / np.pi}")

        # --------------------------------
        plt.subplot(5, 2, 7)
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
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="upper left")
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # --------------------------------
        plt.subplot(5, 2, 8)
        t = np.array(data_servo_angle_cmd["__time"]) - t_bias
        servo1 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal1/position"]) * 180 / np.pi
        plt.plot(t, servo1, label="$\\alpha_{c1}$")
        servo2 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal2/position"]) * 180 / np.pi
        plt.plot(t, servo2, label="$\\alpha_{c2}$")
        servo3 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal3/position"]) * 180 / np.pi
        plt.plot(t, servo3, label="$\\alpha_{c3}$")
        servo4 = np.array(data_servo_angle_cmd["/beetle1/gimbals_ctrl/gimbal4/position"]) * 180 / np.pi
        plt.plot(t, servo4, label="$\\alpha_{c4}$")
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        plt.ylabel("Servo Cmd (deg)", fontsize=label_size)
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="upper left")

        # --------------------------------
        # Subplot (5,1): Extendable joint lengths
        plt.subplot(5, 2, 9)
        t = np.array(data_extendable_links_len["__time"]) - t_bias
        joint1 = 0.2 + np.array(data_extendable_links_len["/beetle1/joint_states/extendable_joint1/position"])
        plt.plot(t, joint1, label="$a_1$")
        joint2 = 0.2 + np.array(data_extendable_links_len["/beetle1/joint_states/extendable_joint2/position"])
        plt.plot(t, joint2, label="$a_2$")
        joint3 = 0.2 + np.array(data_extendable_links_len["/beetle1/joint_states/extendable_joint3/position"])
        plt.plot(t, joint3, label="$a_3$")
        joint4 = 0.2 + np.array(data_extendable_links_len["/beetle1/joint_states/extendable_joint4/position"])
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)
        plt.plot(t, joint4, label="$a_4$")
        plt.ylabel("Rotor pos(m)", fontsize=label_size)
        plt.xlabel("Time (s)", fontsize=label_size)
        plt.legend(framealpha=legend_alpha, loc="upper left")

        # --------------------------------
        # Subplot (5,2): Extend torque
        plt.subplot(5, 2, 10)
        t = np.array(data_extend_torque["__time"]) - t_bias
        # torque = np.array(data_extend_torque["/beetle1/servo/states/servos[4]/load"])

        # only for valve task
        # Set invalid torque values to 0 instead of deleting them
        torque = np.array(data_extend_torque["/beetle1/servo/states/servos[4]/angle"])
        
        # remove servo value
        # # --------------------valve
        # task_nominal = 150
        # task_diff = 250
        # --------------------grasp
        task_nominal = 200
        task_diff = 200
        mask = np.abs(torque) > task_nominal
        torque[mask] = 0
        
        # Step 2: Compare adjacent values, set larger absolute value to 0 if difference > 400
        for i in range(1, len(torque)):
            # Skip if either current or previous value is already 0 (from step 1)
            if torque[i-1] == 0 or torque[i] == 0:
                continue
                
            diff = abs(torque[i] - torque[i-1])
            if diff > task_diff:
                # Set the one with larger absolute value to 0
                if abs(torque[i]) > abs(torque[i-1]):
                    torque[i] = 0
                else:
                    torque[i-1] = 0
        
        # Step 3: Fill gaps between non-zero values
        i = 0
        while i < len(torque):
            # Find next non-zero value A
            if torque[i] != 0:
                A = torque[i]
                ai = i
                
                # Search for next non-zero value B within next 20 values
                found_B = False
                for j in range(ai + 1, min(ai + 21, len(torque))):
                    if torque[j] != 0:
                        # Found non-zero value B
                        bi = j
                        # Fill values from ai+1 to bi-1 with A
                        for k in range(ai + 1, bi):
                            torque[k] = A
                        
                        # B becomes the new A, continue from B
                        i = bi
                        found_B = True
                        break
                
                if not found_B:
                    # No non-zero value found within 20 values, abandon current A
                    i = ai + 1
            else:
                i += 1
        


        plt.plot(t, torque)
        plt.ylabel("Torque $(N\cdot m)$", fontsize=label_size)
        plt.xlabel("Time (s)", fontsize=label_size)
        # ref_traj duration shaded area
        plt.axvspan(t_ref_start, t_ref_end, alpha=0.2, color='orange', zorder=0)

        # --------------------------------
        plt.tight_layout()
        # make the subplots very compact
        fig.subplots_adjust(hspace=0.2)
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

    # grasp
    t_ref_start = 35.0  # seconds
    t_ref_end = 80.0  # seconds
    # valve
    t_ref_start = 5.0  # seconds
    t_ref_end = 35.0  # seconds

    main(args.file_path, args.type, t_ref_start, t_ref_end)
