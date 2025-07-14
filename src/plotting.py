import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider

import math


def quaternion_to_yaw(x, y, z, w):
    """Compute yaw from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def load_csv(folder, topic):
    path = os.path.join(folder, topic + '.csv')
    if os.path.isfile(path):
        return pd.read_csv(path)
    else:
        print(f"Warning: {path} not found.")
        return None


def plot_rosbag_csv_folder(folder_path):
    # Load all data
    arbitrator = load_csv(folder_path, '_arbitrator_control_thruster_state')
    setpoint = load_csv(folder_path, '_control_thruster_setpoint')
    goal_pose = load_csv(folder_path, '_goal_pose')
    observer_body = load_csv(folder_path, '_odometry_observer_body_velocities')
    observer = load_csv(folder_path, '_odometry_observer')
    planning_goal_pose = load_csv(folder_path, '_planning_goal_pose')

    # 1. Top-down NED position plot
    if observer is not None:
        plt.figure()
        x = observer['_pose__pose__position__x']
        y = observer['_pose__pose__position__y']
        plt.plot(y, x, color='blue', label='Actual Trajectory')

        # Heading arrows for actual trajectory
        psi_list = []
        for i in range(len(observer)):
            qx = observer['_pose__pose__orientation__x'][i]
            qy = observer['_pose__pose__orientation__y'][i]
            qz = observer['_pose__pose__orientation__z'][i]
            qw = observer['_pose__pose__orientation__w'][i]
            psi = quaternion_to_yaw(qx, qy, qz, qw)
            psi_list.append(psi)

        skip = max(1, len(x) // 5)
        for i in range(0, len(x), skip):
            dx = np.cos(psi_list[i])
            dy = np.sin(psi_list[i])
            plt.arrow(y[i], x[i], dy, dx, head_width=0.2, head_length=0.2, color='blue')

        # Add orange arrow for goal pose if available
        if planning_goal_pose is not None and len(planning_goal_pose) > 0:
            goal_x = planning_goal_pose['_pose__position__x'].iloc[-1]
            goal_y = planning_goal_pose['_pose__position__y'].iloc[-1]
            goal_qx = planning_goal_pose['_pose__orientation__x'].iloc[-1]
            goal_qy = planning_goal_pose['_pose__orientation__y'].iloc[-1]
            goal_qz = planning_goal_pose['_pose__orientation__z'].iloc[-1]
            goal_qw = planning_goal_pose['_pose__orientation__w'].iloc[-1]
            goal_psi = quaternion_to_yaw(goal_qx, goal_qy, goal_qz, goal_qw)

            # Plot goal as orange arrow
            plt.arrow(goal_y, goal_x,
                    0.8 * np.sin(goal_psi),
                    0.8 * np.cos(goal_psi),
                    head_width=0.3, head_length=0.4,
                    color='orange', label='Desired Goal')

        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.title('Top-down NED Plot')
        plt.legend()
        plt.axis('equal')
        # plt.gca().invert_xaxis()



    # 2. Body velocities
    if observer_body is not None:
        plt.figure()
        time = observer_body['timestamp'] - observer_body['timestamp'].iloc[0]
        time = time * 1e-9  # to seconds

        plt.subplot(3, 1, 1)
        plt.plot(time, observer_body['_twist__twist__linear__x'])
        plt.ylabel('u (m/s)')
        plt.grid()

        plt.subplot(3, 1, 2)
        plt.plot(time, observer_body['_twist__twist__linear__y'])
        plt.ylabel('v (m/s)')
        plt.grid()

        plt.subplot(3, 1, 3)
        plt.plot(time, observer_body['_twist__twist__angular__z'])
        plt.ylabel('r (rad/s)')
        plt.xlabel('Time (s)')
        plt.grid()

        plt.suptitle('Body Velocities')

    # 3. Actuator states
    if arbitrator is not None and setpoint is not None:
        time = arbitrator['timestamp'] - arbitrator['timestamp'].iloc[0]
        time = time * 1e-9

        time_set = setpoint['timestamp'] - setpoint['timestamp'].iloc[0]
        time_set = time_set * 1e-9

        # wrap angles to [-pi, pi]
        # first convert to rad
        arbitrator['_portside_angle'] = np.deg2rad(arbitrator['_portside_angle'])
        arbitrator['_starboard_angle'] = np.deg2rad(arbitrator['_starboard_angle'])
        # flip sign of desired angles due to frame conventions
        setpoint['_portside_angle'] = -setpoint['_portside_angle']
        setpoint['_starboard_angle'] = -setpoint['_starboard_angle']
        arbitrator['_portside_angle'] = np.rad2deg(np.arctan2(np.sin(arbitrator['_portside_angle']),
                                                     np.cos(arbitrator['_portside_angle'])))
        arbitrator['_starboard_angle'] = np.rad2deg(np.arctan2(np.sin(arbitrator['_starboard_angle']),
                                                     np.cos(arbitrator['_starboard_angle'])))

        plt.figure()

        plt.subplot(4, 1, 1)
        plt.plot(time_set, setpoint['_portside_rpm'], '--', color='orange', label='Desired')
        plt.plot(time, arbitrator['_portside_rpm'], color='blue', label='Actual')
        plt.ylabel('Portside RPM')
        plt.legend()
        plt.grid()

        plt.subplot(4, 1, 2)
        plt.plot(time_set, setpoint['_starboard_rpm'], '--', color='orange', label='Desired')
        plt.plot(time, arbitrator['_starboard_rpm'], color='blue', label='Actual')
        plt.ylabel('Starboard RPM')
        plt.legend()
        plt.grid()

        plt.subplot(4, 1, 3)
        plt.plot(time_set, setpoint['_portside_angle'], '--', color='orange', label='Desired')
        plt.plot(time, arbitrator['_portside_angle'], color='blue', label='Actual')
        plt.ylabel('Portside Angle')
        plt.legend()
        plt.grid()

        plt.subplot(4, 1, 4)
        plt.plot(time_set, setpoint['_starboard_angle'], '--', color='orange', label='Desired')
        plt.plot(time, arbitrator['_starboard_angle'], color='blue', label='Actual')
        plt.ylabel('Starboard Angle')
        plt.xlabel('Time (s)')
        plt.legend()
        plt.grid()

        plt.suptitle('Actuator States')


    # 4. X, Y, Psi over time
    # 4. X, Y, Psi over time
    if observer is not None:
        time = observer['timestamp'] - observer['timestamp'].iloc[0]
        time = time * 1e-9

        plt.figure()

        plt.subplot(3, 1, 1)
        plt.plot(time, observer['_pose__pose__position__x'], color='blue', label='Actual')
        if planning_goal_pose is not None and len(planning_goal_pose) > 0:
            plt.axhline(planning_goal_pose['_pose__position__x'].iloc[0], color='orange', linestyle='--', label='Desired')
        plt.ylabel('X (m)')
        plt.legend()
        plt.grid()

        plt.subplot(3, 1, 2)
        plt.plot(time, observer['_pose__pose__position__y'], color='blue', label='Actual')
        if planning_goal_pose is not None and len(planning_goal_pose) > 0:
            plt.axhline(planning_goal_pose['_pose__position__y'].iloc[0], color='orange', linestyle='--', label='Desired')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.grid()

        plt.subplot(3, 1, 3)
        psi_list = [
            quaternion_to_yaw(
                observer['_pose__pose__orientation__x'].iloc[i],
                observer['_pose__pose__orientation__y'].iloc[i],
                observer['_pose__pose__orientation__z'].iloc[i],
                observer['_pose__pose__orientation__w'].iloc[i]
            ) for i in range(len(observer))
        ]
        plt.plot(time, psi_list, color='blue', label='Actual')
        if planning_goal_pose is not None and len(planning_goal_pose) > 0:
            goal_qx = planning_goal_pose['_pose__orientation__x'].iloc[0]
            goal_qy = planning_goal_pose['_pose__orientation__y'].iloc[0]
            goal_qz = planning_goal_pose['_pose__orientation__z'].iloc[0]
            goal_qw = planning_goal_pose['_pose__orientation__w'].iloc[0]
            goal_psi = quaternion_to_yaw(goal_qx, goal_qy, goal_qz, goal_qw)
            plt.axhline(goal_psi, color='orange', linestyle='--', label='Desired')
        plt.ylabel('Psi (rad)')
        plt.xlabel('Time (s)')
        plt.legend()
        plt.grid()

        plt.suptitle('X, Y, Psi over Time')


    plt.show()


def plot_interactive_rosbag_csv(folder_path):
    observer = load_csv(folder_path, '_odometry_observer')
    planning_goal_pose = load_csv(folder_path, '_planning_goal_pose')

    if observer is None:
        print("Observer data not found. Cannot plot.")
        return

    # Extract time series (seconds from start)
    time_series = (observer['timestamp'] - observer['timestamp'].iloc[0]) * 1e-9

    obs_x = observer['_pose__pose__position__x']
    obs_y = observer['_pose__pose__position__y']
    psi_list = [
        quaternion_to_yaw(
            observer['_pose__pose__orientation__x'].iloc[i],
            observer['_pose__pose__orientation__y'].iloc[i],
            observer['_pose__pose__orientation__z'].iloc[i],
            observer['_pose__pose__orientation__w'].iloc[i]
        ) for i in range(len(observer))
    ]

    if planning_goal_pose is not None and len(planning_goal_pose) > 0:
        goal_x = planning_goal_pose['_pose__position__x'].iloc[-1]
        goal_y = planning_goal_pose['_pose__position__y'].iloc[-1]
        goal_qx = planning_goal_pose['_pose__orientation__x'].iloc[-1]
        goal_qy = planning_goal_pose['_pose__orientation__y'].iloc[-1]
        goal_qz = planning_goal_pose['_pose__orientation__z'].iloc[-1]
        goal_qw = planning_goal_pose['_pose__orientation__w'].iloc[-1]
        goal_psi = quaternion_to_yaw(goal_qx, goal_qy, goal_qz, goal_qw)
    else:
        print("Planning goal pose not found. Cannot plot.")
        return

    all_x = list(obs_x) + [goal_x]
    all_y = list(obs_y) + [goal_y]
    x_min, x_max = min(all_y) - 2, max(all_y) + 2
    y_min, y_max = min(all_x) - 2, max(all_x) + 2

    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.20)

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    # ax.invert_xaxis()
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Interactive NED Trajectory with Slider')
    ax.grid(True)

    ax.plot(obs_y, obs_x, color='gray', linestyle='--', label='Trajectory')

    # Initial goal and vessel arrows
    goal_arrow = ax.arrow(goal_y, goal_x,
                          0.8 * np.sin(goal_psi),
                          0.8 * np.cos(goal_psi),
                          head_width=0.3, head_length=0.4,
                          fc='orange', ec='orange', label='Desired Goal')

    vessel_arrow = ax.arrow(obs_y.iloc[0], obs_x.iloc[0],
                            0.5 * np.sin(psi_list[0]),
                            0.5 * np.cos(psi_list[0]),
                            head_width=0.3, head_length=0.4,
                            fc='blue', ec='blue', label='Actual Vessel')

    time_text = ax.text(0.95, 0.95, f"Time: {time_series.iloc[0]:.1f} s",
                        transform=ax.transAxes,
                        ha='right', va='top', fontsize=10,
                        bbox=dict(facecolor='white', alpha=0.7))

    ax.legend()

    ax_slider = plt.axes([0.15, 0.05, 0.7, 0.04])
    slider = Slider(ax_slider, 'Frame', 0, len(obs_x) - 1, valinit=0, valstep=1)

    animation_running = [True]
    current_frame = [0]

    def draw_frame(frame):
        nonlocal vessel_arrow, goal_arrow

        # Remove old arrows
        vessel_arrow.remove()
        goal_arrow.remove()

        # Draw updated vessel arrow
        vessel_arrow = ax.arrow(obs_y.iloc[frame], obs_x.iloc[frame],
                                0.5 * np.sin(psi_list[frame]),
                                0.5 * np.cos(psi_list[frame]),
                                head_width=0.3, head_length=0.4,
                                fc='blue', ec='blue')

        # Draw updated goal arrow (static position but same heading)
        goal_arrow = ax.arrow(goal_y, goal_x,
                              0.8 * np.sin(goal_psi),
                              0.8 * np.cos(goal_psi),
                              head_width=0.3, head_length=0.4,
                              fc='orange', ec='orange')

        time_text.set_text(f"Time: {time_series.iloc[frame]:.1f} s")
        fig.canvas.draw_idle()

    def on_slider_change(val):
        frame = int(slider.val)
        current_frame[0] = frame
        draw_frame(frame)
        slider.valtext.set_text(f"{time_series.iloc[frame]:.1f} s")

    slider.on_changed(on_slider_change)

    def update_anim(_):
        if animation_running[0]:
            current_frame[0] += 1
            if current_frame[0] >= len(obs_x):
                current_frame[0] = 0
            slider.set_val(current_frame[0])

    ani = animation.FuncAnimation(fig, update_anim, frames=len(obs_x), interval=50, blit=False)

    def on_press(event):
        if event.inaxes == ax_slider:
            animation_running[0] = False

    def on_release(event):
        if event.inaxes == ax_slider:
            animation_running[0] = True

    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('button_release_event', on_release)

    plt.show()



if __name__ == "__main__":
    # folder = "/home/damenadmin/Projects/RosbagReader/Rosbags_csv/DP_test_1_disturbance"  # surge disturbance
    folder = "/home/damenadmin/Projects/RosbagReader/Rosbags_csv/DP_test_2_disturbance"  # sway disturbance
    # folder = "/home/damenadmin/Projects/RosbagReader/Rosbags_csv/DP_test_3_disturbance"  # yaw disturbance
    # folder = "/home/damenadmin/Projects/RosbagReader/Rosbags_csv/manual_wp_sailing_DP_controller"
    # folder = "/home/damenadmin/Projects/RosbagReader/Rosbags_csv/sway_motion_DP_controller"
    # plot_rosbag_csv_folder(folder)
    plot_interactive_rosbag_csv(folder)
