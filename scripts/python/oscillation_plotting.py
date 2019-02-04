#!/usr/bin/env python3
import sys
import numpy as np
from collections import namedtuple
import matplotlib.pyplot as plt

if len(sys.argv) < 4:
    print("Usage: ", sys.argv[0], "<ssl vision id> <position and trajectory file> <acceleration file>")
    exit(-1)

Pose2d = namedtuple("Pose2d", "angle position")
DataPoint = namedtuple("DataPoint", "world_time observed_time monotonic_time raw_vision_pos filtered_not_fp_pos filtered_not_fp_vel current_pos current_vel goal_pos ntoc_accel linear_stages rotational_stages")
LinearStage = namedtuple("LinearStage", "accel time")
RotationalStage = namedtuple("RotationalStage", "accel time")
AccelerationCommand = namedtuple("AccelerationCommand", "world_time ssl_vision_id command")

def check_type(e, e_type):
    try:
        assert(type(e) == e_type)
        if e_type is Pose2d:
            check_type(e.angle, float)
            check_type(e.position, np.ndarray)
    except AssertionError:
        print('ERROR: "e" of type {} instead of type {}'.format(type(e), e_type))
        raise

def check_is_list_of_type(lst, lst_type):
    assert(type(lst) == list)
    for e in lst:
        check_type(e, lst_type)

def setup_esc():
    def quit_figure(event):
        if event.key == 'escape':
            plt.close(event.canvas.figure)

    cid = plt.gcf().canvas.mpl_connect('key_press_event', quit_figure)

def ppvec(v):
    check_type(v, np.ndarray)
    return "[{}, {}]".format(v[0], v[1])

def pppose2d(p2d):
    check_type(p2d, Pose2d)
    assert(type(p2d) == Pose2d)
    return "({}, {})".format(p2d.angle, p2d.position)

def parse_acceleration_file_line(l):
    check_type(l, str)
    split = [float(e) for e in l.split(',')]
    world_time = split[0]
    ssl_vision_id = split[1]
    command = Pose2d(split[4], np.array([split[2], split[3]]))
    check_type(command, Pose2d)
    return AccelerationCommand(world_time, ssl_vision_id, command)

def parse_trajectory_file_line(l):
    check_type(l, str)
    assert('#' in l)
    l = l.replace('#', '')
    split = [float(e) for e in l.split(',')[:-1]]
    # Timestamp
    world_time = split[0]
    observed_time = split[1]
    monotonic_time = split[2]
    check_type(world_time, float)
    check_type(observed_time, float)
    check_type(monotonic_time, float)

    # Raw Vision Pose
    raw_vision_pos_position = np.array([split[3], split[4]])
    raw_vision_pos_angle = split[5]
    raw_vision_pos = Pose2d(raw_vision_pos_angle, raw_vision_pos_position)
    check_type(raw_vision_pos, Pose2d)

    # Filtered Not FP Position Pose
    filtered_not_fp_pos_position = np.array([split[6], split[7]])
    filtered_not_fp_pos_angle =  split[8]
    filtered_not_fp_pos = Pose2d(filtered_not_fp_pos_angle, filtered_not_fp_pos_position)
    check_type(filtered_not_fp_pos, Pose2d)

    # Filtered Not FP Velocity Pose
    filtered_not_fp_vel_position = np.array([split[9], split[10]])
    filtered_not_fp_vel_angle =  split[11]
    filtered_not_fp_vel = Pose2d(filtered_not_fp_vel_angle, filtered_not_fp_vel_position)
    check_type(filtered_not_fp_vel, Pose2d)

    # Position Pose
    current_pos_position = np.array([split[12], split[13]])
    current_pos_angle =  split[14]
    current_pos = Pose2d(current_pos_angle, current_pos_position)
    check_type(current_pos, Pose2d)

    # Velocity Pose
    current_vel_position = np.array([split[15], split[16]])
    current_vel_angle = split[17]
    current_vel = Pose2d(current_vel_angle, current_vel_position)
    check_type(current_vel, Pose2d)

    # Goal Pose
    goal_pos_position = np.array([split[18], split[19]])
    goal_pos_angle = split[20]
    goal_pos = Pose2d(goal_pos_angle, goal_pos_position)
    check_type(goal_pos, Pose2d)

    # NTOC Acceleration command
    ntoc_accel_position = np.array([split[21], split[2]])
    ntoc_accel_angle = split[23]
    ntoc_accel = Pose2d(ntoc_accel_angle, ntoc_accel_position)
    check_type(ntoc_accel, Pose2d)

    # Counts
    linear_control_phase_count = int(split[24])
    rotational_control_phase_count = int(split[25])
    check_type(linear_control_phase_count, int)
    check_type(rotational_control_phase_count, int)


    assert(int(linear_control_phase_count) == linear_control_phase_count)
    assert(int(rotational_control_phase_count) == rotational_control_phase_count)

    linear_stages = []
    for i in range (linear_control_phase_count):
        base_index = i * 3 + 26
        accel = np.array(split[base_index:base_index + 2])
        time = split[base_index + 2]
        linear_stages.append(LinearStage(accel, time))

    rotational_stages = []
    for j in range(rotational_control_phase_count):
        base_index = j * 2 + linear_control_phase_count * 3 + 26
        accel = split[base_index]
        time = split[base_index + 1]
        rotational_stages.append(RotationalStage(accel, time))
    
    return DataPoint(world_time, observed_time, monotonic_time,
                     raw_vision_pos, filtered_not_fp_pos, filtered_not_fp_vel,
                     current_pos, current_vel, goal_pos, ntoc_accel,
                     linear_stages, rotational_stages)

def print_datapoint(d):
    check_type(d, DataPoint)
    print("Pos:", pppose2d(d.current_pos),
          "Vel:", pppose2d(d.current_vel),
          "Goal:", pppose2d(d.goal_pos))
    print("\tNTOC Linear Stages:")
    for idx, s in enumerate(d.linear_stages):
        print("\t{}: {} t: {}".format(idx, ppvec(s.accel), s.time))
    print("\tNTOC Rotational Stages:")
    for idx, s in enumerate(d.rotational_stages):
        print("\t{}: {} t: {}".format(idx, s.accel, s.time))

def execute_accel_position(pos, vel, accel, t):
    check_type(pos, Pose2d)
    check_type(vel, Pose2d)
    check_type(accel, np.ndarray)
    final_pos_position = vel.position * t + accel * (0.5 * t * t) + pos.position
    final_vel_position = vel.position + accel * t
    return (Pose2d(pos.angle, final_pos_position), Pose2d(vel.angle, final_vel_position))

def normalize_times(data_points, accelerations):
    check_is_list_of_type(data_points, DataPoint)
    check_is_list_of_type(accelerations, AccelerationCommand)
    assert(len(data_points) > 0)
    assert(len(accelerations) > 0)
    d0 = data_points[0]
    a0 = accelerations[0]
    min_time = min(d0.observed_time, d0.world_time, a0.world_time)
    # world_time observed_time monotonic_time
    return ([DataPoint(d.world_time - min_time,
                       d.observed_time - min_time,
                       d.monotonic_time - d0.monotonic_time,
                       d.raw_vision_pos,
                       d.filtered_not_fp_pos,
                       d.filtered_not_fp_vel,
                       d.current_pos,
                       d.current_vel,
                       d.goal_pos,
                       d.ntoc_accel,
                       d.linear_stages,
                       d.rotational_stages) for d in data_points],
            [AccelerationCommand(a.world_time - min_time,
                                 a.ssl_vision_id,
                                 a.command) for a in accelerations])

def plot_control_points(pos, vel, start_t, stages):
    check_type(pos, Pose2d)
    check_type(vel, Pose2d)
    check_type(start_t, float)
    check_is_list_of_type(stages, LinearStage)
    kSamplingAmt = 40
    xs = [pos.position[0]]
    ts = [start_t]
    for s in stages:
        accel = s.accel
        t = s.time
        for i in range(kSamplingAmt):
            pos, vel = execute_accel_position(pos, vel, accel, t / kSamplingAmt)
            xs.append(pos.position[0])
            ts.append(t / kSamplingAmt + ts[-1])
    check_is_list_of_type(xs, np.float64)
    check_is_list_of_type(ts, float)
    plt.plot(ts, xs, color='green', alpha=0.5)
    plt.scatter(ts[0:1], xs[0:1], color='green', alpha=0.5)

def plot_velocity_command(start_t, vel, stages):
    check_type(start_t, float)
    check_type(vel, Pose2d)
    check_is_list_of_type(stages, LinearStage)
    vs = [vel.position[0]]
    ts = [start_t]
    for s in stages:
        check_type(s.accel, np.ndarray)
        check_type(vel, Pose2d)
        vel = Pose2d(vel.angle, vel.position + s.accel * s.time)
        check_type(vel.angle, float)
        check_type(vel.position, np.ndarray)
        vs.append(vel.position[0])
        ts.append(ts[-1] + s.time)
    check_is_list_of_type(vs, np.float64)
    check_is_list_of_type(ts, float)

    plt.plot(ts, vs, color='green', alpha=0.5)
    plt.scatter(ts[0:1], vs[0:1], color='green', alpha=0.5)

def verify_data_points_end_positions(data_points):
    check_is_list_of_type(data_points, DataPoint)
    for idx, d in enumerate(data_points):
        pos = d.current_pos
        vel = d.current_vel
        for s in d.linear_stages:
            accel = s.accel
            t = s.time
            pos, vel = execute_accel_position(pos, vel, accel, t)
        final_dist = np.linalg.norm((pos.position - d.goal_pos.position))
        final_speed = np.linalg.norm(vel.position)
        if (final_dist > 0.02) or (final_speed > 0.02):
            print_datapoint(d)
            print("Final location diff: {} Speed: {}".format(final_dist, final_speed))

def plot_diffed_velocity_position(data_points):
    check_is_list_of_type(data_points, DataPoint)
    vels = []
    for i in range(len(data_points) - 1):
        d2 = data_points[i + 1]
        d1 = data_points[i]
        delta_p = (d2.raw_vision_pos.position - d1.raw_vision_pos.position)
        delta_t = (d2.observed_time - d1.observed_time)
        vels.append(delta_p[0] / delta_t)
    times = [d.world_time for d in data_points[1:]]
    plt.plot(times, vels, color="red", label="Kalman Position Est Velocity")

def plot_linear_control_stages(data_points):
    check_is_list_of_type(data_points, DataPoint)
    for idx, d in enumerate(data_points):
        if len(d.linear_stages) <= 0:
            continue
        pos = d.current_pos
        vel = d.current_vel
        if (idx % 1 == 0):
            plot_control_points(pos, vel, d.world_time, d.linear_stages)

def plot_kalman_positions(data_points):
    check_is_list_of_type(data_points, DataPoint)
    times = [d.world_time for d in data_points]
    positions = [d.current_pos.position[0] for d in data_points]
    check_is_list_of_type(times, float)
    check_is_list_of_type(positions, np.float64)
    plt.plot(times, positions, color='blue', label="Kalman Positions")

def plot_raw_vision_positions(data_points):
    check_is_list_of_type(data_points, DataPoint)
    times = [d.world_time for d in data_points]
    positions = [d.raw_vision_pos.position[0] for d in data_points]
    check_is_list_of_type(times, float)
    check_is_list_of_type(positions, np.float64)
    plt.plot(times, positions, color='orange', label="Raw vision")

def plot_filtered_not_fp_positions(data_points):
    check_is_list_of_type(data_points, DataPoint)
    times = [d.world_time for d in data_points]
    positions = [d.filtered_not_fp_pos.position[0] for d in data_points]
    check_is_list_of_type(times, float)
    check_is_list_of_type(positions, np.float64)
    plt.plot(times, positions, color='black', label="Filtered not FP")

def plot_kalman_velocities(data_points):
    check_is_list_of_type(data_points, DataPoint)
    times = [d.world_time for d in data_points]
    velocities = [d.current_vel.position[0] for d in data_points]
    check_is_list_of_type(times, float)
    check_is_list_of_type(velocities, np.float64)
    plt.plot(times, velocities, color='blue', label="Kalman Velocities")

def plot_velocity_commands(data_points):
    check_is_list_of_type(data_points, DataPoint)
    for idx, d in enumerate(data_points):
        if (idx % 1 == 0):
            plot_velocity_command(d.world_time, d.current_vel, d.linear_stages)

def plot_radio_commanded_velocity(velocities):
    check_is_list_of_type(velocities, AccelerationCommand)
    vel_xs = [v.command.position[0] for v in velocities]
    ts = [v.world_time for v in velocities]
    plt.plot(ts, vel_xs, color="purple")

def read_data():
    ssl_vision_id = int(sys.argv[1])
    position_and_trajectory_filename = sys.argv[2]
    acceleration_filename = sys.argv[3]
    f1 = open(position_and_trajectory_filename)
    f2 = open(acceleration_filename)
    data_points, accelerations = normalize_times([parse_trajectory_file_line(l) for l in f1.readlines()],
                                  [parse_acceleration_file_line(l) for idx, l in enumerate(f2.readlines()) if idx % 2 == 0])
    accelerations = [a for a in accelerations if a.ssl_vision_id == ssl_vision_id]
    check_is_list_of_type(data_points, DataPoint)
    check_is_list_of_type(accelerations, AccelerationCommand)

    return data_points, accelerations

def linear_position_plot(data_points):
    check_is_list_of_type(data_points, DataPoint)
    plot_kalman_positions(data_points)
    plot_raw_vision_positions(data_points)
    plot_filtered_not_fp_positions(data_points)
    plot_linear_control_stages(data_points)
    plt.ylabel("X Position in world")
    plt.xlabel("Time (Seconds)")
    plt.title("Forward Predicted X vs Time for ForwardBackward")
    plt.legend(['Kalman Positions',
                'Raw Vision Positions',
                'Filtered, Not FP Positions',
                'Linear Control Stages'])

def linear_velocity_plot(data_points, velocities):
    check_is_list_of_type(data_points, DataPoint)
    plot_diffed_velocity_position(data_points)
    plot_radio_commanded_velocity(velocities)
    plot_velocity_commands(data_points)
    plot_kalman_velocities(data_points)
    plt.xlabel("Time (Seconds)")
    plt.ylabel("X Velocity")
    plt.title("X velocity vs Time")
    plt.legend([
#        'Kalman Velocity',
                'Diffed Raw Vision Position Velocity',
                'World Frame Radio Commands',
    #            'Velocity Commands'
    ])

data_points, velocities = read_data()
verify_data_points_end_positions(data_points)

ax1 = plt.subplot(211)
linear_position_plot(data_points)
ax1.set_xticks([d.world_time for d in data_points])
ax1.set_xticklabels(
        ["{0:.3f}".format(d.world_time) if idx % 5 == 0 else "" for idx, d in enumerate(data_points)]
        )
plt.grid()

plt.subplot(212, sharex=ax1)
linear_velocity_plot(data_points, velocities)

setup_esc()
plt.grid()
plt.show()
