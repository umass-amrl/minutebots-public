#!/usr/bin/env python3
import sys
import re
from matplotlib import pyplot as plt
import numpy as np
import numpy.polynomial.polynomial as poly
import pdb

def print_usage_and_exit():
    print("Usage: {} [show individual fits] [file]*".format(sys.argv[0]))
    exit(-1)

def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")

def get_show_individual_fits():
    string = sys.argv[1].lower()
    dict = {'t':True, 'true':True,'f':False,'false':False}
    result = dict.get(string, None)
    if result is None:
        print_usage_and_exit()
    return result

def extract_info_from_name(name):
    search_obj = re.search(r'(kick_tuning_ssl_)(1[0-9]|[0-9])(_power_)([0-9][0-9][0-9]|[0-9][0-9]|[0-9])(.[0-9]*_itr_)([0-9][0-9]|[0-9])', name, re.M|re.I)
    return (search_obj.group(2), search_obj.group(4), search_obj.group(6))

def process_file_line(line):
    x = float(line.split(',')[0])
    y = float(line.split(',')[1].split('@')[0])
    t = float(line.split('@')[1])
    return (x, y, t)

def read_file(name):
    f = open(name, 'r')
    return [process_file_line(l) for l in f.readlines()]

def normalize_data(file_lines):
    first_x, first_y, first_time = file_lines[5]
    normed_lines = [(x - first_x, y - first_y, t - first_time) for x, y, t in file_lines[5:-5]]
    for l in normed_lines:
        assert(l[2] >= 0)
    return normed_lines

def setup_esc():
    def quit_figure(event):
        if event.key == 'escape':
            plt.close(event.canvas.figure)

    cid = plt.gcf().canvas.mpl_connect('key_press_event', quit_figure)


def dist_point_line(p1, p2, p3):
    assert(len(p1) == len(p2))
    if np.array_equal(p1, p2):
        return float('inf')
    normed_line = (p2 - p1) / np.linalg.norm(p2 - p1)
    vec_on_line = np.dot(normed_line, (p3 - p1)) * normed_line
    return np.abs(np.linalg.norm((p3 - p1) - (vec_on_line)))

def dist_from_y_given_x(p1, p2, p3):
    assert(len(p1) == 2)
    assert(len(p2) == 2)
    assert(len(p1) == len(p2))
    if np.array_equal(p1, p2):
        return float('inf')

    normed_line = (p2 - p1) / np.linalg.norm(p2 - p1)
    scale_amount = p3[0] / normed_line[0]
    expected_position = p1 + normed_line * scale_amount
    return np.abs(expected_position[1] - p3[1])

def dist_from_z_given_xy(p1, p2, p3):
    assert(len(p1) == 3)
    assert(len(p2) == 3)
    assert(len(p1) == len(p2))
    if np.array_equal(p1, p2):
        return float('inf')

    normed_line = (p2 - p1) / np.linalg.norm(p2 - p1)

    p1_xy = np.array([p1[0], p1[1]])
    p3_xy = np.array([p3[0], p3[1]])
    xy_normed = np.array([normed_line[0], normed_line[1]])
    scale_amount = np.dot(p3_xy - p1_xy, xy_normed)
    expected_position = p1 + normed_line * scale_amount
    delta = np.abs(expected_position[2] - p3[2])
    assert(delta >= 0)
    return delta


def run_modified_RANSAC(data, epsilon, time_epsilon):
    if (len(data) < 2):
        print("Cannot run RANSAC on {} datapoints".format(len(data)))
        return None
    first_point = data[0]
    sample_point = lambda: data[np.random.randint(1, len(data) - 1)]

    def count_all_in_epsilon(p1, p2, epsilon, time_epsilon):
        p1 = np.array([p1[0], p1[1], p1[2]])
        p2 = np.array([p2[0], p2[1], p2[2]])
        count = 0
        np_data = [np.array([d[0], d[1], d[2]]) for d in data]
        filtered_np_data = [d for d in np_data if dist_point_line(p1, p2, d) < epsilon and dist_from_z_given_xy(p1, p2, d) < time_epsilon]
        xs = [d[0] for d in filtered_np_data]
        ys = [d[1] for d in filtered_np_data]
        coefs = poly.polyfit(xs, ys, 1)
        ffit = poly.polyval(xs, coefs)
        # norms = [np.linalg.norm(d-p1) for d in np_data]
        # normed_vec = (p2-p1) / np.linalg.norm(p2-p1)
        # scaled_vec = normed_vec * max(norms)
        return ([(np.array([x, y]), t) for x, y, t in filtered_np_data], ffit)

    lines = [count_all_in_epsilon(first_point, sample_point(), epsilon, time_epsilon) for i in range(100)]
    best_line = lines[0]
    for l in lines:
        if len(best_line[0]) < len(l[0]):
            best_line = l
    
    return best_line


def RANSAC_velocities(velocities, times, epsilon):
    vel_times = [np.array([v, t]) for v, t in  zip(velocities, times)]
    random_point = lambda: vel_times[np.random.randint(0, len(vel_times) - 1)]

    def get_all_in_epsilon(epsilon):
        p1 = random_point()
        p2 = random_point()

        max_v_point = vel_times[0]
        for d in vel_times:
            if d[0] > max_v_point[0]:
                max_v_point = d

        filtered_data = [d for d in vel_times if dist_from_y_given_x(p1, p2, d) < epsilon]
        return filtered_data

    lines = [get_all_in_epsilon(epsilon) for i in range(100)]
    best_line = lines[0]
    for l in lines:
        if len(best_line) < len(l):
            best_line = l
    return ([e[0] for e in best_line], [e[1] for e in best_line])

def compute_velocity(position_times, epsilon):
    vels = []
    for i in range(len(position_times) - 1):
        p0, t0 = position_times[i]
        p1, t1 = position_times[i+1]
        assert(t1 - t0 > 0)
        v = np.linalg.norm(p1 - p0) / (t1 - t0)
        if (len(vels) == 0 or np.abs(v - vels[-1]) < 1000):
            vels.append(v)

    if len(vels) <= 1:
        return (None, None, None, None)
    ransacd_vel, ransacd_filtered_t = RANSAC_velocities(vels, filtered_t[1:], epsilon)
    if len(ransacd_vel) <= 0:
        return (None, None, None, None)
    coefs = poly.polyfit(ransacd_filtered_t, ransacd_vel, 1)
    ffit = poly.polyval(ransacd_filtered_t, coefs)
    return (ransacd_vel, ransacd_filtered_t, ffit, coefs)

if len(sys.argv) < 3:
    print_usage_and_exit()

kShowIndividualFits = get_show_individual_fits()

power_to_velocity = []

for file in  sys.argv[2:]:
    ssl_id, power, iteration = extract_info_from_name(file)
    kEpsilon = 20
    kTimeEpsilon = 0.2
    data = normalize_data(read_file(file))
    if (len(data) == 0):
        print("ERROR: Bad data collection for ID {} Power {}".format(ssl_id, power))
        continue
    data_x = [d[0] for d in data]
    data_y = [d[1] for d in data]
    data_t = [d[2] for d in data]

    filtered_data, ffit = run_modified_RANSAC(data, kEpsilon, kTimeEpsilon)

    filtered_x = [d[0] for d, t in filtered_data]
    filtered_y = [d[1] for d, t in filtered_data]
    filtered_t = [t for d, t in filtered_data]

    velocities, vel_ts, velfit, velcoef = compute_velocity(filtered_data, kEpsilon)

    if velocities is None:
        continue

    def plot_figures():
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
        fig.suptitle("ID {}; Power {}; Itr {}".format(ssl_id, power, iteration), fontsize=14, fontweight='bold')
        ax1.set_title("Raw data with fitline")
        ax1.scatter(data_x, data_y, c=data_t, cmap='gray')
        #plt.scatter(filtered_x, filtered_y, c=filtered_t, cmap='autumn')
        ax1.plot(filtered_x, ffit, linewidth=2.0, color='red')
        ax1.set_aspect('equal', 'datalim')
        ax1.set_xlabel("Position X $(mm)$")
        ax1.set_ylabel("Position Y $(mm)$")

        ax2.set_title("Filtered data with fitline")
        ax2.scatter(filtered_x, filtered_y, c=filtered_t, cmap='gray')
        ax2.plot(filtered_x, ffit, linewidth=2.0, color='red')
        ax2.set_aspect('equal', 'datalim')
        ax2.set_xlabel("Position X $(mm)$")
        ax2.set_ylabel("Position Y $(mm)$")

        ax3.set_title("Velocity Fit")
        ax3.scatter(vel_ts, velocities, c=vel_ts, cmap='gray')
        ax3.plot(vel_ts, velfit, color='red', )
        ax3.text(max(vel_ts) * 0.1, 1,
                 "Intercept: {}\nSlope:      {}".format(velcoef[0], velcoef[1]))
        ax3.set_xlabel("Time $(seconds)$")
        ax3.set_ylabel("Single timestep delta velocity $(mm/s)$")
        ax3.set_ylim((max(velocities)* -0.1, max(velocities) * 1.1))
        ax3.set_xlim(0, max(vel_ts) * 1.05)
        
        setup_esc()
        #plt.tight_layout()
        plt.show()

    if kShowIndividualFits:
        plot_figures()
    power_to_velocity.append((int(power), velcoef[0]))


def  build_velocity_to_power_function():
    power_to_velocity.sort(key=lambda tup: tup[0])
    powers = [p[0] for p in power_to_velocity]
    velocities = [p[1] for p in power_to_velocity]

    coefs = poly.polyfit(velocities, powers, 2)
    ffit = poly.polyval(sorted(velocities), coefs)

    print(coefs)
    plt.scatter(velocities, powers)
    plt.plot(sorted(velocities), ffit)
    plt.title("Ball Velocity vs Kicker Power for Robot {}\n$ {}x^2 + {}x + {} $".format(ssl_id, coefs[2], coefs[1], coefs[0]))
    plt.xlabel("Ball Velocity")
    plt.ylabel("Kicker Power")
    setup_esc()
    plt.show()
    return coefs

def write_to_radio_file(coefs):
    filename = "kick_curve_id_{}.txt".format(ssl_id)
    if not query_yes_no("Do you want to write to radio file?", default="no"):
        return
    output_file = open("src/configs/{}".format(filename), 'w')
    output_file.write("{} {} {}".format(coefs[0], coefs[1], coefs[2]))
    output_file.close()

coefs = build_velocity_to_power_function()
write_to_radio_file(coefs)
