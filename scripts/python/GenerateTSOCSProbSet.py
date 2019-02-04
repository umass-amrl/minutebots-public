import numpy as np
import itertools
import math

write_to = "../../src/motion_control/data/ProblemSets/"
max_num_process = 5000

robot_max_vel = 4. / 7.
robot_max_pos = math.sqrt(9 * 9 + 6 * 6) / 7
print robot_max_pos, robot_max_vel

def GetProblemSet(num_param_values, start_multr) :
    num_param_values -= 1 - 1e-7 * (robot_max_vel + robot_max_pos)

    x = np.arange(start_multr[0] * robot_max_pos,
                  1.0001 * robot_max_pos,
                  robot_max_pos * (1 - start_multr[0])
                  / num_param_values[0])
    v0x = np.arange(start_multr[1] * robot_max_vel,
                    1.0001 * robot_max_vel,
                    robot_max_vel * (1 - start_multr[1])
                    / num_param_values[1]) / math.sqrt(2.)
    v0y = np.arange(start_multr[2] * robot_max_vel,
                    1.0001 * robot_max_vel,
                    robot_max_vel * (1 - start_multr[2])
                    / num_param_values[2]) / math.sqrt(2.)
    vfx = np.arange(start_multr[3] * robot_max_vel,
                    1.0001 * robot_max_vel,
                    robot_max_vel * (1 - start_multr[3])
                    / num_param_values[3]) / math.sqrt(2.)
    vfy = np.arange(start_multr[4] * robot_max_vel,
                    1.0001 * robot_max_vel,
                    robot_max_vel * (1 - start_multr[4])
                    / num_param_values[4]) / math.sqrt(2.)

    return np.array(list(itertools.product(x, v0x, v0y, vfx, vfy)))

def WriteProblemSetToFiles(prob_set, sub_dir) : 
    num_rows = float(prob_set.shape[0])
    rows_per_file = int(num_rows / max_num_process)
    print "rows per process:", rows_per_file
    num_iters = int(num_rows / rows_per_file + .5)
    for i in xrange(0, num_iters):
        sub_set = prob_set[i*rows_per_file:(i+1)*rows_per_file,:]
        file_name = "%s/%s/SubSet%d.csv" % (write_to, sub_dir, i+1)
        np.savetxt(file_name, sub_set, delimiter=",")
    sub_set = prob_set[num_iters*rows_per_file:,:]
    file_name = "%s/%s/SubSet%d.csv" % (write_to, sub_dir, num_iters+1)
    np.savetxt(file_name, sub_set, delimiter=",")

# x-pos, v0-x, v0-y, vf-x, vf-y
ps1 = GetProblemSet(np.array([100., 1., 1., 100., 100.]), [0, 0, 0, -1, 0])
print ps1.shape
WriteProblemSetToFiles(ps1, "PS1")
ps2 = GetProblemSet(np.array([20., 20., 20., 20., 20.]), [0, -1, -1, -1, 0])
print ps2.shape
WriteProblemSetToFiles(ps2, "PS2")
ps3 = GetProblemSet(np.array([1., 100., 1., 100., 100.]), [0, 0, 0, -1, 0])
print ps3.shape
WriteProblemSetToFiles(ps3, "PS3")

