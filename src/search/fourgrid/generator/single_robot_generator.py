#! /usr/bin/env python3
import sys
import itertools
import collision_checks as cc

if len(sys.argv) < 2:
    print("Args: <scenario name>")
    exit()

def import_pkg(pkg_name):
    return __import__(pkg_name)

scenario_map = {}
scenario_map["split"] = "scenario_1d_split_corner"
scenario_map["dstar"] = "scenario_1d_dstar_lite"

##########################################
scenario_name = sys.argv[1]
scenario_pkg_name = scenario_map[scenario_name]
scenario = import_pkg(scenario_pkg_name)
verticies, edges, starts, goals = scenario.generate_unblocked()

def position_writer(jp):
    letters = ["A", "B", "C", "D"]
    s = ""
    for idx, p in enumerate(jp):
        letter = letters[idx]
        s += "{} {} {} ".format(letter, p[0], p[1])
    return s

def edges_fromto_position(p, goal):
    if p == goal:
        return [(p, p, 0)]
    else:
        return [e for e in edges if e[0] == p] + [(e[1], e[0], e[2]) for e in edges if e[1] == p] + [(p, p, 1)]

def next_positions_from_joint_position(jp, goals):
    next_positions = [[(e[1], e[2]) for e in edges_fromto_position(p, goal)] for p, goal in zip(jp, goals)]
    iter_result = [list(t) for t in itertools.product(*next_positions, repeat=1)]
    positions_and_cost = [([t[0] for t in p], sum([t[1] for t in p])) for p in iter_result]
    filtered_positions = [p for p in positions_and_cost if not cc.joint_position_collides(p[0]) and not cc.no_forward_progress(p[0], jp, p[1])]
    filtered_edges = [p for p in filtered_positions if not cc.joint_edge_collides(jp, p[0])]
    return filtered_edges

def generate_individual_starts_goals():
    assert(type(starts) == list)
    assert(type(goals) == list)
    assert(len(starts) == 1)
    assert(len(goals) == 1)
    p = starts
    output_file = "fourgrid3_single_robot_{}_individual_start.txt".format(scenario_name)
    file = open(output_file, 'w')
    file.write(position_writer(p))
    file.close()
    print("Done. Written to {}".format(output_file))

    p = goals
    output_file = "fourgrid3_single_robot_{}_individual_goal.txt".format(scenario_name)
    file = open(output_file, 'w')
    file.write(position_writer(p))
    file.close()
    print("Done. Written to {}".format(output_file))

def generate_graph(verticies, edges, starts, goals, string):
    assert(len(starts) == 1)
    output_file = "fourgrid3_single_robot_{}_{}.txt".format(scenario_name, string)
    file = open(output_file, 'w') 
    for current_position in [list(p) for p in itertools.product(verticies,
                                                                repeat=len(starts))
                             if len(set(p)) == len(p)]:
        for next_position, cost in next_positions_from_joint_position(current_position,
                                                                      goals):
            file.write(position_writer(current_position) + "=" + str(cost)
                       + "> " + position_writer(next_position) + '\n')
    file.close()
    print("Done. Written to {}".format(output_file))

generate_individual_starts_goals()

verticies, edges, starts, goals = scenario.generate_blocked()
generate_graph(verticies, edges, starts, goals, "blocked")

verticies, edges, starts, goals = scenario.generate_unblocked()
generate_graph(verticies, edges, starts, goals, "unblocked")
