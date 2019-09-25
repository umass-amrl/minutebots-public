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
scenario_map["split"] = "scenario_2d_split"
scenario_map["split_dual"] = "scenario_2d_split_dual"
scenario_map["corners"] = "scenario_2d_corners"
scenario_map["straight"] = "scenario_2d_straight"
scenario_map["split3"] = "scenario_3d_split"

##########################################
scenario_name = sys.argv[1]
scenario_pkg_name = scenario_map[scenario_name]
scenario = import_pkg(scenario_pkg_name)
verticies, edges, starts, goals = scenario.generate()

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

def position_writer(jp):
    letters = ["A", "B", "C", "D"]
    s = ""
    for idx, p in enumerate(jp):
        letter = letters[idx]
        s += "{} {} {} ".format(letter, p[0], p[1])
    return s

def generate_joint_graph():
    output_file = "fourgrid3_{}_joint.txt".format(scenario_name)
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

def generate_individual_graphs():
    for i in range(len(starts)):
        start_i = [starts[i]]
        goal_i = [goals[i]]
        output_file = "fourgrid3_{}_individual_{}.txt".format(scenario_name, i)
        file = open(output_file, 'w') 
        for current_position in [[p] for p in verticies]:
            for next_position, cost in next_positions_from_joint_position(current_position,
                                                                          goal_i):
                file.write(position_writer(current_position) + "=" + str(cost)
                           + "> " + position_writer(next_position) + '\n')
        file.close()
        print("Done. Written to {}".format(output_file))

def generate_individual_starts_goals():
    for i, p in enumerate([[p] for p in starts]):
        output_file = "fourgrid3_{}_individual_{}_start.txt".format(scenario_name, i)
        file = open(output_file, 'w')
        file.write(position_writer(p))
        file.close()
        print("Done. Written to {}".format(output_file))
        
    for i, p in enumerate([[p] for p in goals]):
        output_file = "fourgrid3_{}_individual_{}_goal.txt".format(scenario_name, i)
        file = open(output_file, 'w')
        file.write(position_writer(p))
        file.close()
        print("Done. Written to {}".format(output_file))
        
        
def generate_joint_starts_goals():
    output_file = "fourgrid3_{}_joint_start.txt".format(scenario_name)
    file = open(output_file, 'w')
    file.write(position_writer(starts))
    file.close()
    print("Done. Written to {}".format(output_file))
    

    output_file = "fourgrid3_{}_joint_goal.txt".format(scenario_name)
    file = open(output_file, 'w')
    file.write(position_writer(goals))
    file.close()
    print("Done. Written to {}".format(output_file))

generate_joint_graph()
generate_individual_graphs()

generate_individual_starts_goals()
generate_joint_starts_goals()

