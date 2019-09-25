def joint_position_collides(jp):
    assert(type(jp) == list)
    robot_count = len(jp)
    if robot_count == 1:
        return False
    if robot_count == 2:
        return jp[0] == jp[1]
    if robot_count == 3:
        return jp[0] == jp[1] or jp[1] == jp[2] or jp[2] == jp[0]
    else:
        raise Exception("Unhandled size {}".format(robot_count))

def joint_edge_collides(start, goal):
    assert(type(start) == list)
    assert(type(goal) == list)
    assert(len(goal) == len(start))
    robot_count = len(start)
    if robot_count == 1:
        return False
    if robot_count == 2:
        return start[0] == goal[1] and goal[0] == start[1]
    if robot_count == 3:
        return (start[0] == goal[1] and goal[0] == start[1]) or (start[1] == goal[2] and goal[1] == start[2]) or (start[2] == goal[0] and goal[2] == start[0])
    else:
        raise Exception("Unhandled size {}".format(robot_count))

def no_forward_progress(prev_jp, next_jp, weight):
    assert(type(prev_jp) == list)
    assert(type(next_jp) == list)
    assert(len(prev_jp) == len(next_jp))
    robot_count = len(next_jp)
    return prev_jp == next_jp and weight > 0
