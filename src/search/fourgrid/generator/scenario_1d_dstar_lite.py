import grid_generator as gg

def generate_unblocked():
    invalid_positions = [(6, y) for y in range(3, 12) if y != 5 ] + \
                        [(x, 10) for x in range(5)] + \
                        [(x, 4) for x in range(5)] + [(4, 3), (6, 3), (7, 4) , (8, 4), (7, 6) , (8, 6)]
    verticies, edges = gg.generate_grid(0, 15, 0, 11, gg.make_predicate(invalid_positions))
    starts = [(1, 11)]
    goals = [(14, 1)]
    gg.plot_grid(verticies, edges, start=starts, goal=goals)
    return (verticies, edges, starts, goals)

def generate_blocked():
    invalid_positions = [(6, y) for y in range(3, 12)] + \
                        [(x, 10) for x in range(5)] + \
                        [(x, 4) for x in range(5)] + [(4, 3), (6, 3), (7, 4) , (8, 4), (7, 6) , (8, 6)]
    verticies, edges = gg.generate_grid(0, 15, 0, 11, gg.make_predicate(invalid_positions))
    starts = [(1, 11)]
    goals = [(14, 1)]
    gg.plot_grid(verticies, edges, start=starts, goal=goals)
    return (verticies, edges, starts, goals)
