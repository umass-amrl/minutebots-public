import grid_generator as gg

def generate():
    invalid_positions = [(5, y) for y in range(11) if y != 5 ]
    verticies, edges = gg.generate_grid(0, 10, 0, 10, gg.make_predicate(invalid_positions))
    starts = [(0, 0), (10, 10)]
    goals = [(10, 10), (0, 0)]
    gg.plot_grid(verticies, edges)
    return (verticies, edges, starts, goals)

