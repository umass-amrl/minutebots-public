import numpy as np
import matplotlib.pyplot as plt
import matplotlib

def set_font(size):
    matplotlib.rcParams.update({# Use mathtext, not LaTeX
                            'text.usetex': False,
                            # Use the Computer modern font
                            'font.family': 'serif',
                            'font.serif': 'cmr10',
                            'font.size' : size,
                            'mathtext.fontset': 'cm',
                            # Use ASCII minus
                            'axes.unicode_minus': False,
                            })

set_font(6)
fig = plt.gcf()
fig.set_size_inches(8.5/2.5, 8.5/2.5 / 1.61, forward=True)
linewidth = 0.5
minor_tick_color = (0.9, 0.9, 0.9)
plt.gca().set_axisbelow(True)
plt.grid(linewidth=linewidth/2)
plt.grid(which='minor', color=minor_tick_color, linestyle='--', alpha=0.7, clip_on=True, linewidth=linewidth/4)

def get_plot_color(count, total_elements):
    start = 0.2
    stop = 0.8
    cm_subsection = np.linspace(start, stop, total_elements) 
    return [ matplotlib.cm.magma(x) for x in cm_subsection ][count]

plot_ci_calls = 0
def plot_ci(keys_lst, ci, total_cis, label):
    global plot_ci_calls
    assert(plot_ci_calls  < total_cis)
    color = get_plot_color(plot_ci_calls, total_cis)
    bottom_lst, mean_lst, top_lst = zip(*ci)
    plt.plot(keys_lst, mean_lst, '-', color=color, label=label, linewidth=linewidth )
    plt.plot(keys_lst, bottom_lst, '-', color=color, linewidth=linewidth )
    plt.plot(keys_lst, top_lst, '-', color=color, linewidth=linewidth )
    plt.fill_between(keys_lst, bottom_lst, top_lst,
                     where=bottom_lst <= top_lst,
                     facecolor=(color[0], color[1], color[2], 0.5), interpolate=True, linewidth=0.0)
    plot_ci_calls += 1
    
    
plot_calls = 0
def plot(keys_lst, lst, total_plots, label):
    global plot_calls
    assert(plot_calls  < total_plots)
    color = get_plot_color(plot_calls, total_plots)
    plt.plot(keys_lst, lst, '-', color=color, label=label, linewidth=linewidth*2 )
    plot_calls += 1
    #plt.grid()
    #plt.grid(which='minor', color=minor_tick_color, linestyle='--')

    
def add_dots_to_robocup_field(positions, radius, color, xlim=(-1000.0, 1000.0), ylim=(-1000.0, 1000.0)):
    assert(type(positions) == list)
    ax = plt.gca()
    for center in positions:
        ax.add_artist(plt.Circle(center, radius, color=color))
        
    plt.xlim(xlim)
    plt.ylim(ylim)
    plt.gca().set_aspect('equal', adjustable='box')
    #plt.grid()
    #plt.grid(which='minor', color=minor_tick_color, linestyle='--')


def legend(loc):
    leg = plt.legend(loc=loc)
    # set the linewidth of each legend object
    for legobj in leg.legendHandles:
        legobj.set_linewidth(linewidth * 2)


def save_fig(filename):
    plt.savefig("{}.pgf".format(filename), bbox_inches='tight')
    plt.savefig("{}.png".format(filename), bbox_inches='tight')
