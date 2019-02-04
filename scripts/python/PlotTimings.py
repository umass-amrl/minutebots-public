#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.markers as markers
import sys

lines_list = []
for filename in sys.argv[1:]:
    f = open(sys.argv[1], 'r')
    lines_list.append([float(l.strip()) for l in f.readlines()])

min_element = min([l[0] for l in lines_list])

lines_list = [np.array([ ((e - min_element) * 1000, 0.0001) for e in lst]) for lst in lines_list]
print(lines_list)

def get_color(idx):
    return ['red', 'green', 'blue'][idx]

def get_marker(idx):
    return [markers.TICKUP, '|', markers.TICKDOWN][idx]

patches_lst = []
fig,ax=plt.subplots(figsize=(6,3))

for i, line in enumerate(lines_list):
    print(line)
    ax.broken_barh(line, (i*0.21-0.4,0.2), color=get_color(i) )
    patches_lst.append(patches.Patch(color=get_color(i), label=sys.argv[i + 1]))

labels=[]
# for i, task in enumerate(df.groupby("Task")):
#     labels.append(task[0])
#     for r in task[1].groupby("Resource"):
#         data = r[1][["Start", "Diff"]]
#         ax.broken_barh()
#         ax.broken_barh(data.values, (i-0.4,0.8), color=color[r[0]] )

ax.set_yticks(range(len(labels)))
ax.set_yticklabels(labels) 
ax.set_xlabel("time [ms]")
plt.xlim([1/30 * 1000*600 - 0.1, 1/30 * 1000 * 800 + 0.1])
plt.legend(handles=patches_lst)
plt.tight_layout()
plt.show()


# for idx, lst in enumerate(lines_list):
#     plt.plot(lst, np.zeros_like(lst) , 'x', color=get_color(idx), marker=get_marker(idx), mew=3)
#     ax.add_patch(patches.Rectangle((50,100),40,30,linewidth=1,edgecolor='r',facecolor='none'))
#     patches_lst.append(patches.Patch(color=get_color(idx), label=sys.argv[idx + 1]))


# plt.xlabel("Time (milliseconds)")
# plt.ylabel("Ignore This")
# plt.xlim([0.0, 1/30 * 1000])
# plt.legend(handles=patches_lst)
# plt.show()
