import matplotlib.pyplot as plt
import networkx as nx
import time

# fig, axes = plt.subplots(1,1)
#
# plt.sca(axes)
# G = nx.DiGraph()
# edges = [(1,2),(1,3),(2,3)]
# nodes = [1,2,3]
# G.add_edges_from(edges)
# G.add_nodes_from(nodes)
# nx.draw_networkx(G)
# plt.xlim(-2,2)
# plt.ylim(-2,2)
# plt.ion()
# #plt.show()
# for t in range(1,20):
#     plt.pause(0.5)
#     G.clear()
#     plt.cla()
#     nodes = [1+t, 2+t, 3+t,4+t,5+t]
#     edges = [(1+t,2+t),(1+t,3+t),(2+t,3+t),(3+t,2+t)]
#     G.add_edges_from(edges)
#     G.add_nodes_from(nodes)
#     nx.draw_networkx(G)

# roundabout_id = {30001, 30002, 30005, 30016, 30018, 30030, 30036, 30040, 30042, 30047, 30004, 30017, 30023}
#
# for i in range(8):
#     if 30001 in roundabout_id:
#         print('30001 is in roundabout')
calculated_pair = {}
j = 5
m = 6
calculated_pair.setdefault(j,[]).append(1)
calculated_pair.setdefault(j,[]).append(2)
calculated_pair.setdefault(j,[]).append(3)
calculated_pair.setdefault(j,[]).append(4)
calculated_pair.setdefault(m,[]).append(4)

print(calculated_pair)
if 3 in calculated_pair[j]:
    print('yes')
else:
    print('no')
