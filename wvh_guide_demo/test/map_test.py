#! /usr/bin/env python3
import matplotlib.pyplot as plt

import matplotlib.image as mpimg
import networkx as nx
from PIL import Image

# # import Image

# # img = Image.open('/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH_1.jpg')
# # print(img.size)

# # img = img.resize((160, 240), Image.ANTIALIAS)

# # print(img.size)

# plt.figure(figsize=(10,8))
# img=mpimg.imread('/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH_1.jpg')
# plt.imshow(img)
# # nx.draw(G,pos)
# plt.show()
# # plt.savefig('/home/test.png')

'''import networkx as nx 
import matplotlib.pyplot as plt
 
G = nx.Graph()
 
plt.figure(figsize =(9, 12))
G.add_edges_from([(1, 2), (1, 3), (2, 3), (2, 4), (2, 5), (3, 4), 
                         (4, 5), (4, 6), (5, 7), (5, 8), (7, 8)])
 
# First Graph created
plt.subplot(311)
nx.draw_networkx(G)
 
H = nx.Graph()
H.add_edges_from([(13, 14), (13, 15), (13, 9),
                  (14, 15), (15, 10), (9, 10)])
 
# Second Graph created
plt.subplot(312)
nx.draw_networkx(H)
 
 
I = nx.union(G, H)
plt.subplot(313)
nx.draw_networkx(I)
plt.show()'''
import networkx as nx
import matplotlib.pyplot as plt

# # Create multiple graphs
# G1 = nx.Graph()
# G1.add_edges_from([(1, 2), (1, 3), (2, 4)])

# G2 = nx.DiGraph()
# G2.add_edges_from([(1, 2), (2, 3), (3, 1)])

# G3 = nx.Graph()
# G3.add_edges_from([(1, 3), (3, 4), (4, 5), (5, 1)])

# # List of graphs to plot
# graphs = [G1, G2, G3]

# # Plot each graph in a separate window simultaneously
# for i, graph in enumerate(graphs):
#     plt.figure()  # Create a new figure for each graph
#     nx.draw(graph, with_labels=True, node_color='lightblue', node_size=700)
#     plt.title(f"Graph {i + 1}")
#     plt.show(block=False)  # Show the plot without blocking

# # Keep the script running until all windows are closed
# plt.show()
# img = Image.open(f'/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_{4}.jpg')
# print(img.size)

# img = img.resize((2357,3316), Image.ANTIALIAS)
# img.save(f'/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_{4}_size.jpg')


# fig = plt.figure(figsize = (12.5,3))
# ax = plt.subplot(1,3,1)
# img=mpimg.imread('/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_1.jpg')
# plt.imshow(img)
# ax = plt.subplot(1,3,2)
# img=mpimg.imread('/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_2.jpg')
# plt.imshow(img)
# ax = plt.subplot(1,3,3)
# img=mpimg.imread('/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_3.jpg')
# plt.imshow(img)
# plt.colorbar()

fig, axes = plt.subplots(nrows=2, ncols=2, figsize = [10, 8])
# for i in range(4):
image = mpimg.imread(f'/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_{1}.jpg')
axes[0,0].imshow(image)
image = mpimg.imread(f'/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_{2}.jpg')
axes[0,1].imshow(image)
image = mpimg.imread(f'/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_{3}.jpg')
axes[1,0].imshow(image)
image = mpimg.imread(f'/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_{4}.jpg')
axes[1,1].imshow(image)
# Tweak the following numbers until it is right for you
plt.subplots_adjust(right = .91)
# cbar_ax = fig.add_axes([.92, 0.108, 0.03, 0.772]) 
# cbar = fig.colorbar(image, cax=cbar_ax)
plt.show()