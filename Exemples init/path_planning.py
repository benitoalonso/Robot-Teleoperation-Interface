import numpy as np
from lab_utils.plan_utils import *
from lab_utils.astart import AStarPlanner
from lab_utils.rrt import RRTPlanner
from matplotlib import pyplot as plt
from matplotlib import image as mpimg
import os

 
## IMPORTATION DE LA CARTE
# Lecture de l'image
name = "a2230_map_closed_fliped.png"
path_map = os.path.join(os.getcwd(), "Exemples prof")
path_map = os.path.join(path_map, name)
image = mpimg.imread(path_map)
# Définition du point de départ et d'arrivée
#start = Point(1273, 1415)
#end = Point(1214, 1225)
start = Point(1490, 1414)
end = Point(1490, 988)

# Affichage de la carte avec points de départ et d'arrivée
fig, ax = plt.subplots()
plt.imshow(image)
ax.scatter(start.x, start.y, s=20, color='r', label='Start')
ax.scatter(end.x, end.y, s=20, color='b', label='End')
plt.title('Importation de la carte')
plt.legend()
plt.savefig('carte')

# Conversion de l'image en matrice
map_img = 1-np.array(image[:,:,1])
mat_map = map_img
map = BMPMap(width=map_img.shape[1], height=map_img.shape[0], mat=mat_map)


## Trajectoire A*
# Initialisation du Planner
astarPlanner = AStarPlanner(map=map, step_size=10, heuristic_dist='Euclidean')
astarPlanner.plan(start=start, target=end)

# Affichage des trajectoires étudiées
fig = plt.figure(figsize=(16,5))
ax = fig.add_subplot(1, 2, 1)
plt.imshow(image)
ax.scatter(start.x, start.y, s=20, color='r', label='Start')
ax.scatter(end.x, end.y, s=20, color='b', label='End')
for i in range(len(astarPlanner.open_list)-1):
    pt = astarPlanner.open_list[i].pos.tuple()
    ax.add_artist(plt.Circle((pt[0], pt[1]), 1, color='gray'))
for i in range(len(astarPlanner.close_list)-1):
    pt = astarPlanner.close_list[i].pos.tuple()
    ax.add_artist(plt.Circle((pt[0], pt[1]), 1, color='gray'))
ax.set_title('Possible trajectories A*')

# Affichage de la trajectoire retenue
ax2 = fig.add_subplot(1, 2, 2)
plt.imshow(image)
ax2.scatter(start.x, start.y, s=20, color='r', label='Start')
ax2.scatter(end.x, end.y, s=20, color='b', label='End')
for i in range(len(astarPlanner.finalPath)-1):
    pt = astarPlanner.finalPath[i].tuple()
    ax2.add_artist(plt.Circle((pt[0], pt[1]), 5, color='g'))
ax2.set_title('Path planning A*')
plt.legend()
plt.show()
plt.savefig('Path A')


## Trajectoire avec RRT
# Initialisation du planner
rrtPlanner = RRTPlanner(map, epsilon=0.05, stepSize=20)
rrtPlanner.plan(start=start, target=end)

# Affichage des trajectoires étudiées
fig = plt.figure(figsize=(16,5))
ax = fig.add_subplot(1, 2, 1)
plt.imshow(image)
ax.scatter(start.x, start.y, s=20, color='r', label='Start')
ax.scatter(end.x, end.y, s=20, color='b', label='End')
for i in range(len(rrtPlanner.nodeList)-1):
    pt = rrtPlanner.nodeList[i].pos.tuple()
    ax.add_artist(plt.Circle((pt[0], pt[1]), 1, color='gray'))
ax.set_title('Possible trajectories RRT')

# Affichage de la trajectoire retenue
ax2 = fig.add_subplot(1, 2, 2)
plt.imshow(image)
ax2.scatter(start.x, start.y, s=20, color='r', label='Start')
ax2.scatter(end.x, end.y, s=20, color='b', label='End')
for i in range(len(rrtPlanner.finalPath)-1):
    pt = rrtPlanner.finalPath[i].tuple()
    ax2.add_artist(plt.Circle((pt[0], pt[1]), 5, color='g'))
ax2.set_title('Path planning RRT')
plt.legend()
plt.savefig('Path RRT')