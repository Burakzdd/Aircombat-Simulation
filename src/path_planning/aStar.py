import numpy as np
import heapq
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def astar(start, goal, obstacles, obstacle_radius):
    open_list = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    
    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        # Bitiş noktasına 1 birim mesafe kaldığında dur
        if np.linalg.norm(np.array(current_node) - np.array(goal)) <= 1:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            path.reverse()
            return path
        
        for neighbor, weight in get_neighbors(current_node, obstacles, obstacle_radius).items():
            tentative_g_score = g_score[current_node] + weight
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + manhattenDistance(neighbor, goal)
                heapq.heappush(open_list, (f_score, neighbor))
                came_from[neighbor] = current_node
    
    return None

def get_neighbors(node, obstacles, obstacle_radius):
    neighbors = {}
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                neighbor = (node[0] + dx, node[1] + dy, node[2] + dz)
                too_close = False
                for obstacle in obstacles:
                    if np.linalg.norm(np.array(neighbor) - np.array(obstacle)) < obstacle_radius:
                        too_close = True
                        break
                if too_close:
                    continue
                neighbors[neighbor] = 1  # Ağırlık her zaman 1 olarak alınmıştır
    return neighbors

def manhattenDistance(current, goal):
    return np.abs(current[0] - goal[0]) + np.abs(current[1] - goal[1]) + np.abs(current[2] - goal[2])

# # Örnek kullanım:
# start = (4, 0, 10)
# goal = (-6, -6, 13)
# obstacles = [[ 4, -8, 13], [-8,  6, 13], [ 7,  7, 13]]
# obstacle_radius = 2

# path = astar(start, goal, obstacles, obstacle_radius)
# print("AStar ile hesaplanan yol:", path)

# # 3B olarak çizim
# fig = plt.figure(figsize=(10, 7))  # Daha geniş bir figür boyutu belirle
# ax = fig.add_subplot(111, projection='3d')

# # Obstacles
# obs_x, obs_y, obs_z = zip(*obstacles)
# ax.scatter(obs_x, obs_y, obs_z, c='r', marker='o', s=100, label='Obstacles')

# # Path
# if path:
#     path_x, path_y, path_z = zip(*path)
#     ax.plot(path_x, path_y, path_z, c='b', label='Path')
#     # Path üzerindeki noktalara işaret ekleme
#     ax.scatter(path_x, path_y, path_z, c='b', marker='o', s=50, label='Path Points')
# else:
#     print("Bir yol bulunamadı.")

# # Start and Goal
# ax.scatter(start[0], start[1], start[2], c='g', marker='x', s=100, label='Start')
# ax.scatter(goal[0], goal[1], goal[2], c='k', marker='x', s=100, label='Goal')

# ax.set_xlabel('X axis')
# ax.set_ylabel('Y axis')
# ax.set_zlabel('Z axis')

# # Yan tarafta bilgi paneli
# info_text = "Start: {}\nGoal: {}\n\nObstacles:\n".format(start, goal)
# for obs in obstacles:
#     info_text += "{}\n".format(obs)
# if path:
#     info_text += "\nPath:\n"
#     for p in path:
#         info_text += "{}\n".format(p)
# else:
#     info_text += "\nNo path found."

# # Bilgi metnini sağ üst köşeye yerleştir
# plt.figtext(0.15, 0.85, info_text, fontsize=10, va='top', ha='left', bbox=dict(facecolor='white', alpha=0.8))

# plt.legend()
# plt.show()

