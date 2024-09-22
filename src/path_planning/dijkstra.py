import numpy as np
import heapq

def dijkstra(start, goal, obstacles, obstacle_radius):
    open_list = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    
    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        # Hedefe ulaşıldığında dur
        if current_node == goal:
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
                heapq.heappush(open_list, (tentative_g_score, neighbor))
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

# # Örnek kullanım:
start = (0, 0, 3)
goal = (5, 6, 10)
obstacles = [(2, 3, 1), (4, 2, 3),(6, 3, 2), (3, 4, 5)]
obstacle_radius = 2


path = dijkstra(start, goal, obstacles, obstacle_radius)
print("Dijkstra ile hesaplanan yol:", path)