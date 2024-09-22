import numpy as np
import matplotlib.pyplot as plt
import time

class AntColonyOptimization:
    def __init__(self, start, goal, obstacles, obstacle_radius, num_ants=10, max_iter=100, alpha=1.0, beta=2.0, evaporation_rate=0.5):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.obstacle_radius = obstacle_radius
        self.num_ants = num_ants
        self.max_iter = max_iter
        self.alpha = alpha
        self.beta = beta
        self.evaporation_rate = evaporation_rate
        self.pheromone = np.ones((goal[0]+1, goal[1]+1, goal[2]+1), dtype=np.float64)

    def run(self):
        best_path = None
        best_path_length = float('inf')

        for _ in range(self.max_iter):
            paths = self.construct_paths(self.pheromone)
            self.update_pheromone(self.pheromone, paths)
            current_best_path, current_best_length = self.get_best_path(paths)

            if current_best_length < best_path_length:
                best_path = current_best_path
                best_path_length = current_best_length

        return best_path

    def construct_paths(self, pheromone):
        paths = []
        for _ in range(self.num_ants):
            path = [self.start]
            current_node = self.start
            while current_node != self.goal:
                neighbors = self.get_feasible_neighbors(current_node)
                probabilities = self.calculate_probabilities(current_node, neighbors, pheromone)
                next_node = np.random.choice(neighbors, p=probabilities)
                path.append(next_node)
                current_node = next_node
            paths.append(path)
        return paths

    def calculate_probabilities(self, current_node, neighbors, pheromone):
        probabilities = []
        total = 0
        for neighbor in neighbors:
            if neighbor not in current_node:
                heuristic = 1.0 / np.linalg.norm(np.array(neighbor) - np.array(self.goal))
                pheromone_level = pheromone[neighbor[0]][neighbor[1]][neighbor[2]]
                probabilities.append((pheromone_level ** self.alpha) * (heuristic ** self.beta))
                total += probabilities[-1]
        return [p / total for p in probabilities]

    def update_pheromone(self, pheromone, paths):
        pheromone *= (1 - self.evaporation_rate)  # Evaporation
        for path in paths:
            for node in path:
                pheromone[node[0]][node[1]][node[2]] += 1.0 / len(path)

    def get_feasible_neighbors(self, node):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neighbor = (node[0] + dx, node[1] + dy, node[2] + dz)
                    too_close = False
                    for obstacle in self.obstacles:
                        if np.linalg.norm(np.array(neighbor) - np.array(obstacle)) < self.obstacle_radius:
                            too_close = True
                            break
                    if not too_close:
                        neighbors.append(neighbor)
        return neighbors

    def get_best_path(self, paths):
        best_path = None
        best_length = float('inf')
        for path in paths:
            length = self.calculate_path_length(path)
            if length < best_length:
                best_path = path
                best_length = length
        return best_path, best_length

    def calculate_path_length(self, path):
        length = 0
        for i in range(len(path) - 1):
            length += np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
        return length

# Örnek kullanım
start = (0, 0, 0)
goal = (5, 5, 5)
obstacles = [(2, 2, 2), (3, 3, 3)]
obstacle_radius = 1

aco = AntColonyOptimization(start, goal, obstacles, obstacle_radius)
best_path = aco.run()
print("Best path:", best_path)
