class Graph:
    def __init__(self):
        self.graph = {}
    
    def add_edge(self, u, v, w):
        if u in self.graph:
            self.graph[u].append((v, w))
        else:
            self.graph[u] = [(v, w)]
    
    def astar(self, start, goal):
        open_set = [(0, start)]
        came_from = {}
        g_score = {vertex: float('inf') for vertex in self.graph}
        g_score[start] = 0
        f_score = {vertex: float('inf') for vertex in self.graph}
        f_score[start] = self.heuristic(start, goal)
        
        while open_set:
            current_f_score, current_vertex = min(open_set)
            open_set.remove((current_f_score, current_vertex))
            
            if current_vertex == goal:
                return self.reconstruct_path(came_from, goal)
            
            for neighbor, weight in self.graph.get(current_vertex, []):
                tentative_g_score = g_score[current_vertex] + weight
                
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current_vertex
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    open_set.append((f_score[neighbor], neighbor))
        
        return None
    
    def heuristic(self, current, goal):
        # This heuristic function can be replaced with any appropriate
        # heuristic function, such as Euclidean distance or Manhattan distance.
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])
    
    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

# Example usage:
g = Graph()
g.add_edge((0, 0), (0, 1), 1)
g.add_edge((0, 0), (1, 0), 1)
g.add_edge((0, 1), (1, 1), 1)
g.add_edge((1, 0), (1, 1), 1)
g.add_edge((1, 0), (2, 0), 1)
g.add_edge((1, 1), (2, 1), 1)
g.add_edge((2, 0), (2, 1), 1)
g.add_edge((2, 0), (3, 0), 1)
g.add_edge((2, 1), (3, 1), 1)

start = (0, 0)
goal = (3, 1)
path = g.astar(start, goal)
print("A* Path from", start, "to", goal, ":", path)
