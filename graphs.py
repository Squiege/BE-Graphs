import heapq

class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}

    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight  # For undirected graph

    def get_neighbors(self, vertex):
        if vertex in self.vertices:
            return self.vertices[vertex]
        else:
            return {}

    def dijkstra(self, source):
        # Initialize distances and priority queue
        distances = {vertex: float('inf') for vertex in self.vertices}
        distances[source] = 0
        priority_queue = [(0, source)]  # (distance, vertex)
        visited = set()
        previous = {vertex: None for vertex in self.vertices}  # Track shortest path tree

        while priority_queue:
            current_distance, current_vertex = heapq.heappop(priority_queue)

            if current_vertex in visited:
                continue

            visited.add(current_vertex)

            # Update neighbors
            for neighbor, weight in self.get_neighbors(current_vertex).items():
                if neighbor not in visited:
                    distance = current_distance + weight

                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        previous[neighbor] = current_vertex
                        heapq.heappush(priority_queue, (distance, neighbor))

        # Return distances and shortest path
        return distances, previous

    def reconstruct_path(self, previous, target):
        # Reconstruct path to the target
        path = []
        while target is not None:
            path.append(target)
            target = previous[target]
        path.reverse()
        return path

# Creating Nodes and Edges
graph = Graph()
graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_vertex('D')
graph.add_edge('A', 'B', 1)
graph.add_edge('B', 'C', 3)
graph.add_edge('A', 'C', 10)
graph.add_edge('B', 'D', 2)
graph.add_edge('C', 'D', 1)

# Compute shortest paths
distances, previous = graph.dijkstra('A')

# Print shortest distances from A
print("Shortest distances:", distances)

# Reconstruct the shortest path from A to D
path_to_d = graph.reconstruct_path(previous, 'D')
print("Shortest path to D:", path_to_d)
