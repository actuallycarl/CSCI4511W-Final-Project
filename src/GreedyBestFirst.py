import networkx as nx
import osmnx as ox
import heapq

def greedy_best_first_search(G, start_node, goal_node, heuristic_function):
    """
    Dirty Best-First Search algorithm for OSMnx graphs.

    Args:
        G: OSMnx graph.
        start_node: Origin node ID.
        goal_node: Destination node ID.
        heuristic_function: A function that takes two node IDs and returns an estimated cost.

    Returns:
        A list of node IDs representing the shortest path.
    """

    open_set = [(heuristic_function(start_node, goal_node), start_node)]  # Priority queue: (heuristic_cost, node)
    closed_set = set()
    came_from = {}

    while open_set:
        current_node = heapq.heappop(open_set)[1]
        if current_node == goal_node:
            return reconstruct_path(came_from, current_node)

        closed_set.add(current_node)

        for neighbor in G.neighbors(current_node):
            if neighbor not in closed_set:
                heapq.heappush(open_set, (heuristic_function(neighbor, goal_node), neighbor))
                came_from[neighbor] = current_node

    return None
def reconstruct_path(came_from, current_node):
    """
        Reconstructs the path from the origin to the destination node.

    Args:
        came_from: A dictionary mapping a node to its predecessor.
        current_node: The destination node.

    Returns:
        A list of node IDs representing the shortest path.
    """

    path = [current_node]
    count = 0
    while current_node in came_from:
        current_node = came_from[current_node]
        path.insert(0,current_node)
        count = count + 1
    print(count)
    return path


def euclidean_distance_heuristic(node1, node2):
    pos1 = G.nodes[node1]['x'], G.nodes[node1]['y']
    pos2 = G.nodes[node2]['x'], G.nodes[node2]['y']
    return ((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)**0.5

# Download a street network
G = ox.graph_from_place('Manhattan, New York')

# Get node IDs for origin and destination
origin_node = ox.nearest_nodes(G,-73.968285, 40.785091)  # Example coordinates
destination_node = ox.nearest_nodes(G, -74.00611, 40.712776)

# Find the shortest path using Dijkstra's algorithm
shortest_path = greedy_best_first_search(G, origin_node, destination_node, euclidean_distance_heuristic)

# Visualize the path
fig, ax = ox.plot_graph_route(G, shortest_path)