import networkx as nx
import osmnx as ox
import heapq

def a_star_search_osmnx(G, origin_node, destination_node, heuristic_function):
    """
    A* search algorithm adapted for OSMnx graphs.

    Args:
        G: OSMnx graph.
        origin_node: Origin node ID.
        destination_node: Destination node ID.
        heuristic_function: A function that takes two node IDs and returns an estimated cost.

    Returns:
        A list of node IDs representing the shortest path.
    """

    open_set = [(0, origin_node)]  # Priority queue: (f_score, node)
    closed_set = set()
    came_from = {}
    g_score = {node: float('infinity') for node in G.nodes}
    g_score[origin_node] = 0

    count = 0
    while open_set:
        current_node = heapq.heappop(open_set)[1]
        print(current_node)
        print(destination_node)
        if current_node == destination_node:
            return reconstruct_path(came_from, current_node)
        print(count)
        closed_set.add(current_node)

        for neighbor in G.neighbors(current_node):
            tentative_g_score = g_score[current_node] + G.get_edge_data(current_node,neighbor)[0]['travel_time']
            if neighbor in closed_set and tentative_g_score >= g_score[neighbor]:
                continue
            if neighbor not in [node for _, node in open_set] or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score = g_score[neighbor] + heuristic_function(neighbor, destination_node)
                heapq.heappush(open_set, (f_score, neighbor))

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
# Example usage:
# Download a street network
G = ox.graph_from_place('Manhattan, New York', network_type='drive')  # This might include lengths
G = ox.add_edge_speeds(G)
G = ox.add_edge_travel_times(G)

# Get node IDs for origin and destination
origin_node = ox.nearest_nodes(G, -73.99845984, 40.712776)  # Example coordinates
destination_node = ox.nearest_nodes(G, -73.968285, 40.785091)

print(origin_node)
print(destination_node)
# Define a simple heuristic (e.g., Euclidean distance)
def euclidean_distance_heuristic(node1, node2):
    pos1 = G.nodes[node1]['x'], G.nodes[node1]['y']
    pos2 = G.nodes[node2]['x'], G.nodes[node2]['y']
    return ((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)**0.5

# Find the shortest path
shortest_path = a_star_search_osmnx(G, origin_node, destination_node, euclidean_distance_heuristic)

# Visualize the path
fig, ax = ox.plot_graph_route(G, shortest_path)