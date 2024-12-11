import networkx as nx
import osmnx as ox
import heapq
import numpy as np
import time

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
        if current_node == destination_node:
            return reconstruct_path(came_from, current_node)
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
    while current_node in came_from:
        current_node = came_from[current_node]
        path.insert(0,current_node)
    return path
# Example usage:
# Download a street network
G = 0 #declare g
# Get node IDs for origin and destination
min_costs = []
landmarks = []
fastmap_embedding = {}
num_landmarks = 10 # good test case for landmarks

# Define a simple heuristic (e.g., Euclidean distance)
def euclidean_distance_heuristic(node1, node2):
    pos1 = G.nodes[node1]['x'], G.nodes[node1]['y']
    pos2 = G.nodes[node2]['x'], G.nodes[node2]['y']
    return ((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)**0.5

def greedy_landmark_selection(G, num_landmarks, origin_node):
  # Greedy Refinement
  landmarks = []
  landmarks.append(origin_node)
  num_nodes = G.number_of_nodes()

  for _ in range(num_landmarks):
    max_avg_time = 0
    next_landmark = None

    for v in G.nodes():
      if v not in landmarks:
        avg_time = 0
        for l in landmarks:
            avg_time = avg_time + euclidean_distance_heuristic(l,v)
        avg_time = avg_time / len(landmarks)
        if avg_time > max_avg_time:
          max_avg_time = avg_time
          next_landmark = v

    landmarks.append(next_landmark)

  return landmarks

def compute_minimal_costs(G, landmarks):
  num_nodes = G.number_of_nodes()
  min_costs = {}


  for landmark in landmarks:
    min_costs[landmark] = {}
    distances = nx.single_source_dijkstra_path_length(G, landmark)
    for node in distances.items():
      min_costs[landmark][node] = float('inf')
      for travel_time in distances.items():
        min_costs[landmark][node] = min(min_costs[landmark][node], travel_time[0])

  return min_costs

# differential landmark heuristic
def alt_heuristic(node1, node2):
    """
    Calculates the ALT heuristic between two nodes using landmarks.

    Args:
        G: The graph.
        node1: The first node.
        node2: The second node.
        landmarks: A list of landmark nodes.

    Returns:
        The estimated distance between node1 and node2.
    """

    max_diff = 0
    if (node1 == node2):
        return 0
    for landmark in landmarks:
        if node1 in min_costs[landmark]:
            diff = abs(min_costs[landmark][node1] - min_costs[landmark][node2])
            max_diff = max(max_diff, diff)
    return max_diff

def find_furthest_vertices(G):
    """
    Finds the furthest pair of vertices in a graph G.

    Args:
        G: NetworkX graph.

    Returns:
        A tuple (a1, b1) representing the furthest vertices.
    """

    # Compute all-pairs shortest path distances
    path_lengths = dict(nx.all_pairs_shortest_path_length(G))

    max_distance = 0
    furthest_pair = None
    for u, distances in path_lengths.items():
        for v, distance in distances.items():
            if u != v and distance > max_distance:
                max_distance = distance
                furthest_pair = (u, v)

    return furthest_pair

def embed_fastmap(G):
    furthest_pair = find_furthest_vertices(G)
    fastmap_embedding = {}
    distances_a = nx.single_source_dijkstra_path_length(G, furthest_pair[0])
    distances_b = nx.single_source_dijkstra_path_length(G, furthest_pair[1])
    for node in distances_a.items():
        min_av = float('inf')
        min_bv = float('inf')
        for travel_time in distances_a.items():
            min_av = min(min_av, travel_time[0])
        for travel_time in distances_b.items():
            min_bv = min(min_bv, travel_time[0])
        fastmap_embedding[node] = .5 * (min_av-min_bv)

    return fastmap_embedding

def fastmap_heuristic(node1, node2):
    # Extract embeddings
    if node1 not in fastmap_embedding or node2 not in fastmap_embedding:
        return float('inf')
    embedding1 = fastmap_embedding[node1]
    embedding2 = fastmap_embedding[node2]

    distance = abs(embedding1 - embedding2)

    return distance

def manhattan_distance_heuristic(node1, node2):
    """
    Calculates the Manhattan distance heuristic between two nodes.

    Args:
        node1: The first node.
        node2: The second node.

    Returns:
        The Manhattan distance between the two nodes.
    """

    pos1 = G.nodes[node1]['x'], G.nodes[node1]['y']
    pos2 = G.nodes[node2]['x'], G.nodes[node2]['y']
    return abs(pos2[0] - pos1[0]) + abs(pos2[1] - pos1[1])

def find_travel_time(G, path):
    """
    Calculates the total travel time along a given path in a graph.

    Args:
        G: The NetworkX graph.
        path: A list of nodes representing the path.

    Returns:
        The total travel time along the path.
    """

    total_time = 0
    gdf = ox.utils_graph.route_to_gdf(G, path, "travel_time")
    total_time = gdf["travel_time"].sum()
        

    return total_time


def run_batch(Map, start_coordinates, end_coordinates):
    global G, landmarks, min_costs, fastmap_embedding
    G = Map
    start_time = time.time()
    landmarks = greedy_landmark_selection(G, num_landmarks, ox.nearest_nodes(G, start_coordinates[0][0], start_coordinates[0][1])) # Landmark generation
    min_costs = compute_minimal_costs(G, landmarks) # ALT embedding
    elapsed_time = time.time() - start_time
    print("ALT landmark selection setup time: ")
    print(elapsed_time)
    start_time = time.time()
    fastmap_embedding = embed_fastmap(G) # Generate Fastmap Embedding
    elapsed_time = time.time() - start_time
    print("Fastmap embedding setup time: ")
    print(elapsed_time)

    # Run each pair of nodes and report elapsed times
    for i in range(0, len(start_coordinates)):
        print("---------- Route (%2.5f, %2.5f) to (%2.5f, %2.5f) -------------"  % (start_coordinates[i][0], start_coordinates[i][1],end_coordinates[i][0], end_coordinates[i][1]))
        origin_node = ox.nearest_nodes(G, start_coordinates[i][0], start_coordinates[i][1])
        destination_node = ox.nearest_nodes(G, end_coordinates[i][0], end_coordinates[i][1])
        start_time = time.time()
        shortest_path_euclidean = a_star_search_osmnx(G, origin_node, destination_node, euclidean_distance_heuristic)
        elapsed_time = time.time() - start_time
        print("A* Euclidiean distance heuristic: %2.8f s" % elapsed_time)
        travel_time = find_travel_time(G, shortest_path_euclidean)
        print("travel_time: %6.3f" % travel_time)
        start_time = time.time()
        shortest_path_manhattan = a_star_search_osmnx(G, origin_node, destination_node, manhattan_distance_heuristic)
        elapsed_time = time.time() - start_time
        print("A* Manhattan distance heuristic: %2.8f s" % elapsed_time)
        travel_time = find_travel_time(G, shortest_path_manhattan)
        print("travel_time: %6.3f" % travel_time)
        start_time = time.time()
        shortest_path_differential = a_star_search_osmnx(G, origin_node, destination_node, alt_heuristic)
        elapsed_time = time.time() - start_time
        print("A* ALT Heuristic: %2.8f s" % elapsed_time)
        travel_time = find_travel_time(G, shortest_path_differential)
        print("travel_time: %6.3f" % travel_time)
        start_time = time.time()
        shortest_path_fastmap = a_star_search_osmnx(G, origin_node, destination_node, fastmap_heuristic)
        elapsed_time = time.time() - start_time
        print("A* Fastmap Heuristic: %2.8f s" % elapsed_time)
        travel_time = find_travel_time(G, shortest_path_fastmap)
        print("travel_time: %6.3f" % travel_time)
        # Djikstra's
        start_time = time.time()

        shortest_path_djikstras = nx.dijkstra_path(G, origin_node, destination_node, weight='travel_time')
        elapsed_time = time.time() - start_time
        print("Djikstras: %2.8f s" % elapsed_time)
        travel_time = find_travel_time(G, shortest_path_djikstras)
        print("travel_time: %6.3f" % travel_time)

        # Greedy Best first search
        origin_node = ox.nearest_nodes(G, start_coordinates[i][0], start_coordinates[i][1])
        destination_node = ox.nearest_nodes(G, end_coordinates[i][0], end_coordinates[i][1])
        start_time = time.time()
        greedy_path_euclidean = greedy_best_first_search(G, origin_node, destination_node, euclidean_distance_heuristic)
        elapsed_time = time.time() - start_time
        print("Greedy Euclidiean Heuristic: %2.8f s" % elapsed_time)
        travel_time = find_travel_time(G, greedy_path_euclidean)
        print("travel_time: %6.3f" % travel_time)
        start_time = time.time()
        greedy_path_differential = greedy_best_first_search(G, origin_node, destination_node, alt_heuristic)
        elapsed_time = time.time() - start_time
        print("Greedy ALT Heuristic: %2.8f s" % elapsed_time)
        travel_time = find_travel_time(G, greedy_path_differential)
        print("travel_time: %6.3f" % travel_time)
        start_time = time.time()
        greedy_path_manhattan = a_star_search_osmnx(G, origin_node, destination_node, manhattan_distance_heuristic)
        elapsed_time = time.time() - start_time
        print("Greedy Manhattan distance heuristic: %2.8f s" % elapsed_time)
        travel_time = find_travel_time(G, greedy_path_manhattan)
        print("travel_time: %6.3f" % travel_time)
        start_time = time.time()
        greedy_path_fastmap = greedy_best_first_search(G, origin_node, destination_node, fastmap_heuristic)
        elapsed_time = time.time() - start_time
        print("Greedy Fastmap Heuristic: %2.8f s" % elapsed_time)
        travel_time = find_travel_time(G, greedy_path_fastmap)
        print("travel_time: %6.3f" % travel_time)

# Visualize the path
# fig, ax = ox.plot_graph_route(G, shortest_path_differential)