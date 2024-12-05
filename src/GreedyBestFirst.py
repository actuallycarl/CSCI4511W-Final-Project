import networkx as nx
import osmnx as ox
import heapq

def dirty_best_first_search(G, start_node, goal_node, heuristic_function):
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

# Example usage:
# ... (same as previous examples)
