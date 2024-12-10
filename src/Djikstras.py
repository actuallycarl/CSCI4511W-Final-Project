import networkx as nx
import osmnx as ox

def dijkstra_search_osmnx(G, origin_node, destination_node):
    """
    Dijkstra's algorithm for finding the shortest path in an OSMnx graph.

    Args:
        G: OSMnx graph.
        origin_node: Origin node ID.
        destination_node: Destination node ID.

    Returns:
        A list of node IDs representing the shortest path.
    """

    path = nx.dijkstra_path(G, origin_node, destination_node, weight='length')
    return path

# Example usage:
# Download a street network
G = ox.graph_from_place('Manhattan, New York')

# Get node IDs for origin and destination
origin_node = ox.nearest_nodes(G,-73.968285, 40.785091)  # Example coordinates
destination_node = ox.nearest_nodes(G, -74.00611, 40.712776)

# Find the shortest path using Dijkstra's algorithm
shortest_path = dijkstra_search_osmnx(G, origin_node, destination_node)

# Visualize the path
fig, ax = ox.plot_graph_route(G, shortest_path)