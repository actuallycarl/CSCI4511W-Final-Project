import networkx as nx
import osmnx as ox
import heapq
import numpy as np
import time
import AStarSearch

if __name__ == "__main__":
    
    G = ox.graph_from_place('Lakeville, Minnesota', network_type='drive')  # This might include lengths
    G = ox.add_edge_speeds(G)
    G = ox.add_edge_travel_times(G)


    start_coordinates = [
    (-93.261524, 44.657022),
    (-93.291530, 44.700968),
    (-93.220297, 44.698576),
    (-93.224771, 44.704706)
    ]
    end_coordinates = [
    (-93.290075, 44.713909),
    (-93.233650, 44.636421),
    (-93.257610, 44.681432),
    (-93.218960, 44.674058)
    ]

    AStarSearch.run_batch(G, start_coordinates, end_coordinates)

    G1 = ox.graph_from_place('Duluth, Minnesota', network_type='drive')  
    G1 = ox.add_edge_speeds(G1)
    G1 = ox.add_edge_travel_times(G1)

    start_coordinates = [
        (-92.105083, 46.786639),  # 2000 W Superior St, Duluth, MN 55806
        (-92.097222, 46.783611),  # 506 W Superior St, Duluth, MN 55802
        (-92.104722, 46.777778),  # 120 N 1st Ave E, Duluth, MN 55802
        (-92.086944, 46.783889),  # 600 E Superior St, Duluth, MN 55802
        (-92.104722, 46.783889),  # 200 S Lake Ave, Duluth, MN 55802
        (-92.121436, 46.772293),  # 1605 Enger Tower Dr, Duluth, MN 55806
    ]

    end_coordinates = [
        (-92.104722, 46.777778),  # 120 N 1st Ave E, Duluth, MN 55802
        (-92.086944, 46.783889),  # 600 E Superior St, Duluth, MN 55802
        (-92.104722, 46.783889),  # 200 S Lake Ave, Duluth, MN 55802
        (-92.104722, 46.791111),  # 2400 W Superior St, Duluth, MN 55806
        (-92.097222, 46.783611),  # 506 W Superior St, Duluth, MN 55802
        (-92.171602, 46.737714),  # 5830 Grand Ave, Duluth, MN 55807
    ]

    AStarSearch.run_batch(G1, start_coordinates, end_coordinates)


    G2 = ox.graph_from_place('Minneapolis, Minnesota', network_type='drive')  
    G2 = ox.add_edge_speeds(G2)
    G2 = ox.add_edge_travel_times(G2)

    start_coordinates = [
        (-93.261524, 44.977778),  # 100 N 6th St, Minneapolis, MN 55403
        (-93.273611, 44.977778),  # 200 S 6th St, Minneapolis, MN 55402
        (-93.261524, 44.977778),  # 100 N 6th St, Minneapolis, MN 55403
        (-93.255000, 44.983333),  # 300 Nicollet Mall, Minneapolis, MN 55401
        (-93.261524, 44.977778),  # 100 N 6th St, Minneapolis, MN 55403
        (-93.223651, 44.977781)   # 420 23rd Ave SE, Minneapolis, MN 55455
    ]

    end_coordinates = [
        (-93.273611, 44.977778),  # 200 S 6th St, Minneapolis, MN 55402
        (-93.261524, 44.977778),  # 100 N 6th St, Minneapolis, MN 55403
        (-93.255000, 44.983333),  # 300 Nicollet Mall, Minneapolis, MN 55401
        (-93.261524, 44.977778),  # 100 N 6th St, Minneapolis, MN 55403
        (-93.273611, 44.977778),  # 200 S 6th St, Minneapolis, MN 55402
        (-93.272317, 44.959563)   # 2400 3rd Ave S, Minneapolis, MN 55404
    ]

    AStarSearch.run_batch(G2, start_coordinates, end_coordinates)
