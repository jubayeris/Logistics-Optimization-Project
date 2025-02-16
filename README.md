# Logistics Optimization Project
# Author: Islam Md Jubayer
# Description: This script optimizes delivery routes using OR-Tools and visualizes them using Folium.

try:
    import ortools
    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp
except ModuleNotFoundError:
    raise ImportError("Error: OR-Tools library is not installed. Please install it manually using 'pip install ortools'.")

import folium
import numpy as np
from geopy.distance import great_circle

def create_distance_matrix(locations):
    size = len(locations)
    distance_matrix = np.zeros((size, size))
    
    for i in range(size):
        for j in range(size):
            if i != j:
                distance_matrix[i][j] = great_circle(locations[i], locations[j]).km
    
    return distance_matrix

def solve_vrp(locations):
    if not locations:
        raise ValueError("Error: Locations list is empty.")
    
    distance_matrix = create_distance_matrix(locations)
    num_locations = len(distance_matrix)
    num_vehicles = 1
    depot = 0
    
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)
    
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node] * 1000)
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    
    solution = routing.SolveWithParameters(search_parameters)
    
    if solution:
        route = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        return route
    else:
        raise RuntimeError("Error: No valid route found.")

def plot_route(locations, route):
    if not route:
        print("No valid route found.")
        return
    
    map_center = locations[0]
    map_route = folium.Map(location=map_center, zoom_start=12)
    
    for i, loc in enumerate(locations):
        folium.Marker(loc, popup=f"Location {i}").add_to(map_route)
    
    route_coords = [locations[i] for i in route]
    folium.PolyLine(route_coords, color='blue', weight=2.5, opacity=1).add_to(map_route)
    map_route.save("optimized_route.html")
    print("Route saved as optimized_route.html")

if __name__ == "__main__":
    delivery_locations = [(37.5665, 126.9780), (37.5759, 126.9768), (37.5800, 126.9700), (37.5622, 126.9832)]
    
    try:
        optimal_route = solve_vrp(delivery_locations)
        print("Optimized Delivery Route:", optimal_route)
        plot_route(delivery_locations, optimal_route)
    except (ValueError, RuntimeError) as e:
        print(str(e))
