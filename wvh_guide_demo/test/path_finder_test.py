#! /usr/bin/env python3
import geojson
import heapq
import numpy as np
def set_locations():
    graph = {}
    with open('/home/masettizan/ros2_ws/src/wvh_guide_demo/json/floors.geojson') as f:
        data = geojson.load(f)
        
    for feature in data['features']:
        info = {}
        info['x'] = float(feature['geometry']['coordinates'][0])
        info['y'] = float(feature['geometry']['coordinates'][1])
        info['neighbors'] = feature['properties']['neighbors']
        info['floor'] = feature['properties']['floor']
        graph[feature['properties']['id']] = info
    return graph


def find_path(graph, start, goal):
    explored = []
    queue = [[start]]

    if start == goal:
        return 
    
    while queue:
        #pop first path
        path = queue.pop(0)
        #get last node from path
        node = path[-1]

        if node not in explored:
            neighbors = graph[node]['neighbors']
            # go through neighbor nodes
            for neighbor in neighbors:
                new_path = list(path)
                new_path.append(neighbor)
                #push to queue
                queue.append(new_path)

                if neighbor == goal:
                    return new_path
                
            explored.append(node)
    return #error path does not exist

def dijkstra(graph, start, goal):
    # Priority queue to store (cost, node)
    queue = []
    heapq.heappush(queue, (0, start))
    
    # Dictionary to store the shortest path to each node
    shortest_paths = {start: (None, 0)}
    
    while queue:
        # Get the node with the lowest cost
        current_cost, current_node = heapq.heappop(queue)

        # If we reached the goal, reconstruct the path
        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = shortest_paths[current_node][0]
            return path[::-1], current_cost  # Return reversed path and cost

        # Explore neighbors
        for neighbor, weight in graph[current_node].items():
            cost = current_cost + weight
            
            # If this path is shorter, update shortest_paths and add to queue
            if neighbor not in shortest_paths or cost < shortest_paths[neighbor][1]:
                shortest_paths[neighbor] = (current_node, cost)
                heapq.heappush(queue, (cost, neighbor))
    
    return None, float('inf')  # Return None if no path exists

def calculate_edge_cost(graph, ele, st, node_name, neighbor_name):
    node = graph[node_name]
    neighbor = graph[neighbor_name]

    if node['floor'] != neighbor['floor']:
        if node_name in ele and neighbor_name in ele:
            return 1
        elif node_name in st and neighbor_name in st:
            return 10

    a = np.asarray((node['x'], node['y'])) 
    b = np.asarray((neighbor['x'], neighbor['y']))

    distance = b - a  

    weight = np.sqrt(distance[0]**2 + distance[1]**2)
    return weight

def dijkstra(graph, start, goal, ele, st):
    queue = []

    heapq.heappush(queue, (0, start))
    path = {start: (None, 0)} #(0,None)

    while queue:
        cost, node = heapq.heappop(queue)
        # print(cost, node)

        if node == goal:
            result = []
            while node is not None:
                result.append(node)
                # print(path)
                node = path[node][0]
            return result[::-1], cost  # Return reversed path and cost

        for neighbor in graph[node]['neighbors']:
            weight = cost + calculate_edge_cost(graph, ele, st, node, neighbor)
            
            if (neighbor not in path) or (weight < path[neighbor][1]):
                path[neighbor] = (node, weight)
                heapq.heappush(queue, (weight, neighbor))
            # print(path)
    return path


def generate_edges(path): 
    edges = [] 

    for idx in range(len(path) - 1):
        node = path[idx]
        next_node = path[idx + 1]
        # if edge exists then append 
        edges.append((node, next_node)) 
    return edges 

graph = set_locations()

elevators = ['f1_p7', 'f2_p1', 'f3_p1']
stairs = ['f1_p18', 'f1_p25', 'f2_p14', 'f2_p15', 'f3_p3', 'f3_p17']
path = dijkstra(graph, 'f1_p1', 'f3_p2', elevators, stairs)
print(path)
print(generate_edges(path[0]))
