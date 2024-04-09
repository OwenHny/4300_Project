import pandas as pd
import random
import heapq
import ast
import csv
import copy
import math

def add_edge(graph, vertex, neighbor):
   
    # give random latency to each edge
    latency = random.randint(10,500)
    add_edge(graph, vertex, neighbor, latency)


def add_edge(graph, vertex, neighbor, latency):
    """
    Add a new vertex to the graph adjacency map.

    Parameters:
    - graph (dict): The existing graph adjacency map.
    - vertex: The new vertex to be added.
    - neighbor : The new  neighbor for the vertex.
    """
     
    if vertex != neighbor: # don't allow duplicate points
        if vertex in graph:
            if neighbor not in graph[vertex]:#graph.get(vertex, []):
                graph[vertex].append((latency,neighbor))    
        else:
            graph[vertex] = [(latency,neighbor)]        

        if neighbor in graph:
            if vertex not in graph[neighbor]:#graph.get(neighbor, []):
                graph[neighbor].append((latency, vertex))
        else:
            graph[neighbor] = [(latency, vertex)]

   

def remove_edge(graph, point1, point2):
    if point2 in graph[point1]:
        graph[point1].remove(point2)
    for point in graph[point2[1]]:
        if point[1] == point1:
            graph[point2[1]].remove(point)
            break

def dijkstra(graph, start, end):
    visited = set()
    distances = {node: [float('infinity'),[]] for node in graph}
    distances[start] = [0,[]]
    priority_queue = [(0, (0,start))]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node[1] not in visited:
            visited.add(current_node[1])
        
            for neighbor in graph[current_node[1]]:
                distance = current_distance + 1

                if distance < distances[neighbor[1]][0]:
                    distances[neighbor[1]][1] = copy.deepcopy(distances[current_node[1]][1])
                    distances[neighbor[1]][1].append(current_node)
                    if neighbor[1] == end:
                        distances[neighbor[1]][1].append(neighbor)

                    distances[neighbor[1]][0] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))

    return distances[end]

def dijkstra_weight(graph, start, end):
    visited = set()
    distances = {node: [float('infinity'),[]]for node in graph}
    distances[start] = [0,[]]
    priority_queue = [(0, (0,start))]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node[1] not in visited:
            visited.add(current_node[1])
        
            for neighbor in graph[current_node[1]]:
                distance = current_distance + neighbor[0]

                if distance < distances[neighbor[1]][0]:
                    distances[neighbor[1]][1] = copy.deepcopy(distances[current_node[1]][1])
                    distances[neighbor[1]][1].append(current_node)
                    if neighbor[1] == end:
                        distances[neighbor[1]][1].append(neighbor)


                    distances[neighbor[1]][0] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))

    return distances[end]


def build_graph(graph):
    # import csv
    location = pd.read_csv('out.csv')
    links = pd.read_csv('links2.csv')

    location['-_x'] = location['-_x'].astype(str)
    links['to'] = links['to'].astype(str)
    links['from'] = links['from'].astype(str)

    locations = {}

    # loop through all locations, add to dict
    for index, row in location.iterrows():
        if index % 10000 == 0:
            print(index)

        match = row['-_x']
        if match != '-':
            locations[row['-_x']] = (row['0.000000'], row['0.000000.1'])

    # for each link in links, add edge
    for  index, row in links.iterrows():
        if index % 10000 == 0:
            print(index)

        if row['to'] in locations and row['from'] in locations:
            start = locations[row['to']]
            end = locations[row['from']]
            add_edge(graph, start, end)

    with open("graph.txt", mode='w', newline='') as file:
        file.write(str(graph) )

# pick a bunch of random nodes to route between
def make_paths(graph):
    num_keys = len(graph.keys())
    with open("tests.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["start", "end", "distance", "latency"])
        for routes in range(300):
            start = random.randrange(num_keys)
            start_node = list(graph.keys())[start] 
            end_node = start_node
            end = start
            distance = 0
            latency = 0
            while end == start and distance < 2: 
                end = random.randrange(num_keys)
                end_node = list(graph.keys())[end]
                # calc shortest path and subsequent latency
                distance = dijkstra(graph,start_node, end_node)     
                latency = dijkstra_weight(graph, start_node, end_node)
            print(start_node, end_node, distance[0], distance[1], latency)
            writer.writerow([start_node, end_node, distance, latency])

def distance_calc(point1, point2):
    return (math.sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))) 

def planar(graph):
    planar = copy.deepcopy(graph) 
    for node in graph: # loop for each node in graph
        for neighbor in graph[node]: # loop through all neighbors of a node
            cx = (neighbor[1][0] - node[0])/2 + node[0]
            cy = (neighbor[1][1] - node[1])/2 + node[1]
            r = distance_calc(node, (cx, cy))
            for point in graph[node]: # check if any other neighbors of the node are in the collision range, if so remove the edge
                point = point[1]
                if neighbor != point and distance_calc(point, (cx,cy)) < r:
                    remove_edge(planar, node, neighbor)         
                    break
    empty = 0
    for node in graph:
        if len(planar[node]) == 0: # if all neighbors have been removed, add the one with the least distance back
            closest = graph[node][0]
            for neighbor in graph[node]:
                if distance_calc(neighbor[1], node) < distance_calc(closest[1], node):
                    closest = neighbor
            add_edge(planar, node, closest[1], closest[0])
            empty +=1

    print(empty)
    with open("planar.txt", mode='w', newline='') as file:
        file.write(str(planar))

def unitdisk(graph):
    disk = copy.deepcopy(graph)
    for node in graph:
        for neighbor in graph[node]:
            if distance_calc(node, neighbor[1]) > 50:
                remove_edge(disk, node, neighbor)

    empty = 0
    for node in graph:
        if len(disk[node]) == 0:
            closest = graph[node][0]
            for neighbor in graph[node]:
                if distance_calc(neighbor[1], node) < distance_calc(closest[1], node):
                    closest = neighbor
            add_edge(disk, node, closest[1], closest[0])
            empty +=1
    
    print(empty)
    with open("disk.txt", mode='w', newline='') as file:
        file.write(str(disk))

if __name__ == '__main__':
    with open("graph.txt", "r") as file:
        graph = ast.literal_eval(file.read())
        total = 0
        for node in graph:
            total += len(graph[node])
        print(len(graph), total / len(graph))
        #unitdisk(graph) 
        #planar(graph)

        #make_paths(graph)

    #graph = {}
    #build_graph(graph)
