import pandas
import math
import ast
import random
import csv

BUMPED_ZERO = 0.000001
ROUTING_FAILURE = 3000

num_steps = 0
path = []
time_total = 0

def route(graph_file, routes_file, output_name):
    global num_steps, path, time_total
    with open(graph_file, "r") as graph_txt:
        graph = ast.literal_eval(graph_txt.read())
        

        print("graph in")
        with open(routes_file, "r") as file:
            with open(output_name + ".csv", "w") as output_file:
                output = csv.writer(output_file)

                csvFile = pandas.read_csv(file)

                fail = 0
                success = 0

                for case in csvFile.iterrows():
                    print(fail, success)
                    if fail > success * 3 and fail > 10: # routing does not work abort
                        break
                    start = ast.literal_eval(case[1].start)
                    end = ast.literal_eval(case[1].end)

                    num_steps = 0
                    path = []
                    path.append((0,start))
                    #greedy(start, end, graph)
                    compass(start, end, graph)
                    
                    if num_steps < ROUTING_FAILURE:
                        latency = ast.literal_eval(case[1].latency)
                        distance = ast.literal_eval(case[1].distance)
                        time = sum_latency(path, latency[0])
                                
                        output.writerow([len(path)/distance[0], time, path])
                        time_total += time
                        success += 1
                        #print(num_steps, path, case[1].distance, case[1].latency)
                        #print(num_steps, case[1].distance)
                        #print(success, fail)
                    else:
                        fail += 1
                        print("fail")
                        output.writerow([[],0])
                
                output.writerow([success, fail, fail/success, time_total/success])

def sum_latency(path, least):
    path_total = 0
    for node in path:
        path_total += node[0]

    return path_total / least

def add_node(node, neighborhood):
    global path
    test = len(path) 
    for neighbor in neighborhood:
        if node == neighbor[1]:
            path.append(neighbor) 
            break

def distance_calc(point1, point2):
    return (math.sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))) 

def calc_angle(center, point1, point2):
    return math.atan2(point2[1] - center[1], point2[0] - center[0]) - math.atan2(point1[1] - center[1], point1[0] - center[0])

def calc_intersect_angle(point1, point2, point3, point4):
    A = point4[1] - point3[1]
    B = point3[0] - point4[0]
    C = point3[1]*(point4[0] - point3[0]) - point3[0]*(point4[1] - point3[1])
    line = [A,B,C]

    A = point2[1] - point1[1]
    B = point1[0] - point2[0]
    C = point1[1]*(point2[0] - point1[0]) - point1[0]*(point2[1] - point1[1])

    if line[0]*B - A * line[1] != 0:
        x = (line[1] * C - B * line[2])/(line[0]*B - A * line[1])
        y = (A * line[2] - line[0] * C)/(line[0]*B - A * line[1])

        if (x <= point1[0] and x >= point2[0] or x >= point1[0] and x <= point2[0]) and (y <= point1[1] and y >=  point2[1] or y >= point1[1] and y <= point2[1]):
            if (x <= point3[0] and x >= point4[0] or x >= point3[0] and x <= point4[0]) and (y <= point3[1] and y >= point4[1] or y >= point3[1] and y <= point4[1]):
                return calc_angle((x,y), point2, point4)
    return 0
    
def crossing_points(point1, point2, point3, point4):
    
    A = point4[1] - point3[1]
    B = point3[0] - point4[0]
    C = point3[1]*(point4[0] - point3[0]) - point3[0]*(point4[1] - point3[1])
    line = [A,B,C]

    A = point2[1] - point1[1]
    B = point1[0] - point2[0]
    C = point1[1]*(point2[0] - point1[0]) - point1[0]*(point2[1] - point1[1])

    if line[0]*B - A * line[1] != 0:
        x = (line[1] * C - B * line[2])/(line[0]*B - A * line[1])
        y = (A * line[2] - line[0] * C)/(line[0]*B - A * line[1])

        if (x < point1[0] and x > point2[0] or x > point1[0] and x < point2[0]) and (y < point1[1] and y >  point2[1] or y > point1[1] and y < point2[1]):
            if ((x < point3[0] and x > point4[0] or x > point3[0] and x < point4[0]) and (y < point3[1] and y > point4[1] or y > point3[1] and y < point4[1])):
                return distance_calc((x,y), point4)
                
    return 0

def crossing_point(point1, point2, Line, end):
    A = point2[1] - point1[1]
    B = point1[0] - point2[0]
    C = point1[1]*(point2[0] - point1[0]) - point1[0]*(point2[1] - point1[1])

    if Line[0]*B - A * Line[1] != 0:
        x = (Line[1] * C - B * Line[2])/(Line[0]*B - A * Line[1])
        y = (A * Line[2] - Line[0] * C)/(Line[0]*B - A * Line[1])

        if (x <= point1[0] and x >= point2[0] or x >= point1[0] and x <= point2[0]) and (y <= point1[1] and y >= point2[1] or y >= point1[1] and y <= point2[1]):
            return distance_calc((x,y), end)
    return 0

def greedy(start, end, graph):
    global num_steps, path
    node = start

    while node != end:
        if num_steps > ROUTING_FAILURE:
            return 
        next_node = node
        distance_total = distance_calc(node, end) 
        distance = 0

        for neighbor in graph[node]:
            if neighbor[1] == end:
                path.append(neighbor)
                num_steps += 1
                return
            if (distance_total - distance_calc(neighbor[1], end))/neighbor[0] > distance :
                next_node = neighbor
                distance = (distance_total - distance_calc(neighbor[1], end))/neighbor[0] 
        if next_node == node:          
            #node = void(node, end, graph)
            #node = left_nonplanar(node, end, graph)
            #node = left(node, end, graph)
            node = random_recovery(node, end, graph)
        else:
            path.append(next_node)
            node = next_node[1]
            num_steps = num_steps + 1 

def compass(start, end, graph,):
    global num_steps, path 
    node = start

    while node != end:
        if num_steps > ROUTING_FAILURE:
            return
        next_node = node
        min_angle = math.pi/2
        for neighbor in graph[node]:
            if neighbor[1] == end:
                path.append(neighbor)
                num_steps = num_steps + 1
                return None
            angle = abs(calc_angle(node,end, neighbor[1]))
            if angle > math.pi:
                angle = math.pi * 2 - angle
            if min_angle >= angle: 
                min_angle = angle
                next_node = neighbor

        if next_node == node: #or num_steps > step_limit:
            node = random_recovery(node, end, graph)
        else:
            path.append(next_node)
            node = next_node[1]
            num_steps = num_steps + 1

def random_recovery(node, end, graph):
    global num_steps, path
    
    counter = 0
    while counter < 5:
        for neighbor in graph[node]:
            if neighbor[1] == end:
                path.append(neighbor)
                return end
        next = 0
        for x in range(5):
            index = random.randint(0, len(graph[node]) - 1)
            if next == 0 or graph[node][index][0] < next[0]:
                next = graph[node][index]
        path.append(next)
        node = next[1]
        counter += 1
        num_steps +=1

    return node
        

def left(node, end, graph):
    # left routing, doing planar calcs during forwarding descision
    global num_steps, path
    stop = node
    next = node
    last = end

    while node != end and distance_calc(node, end) >= distance_calc(stop, end): 
        if num_steps > ROUTING_FAILURE:
            print("Left failure")
            return end
            #raise Exception("stop")
        
        angle = math.pi * 3
        for neighbor in graph[node]:
            if neighbor[1] == end:
                path.append(neighbor)
                return end
            neighbor = neighbor[1]
            if distance_calc(neighbor, end) < distance_calc(stop, end):
                return neighbor
            test_angle  = calc_angle(node, last, neighbor)
            if test_angle == -0.0:
                test_angle = 0
            if test_angle == 0 or neighbor == last:
                test_angle = math.pi * 2
            if test_angle < 0:
                test_angle = math.pi * 2 + test_angle 

            if test_angle < angle:
                next = neighbor
                angle = test_angle

        num_steps += 1
        add_node(next, graph[node])
        last = node
        node = next
    return node

def left_nonplanar(node, end, graph):
    # left routing, doing planar calcs during forwarding descision
    global num_steps, path
    stop = node
    next = node
    last = end

    while node != end and distance_calc(node, end) >= distance_calc(stop, end): 
        if num_steps > ROUTING_FAILURE:
            print("Left non Planar failure")
            return end

        angle = math.pi * 3
        for neighbor in graph[node]:
            if neighbor[1] == end:
                path.append(neighbor)
                return end
            neighbor = neighbor[1]
            if distance_calc(neighbor, end) < distance_calc(stop, end):
                return neighbor
            test_angle  = calc_angle(node, last, neighbor)
            if test_angle == -0.0:
                test_angle = 0
            if test_angle == 0 or neighbor == last:
                test_angle = math.pi * 2
            if test_angle < 0:
                test_angle = math.pi * 2 + test_angle 

            if test_angle < angle:
                planar = True
                cx = (neighbor[0] - node[0])/2 + node[0]
                cy = (neighbor[1] - node[1])/2 + node[1]
                r = distance_calc(node, (cx, cy))
                for point in graph[node]: # check if any other neighbors of the node are in the collision range, if so remove the edge
                    point = point[1]
                    if neighbor != point and distance_calc(point, (cx,cy)) < r:
                        planar = False
                        break
                if planar:
                    next = neighbor
                    angle = test_angle

        num_steps += 1
        add_node(next, graph[node])
        last = node
        node = next
    return node


# traverse void of non planar graph
def void(node, end, graph):
    global num_steps, path
    stop = node
    last = end
    next = None
    last_intersection = None
    direction = 1
    best_crossing = distance_calc(node, end)

    A = end[1] - node[1]
    B = node[0] - end[0]
    C = node[1]*(end[0] - node[0]) - node[0]*(end[1] - node[1])

    while node != end and distance_calc(node, end) >= distance_calc(stop, end): 
        print(num_steps)
        if num_steps > ROUTING_FAILURE:
            print("void failure")
            return end

 
        for neighbor in graph[node]:
            if neighbor[1] == end:
                path.append(neighbor)
                return end

        if next == None:
            angle = math.pi * 3
            set_next = None
            face_intersection = None
            
            # check if previous edge is intersected by any two hop neighor edge
            # else normal face routing:    
            for neighbor in graph[node]:
                #print(len(graph[node]))
                neighbor = neighbor[1]
                if distance_calc(neighbor, end) < distance_calc(stop, end):
                    #print("here", node)
                    num_steps += 1
                    path.append(neighbor)
                    return neighbor

                for two_hop in graph[neighbor]:
                    #print(len(graph[neighbor]))
                    two_hop = two_hop[1]
                    if two_hop != node  and two_hop != last and neighbor != last and  last != end:
                        crossing = crossing_points(neighbor, two_hop, node, last) 

                        if (crossing > BUMPED_ZERO and crossing < distance_calc(node, last) - BUMPED_ZERO and 
                            (face_intersection == None or (crossing < face_intersection - BUMPED_ZERO) or 
                             (crossing <= face_intersection + BUMPED_ZERO and distance_calc(next, two_hop) < distance_calc(next, set_next)))  and
                            (last_intersection == None or crossing > last_intersection + BUMPED_ZERO )):

                            test_angle = calc_intersect_angle(node, last, neighbor, two_hop) * direction

                            vec1 = (last[0] - node[0], last[1] -node[1] )
                            vec2 = (two_hop[0] - neighbor[0], two_hop[1] - neighbor[1] )

                            cos_angle = (vec1[0] * vec2[0] + vec1[1] * vec2[1]) / math.sqrt((vec1[0]**2 + vec1[1]**2) * (vec2[0]**2 + vec2[1]**2))
                           
                            #if ((test_angle > 0  and test_angle < math.pi) or test_angle < math.pi * -1) and cos_angle < 0.97 and (test_angle > math.pi/4 or cos_angle < .8): 
                            if ((test_angle > 0  and test_angle < math.pi) or test_angle < math.pi * -1) and (test_angle > math.pi/4 or cos_angle < .5): 
                                #print("intersect", neighbor, two_hop, cos_angle, test_angle)

                                #print(test_angle > math.pi/4, cos_angle < .8)
                                if face_intersection and crossing > face_intersection - BUMPED_ZERO: # check that bumped equvilant crossing is going more left than current option
                                    vec3 = (set_next[0] - next[0], set_next[1] - next[1] )
                                    cos_angle2 = (vec1[0] * vec3[0] + vec1[1] * vec3[1]) / math.sqrt((vec1[0]**2 + vec1[1]**2) * (vec3[0]**2 + vec3[1]**2))
                                    if cos_angle2 < cos_angle + .1: 
                                        face_intersection = crossing
                                        next =  neighbor
                                        set_next = two_hop 
                                else:
                                    face_intersection = crossing
                                    next =  neighbor
                                    set_next = two_hop 

                if face_intersection == None:
                    test_angle = calc_angle(node, last, neighbor) * direction
                    if test_angle == -0.0:
                        test_angle = 0 
                    if test_angle == 0 or neighbor == last or test_angle > 0 - BUMPED_ZERO and test_angle < BUMPED_ZERO:
                        test_angle = math.pi * 2
                    elif test_angle < 0 :
                        test_angle = math.pi * 2 + test_angle 

                    if test_angle < angle: 
                        angle = test_angle 
                        next = neighbor
                    if test_angle == angle and distance_calc(node, neighbor) < distance_calc(node, next):
                        next = neighbor

            side = (A * node[0] + B * node[1] + C ) # ensures that the direction is not switched on entry and exit to a point on the line 
            if face_intersection != None:
                crossing = crossing_point(node, set_next, (A,B,C), end)
                
                last_intersection= crossing_points(node, last, set_next, next)
                last = node
                node = next
                next = set_next 
                
            else:
                crossing = crossing_point(node, next, (A,B,C), end)

                last = node
                node = next
                next = None
                last_intersection = None
            
            # check if the next node is on the other side of the line from the current node 
            if crossing == best_crossing and last != end and side != 0 and last != next:
                direction = direction * -1
            elif crossing and crossing < best_crossing and last != end and side != 0:  
                best_crossing = crossing
        else:
            last = node
            node = next
            next = None
        num_steps += 1
        add_node(node, graph[last])

    return node

if __name__ == '__main__': 
    output = input("give test type\n")
    #route("graph.txt", "tests.csv", output)
    #route("planar.txt", "tests.csv", output)
    route("disk.txt", "tests.csv", output)