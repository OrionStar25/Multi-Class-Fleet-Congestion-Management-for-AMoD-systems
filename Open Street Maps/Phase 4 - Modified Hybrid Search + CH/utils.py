#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
import networkx as nx
import osmnx as ox

from rtree import index


def process_speed_band(df):

    # Define Locations

    def truncate(number, digits):
        stepper = 10.0 ** digits
        return math.trunc(stepper * number) / stepper

    location = df['Location'].values
    i = 0

    for loc in location:
        (x1, y1, x2, y2) = [float(n) for n in loc.split(' ')]

        x1 = truncate(x1, 7)
        y1 = truncate(y1, 7)
        x2 = truncate(x2, 7)
        y2 = truncate(y2, 7)

        if y1 < y2:
            bottom = y1
            top = y2
        else:
            bottom = y2
            top = y1

        if x1 < x2:
            left = x1
            right = x2
        else:
            left = x2
            right = x1

        df['Location'].values[i] = (left, bottom, right, top)
        i += 1

    # Process maximum speeds
    i = 0
    for x in df['SpeedBand']:
        if x == 0:
            df['MaximumSpeed'][i] = '80'
        i += 1


def calculate_congestion(G, edges, speed_bands):

    # Define Location for edges

    location = edges['geometry'].values
    i = 0
    Location = []

    for loc in location:
        x1 = loc.xy[1][-1]
        y1 = loc.xy[0][-1]
        x2 = loc.xy[1][0]
        y2 = loc.xy[0][0]

        if y1 < y2:
            bottom = y1
            top = y2
        else:
            bottom = y2
            top = y1

        if x1 < x2:
            left = x1
            right = x2
        else:
            left = x2
            right = x1

        Location.append((left, bottom, right, top))
        i += 1

    # Define observed_speed using speedband dataset
    # key: maximum speed ,value: location

    idx = index.Index()

    for (mx, loc) in zip(speed_bands['MaximumSpeed'], speed_bands['Location']):
        idx.insert(int(mx), loc)

    # Find intersections
    observed_speed = []

    for (loc, speed) in zip(Location, edges['maxspeed']):
        max_speeds = list(idx.intersection(loc))

        if len(max_speeds) == 0:  # edge didn't intersect with any speed band
            observed_speed.append(speed)
        else:
            observed_speed.append(sum(max_speeds) / len(max_speeds))

    edges['observed_speed'] = observed_speed
    

    # Define BPR heuristic for each road link
    bpr = dict()
    n = len(edges['u'])

    for i in range(n):
        u = edges['u'][i]
        v = edges['v'][i]
        key = edges['key'][i]
        time = float(edges['travel_time'][i])
        flow = float(edges['maxspeed'][i])
        capacity = float(edges['observed_speed'][i])

        bpr[(u, v, key)] = time * (1 + 0.15 * (flow / capacity) ** 4)

    nx.set_edge_attributes(G, bpr, 'BPR')


def haversine_distance(nodes, source, dest):
    radius = 6371 * 1000  # metres

    if type(source) == tuple:
        lat1 = source[0]
        lon1 = source[1]
    else:
        lat1 = nodes[nodes['osmid'] == source]['y'].values[0]
        lon1 = nodes[nodes['osmid'] == source]['x'].values[0]

    if type(dest) == tuple:
        lat2 = dest[0]
        lon2 = dest[1]
    else:
        lat2 = nodes[nodes['osmid'] == dest]['y'].values[0]
        lon2 = nodes[nodes['osmid'] == dest]['x'].values[0]

    # euclidean distance in metres

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) * math.sin(dlat / 2) \
        + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) \
        * math.sin(dlon / 2) * math.sin(dlon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    ed = radius * c
    return ed


def get_length_dict(nodes, sources, destinations):
    distances = dict()
  
    for source in sources:        
        for dest in destinations:
            ed = haversine_distance(nodes, source, dest)
            distances[source,dest] = ed/13.8889
       
    # sort the dictionary in ascending order using distances
    x = sorted(distances.items(), key = lambda kv:(kv[1], kv[0]))
    return x  


def find_nearest_nodes(nodes, source, dist, idx):
    # Create a bounding box around source of min distance in all directions
    (north, south, east, west) = ox.bbox_from_point(point=source, distance=dist)
    
    candidate_nodes = []
    initial = list(idx.intersection((west, south, east, north)))   

    for node in initial:
        x = haversine_distance(nodes, node, source)
        if x <= dist:
            candidate_nodes.append(node)

    return candidate_nodes
