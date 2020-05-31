#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
import networkx as nx
import osmnx as ox
import json
import requests

from rtree import index
from random import randint
from collections import defaultdict


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

    for (mx, loc) in zip(speed_bands['MaximumSpeed'],
                         speed_bands['Location']):
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

    # Define capacity for each road link using no. of lanes and speed bands

    capacity = []
    n = len(edges['u'])

    for i in range(n):
        l = float(edges['length'][i])
        o = float(edges['observed_speed'][i])
        m = float(edges['maxspeed'][i])
        k = float(edges['key'][i])

        c = math.ceil(3 * (1 + k) * l / 6 * (o / m))
        capacity.append(c)

    edges['capacity'] = capacity

    # Define BPR heuristic for each road link

    bpr = dict()

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


def get_length_dict(
    nodes,
    sources,
    destinations,
    method='euclidean',
    ):

    distances = dict()

    if method == 'euclidean':
        for source in sources:
            for dest in destinations:
                ed = haversine_distance(nodes, source, dest)
                distances[source, dest] = ed / 13.8889
    else:
        for source in sources:
            for dest in destinations:
                source_x = nodes[nodes['osmid'] == source]['x'
                        ].values[0]
                source_y = nodes[nodes['osmid'] == source]['y'
                        ].values[0]
                dest_x = nodes[nodes['osmid'] == dest]['x'].values[0]
                dest_y = nodes[nodes['osmid'] == dest]['y'].values[0]
                url = \
                    'http://0.0.0.0:5000/route/v1/driving/{},{};{},{}'.format(source_x,
                        source_y, dest_x, dest_y)
                r = requests.get(url)
                json = r.json()
                if json['code'] == 'Ok':
                    d = json['routes'][0]['weight']
                else:
                    d = float('inf')
                distances[(source, dest)] = d

    # sort the dictionary in ascending order using distances

    x = sorted(distances.items(), key=lambda kv: (kv[1], kv[0]))
    return x


def hybrid_search_length(euclidean, real_lengths):
    n = len(euclidean)
    min_pair = euclidean[0]
    source = min_pair[0][0]
    dest = min_pair[0][1]
    min_tt = float('inf')

    for i in range(n):
        travel_time = real_lengths[(source, dest)]

        if travel_time < min_tt:
            min_tt = travel_time
            min_pair = euclidean[i]

            if i < n - 1:
                if travel_time <= euclidean[i + 1][1]:
                    break
                else:
                    source = euclidean[i + 1][0][0]
                    dest = euclidean[i + 1][0][1]
            else:
                break
        else:
            if i < n - 1:
                if min_tt <= euclidean[i + 1][1]:
                    break
                else:
                    source = euclidean[i + 1][0][0]
                    dest = euclidean[i + 1][0][1]
            else:
                break

    return (min_tt, i + 1)


def modified_hybrid_search(nodes, x):
    n = len(x)
    cutoff = math.floor(n / math.exp(1))
    min_pair = x[0]
    source = min_pair[0][0]
    dest = min_pair[0][1]
    min_tt = float('inf')
    mhq = 0
    ht = 0
    hq = 0
    mht = 0
    flag = 0

    for i in range(n):
        source_x = nodes[nodes['osmid'] == source]['x'].values[0]
        source_y = nodes[nodes['osmid'] == source]['y'].values[0]
        dest_x = nodes[nodes['osmid'] == dest]['x'].values[0]
        dest_y = nodes[nodes['osmid'] == dest]['y'].values[0]

        url = \
            'http://0.0.0.0:5000/route/v1/driving/{},{};{},{}'.format(source_x,
                source_y, dest_x, dest_y)
        r = requests.get(url)
        json = r.json()

        if json['code'] == 'Ok':
            travel_time = json['routes'][0]['weight']
        else:
            continue

        # have reached cutoff and didnt find optimal yet

        if i > cutoff and flag == 0:
            if travel_time < min_tt:
                mht = travel_time
                mhq = i + 1
                i -= 1
                flag = 1  # Don't break, instead skip this block forever
            else:
                mht = min_tt
                mhq = i + 1
                i -= 1
                flag = 1  # Don't break, instead skip this block forever
        else:

              # Pure Hybrid

            if travel_time < min_tt:
                min_tt = travel_time

                if i < n - 1:
                    if travel_time < x[i + 1][1]:
                        ht = min_tt
                        hq = i + 1
                        break
                    else:
                        source = x[i + 1][0][0]
                        dest = x[i + 1][0][1]
                else:
                    ht = min_tt
                    hq = i + 1
                    break
            else:
                if i < n - 1:
                    if min_tt < x[i + 1][1]:
                        ht = min_tt
                        hq = i + 1
                        break
                    else:
                        source = x[i + 1][0][0]
                        dest = x[i + 1][0][1]
                else:
                    ht = min_tt
                    hq = i + 1
                    break

    if i <= cutoff:  # Hybrid stopped before cutoff
        mht = ht
        mhq = hq

    return (ht, hq, mht, mhq)


def find_nearest_nodes(
    nodes,
    source,
    dist,
    idx,
    ):

    # Create a bounding box around source of min distance in all directions

    (north, south, east, west) = ox.bbox_from_point(point=source,
            distance=dist)

    candidate_nodes = []
    initial = list(idx.intersection((west, south, east, north)))

    for node in initial:
        x = haversine_distance(nodes, node, source)
        if x <= dist:
            candidate_nodes.append(node)

    return candidate_nodes


def make_nodes_tntp(nodes):
    f = open('data/Singapore_node.tntp', 'w')
    f.write('Node\tX\tY\t;\n')

    for n in nodes.values:
        f.write('{}\t{}\t{}\t;\n'.format(n[2], n[1], n[0]))
    f.close()


def make_net_tntp(edges):
    f = open('data/Singapore_net.tntp', 'w')
    f.write('''<NUMBER OF ZONES> 23219 
<NUMBER OF NODES> 23219 
<FIRST THRU NODE> 1 
<NUMBER OF LINKS> 44612 
<END OF METADATA> 

''')
    f.write('~\tInit node\tTerm node\tCapacity\tLength\tFree Flow Time\tB\tPower\tSpeed limit\tToll\tType\t;\n'
            )

    for e in edges.values:
        f.write('\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t;\n'.format(
            e[0],
            e[1],
            e[13],
            e[8],
            e[9],
            0.15,
            4,
            e[7],
            0,
            1,
            ))

    f.close()


def make_trips_tntp(demands):
    trips = defaultdict(lambda : list())
    for d in demands:
        trips[d[0]].append(d[1])

    f = open('data/Singapore_trips.tntp', 'w')
    f.write('''<NUMBER OF ZONES> 23219 
<TOTAL OD FLOW> 
<END OF METADATA> 

''')
    demand = 0

    for (u, l) in trips.items():
        f.write('Origin\t{}\t\n'.format(u))

        count = 0
        for v in l:
            d = randint(1, 10)
            f.write('\t\t{} : {};'.format(v, d))
            demand += d

        count += 1
        if count % 5 == 0:
            f.write('  \n')
        f.write('''

''')

    f.close()
    print ('No. of requests: ', demand)


# use pedestrain network only

def print_route(G, route, length):
    i = 0
    n = len(route)

    for i in range(n):
        for (nei, w) in G[route[i]].items():
            if i + 1 != n:
                if nei == route[i + 1]:
                    print (w[0].get('highway'), w[0].get('length'))

    print ('Distance from X/B to A/Y: ', length)  # in metres
