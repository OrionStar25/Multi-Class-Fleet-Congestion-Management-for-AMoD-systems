import networkx as nx
import scipy.integrate as integrate 
from scipy.optimize import minimize_scalar
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math
import time

import TransportationNetworks as tn
from networkx.algorithms.flow import edmonds_karp
from heapq import heappush, heappop
from itertools import count

class Run:
    """
    Class of implementing Frank-Wolfe algorithm for networks privided from 
    Transportation Networks for Research Core Team (https://github.com/bstabler/TransportationNetworks)
    
    Parameters
    ----------
    link_file :     string
                    file path of network file, which containing various link information
                    
    trip_file :     string
                    file path of trip table. An Origin label and then Origin node number, followed by Destination node numders and OD flow
                    
    node_file :     string
                    file path of node file, which containing coordinates information of nodes
                    
    SO:             boolean
                    True if objective is to find system optimal solution,
                    False if objective is to find user equilibrium
                    
    Attributes
    ----------
    graph:          networkx DiGraph
                    graph of links when completing the algorithm
                    
    network:        nested dictionary
                    dictionary of links information and history of Frank-Wolfe algorithm implementation by iteration
    
    fwResult:       dictionary
                    dictionary of theta (optimal move size) and objective function value over iterations
                    
    Example
    -------
    A Quick example
    
    #Set the paths of Transportation Networks file
    
    >>> directory = ".\\Data\\TransportationNetworks\\SiouxFalls\\" 
    >>> link_file = '{}SiouxFalls_net.tntp'.format(directory) 
    >>> trip_file = '{}SiouxFalls_trips.tntp'.format(directory) 
    >>> node_file = '{}SiouxFalls_node.tntp'.format(directory)
    >>> SO = False \n
    
    #Implement Frank-Wolfe algorithm
    
    >>> fw = Run(link_file, trip_file, node_file, SO)
    >>> fw.showODFlow()
    >>> fw.showODFlowMap()
    """
    def __init__(self, link_file, trip_file, node_file, SO=True):

        """

        :param link_file: Link file - Bargera Network type
        :param trip_file: OD file - Bargera Network type
        :param node_file: Node file - Bargera Network type
        :param SO: Whether the objective function is of System Optimum or not (User Equilibrium)
        """
        self.SO = SO
        
        nw = tn.Network(link_file, trip_file, node_file, self.SO)
        self.od_vols = nw.od_vols

        ## initialization
        self.network = {(u,v): {'t0':d['object'].t0, 'alpha':d['object'].alpha, \
                   'beta':d['object'].beta, 'capa':d['object'].capacity, 'flow':[], \
                   'auxiliary':[], 'cost':[]} for (u, v, d) in nw.graph.edges(data=True)}
        
        self.fwResult = {'theta':[], 'z':[]}
        
        nw.all_or_nothing_assignment()
        nw.update_linkcost()
        
        for linkKey, linkVal in self.network.items():
            linkVal['cost'].append(nw.graph[linkKey[0]][linkKey[1]]['weight'])
            linkVal['auxiliary'].append(nw.graph[linkKey[0]][linkKey[1]]['object'].vol)
            linkVal['flow'].append(nw.graph[linkKey[0]][linkKey[1]]['object'].vol)
            
        ## iterations
        iterNum=0
        iteration = True
        start = time.time()
        PERIOD_OF_TIME = 300 # 5min

        while iteration:
            iterNum += 1
            # print(iterNum)
            nw.all_or_nothing_assignment()
            nw.update_linkcost()
            
            for linkKey, linkVal in self.network.items():
                linkVal['auxiliary'].append(nw.graph[linkKey[0]][linkKey[1]]['object'].vol)
                
            theta = self.lineSearch()
            self.fwResult['theta'].append(theta)
            
            for linkKey, linkVal in self.network.items():
                aux = linkVal['auxiliary'][-1]
                flow = linkVal['flow'][-1]
                linkVal['flow'].append(flow + theta*(aux-flow))
                
                nw.graph[linkKey[0]][linkKey[1]]['object'].vol =  flow + theta * (aux - flow)
                nw.graph[linkKey[0]][linkKey[1]]['object'].flow = flow + theta * (aux - flow)
                
            
            nw.update_linkcost()
            
            z=0
            for linkKey, linkVal in self.network.items():
                linkVal['cost'].append(nw.graph[linkKey[0]][linkKey[1]]['weight'])
                totalcost = nw.graph[linkKey[0]][linkKey[1]]['object'].get_objective_function()
                z+=totalcost
                
            self.fwResult['z'].append(z)        
            
            if iterNum == 1:
                iteration = True
            else:
                print(iterNum, abs(self.fwResult['z'][-2] - self.fwResult['z'][-1]))
                if abs(self.fwResult['z'][-2] - self.fwResult['z'][-1]) <= 0.002 or \
                   iterNum==1000 or \
                   time.time() > start + PERIOD_OF_TIME:
                   iteration = False
            
        self.graph = nw.graph
                    
    def BPR(self, t0, xa, ca, alpha, beta):
        """
        Method for calculating link travel time based on BPR function
        
        Parameters
        ----------
        t0:     float
                link travel time under free flow speed
                
        xa:     float
                traffic link flow
        
        ca:     float
                capacity of link
                
        alpha:  float
                first BPR function parameter, usually 0.15
                        
        beta:   float
                second BPR function parameter, usually 4.0
                
        Return
        ------
        ta:     float
                link travel time under the current traffic flow
        """
        ta = t0 * (1 + alpha * pow((xa/ca), beta))
        return ta
    
    def calculateZ(self, theta):
        """
        Method for calculating objective function value
        
        Parameters
        ----------
        theta:      float
                    optimal move size
                    
        Return
        ------
        float
            objective function value
                    
        """
        z = 0
        for linkKey, linkVal in self.network.items():
            t0 = linkVal['t0']
            ca = linkVal['capa']
            beta = linkVal['beta']
            alpha = linkVal['alpha']
            aux = linkVal['auxiliary'][-1]
            flow = linkVal['flow'][-1]
            
            if SO == False:
                z += integrate.quad(lambda x: self.BPR(t0, x, ca, alpha, beta), 0, flow+theta*(aux-flow))[0]
            elif SO == True:
                z += list(map(lambda x : x * self.BPR(t0, x, ca, alpha, beta), [flow+theta*(aux-flow)]))[0]
        return z
    
    def lineSearch(self):
        """
        Method for estimating theta (optimal move size)
        
        Return
        ------
        float
            optimal move size (rate) between 0 and 1
        """
        theta = minimize_scalar(lambda x: self.calculateZ(x), bounds = (0,1), method = 'Bounded')
        return theta.x


    def shortest_successive_path(self, source, target):
        if target == source:
            return [target]

        paths = {source: [source], target: []} # dictionary of paths
        G_succ = self.graph._succ
        push = heappush
        pop = heappop        
        dist = {}  # dictionary of final width
        c = count() # use the count c to avoid comparing nodes
        fringe = [] # fringe is heapq with 3-tuples (distance,c,node)

        for n in self.graph.nodes:
            dist[n] = float('inf')
        dist[source] = 0
        
        push(fringe, (dist[source], next(c), source))
        while fringe:
            (w, _, v) = pop(fringe)
            if v == target:
                break

            for u, e in G_succ[v].items():
                # Check for only those edges who have enough capacity left
                if e['capacity'] > 0:
                    dist_vu = e['weight']
                    alt = dist[v] + dist_vu

                    if alt < dist[u]:
                        dist[u] = alt
                        push(fringe, (dist[u], next(c), u))
                        paths[u] = paths[v] + [u]

        return paths[target]

    
    def showODPath(self, SO):
        """
        Method for presenting table of the optimal traffic assignment of the Frank-Wolfe algorithm procedure
        """
        capacity = dict()
        for (u, v, d) in self.graph.edges(data=True):
            capacity[(u,v)] = math.ceil(d['object'].vol)

        nx.set_edge_attributes(self.graph, capacity, name='capacity')

        # Decomposing flow into a path for every request
        infeasible = dict()
        cost = 0

        for (origin, dest), demand in self.od_vols.items():
            while demand > 0:
                path = self.shortest_successive_path(origin, dest)

                if path == []: # Add to waiting queue
                    infeasible[(origin, dest)] = demand
                    break
                else:
                    # Decrement capacity of chosen path by 1
                    for i in range(len(path)-1):
                        u = path[i]
                        v = path[i+1]
                        self.graph[u][v]['capacity'] -= 1
                        cost += self.graph[u][v]['weight']
                    demand = demand - 1                   

        # Route infeasible paths in a user equilibrium way
        if len(infeasible) > 0:
            new_capacity = dict()

            for (u, v, d) in self.graph.edges(data=True):
                new_capacity[(u,v)] = d['object'].capacity -\
                                      math.ceil(d['object'].vol) +\
                                      d['capacity']

            nx.set_edge_attributes(self.graph, new_capacity, name='capacity')
            
            count = 0
            for (origin, dest), demand in infeasible.items():
                while demand > 0:
                    path = self.shortest_successive_path(origin, dest)

                    if path == []: # Add to waiting queue
                        count += demand
                        break
                    else:
                        # Decrement capacity of chosen path by 1
                        for i in range(len(path)-1):
                            u = path[i]
                            v = path[i+1]
                            self.graph[u][v]['capacity'] -= 1
                            cost += self.graph[u][v]['weight']
                        demand = demand - 1 

            print("Number of infeasible trips:", count)
            print("Cost of system:", cost)                  

        print("DONE!")

            

if __name__ == "__main__":
    directory = "../Data/Singapore/"
    link_file = '{}Singapore_net_off.tntp'.format(directory)
    trip_file = '{}Singapore_trips_1000.tntp'.format(directory)
    node_file = '{}Singapore_node.tntp'.format(directory)
    SO = True
    
    fw = Run(link_file, trip_file, node_file, SO)
    fw.showODPath()
