import math
import networkx as nx

from rtree import index


def process_speed_band(df):
    
    # Define Locations
    def truncate(number, digits) -> float:
        stepper = 10.0 ** digits
        return math.trunc(stepper * number) / stepper
    
    location = df['Location'].values
    i = 0
    
    for loc in location:
        x1, y1, x2, y2 = [float(n) for n in loc.split(' ')]
        
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
    # Define Location    
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
    Location = speed_bands['Location']
    Speed = speed_bands['MaximumSpeed'].astype(float)

    for speed, loc in zip(Speed, Location):
        idx.insert(int(speed), loc)
    
    # Find intersections
    observed_speed = []
    speeds =  edges['maxspeed']

    for loc, speed in zip(Location, speeds):
      max_speeds = list(idx.intersection(loc))

      if len(max_speeds) == 0: # edge didn't intersect with any speed band
          observed_speed.append(speed)
      else:
          observed_speed.append(sum(max_speeds)/len(max_speeds))

    edges['observed_speed'] = observed_speed
    
    
    # Define BPR heuristic for each road link
    bpr = dict()
    n = len(edges['u'])

    for i in range (n):
        u = edges['u'][i]
        v = edges['v'][i]
        key = edges['key'][i]
        time = float(edges['travel_time'][i])
        flow = float(edges['observed_speed'][i])
        capacity = float(edges['maxspeed'][i])

        bpr[(u,v,key)] = time * (1 + 0.15*(flow/capacity)**4)
        
    nx.set_edge_attributes(G, bpr, 'BPR')


def haversine_distance(nodes, source, dest):
	radius = 6371 * 1000 #metres

	if type(source) == tuple:
		lat1 = source[0]
		lon1 = source[1]
	else:
		lat1 = nodes[nodes['osmid']==source]['y'].values[0]
		lon1 = nodes[nodes['osmid']==source]['x'].values[0]

	if type(dest) == tuple:
		lat2 = dest[0]
		lon2 = dest[1]
	else:
		lat2 = nodes[nodes['osmid']==dest]['y'].values[0]
		lon2 = nodes[nodes['osmid']==dest]['x'].values[0]
	    
	# euclidean distance in metres
	dlat = math.radians(lat2-lat1)
	dlon = math.radians(lon2-lon1)
	a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(math.radians(lat1)) \
	  * math.cos(math.radians(lat2)) * math.sin(dlon/2) * math.sin(dlon/2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	ed = radius * c
	return ed


def hybrid_search_length(G, euclidean, real_lengths):
    n = len(euclidean)
    min_pair = euclidean[0]
    source = min_pair[0][0]
    dest = min_pair[0][1]
    min_tt = float('inf')

    for i in range(n):
      travel_time = real_lengths[(source,dest)]

      if travel_time < min_tt:
          min_tt = travel_time
          min_pair = euclidean[i]

          if i < n-1:
              if travel_time < euclidean[i+1][1]:
                  break
              else:
                  source = euclidean[i+1][0][0]
                  dest = euclidean[i+1][0][1]
          else:
              break
      else:
          break
    
    return min_tt
