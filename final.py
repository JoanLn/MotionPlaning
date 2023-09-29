import numpy as np
from scipy.spatial import Voronoi, KDTree
from graph import RoadmapVertex, RoadmapEdge, Roadmap
import math
from utils import *

import tkinter as tk

random_agent = True
num_agent = 50

def build_voronoi(points):
	
	# Compute the Voronoi diagram
    vor = Voronoi(points)
	
	# Extract the Voronoi vertices and edges
    vertices = vor.vertices
    edges = vor.ridge_vertices
	
    return vor
    
def assign_goal(vor, points, agent):
   
    region_indices = [] # region index of each points
    point_in_regions = [[-1, -1] for num in range(len(points)) ] # point index in each region 
    for i, point in enumerate(points):
        region_index = vor.point_region[i]
        region_indices.append(region_index)
        point_in_regions[region_index-1] = i
    
    agents_region_indices = []
    # Find the index of the closest point in the Voronoi diagram
    tree = KDTree(points)

    x, y = agent.pos
    distance, index = tree.query(agent.pos)
    # Find the index of the Voronoi region containing the closest point
    region_index = vor.point_region[index]
    agents_region_index = region_index
    
    agents_goals = []
    #for i in range(len(agents)):
    agent_region = agents_region_index
    point_index = point_in_regions[agent_region-1]
    #print(f'agent_region: {agent_region}, points index of that region: {point_index}')
    agents_goal = points[point_index]
    #agents_goals.append(agents_goal)
    #print(f'line: {agents_goal}, {agents[i]}')
	
    return agents_goal   

def build_roadmap(obstacle_points, obstacles):
	
    graph = Roadmap()
    
    for obp in obstacle_points:
        point = graph.addVertex((obp[0], obp[1]))
    count = 0
    countF = 0
    #print(graph.vertices)    
    # Edges
    for vertice in graph.vertices:
        for vertice_n in graph.vertices:
            count = count + 1
            if vertice == vertice_n:
                continue
            else:                
                path = interpolate(vertice.getConfiguration(), vertice_n.getConfiguration(), 1, 700)
                #print(path)
                in_obj = False       
                for p in path:					
                    for obs in obstacles:
                        #print(f'obs.x_min: {obs.x_min}, p: {p}')
                        if (obs.x_min < p[0] < obs.x_max) and (obs.y_min < p[1] < obs.y_max):
                            in_obj = True
                            countF = countF + 1
                            break
                    if in_obj == True:
                        break
                if in_obj == False:
                    dist = distance(vertice, vertice_n)			
                    vertice.addEdge(vertice_n.id, dist, path)

    print(f'count: {count}, countF:{countF}')
    return graph

def find_path(goal, agent, graph, obstacles):
    path  = [] 
    #path.append(agent)
    
    #print(f'agent: {agent}, goal: {goal}')		    
    line = interpolate(agent, goal, 1, 700)
    in_obj = False
    for p in line:					
        for obs in obstacles:
            if (obs.x_min < p[0] < obs.x_max) and (obs.y_min < p[1] < obs.y_max):
                in_obj = True
                break
        if in_obj:
            break
    if in_obj==False:
        path.append(agent)
        path.append(goal)
        print("Goal!")
        return path


    start = graph.addVertex(agent)
    goal = graph.addVertex(goal)
        
    vertices = graph.getVertices()

    parent = [' ' for row in range(len(vertices))]
        
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use the PriorityQueue for the open list
    open_set = PriorityQueueValue(order=min, f=lambda v: v.f)
 
    g = 0
    h = distance(start, goal)
    f = g + h  
    open_set.put(start, Value(f=f,g=g))
    
    # Add edges for start and goal   
    min_s = 10000
    min_g = 10000      

    for v in vertices:
        if v.id != start.id:
            line = interpolate(start.getConfiguration(), v.getConfiguration(), 1, 700)
            #print(path)
            in_obj = False       
            for p in line:					
                for obs in obstacles:
                    #print(f'obs.x_min: {obs.x_min}, p: {p}')
                    if (obs.x_min < p[0] < obs.x_max) and (obs.y_min < p[1] < obs.y_max):
                        #print(f'vertice: {vertice.getConfiguration()}, vertice_n: {vertice_n.getConfiguration()}')
                        in_obj = True
                        break
                if in_obj == True:
                    break
            if in_obj == False:
                dist = distance(start, v)			
                start.addEdge(v.id, dist, path)
			
        if v.id != goal.id:
            line = interpolate(goal.getConfiguration(), v.getConfiguration(), 1, 700)
            #print(path)
            in_obj = False       
            for p in line:					
                for obs in obstacles:
                    #print(f'obs.x_min: {obs.x_min}, p: {p}')
                    if (obs.x_min < p[0] < obs.x_max) and (obs.y_min < p[1] < obs.y_max):
                        #print(f'vertice: {vertice.getConfiguration()}, vertice_n: {vertice_n.getConfiguration()}')
                        in_obj = True
                        break
                if in_obj == True:
                    break
            if in_obj == False:
                dist = distance(goal, v)			
                v.addEdge(goal.id, dist, path)
    
    #print(f'agent: {start.getConfiguration()}, n_s: {n_s.getConfiguration()}, goal: {goal.getConfiguration()}, n_g: {n_g.getConfiguration()}')
    #graph.addEdge(start, n_s, min_s)
    #graph.addEdge(goal, n_g, min_g)
    
    # A*
    while(len(open_set) != 0):       
        
        cur_ver, cur_val = open_set.pop()
        closed_set.add(cur_ver)
        
        #print(f'cur_ver: {cur_ver.getConfiguration()}')
        
        if cur_ver == goal:            
            end = cur_ver
            for i in range(len(closed_set)):
                if end != start:
                    parent_n = parent[end.id]
                    #print(f'add : {parent_n.getConfiguration()}')
                    path.append(parent_n.getConfiguration())
                    end = parent_n        
            path.reverse()
            path.append(goal.getConfiguration())
            print("Goal!")
            #print(path)
            return path
        
        edges = cur_ver.getEdges()
        
        for edge in edges:
            child_ver = vertices[edge.id]
            
            if child_ver not in closed_set:
     
                child_g = distance(cur_ver, child_ver) + cur_val.g
                child_h = distance(goal, child_ver)		
                child_f = child_g + 1.5*child_h
                #print(f'child: {child_ver.getConfiguration()}, g: {child_g}, h: {child_h}, f: {child_f}')
                if child_ver not in closed_set:# or child_ver not in open_set:
                    if child_ver not in open_set:
                        open_set.put(child_ver, Value(child_f, child_g))
                        parent[edge.id] = cur_ver
                    elif open_set.get(child_ver).f > child_f:
                        open_set.remove(child_ver)
                        open_set.put(child_ver, Value(child_f, child_g))
                        parent[edge.id] = cur_ver#.getConfiguration()               
    
    return path

def interpolate (q1, q2, stepsize, window):
    """
        Returns an interpolated local path between two given configurations. 
        It can be used to determine whether an edge between vertices is collision-free. 
    """
    dist = math.sqrt( (q1[0]-q2[0])**2 + (q1[1]-q2[1])**2)
    step = int(dist*window/stepsize)
    
    x_space = (q2[0]-q1[0]) / (step+1)
    y_space = (q2[1]-q1[1]) / (step+1)
    
    path = []
    
    for i in range(step):
        x = q1[0] + i*x_space
        y = q1[1] + i*y_space
        path.append((x,y))
  
    return path 

def distance (q1, q2): 
    """
        Returns the distance between two configurations. 
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration  
    """

    x1 = q1.q[0];
    y1 = q1.q[1]
    x2 = q2.q[0];
    y2 = q2.q[1]
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    return dist

 
if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk
    
    dt = 0.05
    framedelay = 30

    win = tk.Tk()
    scene = Scene('sceneInfo.csv',(build_voronoi, assign_goal, build_roadmap, find_path), num_agent, random_agent, win)
    win.after(framedelay,lambda: scene.drawFrame(dt, win, framedelay))
    win.mainloop()
