# scene.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Authors: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import time, random
import sys
import tkinter as tk
import tkinter.messagebox
import numpy as np 
from obstacles import BoxObstacle
from utils import *
from graph import *
import copy
from agent import TTCAgent
import random

class Scene(tk.Frame):
    random.seed('cpsc8810')
   
    def __init__(self, filename, build_fn, num_agent, random_agent,  master=None, resolution = 700):
        super().__init__(master)
        #self.master = tk.Tk()
        self.build_fn = build_fn
        self.filename = filename
        self.resolution = resolution
        self.num_agent = num_agent
        self.random_agent = random_agent

        # parameters related to the problem
        self.scene_width, self.scene_height = None, None
        self.scene_xmin, self.scene_xmax, self.scene_ymin, self.scene_ymax = None, None, None, None
        self.default_start, self.default_goal = None, None
        self.start, self.goal = None, None
        self.obstacles = None
        self.robot_width, self.robot_height = None, None
        self.roadmap = None
        self.lines = None
        self.vor = None
        self.goals = None
        self.paused = True
        #self.random_agent = True
        if not self.loadProblem(): sys.exit("Failed to load problem file")

        # setup the GUI
        self.master.title("Simulater")        
        self.canvas = tk.Canvas(self.master, width=resolution, height=resolution, bg="white")
        #self.canvas.pack()
        self.bt_new = tk.Button(self.master, text="Generate Roapmap", command=self.generate_roadmap, state=tk.DISABLED)
        self.bt_default = tk.Button(self.master, text="Get Path", command=self.search, state=tk.DISABLED)
        self.bt_findgoal = tk.Button(self.master, text="Get Goal", command=self.assign_goal, state=tk.DISABLED)
        self.bt_voronoi = tk.Button(self.master, text="Generate Voromoi", command=self.generateVoronoi)
        self.bt_start = tk.Button(self.master, text="Start", command=self.startMoving, state=tk.DISABLED)
        self.master.resizable(False, False)

        self.canvas.grid(row=0, columnspan=4,
            sticky=tk.W+tk.E+tk.N+tk.S, padx=10, pady=28
        )
        self.bt_new.grid(row=1, column=2,
            sticky=tk.W, padx=10, pady=(0, 10)
        )
        self.bt_default.grid(row=1, column=3,
            sticky=tk.W, padx=5, pady=(0, 10)
        )
        self.bt_findgoal.grid(row=1, column=1,
            sticky=tk.W, padx=10, pady=(0, 10)
        )
        self.bt_voronoi.grid(row=1, column=0,
            sticky=tk.W, padx=10, pady=(0, 10)
        )
        self.bt_start.grid(row=1, column=4,
            sticky=tk.E, padx=10, pady=(0, 10)
        )
        
        self.master.columnconfigure(0, weight=0)
        self.master.columnconfigure(1, weight=0)
        self.master.columnconfigure(2, weight=0)
        self.master.columnconfigure(3, weight=1)
        self.master.rowconfigure(0, weight=1)
        self.master.rowconfigure(1, weight=0)
       
        self.draw_scene()
        self.draw_agents()
        
        global PAUSED
        #path = self.interpolate(self.default_start,self.default_goal,2)
     
        #self.canvas.bind("<Configure>", self.default_query)

    def loadProblem(self):
        """
            Read a scenario from a file
        """
        try:
            fp = open(self.filename, 'r')
            lines = fp.readlines()
            fp.close()

            # first line reads the dimension 
            # second line reads the dimension of the robot 
            scene_parameters = lines[0].split(',')
            shelter_parameters = lines[1].split(',')
            agent_parameters = lines[2].split(',')
            obstacle_parameters = lines[3].split(',')
            
            #self.scene_xmin, self.scene_xmax, self.scene_ymin, self.scene_ymax = int(scene_parameters[0]), int(scene_parameters[1]), int(scene_parameters[2]), int(scene_parameters[3])
            self.scene_width = self.resolution#int(scene_parameters[0])
            self.scene_height = self.resolution#int(scene_parameters[1])
            
            self.shelter_num = int(shelter_parameters[0])
            self.shelter_radius = float(shelter_parameters[1]) 
            self.shelters = []

            self.agent_num = int(agent_parameters[0])
            self.agent_radius = float(agent_parameters[1]) 
            self.agents = [] 
            
            self.obstacle_num = int(obstacle_parameters[0])
    
            for line in lines[4:4+self.shelter_num]:
                parameters = line.split(',')
                self.shelters.append((float(parameters[0]), float(parameters[1])))
           # print(self.shelters)
            
            self.obstacles = []  
            self.obstacle_points = []              
            for line in lines[4+self.agent_num+self.shelter_num:]:
                parameters = line.split(',')
                vertices = []
                vertices.append((float(parameters[0]), float(parameters[1]))) #y-axis is flipped
                vertices.append((float(parameters[2]), float(parameters[3])))
                vertices.append((float(parameters[4]), float(parameters[5])))
                vertices.append((float(parameters[6]), float(parameters[7])))
                self.obstacles.append(BoxObstacle(vertices))
                for p in vertices:
                    self.obstacle_points.append(p)
                    
            if self.random_agent:
                i = 0
                while len(self.agents) < self.num_agent:
                    posx = random.random()	
                    posy = random.random()	
                    in_obj = False
                    for obs in self.obstacles:
                        if (obs.x_min-self.agent_radius/self.resolution <= posx <= obs.x_max+self.agent_radius/self.resolution) and (obs.y_min-self.agent_radius/self.resolution <= posy <= obs.y_max+self.agent_radius/self.resolution):
                            in_obj = True
                            break
                    if posx<self.agent_radius/self.resolution or posy<self.agent_radius/self.resolution or posx > (1-self.agent_radius/self.resolution) or posy > (1-self.agent_radius/self.resolution):
                        in_obj = True
                    for a in self.agents:
                        if math.sqrt( (posx-a.pos[0])*(posx-a.pos[0]) + (posy-a.pos[1])*(posy-a.pos[1]) ) < (self.agent_radius+5)/self.resolution:
                            in_obj = True
                            break							
                    if in_obj == False:
                        self.agents.append(TTCAgent((i, i, posx, posy, -1, -1, 30/self.resolution, 40/self.resolution, self.agent_radius/self.resolution), 10/self.resolution, 15/self.resolution, 0.5, 5, 0, 0.3)) 
                        i = i+1
            else:
                i = 0
                for line in lines[4+self.shelter_num:4+self.shelter_num+self.agent_num]:
                    parameters = line.split(',')
                    #self.agents.append((float(parameters[0]), float(parameters[1])))
                    #TTCAgent((id, gid, posx, posy, goalx, goaly, prefspeed, maxspeed, radius), goalRadius, dhor, ksi, timehor, epsilon, maxF)) 
                    self.agents.append(TTCAgent((i, i, float(parameters[0]), float(parameters[1]), -1, -1, 30/self.resolution, 40/self.resolution, self.agent_radius/self.resolution), 15/self.resolution, 20/self.resolution, 0.5, 5, 0, 0.3)) 
                    i = i+1
                
            #print(self.agents)
            
        except:
            return False
        
        return True


    def getObstacles(self):
        return self.obstacles
    
    def getRobot(self):
        return self.robot_width, self.robot_height            
    
    # def default_query(self, event=None):
        # self.bt_default.config(state=tk.DISABLED)
        # self.bt_new.config(state=tk.DISABLED)
        # self.bt_voronoi.config(state=tk.DISABLED)
        # self.bt_findgoal.config(state=tk.DISABLED)

 
        # self.canvas.delete("start")
        # self.canvas.delete("goal")
        # self.canvas.delete("path")
        # self.start = self.default_start
        # self.goal = self.default_goal
        # self.draw_config(self.start,"green","start")
        # self.draw_config(self.goal,"blue","goal")
      
        # self.bt_default.config(state=tk.NORMAL)
        # self.bt_new.config(state=tk.NORMAL)
        # self.bt_voronoi.config(state=tk.NORMAL)
        # self.bt_findgoal.config(state=tk.NORMAL)


    # def random_query(self, event=None):
        # self.bt_default.config(state=tk.DISABLED)
        # self.bt_new.config(state=tk.DISABLED)
        # self.bt_voronoi.config(state=tk.DISABLED)
        # self.bt_findgoal.config(state=tk.DISABLED)
 
        # self.canvas.delete("start")
        # self.canvas.delete("goal")
        # self.canvas.delete("path")
        # self.start = (random.uniform(self.scene_xmin, self.scene_xmax), random.uniform(self.scene_ymin, self.scene_ymax), random.uniform(0, np.pi))
        # self.goal = (random.uniform(self.scene_xmin, self.scene_xmax), random.uniform(self.scene_ymin, self.scene_ymax), random.uniform(0, np.pi))
        # #self.draw_config(self.start,"#43a2ca","start")
        # #self.draw_config(self.goal,"#e0f3db","goal")
        # self.draw_config(self.start,"green","start")
        # self.draw_config(self.goal,"blue","goal")
      
        # self.bt_default.config(state=tk.NORMAL)
        # self.bt_new.config(state=tk.NORMAL)
        # self.bt_voronoi.config(state=tk.NORMAL)
        # self.bt_findgoal.config(state=tk.NORMAL)
        
    def search(self):
        self.bt_findgoal.config(state=tk.DISABLED)
        self.bt_new.config(state=tk.DISABLED)
        self.bt_default.config(state=tk.DISABLED)
        self.bt_voronoi.config(state=tk.DISABLED)
        
        if  self.roadmap == None:
            print("Generate roadmap first")
        
        for index in range(len(self.agents)):
            roadmap = copy.deepcopy(self.roadmap)
            p = self.build_fn[3](self.goals[index], self.agents[index].pos, roadmap, self.obstacles)
            if p is None or len(p) == 0:
                tk.messagebox.showinfo("", "Failed to find any solution path.")
            else:
                self.draw_path(p)
                self.agents[index].path = p
                self.agents[index].goal = p[0]
            
        self.bt_findgoal.config(state=tk.NORMAL)
        self.bt_new.config(state=tk.NORMAL)
        self.bt_default.config(state=tk.NORMAL)
        self.bt_voronoi.config(state=tk.NORMAL)
        self.bt_start.config(state=tk.NORMAL)
    
    # def generate(self):
        # self.clear_canvas()
        # self.bt_voronoi.config(state=tk.DISABLED)
        # self.bt_findgoal.config(state=tk.DISABLED)
        # self.bt_new.config(state=tk.DISABLED)
        # self.bt_default.config(state=tk.DISABLED)
     
        # # should return a graph
        # #self.roadmap = self.build_fn[0]([(self.scene_xmin, self.scene_xmax),
        # #(self.scene_ymin, self.scene_ymax), (0,2*np.pi)], (self.robot_width, self.robot_height), self.obstacles)
        
        # self.lines = self.build_fn[0](self.shelters, (self.scene_xmin, self.scene_xmax, self.scene_ymin, self.scene_ymax))

        # # should draw the graph 
        # if self.lines is None :#or len(self.lines.getVertices()) == 0:
            # tk.messagebox.showinfo("", "Failed to construct a roadmap.")
        # else:
            # #self.draw_roadmap(self.roadmap, 0.5)
            # self.drawLinesOnCanvas(self.lines)
       
        # self.bt_findgoal.config(state=tk.NORMAL)
        # self.bt_new.config(state=tk.NORMAL)
        # self.bt_default.config(state=tk.NORMAL)
        # self.bt_voronoi.config(state=tk.NORMAL)

    def generateVoronoi(self):
        self.clear_canvas()
        self.bt_voronoi.config(state=tk.DISABLED)
        self.bt_findgoal.config(state=tk.DISABLED)
        self.bt_new.config(state=tk.DISABLED)
        self.bt_default.config(state=tk.DISABLED)
     
        # should return a graph
        #self.roadmap = self.build_fn[0]([(self.scene_xmin, self.scene_xmax),
        #(self.scene_ymin, self.scene_ymax), (0,2*np.pi)], (self.robot_width, self.robot_height), self.obstacles)
        
        self.vor = self.build_fn[0](self.shelters)
        vertices = self.vor.vertices
        edges = self.vor.ridge_vertices
        world_scale = self.resolution
        # Plot the Voronoi diagram
        for i, edge in enumerate(edges):
            if -1 not in edge:
                x1, y1 = vertices[edge[0]]
                x2, y2 = vertices[edge[1]]
                self.canvas.create_line(x1*world_scale, y1*world_scale, x2*world_scale, y2*world_scale)
       
        self.bt_findgoal.config(state=tk.NORMAL)
        self.bt_new.config(state=tk.NORMAL)
        #self.bt_default.config(state=tk.NORMAL)
        #self.bt_voronoi.config(state=tk.NORMAL)

    def assign_goal(self):
        if self.vor == None:
            print("Please generate Voronoi diagram first")
            return None
        #print(f'agents: {self.agents}')
        world_scale = self.resolution    
        self.goals = [ (-1, -1) for num in range(len(self.agents)) ]
         
        for i in range(len(self.agents)):
            agent = self.agents[i]
            agents_goal = self.build_fn[1](self.vor, self.shelters, agent)
            self.goals[i] = agents_goal
            self.agents[i].goal = (agents_goal[0], agents_goal[1])
            self.canvas.create_line(agents_goal[0]*world_scale, agents_goal[1]*world_scale, agent.pos[0]*world_scale, agent.pos[1]*world_scale, fill = 'black', arrow=tk.FIRST, tag = "goal")
    
    def generate_roadmap(self):
    
        self.roadmap = self.build_fn[2](self.obstacle_points, self.obstacles)
        
        # should draw the graph 
        if self.roadmap is None :#or len(self.lines.getVertices()) == 0:
            tk.messagebox.showinfo("", "Failed to construct a roadmap.")
        else:
            self.draw_roadmap(self.roadmap, 0.5)
        
        self.bt_default.config(state=tk.NORMAL)
    
    def startMoving(self):
			
        if self.paused== True:
            print(f'Start')
            self.bt_default.config(state=tk.NORMAL)
            self.bt_new.config(state=tk.NORMAL)
            self.bt_voronoi.config(state=tk.NORMAL)
            self.bt_findgoal.config(state=tk.NORMAL)
            self.bt_start.config(text="Stop")
            self.paused = False
        else:
            print(f'Stop')
            self.bt_default.config(state=tk.DISABLED)
            self.bt_new.config(state=tk.DISABLED)
            self.bt_voronoi.config(state=tk.DISABLED)
            self.bt_findgoal.config(state=tk.DISABLED)
            self.bt_start.config(text="Start")
            self.paused = True

    def draw_scene(self):
        self.clear_canvas()
        world_scale = self.resolution
        
        radius = self.shelter_radius
        
        for shel in self.shelters:
            u_x = shel[0]
            u_y = shel[1]
            self.canvas.create_oval(world_scale*u_x-radius,world_scale*u_y - radius, 
                world_scale*u_x + radius, world_scale*u_y + radius, fill="red", tag="shelters")
        
        for obst in self.obstacles:
            obs = []
            for p in obst.points:
                obs.append((p[0]*world_scale, p[1]*world_scale))
            self.canvas.create_polygon(obs, fill="green", tag="obstacle")
            #self.canvas.create_polygon(world_scale*obst.x_min, world_scale*obst.y_min, 
            #world_scale*obst.x_max, world_scale*obst.y_max, fill="green", tag="obstacle")

    def draw_agents(self):
        self.canvas.delete("agents")
        world_scale = self.resolution
        
        radius = self.agent_radius
        
        for agent in self.agents:
            if agent.disable!= True:
                u_x = agent.pos[0]
                u_y = agent.pos[1]
                self.canvas.create_oval(world_scale*u_x-radius,world_scale*u_y - radius, 
                    world_scale*u_x + radius, world_scale*u_y + radius, fill="blue", tag="agents")

    def draw_config(self, config, color, name):
        world_scale = self.resolution
        
        points = getRobotPlacement(config, self.robot_width, self.robot_height)
        corners = [(world_scale*(x[0]), world_scale*(x[1]))  for x in points]

        self.canvas.create_polygon(corners,
                fill=color, tag= name
        )
 
    def draw_path(self, path):
        #start_color = np.array([227, 74, 51])
        #end_color =  np.array([254, 232, 200])
        #start_color = np.array([67,162,202])
        #end_color =  np.array([224,255,219])
        self.clear_canvas()

        world_scale = self.resolution
        start_color = np.array([0,255,0])
        end_color =  np.array([0,0,255])
        #print(f'draw path: {path}, len: {len(path)}')
        for i in range(len(path)):
            #color = start_color + float(i)/float(len(path)) * (end_color - start_color)
            #tk_rgb = "#%02x%02x%02x" % tuple(int(c) for c in color)
            
            if i>=1:
                self.canvas.create_line(path[i-1][0]*world_scale, path[i-1][1]*world_scale, path[i][0]*world_scale, path[i][1]*world_scale, fill = "blue", tag = "path")
            #self.draw_config(path[i], tk_rgb, "path")
          
    
    def draw_roadmap(self, roadmap, radius=1.0):
        world_scale = self.resolution
        for i,u in enumerate(roadmap.getVertices()):
            u_x = u.getConfiguration()[0]
            u_y = u.getConfiguration()[1]
            #print(f'u_x: {u_x}, u_y: {u_y}')
            for e in u.getEdges():   # draw edges
                #print(f'edge path: {e.path}')
                #for p in e.path:
                 #   print(f'path point: {p}')
                 #   self.canvas.create_oval(world_scale*p[0], world_scale*p[1], world_scale*p[0], 
                 #       world_scale*p[1], fill="green", tag="roadmap") 
                v = roadmap.getVertices()[e.getId()]
                self.canvas.create_line(world_scale*u_x, world_scale*u_y, world_scale*(v.getConfiguration()[0]), 
                world_scale*(v.getConfiguration()[1]), fill="grey40", dash = (1,1), tag="roadmap") 
            #self.canvas.create_oval(world_scale*(u_x -radius), world_scale*(u_y -radius), world_scale*(u_x + radius), 
            #world_scale*(u_y + radius), fill="black", tag="roadmap")    #draw vertex

    def drawLinesOnCanvas(self, lines):
        world_scale = self.resolution/self.scene_width
        for l in lines:
            self.canvas.create_line(world_scale*l[0], world_scale*l[1], world_scale*l[2], world_scale*l[3], fill='blue')             

    def clear_canvas(self):
        self.canvas.delete("roadmap")
        self.canvas.delete("start")
        self.canvas.delete("goal")
        #self.canvas.delete("agents")
        #self.canvas.delete("obstacle")
    
    def updateSim(self, dt):
        global reachedGoals
        #print('updateSim')
        # compute the forces acting on each agent
        for agent in self.agents:
            agent.computeAction(self.agents, self.obstacles)

        reachedGoals = True    
        for agent in self.agents:
            agent.update(dt)
            if not agent.atGoal:
                reachedGoals = False

    def drawFrame(self, dt, win, framedelay):
        #paused = self.paused

        #elapsed_time = time.time() - start_time
        #start_time = time.time()
        if not self.paused:
            self.updateSim(dt)
            self.draw_agents() 
        
        
        win.after(framedelay,lambda: self.drawFrame(dt, win, framedelay))
