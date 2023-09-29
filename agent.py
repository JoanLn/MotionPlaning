# ageny.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
from abc import ABC, abstractmethod
import math

""" 
    Abstract class for agents
"""
class AbstractAgent(ABC):

    def __init__(self, inputParameters):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(inputParameters[0]) # the id of the agent
        self.gid = int(inputParameters[1]) # the group id of the agent
        self.pos = np.array([float(inputParameters[2]), float(inputParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(inputParameters[4]), float(inputParameters[5])]) # the goal of the agent
        self.prefspeed = float(inputParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(np.sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(inputParameters[7]) # the maximum sped of the agent
        self.radius = float(inputParameters[8]) # the radius of the agent
        self.atGoal = False # has the agent reached its goal?
     
    @abstractmethod
    def computeAction(self, neighbors=[]):
        """
            Performs a sense and act simulation step.
        """
        
        pass

    @abstractmethod    
    def update(self, dt):
        """
            Updates the state of the character, given the time step of the simulation.
        """
        pass
        
    def ttc(radius_self, pos_self, vel_self, radius_tar, pos_tar, vel_tar, timehor, maxF):
        pass

""" 
    Agent class that implements the TTC force-based approach  
"""
class TTCAgent(AbstractAgent):

    def __init__(self, inputParameters, goalRadius=1, dhor = 10, ksi=0.5, timehor=5, epsilon=0, maxF = 10):
        """ 
           Initializes the agent
        """
        super().__init__(inputParameters)
        self.atGoal = False # has the agent reached its goal?
        self.goalRadiusSq = goalRadius * goalRadius # parameter to determine if agent is close to the goal
        self.dhor = dhor # the sensing radius
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.epsilon = epsilon # the error in sensed velocities
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent
        self.path = []
        self.disable = False

    def computeAction(self, neighbors=[], obstacles = []):
        """ 
            Your code to compute the forces acting on the agent. 
        """       

        if not self.atGoal and self.disable != True:
            self.F = np.zeros(2)
            self.F = (self.gvel-self.vel)/self.ksi
            
            
            for n in neighbors:
                d = np.sqrt((self.pos[0]-n.pos[0])**2 + (self.pos[1]-n.pos[1])**2)
                if self.gid != n.gid and d <= self.dhor and n.disable != True:

                    Favoid , mag = self.ttc(self.radius, self.pos, self.vel, n.radius, n.pos, n.vel, self.timehor, self.maxF)
		            # calculate time-to-collision
                    # r = self.radius + n.radius
                    # x = self.pos - n.pos
                    # v = self.vel - n.vel
                
                    # a = np.dot(v, v) - self.epsilon**2
                    # b = np.dot(x, v) - self.epsilon*r
                    # c = np.dot(x, x) - r * r
                    # if (c < 0): #agents are colliding
                        # time = 0
                    # discr = b*b - a*c
		
                    # #print(f'a: {a}, b: {b}, c: {c}, d: {discr}')
		
                    # if (discr <= 0):
                        # time = -1
                    # else:
                        # tau = (-b - np.sqrt(discr)) / a
                        # #print(f'tau: {tau}')
                        # if (tau < 0):
                            # time = -1
                        # else:
                            # time = tau
				
                    # Favoid = self.pos - n.pos + self.vel*time - n.vel*time
                    # if Favoid[0]!=0 and Favoid[1]!=0:
                        # Favoid = Favoid / np.sqrt(np.dot(Favoid, Favoid)) # make Favoid the direction of the force
		
                    # mag = 0
                    # if time>=0 and time<=self.timehor:
                        # mag = max((self.timehor-time),0)/(time+0.001) # avoid divide by 0
                    # if mag>self.maxF:
                        # mag = self.maxF
                    # Favoid = mag * Favoid # calculate the force with magnitude andn direction
			    
                    #print(f'Favoid: {Favoid}, mag: {mag}')
                    self.F = self.F + Favoid
            
            for obs in obstacles:
                line = []
                line.append((obs.x_min, obs.y_min, obs.x_max, obs.y_min))
                line.append((obs.x_min, obs.y_max, obs.x_max, obs.y_max))
                line.append((obs.x_min, obs.y_min, obs.x_min, obs.y_max))
                line.append((obs.x_max, obs.y_min, obs.x_max, obs.y_max))
                maxMag = 0
                Favoid = np.zeros(2)
                maxF = None
                
                for xpos in np.linspace(obs.x_min, obs.x_max, 18):
                    for ypos in np.linspace(obs.y_min, obs.y_max, 18):
                        dis = math.sqrt( (self.pos[0]-xpos)*(self.pos[0]-xpos) + (self.pos[1]-ypos)*(self.pos[1]-ypos) )
                        if dis <= self.dhor:
                            #print(f'obs: {obs.x_min, obs.y_min, obs.x_max, obs.y_max}')
                            Favoid, mag = self.ttc(self.radius, self.pos, self.vel, 0, (xpos, ypos), 0, self.timehor, self.maxF)
                            self.F = self.F + Favoid	
                            #if maxMag<mag:
                            #    maxF = Favoid


    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel += self.F*dt     # update the velocity
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                #print(f'path: {path}')
                self.atGoal = True  # goal has been reached
                self.disable = True
                if len(self.path) != 0:
                    self.goal = self.path[0]
                    del self.path[0]
                    self.atGoal = False
                    self.disable = False
            else: 
                self.gvel = self.gvel/np.sqrt(distGoalSq)*self.prefspeed  
            
    def ttc(self, radius_self, pos_self, vel_self, radius_tar, pos_tar, vel_tar, timehor, maxF):

        r = radius_self + radius_tar
        x = pos_self - pos_tar
        v = vel_self - vel_tar
                
        a = np.dot(v, v)
        b = np.dot(x, v)
        c = np.dot(x, x) - r * r
        if (c < 0): #agents are colliding
            time = 0
        discr = b*b - a*c

        #print(f'r: {r}, x: {x}, pos_self: {pos_self}, pos_tar: {pos_tar}, v: {v}')
        #print(f'a: {a}, b: {b}, c: {c}, d: {discr}')         
		
        if (discr <= 0):
            time = -1
        else:
            tau = (-b - np.sqrt(discr)) / a
            #print(f'tau: {tau}')
            if (tau < 0):
                time = -1
            else:
                time = tau
				
        Favoid = pos_self - pos_tar + vel_self*time - vel_tar*time
        if Favoid[0]!=0 and Favoid[1]!=0:
            Favoid = Favoid / np.sqrt(np.dot(Favoid, Favoid)) # make Favoid the direction of the force
		
        mag = 0
        if time>=0 and time<=timehor:
            mag = max((timehor-time),0)/(time+0.001) # avoid divide by 0
        if mag>maxF:
            mag = maxF
        Favoid = mag * Favoid # calculate the force with magnitude andn direction
        #print(f'mag: {mag}')
        return Favoid, mag

