# -*- coding: utf-8 -*-
"""
Created on Sat May  7 15:55:17 2022

@author: ghisa

This script contains "all" that is needed to run the Agent-Based Model. It is
a modeller: it simulates the simplified physical world of the agents. In this
script you can find the classes, some functions, and the visualiser coming
together. It is intended so that other scripts take care of the algorithms
that have to be implemented as well as the actual main(). Hopefully this method
provides a modular framework within which to simulate as many environments as
possible.

Obstacles and stations are agents, but they are also stored in the Model as
parts of a list and can be accessed in principle by any agent at any time.

Question: how do you visualise non-agent objects? With MESA you can only
visualise agents.

Question: how do you initially place agents? For now, all in one cell at (0,0)

Algos is the input functions that each agent needs to compute where to go and
all of that. Algos is a list of functions that have to behave as follows:
    1. Algos is a function returning, as a tuple, the next step of the
       agent and takes as argument an agent (for which to find the next move),
       and the index of the function that is requested to be run. Index 0 
       corresponds to a moving function for now.
.
Additional libraries to possibly import:
from mesa.datacollection import DataCollector
from mesa.batchrunner import BatchRunner
"""
from mesa import Model, Agent #parent classes
from mesa.space import MultiGrid #}use SingleGrid for one agent allowed per block
from mesa.time import SimultaneousActivation

from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer

from algorithms import algos

import numpy as np

import matplotlib.pyplot as plt

from random import randint as ri

#define classes
class model(Model):
    
    def __init__(self,n_agents,w,h,stations = [],restpoints = [] ,obstacles = []):
        self.n_agents = n_agents
        self.grid = MultiGrid(w,h,torus = False)
        self.schedule = SimultaneousActivation(self)
        self.running = True #for batch execution (?)
        self.stations = stations
        self.restpoints = restpoints
        self.obstacles = obstacles
        self.agents = []
        # Datacollector? no implementation yet
        
        #create agents
        for i in range(self.n_agents):
            unit = agent(i,self) # needs agent class to be defined
            #self.grid.place_agent(unit,(0,0)) #how to place agents?
            self.schedule.add(unit) #probably for making an agent part of the execution
            self.agents.append(unit)
            
        #create obstacles
        for i in range(len(obstacles)):
            self.grid.place_agent(obstacle(i,self), obstacles[i])
            
    def step(self):
        #self.datacollector.collect(self) ???
        self.schedule.step()
        return


class agent(Agent):
    def __init__(self,identifier,model,rest = (0,0),goal = (10,10)):
        super().__init__(identifier,model)
        self.model.grid.place_agent(self,rest)
        self.goal = goal
        self.potential = potential(self) #element to store the potential of an agent
        self.path = []
    
    def find_path(self):
        self.potential.compute()
        self.potential.plot()
        self.path = algos(self,2)
        '''
        for i in range(len(self.path)):
            self.model.grid.place_agent(path(self.unique_id+500,self.model,self),self.path[i])
            #!!! Godel's numbers?
        '''
        return
    
    def move(self):
        if len(self.path) > 0:
            self.model.grid.move_agent(self,self.path[0])
        return
    
    def step(self): #step function is called at any time the step
    
        while self.pos == self.goal or self.goal in self.model.obstacles:
            self.goal = (ri(0,self.model.grid.width),ri(0,self.model.grid.height))
            
        self.find_path()#!!! finding path every step: computationally inefficient
        self.move()
        return

class obstacle(Agent):
    def __init__(self,unique_id,model):
        super().__init__(unique_id,model)
        
    def step(self): pass

class path(Agent):
    def __init__(self,unique_id,model,unit):
        super().__init__(unique_id,model)
        self.agent = unit #an agent has its own path particles
        
    def step(self):
        if self.pos not in self.agent.path:
            #autonomously remove itself if its not in the path of an agent anymore
            self.agent.model.grid.remove_agent(self)

class potential():
    def __init__(self, unit):
        #obstacles here may include both static and dynamic ones
        #for dynamic ones, only position is known for know but their speed
        #may be useful when in play so keep in mind for expaning
        
        self.obstacles = unit.model.obstacles
        self.ground = unit.goal
        self.agent = unit
        self.model = unit.model
        
        if self.ground in self.obstacles:#more actions to be taken?
            print("Goal is an obstacle!!")
            
        self.map = np.zeros((self.model.grid.width,self.model.grid.height)) #not sure these are attributes
            
    def compute(self):
        self.map = algos(self.agent,1)
        return

    def plot(self):
        plt.imshow(self.map,interpolation='nearest',origin='lower')
        plt.colorbar()
        return
    
#create visualiser
def agent_portrayal(unit):
    if type(unit) == agent:
        return {'Shape': 'circle',
             'Color': 'red',
             'Filled':'true',
             'text': unit.unique_id,
             'text_color': 'white',
             'Layer':0,
             'r':0.5}
    elif type(unit) == obstacle:
        return {'Shape': 'rect',
             'Color': 'black',
             'Filled':'true',
             'Layer':0,
             'w':0.5,
             'h':0.5}
    elif type(unit) == path:
        return {'Shape': 'circle',
             'Color': 'green',
             'Filled':'true',
             'text': unit.agent.unique_id,
             'text_color': 'black',
             'Layer':0,
             'w':0.5,
             'h':0.5}

def visualise(n_agents, w, h,
              stations = [],restpoints = [] ,obstacles = []):
    grid = CanvasGrid(agent_portrayal, w, h,w*20,h*20)
    server = ModularServer(model,[grid],
                           'ABM',
                           {'n_agents': n_agents,
                            'w': w,
                            'h': h,
                            'stations': stations,
                            'restpoints': restpoints,
                            'obstacles': obstacles})
    server.port = 8851
    server.launch()
    return
