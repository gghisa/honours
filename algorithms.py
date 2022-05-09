# -*- coding: utf-8 -*-
"""
Created on Sat May  7 17:15:21 2022

@author: ghisa
"""
from mesa import Model
from miscfunctions import *
import numpy as np

def algos(agent,index):
    '''
    function which returns a tuple according to instructions in main.py
    '''
    if index == 0:
        return diagonal(agent,1)
    if index == 1:
        return potential(agent.pos, agent.goal, agent.model)
    if index == 2:
        return findmove(agent.potential)[2]
    
def diagonal(agent,cells=1):
    return (agent.pos[0]+cells,agent.pos[1]+cells)

def potential(origin: tuple, goal: tuple, model: Model,
              distance_method = 'euclid'):
    '''
    

    Parameters
    ----------
    origin : tuple
        Coordinates of the starting point.
    goal : tuple
        Coordinates of the finishing point.
    model : Model
        Mesa system model used.
    distance_method : str, optional
        Method used to calculate the nodes' distance. The default is 'euclid'.

    Returns
    -------
    vals : np.array
        Heatmap of the grid.
    '''
    distance = None
    if distance_method == 'euclid': distance = eucl_dist
    elif distance_method == 'manhattan': distance = manh_dist
    else: distance = tche_dist
    #create open list
    op_list = [goal]
    #set cost of single cell to high. post-multiplying factor to be changed
    vals = np.ones((model.grid.width,model.grid.height))*model.grid.height
    #set cost of goal cell to 0
    vals[goal[0],goal[1]] = 0
    
    while len(op_list) > 0: #while there are still nodes to consider
        op_len = len(op_list)
        
        for i in range(op_len): #for the open nodes present in the list at the beginning of the current loop
            cell = op_list[i] #select the cell
            ns = model.grid.get_neighborhood(cell, moore=True) #get neighbours
            
            for n in ns: #for every neighbouring cell
                if n not in model.obstacles: #if it is not an obstacle
                    new_val = vals[cell[0],cell[1]] + distance(cell,n) #compute new potential distance value
                    if new_val < vals[n[0],n[1]]: #if the new one is lower
                        vals[n[0],n[1]] = new_val #update the current neightbour value
                        op_list.append(n) #then add the node to the open list (only if new value was lower already)
        for i in range(op_len-1,-1,-1): #remove all nodes looped over in the current run
            op_list.pop(i)
    
    return vals

def findmove(potential):
    vals = potential.map
    origin = potential.agent.pos
    goal = potential.ground
    model = potential.agent.model
    #now that the cost values are given, find the path
    #by going through each cell from the starting one and selecting from its neighbours
    #that with the lowest cost
    #!!! if a cell has neibours with the same cost, cost wise it may not change, however it may be relevant
    #for collision avoidance to choose one instead of the other   
    current_cell = origin
    path = []#!!! removed origin from list here
    steps = []
    while current_cell != goal:
        lowest_cost = np.max(vals)
        next_cell = None
        for n in model.grid.get_neighborhood(current_cell, moore=True):
            #it is assumed the robot can move diagonally
            if vals[n[0],n[1]] <= lowest_cost:
                lowest_cost = vals[n[0],n[1]]
                next_cell = n
        #then normalise cell postion to get the step to take
        dx = next_cell[0]-current_cell[0]
        dy = next_cell[1]-current_cell[1]
        current_cell = next_cell
        steps.append((dx,dy))    
        path.append(next_cell)
    #add to heatmap the path
    for cell in path:
        vals[cell[0],cell[1]] = -model.grid.height/4 
    
    return vals,steps,path