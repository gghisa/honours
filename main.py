# -*- coding: utf-8 -*-
"""
Created on Sat May  7 17:40:07 2022

@author: ghisa

BEFORE RUNNING THE SCRIPT REMEMBER TO CLOSE THE SPYDER CONSOLE SO TO OPEN A NEW
ONE TO BE ABLE TO USE THE SAME PORT IN CONSECUTIVE SIMULATIONS (VISUALISATIONS)
"""


from ABMsetup import *

width = 35
height = width
n_agents = 1
stations = []
restpoints =  []
obstacles = [(15,15),(15,16),(15,17),(15,18),(15,19),(15,20),(15,21),(15,22),(15,23),(15,24),
           (15,25),(15,26),(15,27),(15,28),(15,14),(15,13),(15,12),(16,16),(17,16),(18,16),
           (19,16),(20,16),(21,16),(22,16),(23,16),(24,16),(25,16),(26,16),(27,16)]

#make sure grid is large enough for all obstacles, restpoints, stations
for ob in obstacles + restpoints + stations:
    if width <= ob[0]:
        width = ob[0] + 1
    if height <= ob[1]:
        height = ob[1] + 1

visualise(n_agents,width, height, stations, restpoints, obstacles)

