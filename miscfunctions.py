# -*- coding: utf-8 -*-
"""
Created on Thu Aug 19 18:21:15 2021

@author: ghisa
"""

from math import *

def eucl_dist(cell1,cell2):
    return sqrt( (cell1[0]-cell2[0])**2 + (cell1[1]-cell2[1])**2 )

def manh_dist(cell1,cell2):
    return abs(cell1[0]-cell2[0]) + abs(cell1[1] - cell2[1])

def tche_dist(cell1,cell2):
    return max( abs(cell1[0] - cell2[0]), abs(cell1[1],cell2[1]) )

