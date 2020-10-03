import sys

try:
    import agx
except:
    sys.exit("Could not import AGX, run \"C:\Program Files\Algoryx\AGX-2.29.2.0\setup_env.bat\" in terminal, including citation marks.")

import agxCollide
import agxOSG
import agxSDK
import agxPython
import agxIO
import agxModel
import agxRender

import time
import math
import numpy as np


def buildArena(sim, root):
    arena_size = [8,8,0.2]
    arena_pos = [0,0,-1]
    h = 0.7

    floor = agx.RigidBody( agxCollide.Geometry( agxCollide.Box(arena_size[0]/2, arena_size[1]/2, arena_size[2]/2)))
    floor.setPosition(arena_pos[0], arena_pos[1], arena_pos[2]-arena_size[2]/2)
    floor.setMotionControl(1)
    sim.add(floor)
    agxOSG.setDiffuseColor(agxOSG.createVisual(floor, root), agxRender.Color.Gray())

    wall = agx.RigidBody( agxCollide.Geometry( agxCollide.Box(arena_size[0]/2, arena_size[2]/2, h/2)))
    wall.setPosition(arena_pos[0], arena_pos[1]+arena_size[1]/2+arena_size[2]/2, arena_pos[2]-arena_size[2]/2+h/2)
    wall.setMotionControl(1)
    sim.add(wall)
    agxOSG.setDiffuseColor(agxOSG.createVisual(wall, root), agxRender.Color.DarkGray())

    wall = agx.RigidBody( agxCollide.Geometry( agxCollide.Box(arena_size[0]/2, arena_size[2]/2, h/2)))
    wall.setPosition(arena_pos[0], arena_pos[1]-arena_size[1]/2-arena_size[2]/2, arena_pos[2]-arena_size[2]/2+h/2)
    wall.setMotionControl(1)
    sim.add(wall)
    agxOSG.setDiffuseColor(agxOSG.createVisual(wall, root), agxRender.Color.DarkGray())

    wall = agx.RigidBody( agxCollide.Geometry( agxCollide.Box(arena_size[2]/2, arena_size[1]/2 + arena_size[2], h/2)))
    wall.setPosition(arena_pos[0]-arena_size[0]/2-arena_size[2]/2, arena_pos[1], arena_pos[2]-arena_size[2]/2+h/2)
    wall.setMotionControl(1)
    sim.add(wall)
    agxOSG.setDiffuseColor(agxOSG.createVisual(wall, root), agxRender.Color.DarkGray())

    wall = agx.RigidBody( agxCollide.Geometry( agxCollide.Box(arena_size[2]/2, arena_size[1]/2 + arena_size[2], h/2)))
    wall.setPosition(arena_pos[0]+arena_size[0]/2+arena_size[2]/2, arena_pos[1], arena_pos[2]-arena_size[2]/2+h/2)
    wall.setMotionControl(1)
    sim.add(wall)
    agxOSG.setDiffuseColor(agxOSG.createVisual(wall, root), agxRender.Color.DarkGray())

    #                      Arena span in x                                              
    # obstacles(sim, root, [[arena_pos[0]-arena_size[0], arena_pos[0]+arena_size[0]], [arena_pos[1]-arena_size[1], arena_pos[1]+arena_size[1]], [arena_pos[2], arena_pos[2]+h]])

def obstacles(sim, root, grid):
    grid_x = grid[0]
    grid_y = grid[1]
    grid_z = grid[2]
    sx = (grid_x[1]-grid_x[0])/2
    sy = grid_y[1]-grid_y[0]
    sz = grid_z[1]-grid_z[0]

    boxx = agx.RigidBody( agxCollide.Geometry( agxCollide.Box(sx*0.1, 1, 1)))
    boxx.setPosition(0,0,1+grid_z[0]+sz*0.8)
    boxx.setMotionControl(1)
    sim.add(boxx)
    agxOSG.setDiffuseColor(agxOSG.createVisual(boxx, root), agxRender.Color.Red())
