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
import random


def buildArena(sim, root):
    width = 14
    arena_size = [width, width, 0.2]
    arena_pos = [0,0,-1]
    h = 0.7

    floor = agx.RigidBody( agxCollide.Geometry( agxCollide.Box(arena_size[0]/2, arena_size[1]/2, arena_size[2]/2)))
    floor.setPosition(arena_pos[0], arena_pos[1], arena_pos[2]-arena_size[2]/2)
    floor.setMotionControl(1)
    sim.add(floor)
    agxOSG.setDiffuseColor(agxOSG.createVisual(floor, root), agxRender.Color.Gray())

    sides = 8
    side_len = width/(1+np.sqrt(2)) + arena_size[2]/2/np.sqrt(2)
    base_pos = agx.Vec3(arena_pos[0], arena_pos[1], arena_pos[2]-arena_size[2]/2+h/2)
    for w in range(sides):
        theta = -w*np.pi/4
        rot = agx.Quat(theta, agx.Vec3(0,0,1))
        rot_pos = agx.Vec3(np.sin(theta)*width/2, -np.cos(theta)*width/2, 0)

        wall = agx.RigidBody( agxCollide.Geometry( agxCollide.Box(side_len/2, arena_size[2]/2, h/2)))
        wall.setPosition(base_pos + rot_pos)
        wall.setMotionControl(1)
        wall.setRotation(rot)
        sim.add(wall)
        agxOSG.setDiffuseColor(agxOSG.createVisual(wall, root), agxRender.Color.DarkGray())
                                              
    obstacles(sim, root, arena_pos[2])

def obstacles(sim, root, h):
    #start plattform
    dims = [1.5, 1.5, 0.06]
    pos = [-6, 0, h+dims[2]/2]
    addboxx(sim, root, dims, pos)

    dims = [0.1, 1.5, 0.3]
    pos = [-6.75, 0, h+dims[2]/2]
    addboxx(sim, root, dims, pos)
    
    dims = [0.1, 1.5, 0.3]
    pos = [-5.25, 0, h+dims[2]/2]
    addboxx(sim, root, dims, pos)
    
    dims = [1.6, 0.1, 0.3]
    pos = [-6, -0.75, h+dims[2]/2]
    addboxx(sim, root, dims, pos)

    # wall

    dims = [4.0, 0.1, 0.22]
    pos = [-3.3, -0.6, h+dims[2]/2]
    addboxx(sim, root, dims, pos)

    # Random stuff in the first quarter
    dims = [0.2, 0.2, 0.8]
    pos = [-4, 0, h+dims[2]/2]
    addboxx(sim, root, dims, pos)
    
    dims = [0.4, 0.25, 0.2]
    pos = [-3, 1.5, h+dims[2]/2]
    addboxx(sim, root, dims, pos)


    for i in range(30):
        x = -0.5 - random.random()*4.75
        y = -0.5 + random.random()*6
        dims = [random.random()*0.6, random.random()*0.6, random.random()*0.6]
        pos = agx.Vec3(x, y, 0)
        if pos.length() < 1.5:
            pos.setLength(1.5+random.random()*3.75)
        if pos.length() > 7.0:
            pos.setLength(1.5+random.random()*5.5)
        pos.set(h+dims[2]/2, 2)
        addboxx(sim, root, dims, pos)
    
    # Pole in the middle with walls around
    dims = [0.5, 0.5, 1.2]
    pos = [0, 0, h+dims[2]/2]
    addboxx(sim, root, dims, pos)

    dims = [0.3, 2.0, 0.4]
    pos = [-1.15, 0, h+dims[2]/2]
    addboxx(sim, root, dims, pos)

    dims = [2.0, 0.3, 0.4]
    pos = [0, -1.15, h+dims[2]/2]
    addboxx(sim, root, dims, pos)

    dims = [0.3, 2.0, 0.4]
    pos = [1.15, 0, h+dims[2]/2]
    addboxx(sim, root, dims, pos)

    dims = [0.3, 7.0, 0.4]
    pos = [0, 3.5, h+dims[2]/2]
    addboxx(sim, root, dims, pos)
    
    # Seesaw board
    dims = [2.2, 0.25, 0.3]
    pos = [2.1, 0.4, h+dims[2]/2]
    addboxx(sim, root, dims, pos)
    seesaw(sim, root, [4,1,h], -0.8*np.pi, h=0.1)
    dims = [0.5, 3.8, 0.18]
    pos = [4.8, 3.25, h+dims[2]/2]
    addboxx(sim, root, dims, pos)

    # Ballroom wall
    dims = [3.6, 0.25, 0.3]
    pos = [2.35, -2.35, h+dims[2]/2]
    wall_in = addboxx(sim, root, dims, pos)
    dims = [3.2, 0.25, 0.3]
    pos = [4.8, -3.4, h+dims[2]/2]
    wall_out = addboxx(sim, root, dims, pos)
    wall_in.setRotation(agx.Quat(-np.pi/4, agx.Vec3(0,0,1)))
    wall_out.setRotation(agx.Quat(-np.pi/4, agx.Vec3(0,0,1)))

    for i in range(150):
        x = 4.0 + random.random()*2.8
        y = 0.75 - random.random()*2.5
        rad = 0.025 + random.random()*0.075
        pos = agx.Vec3(x, y, h+rad+3*random.random()*rad)
        addball(sim, root, rad, pos, Fixed=False)

def seesaw(sim, root, pos, angle, h=0.08):
    d = 0.8
    # Sides
    dims = [0.6, 0.15, h*3/2]
    pos_s = [pos[0]+(d/2+0.3)*np.cos(angle), pos[1]+(d/2+0.3)*np.sin(angle), pos[2]+h/2]
    sideP = addboxx(sim, root, dims, pos_s)
    dims = [0.6, 0.15, h*3/2]
    pos_s = [pos[0]-(d/2+0.3)*np.cos(angle), pos[1]-(d/2+0.3)*np.sin(angle), pos[2]+h/2]
    sideN = addboxx(sim, root, dims, pos_s)
    # Main board
    dims = [d, 0.9, 0.004]
    pos_s = [pos[0]+0.06*np.sin(angle), pos[1]-0.06*np.cos(angle), pos[2]+h]
    board = addboxx(sim, root, dims, pos_s, Fixed=False)

    sideP.setRotation(agx.Quat(angle, agx.Vec3(0,0,1)))
    sideN.setRotation(agx.Quat(angle, agx.Vec3(0,0,1)))
    board.setRotation(agx.Quat(angle, agx.Vec3(0,0,1)))

    #Some stops under
    bot_tilt = 0.17
    dims = [d, 0.43, 0.004]
    dif = 0.215*np.cos(bot_tilt)-0.002*np.sin(bot_tilt)
    pos_s = [pos[0]+(0.06+dif)*np.sin(angle), pos[1]-(0.06+dif)*np.cos(angle), pos[2]+np.sin(bot_tilt)*dif]
    bottom1 = addboxx(sim, root, dims, pos_s, color=agxRender.Color.DarkGray())
    pos_s = [pos[0]+(0.06-dif)*np.sin(angle), pos[1]-(0.06-dif)*np.cos(angle), pos[2]+np.sin(bot_tilt)*dif]
    bottom2 = addboxx(sim, root, dims, pos_s, color=agxRender.Color.DarkGray())

    
    bottom1.setRotation(agx.Quat( bot_tilt, agx.Vec3(1,0,0)))
    bottom1.setRotation(bottom1.getRotation()*agx.Quat(angle, agx.Vec3(0,0,1)))
    
    bottom2.setRotation(agx.Quat(-bot_tilt, agx.Vec3(1,0,0)))
    bottom2.setRotation(bottom2.getRotation()*agx.Quat(angle, agx.Vec3(0,0,1)))
    

    hf = agx.HingeFrame()
    hf.setAxis(agx.Vec3( np.cos(angle),np.sin(angle),0))
    hf.setCenter(agx.Vec3(pos[0]+(d/2)*np.cos(angle), pos[1]+(d/2)*np.sin(angle), pos[2]+h))
    axleP = agx.Hinge(hf, board, sideP)
    sim.add(axleP)

    hf = agx.HingeFrame()
    hf.setAxis(agx.Vec3(  np.cos(angle),np.sin(angle),0))
    hf.setCenter(agx.Vec3(pos[0]-(d/2)*np.cos(angle), pos[1]-(d/2)*np.sin(angle), pos[2]+h))
    axleN = agx.Hinge(hf, board, sideN)
    sim.add(axleN)



    

def addboxx(sim, root, dims, pos, Fixed=True, color = agxRender.Color.Red()):
    if type(pos) == type([]):
        pos = agx.Vec3(pos[0], pos[1], pos[2])
    boxx = agx.RigidBody( agxCollide.Geometry( agxCollide.Box(dims[0]/2, dims[1]/2, dims[2]/2)))
    boxx.setPosition(pos)
    if(Fixed):
        boxx.setMotionControl(1)
    sim.add(boxx)
    agxOSG.setDiffuseColor(agxOSG.createVisual(boxx, root), color)
    return boxx

def addball(sim, root, rad, pos, Fixed=True):
    if type(pos) == type([]):
        pos = agx.Vec3(pos[0], pos[1], pos[2])
    ball = agx.RigidBody( agxCollide.Geometry( agxCollide.Sphere(rad)))
    ball.setPosition(pos)
    if(Fixed):
        ball.setMotionControl(1)
    sim.add(ball)
    agxOSG.setDiffuseColor(agxOSG.createVisual(ball, root), agxRender.Color.Red())
    return ball
