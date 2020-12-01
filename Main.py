'''

This script is a startup template for scripting with the AGX Dynamics Python API

'''
import sys

try:
    import agx
except:
    sys.exit("Could not import AGX, run \"C:\\Program Files\\Algoryx\\AGX-2.29.2.0\\setup_env.bat\" in terminal, including citation marks.")
# "C:\Program Files\Algoryx\AGX-2.29.2.0\setup_env.bat"

# To add agx to pylint, for autocompleting functions and removing warnings:
# Open the .vscode directory
# Open the settings.json file in the editor
# Add:
#     "python.autoComplete.extraPaths": [
#         "C:/Program Files/Algoryx/AGX-2.29.2.0/bin/x64/agxpy"
#     ],
# to the bottom row, above the }. 
# Change the line if your AGX install directory is different.

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
import arena
import robot
import camera

players = 1
drivetrain = "AWD"
# Drivetrains: "AWD", "FWD", "RWD"

#
# "main" def that creates the scene
#
def buildScene():

    sim = agxPython.getContext().environment.getSimulation()
    app = agxPython.getContext().environment.getApplication()
    root = agxPython.getContext().environment.getSceneRoot()

    dec = app.getSceneDecorator()
    dec.setEnableLogo(False)

    arena.buildArena(sim,root)
    bot_pos = [-6, 0,-0.2]
    if players == 2:
        bot1_pos = [bot_pos[0]-0.35,bot_pos[1],bot_pos[2]]
        bot2_pos = [bot_pos[0]+0.35,bot_pos[1],bot_pos[2]]

        bot1 = robot.buildBot(sim, root, bot1_pos, controller='Arrows', drivetrain = drivetrain)
        bot2 = robot.buildBot(sim, root, bot2_pos, controller='Numpad', drivetrain = drivetrain, color=agxRender.Color.Cyan())

        cameraData                   = app.getCameraData()
        cameraData.eye               = agx.Vec3( 15, 0, 12)
        cameraData.center            = agx.Vec3( 2.25, 0, 0)
        cameraData.up                = agx.Vec3( 0,0,1 )
        cameraData.nearClippingPlane = 0.1
        cameraData.farClippingPlane  = 5000
        app.applyCameraData( cameraData )
    else:
        botBody = robot.buildBot(sim, root, bot_pos, controller='Arrows', drivetrain = drivetrain)
        sim.add(camera.FollowCam(app, botBody))

def main(args):

    ## Create an application with graphics etc.
    app = agxOSG.ExampleApplication()

    ## Create a command line parser. sys.executable will point to python executable
    ## in this case, because getArgumentName(0) needs to match the C argv[0] which
    ## is the name of the program running
    argParser = agxIO.ArgumentParser([sys.executable] + args)

    app.addScene(argParser.getArgumentName(1), "buildScene", ord('1'), True)

    ## Call the init method of ExampleApplication
    ## It will setup the viewer, windows etc.
    if app.init(argParser):
        app.run()
    else:
        print("An error occurred while initializing ExampleApplication.")


## Entry point when this script is loaded with python
if agxPython.getContext() == None:
    init = agx.AutoInit()
    main(sys.argv)


