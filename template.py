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

import src.Components

from tutorials.tutorial_utils import createHelpText

players = 1

# Create a class that is triggered at various steps in the simulation
class FollowCam(agxSDK.StepEventListener):
    def __init__(self, app, object_to_follow, distance=9, angle=25):
        super().__init__(agxSDK.StepEventListener.PRE_COLLIDE+agxSDK.StepEventListener.PRE_STEP+agxSDK.StepEventListener.POST_STEP)
        self.app = app
        self.body = object_to_follow
        self.dist = distance
        self.angl = np.deg2rad(angle)
        self.camData = app.getCameraData()

    def preCollide(self, time):
        return
        # print("preCollide")

    def pre(self, time):
        position = -self.body.getVelocity()
        position.set(0.0, 2)

        if(position.length() > 1E-2):
            looker = self.body.getPosition()
            position.setLength(self.dist)
            position.set(position.x()*np.cos(self.angl), position.y()*np.cos(self.angl), self.dist*np.sin(self.angl))
            position = position + looker     

            cameraData                   = self.app.getCameraData()
            cameraData.eye               = position
            cameraData.center            = looker
            cameraData.up                = agx.Vec3( 0, 0, 1 )
            cameraData.nearClippingPlane = 0.1
            cameraData.farClippingPlane  = 5000
            self.app.applyCameraData( cameraData )
        return
        # print("pre")

    def post(self, time):
        return
        # print("post")

#
# "main" def that creates the scene
#
def buildScene():

    sim = agxPython.getContext().environment.getSimulation()
    app = agxPython.getContext().environment.getApplication()
    root = agxPython.getContext().environment.getSceneRoot()

    # body1 = src.Components.MC095()
    # sim.add(body1)
    # # Create a visual representation
    # agxOSG.setDiffuseColor(agxOSG.createVisual(body1, root), agxRender.Color.Red())
    # # Create a constraint
    # f1 = agx.Frame()
    # spring = agx.DistanceJoint(body1, f1, agx.Vec3())
    # spring.setCompliance(1E-3)
    # # Add the constraint to the simulation
    # sim.add(spring)

    arena.buildArena(sim,root)
    bot_pos = [-6, 0,-0.2]
    if players == 2:
        bot1_pos = [bot_pos[0]-0.35,bot_pos[1],bot_pos[2]]
        bot2_pos = [bot_pos[0]+0.35,bot_pos[1],bot_pos[2]]

        bot1 = robot.buildBot(sim, root, bot1_pos, controller='Arrows', drivetrain = 'FWD')
        bot2 = robot.buildBot(sim, root, bot2_pos, controller='Numpad', drivetrain = 'FWD', color=agxRender.Color.Cyan())

        cameraData                   = app.getCameraData()
        cameraData.eye               = agx.Vec3( 15, 0, 12)
        cameraData.center            = agx.Vec3( 2.25, 0, 0)
        cameraData.up                = agx.Vec3( 0,0,1 )
        cameraData.nearClippingPlane = 0.1
        cameraData.farClippingPlane  = 5000
        app.applyCameraData( cameraData )
    else:
        botBody = robot.buildBot(sim, root, bot_pos, controller='Arrows', drivetrain = 'FWD')
        sim.add(FollowCam(app, botBody))
        # Setup the initial camera pose. (Can be retrieved using the 'C' button)
        cameraData                   = app.getCameraData()
        cameraData.eye               = agx.Vec3( bot_pos[0] - 0.00323694, bot_pos[1] - 8.14261, bot_pos[2] + 2.73292)
        cameraData.center            = agx.Vec3( bot_pos[0] + 3.16354e-05, bot_pos[1] + 0.014163, bot_pos[2] - 0.670644)
        cameraData.up                = agx.Vec3( 0,0,1 )
        cameraData.nearClippingPlane = 0.1
        cameraData.farClippingPlane  = 5000
        app.applyCameraData( cameraData )

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


