import agx
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


# Create a class that is triggered at various steps in the simulation
class FollowCam(agxSDK.StepEventListener):
    def __init__(self, app, object_to_follow, distance=9, angle=25):
        super().__init__(agxSDK.StepEventListener.PRE_COLLIDE+agxSDK.StepEventListener.PRE_STEP+agxSDK.StepEventListener.POST_STEP)
        self.app = app
        self.body = object_to_follow
        self.dist = distance
        self.angl = np.deg2rad(angle)
        
        self.relative_position = self.dist*agx.Vec3(0,-np.cos(self.angl), np.sin(self.angl))
        self.looker = self.body.getPosition()
        self.position = self.relative_position + self.looker 
        self.updateCamera()

    def preCollide(self, time):
        return

    def pre(self, time):
        relative_position = -self.body.getVelocity()
        relative_position.set(0.0, 2)

        if(relative_position.length() > 1E-2):
            relative_position.setLength(self.dist)
            relative_position.set(relative_position.x()*np.cos(self.angl), relative_position.y()*np.cos(self.angl), self.dist*np.sin(self.angl))
            
            relative_position = self.relative_position + relative_position/60
            relative_position.setLength(self.dist)
            relative_position.set(relative_position.x()*np.cos(self.angl), relative_position.y()*np.cos(self.angl), self.dist*np.sin(self.angl))

            self.relative_position = relative_position
            self.looker = self.body.getPosition()
            self.position = self.relative_position + self.looker 

            self.updateCamera()
        return
    
    def updateCamera(self):
        cameraData                   = self.app.getCameraData()
        cameraData.eye               = self.position
        cameraData.center            = self.looker
        cameraData.up                = agx.Vec3( 0, 0, 1 )
        cameraData.nearClippingPlane = 0.1
        cameraData.farClippingPlane  = 5000
        self.app.applyCameraData( cameraData )

    def post(self, time):
        return