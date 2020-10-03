import agx
import agxCollide
import agxOSG
import agxSDK
import agxPython
import agxIO
import agxModel
import agxRender
import numpy as np

# from MiroClasses import MiroComponent as mc

density = {
    'ABS': 950,
    'PVC': 1380,
    'Aluminium': 2700,
    'Steel Forged Concrete': 5000,
}


############### MC0XX ###############
# Plate with mounting sockets in corners on one side
def MC0XX(M1, M2, M3, rot = [0,0,0], pos = [0,0,0], Fixed = False):
    size_x = M1
    size_y = M3
    size_z = M2
    density_brick = density['ABS']   # kg/m^3
    mass_brick = density_brick * size_x * size_y * size_z

    return agx.RigidBody( agxCollide.Geometry( agxCollide.Box(size_x,size_y,size_z)))

    # inertia_brick_xx = (size_y**2 + size_z**2)*mass_brick/3
    # inertia_brick_yy = (size_x**2 + size_z**2)*mass_brick/3
    # inertia_brick_zz = (size_x**2 + size_y**2)*mass_brick/3

    # body_brick = chrono.ChBody()
    # body_brick.SetBodyFixed(Fixed)
    # body_brick.SetCollide(True)
    # # set mass properties
    # body_brick.SetMass(mass_brick)
    # body_brick.SetInertiaXX(chrono.ChVectorD(inertia_brick_xx, inertia_brick_yy, inertia_brick_zz))     

    # # Collision shape
    # body_brick.GetCollisionModel().ClearModel()
    # body_brick.GetCollisionModel().AddBox(size_x/2, size_y/2, size_z/2) # must set half sizes
    # body_brick.GetCollisionModel().BuildModel()

    # # Visualization shape, for rendering animation
    # body_brick_shape = chrono.ChBoxShape()
    # body_brick_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_x/2, size_y/2, size_z/2)
    
    # body_brick_shape.SetColor(chrono.ChColor(0.65, 0.65, 0.6)) # set gray color 
    # body_brick.AddAsset(body_brick_shape)

    # # Apply texture
    # texture = chrono.ChTexture()
    # texture.SetTextureFilename(chrono.GetChronoDataFile('textures/MITstol.jpg'))
    # texture.SetTextureScale(1, 1)
    # body_brick.AddAsset(texture)

    # # Generate MiroComponent with above ChBody
    # COMPONENT = mc.MiroComponent(body_brick)

    # # Top
    # COMPONENT.AddLinkPoint('A', [0, 1, 0], chrono.ChVectorD( (size_x/2-0.02),  size_y/2,  (size_z/2-0.02)))
    # COMPONENT.AddLinkPoint('B', [0, 1, 0], chrono.ChVectorD(-(size_x/2-0.02),  size_y/2,  (size_z/2-0.02)))
    # COMPONENT.AddLinkPoint('C', [0, 1, 0], chrono.ChVectorD( (size_x/2-0.02),  size_y/2, -(size_z/2-0.02)))
    # COMPONENT.AddLinkPoint('D', [0, 1, 0], chrono.ChVectorD(-(size_x/2-0.02),  size_y/2, -(size_z/2-0.02)))
    # # Bottom
    # COMPONENT.AddLinkPoint('E', [0,-1, 0], chrono.ChVectorD( 0, -size_y/2, 0))
    # # Sides
    # COMPONENT.AddLinkPoint('F', [0, 0,-1], chrono.ChVectorD(-(size_x/2-0.02),  0,-size_z/2))
    # COMPONENT.AddLinkPoint('G', [0, 0,-1], chrono.ChVectorD( (size_x/2-0.02),  0,-size_z/2))
    # COMPONENT.AddLinkPoint('H', [0, 0, 1], chrono.ChVectorD(-(size_x/2-0.02),  0, size_z/2))
    # COMPONENT.AddLinkPoint('I', [0, 0, 1], chrono.ChVectorD( (size_x/2-0.02),  0, size_z/2))
    # COMPONENT.AddLinkPoint('J', [-1, 0,0], chrono.ChVectorD(-size_x/2,  0, 0))
    # COMPONENT.AddLinkPoint('K', [ 1, 0,0], chrono.ChVectorD( size_x/2,  0, 0))
    
    # COMPONENT.Rotate(rot)
    # COMPONENT.MoveToPosition(pos)

    # return COMPONENT

# MC01X
def MC011(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.08, 0.08, 0.02, rot, pos, Fixed)
def MC012(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.12, 0.08, 0.02, rot, pos, Fixed)
def MC013(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.16, 0.08, 0.02, rot, pos, Fixed)
def MC014(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.20, 0.08, 0.02, rot, pos, Fixed)
def MC015(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.24, 0.08, 0.02, rot, pos, Fixed)

# MC02X
def MC021(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.08, 0.12, 0.02, rot, pos, Fixed)
def MC022(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.12, 0.12, 0.02, rot, pos, Fixed)
def MC023(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.16, 0.12, 0.02, rot, pos, Fixed)
def MC024(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.20, 0.12, 0.02, rot, pos, Fixed)
def MC025(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.24, 0.12, 0.02, rot, pos, Fixed)

# MC03X
def MC031(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.08, 0.16, 0.02, rot, pos, Fixed)
def MC032(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.12, 0.16, 0.02, rot, pos, Fixed)
def MC033(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.16, 0.16, 0.02, rot, pos, Fixed)
def MC034(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.20, 0.16, 0.02, rot, pos, Fixed)
def MC035(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.24, 0.16, 0.02, rot, pos, Fixed)

# MC04X
def MC041(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.08, 0.20, 0.02, rot, pos, Fixed)
def MC042(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.12, 0.20, 0.02, rot, pos, Fixed)
def MC043(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.16, 0.20, 0.02, rot, pos, Fixed)
def MC044(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.20, 0.20, 0.02, rot, pos, Fixed)
def MC045(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.24, 0.20, 0.02, rot, pos, Fixed)

# MC05X
def MC051(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.08, 0.24, 0.02, rot, pos, Fixed)
def MC052(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.12, 0.24, 0.02, rot, pos, Fixed)
def MC053(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.16, 0.24, 0.02, rot, pos, Fixed)
def MC054(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.20, 0.24, 0.02, rot, pos, Fixed)
def MC055(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.24, 0.24, 0.02, rot, pos, Fixed)

# MC09X
def MC091(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.08, 0.08, 0.06, rot, pos, Fixed)
def MC092(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.12, 0.12, 0.06, rot, pos, Fixed)
def MC093(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.16, 0.16, 0.06, rot, pos, Fixed)
def MC094(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.20, 0.20, 0.06, rot, pos, Fixed)
def MC095(rot = [0,0,0], pos = [0,0,0], Fixed = False):
    return MC0XX(0.24, 0.24, 0.06, rot, pos, Fixed)