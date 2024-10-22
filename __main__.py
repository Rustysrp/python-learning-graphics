# Example file showing a basic pygame "game loop"
import pygame
import pymunk
import pymunk.pygame_util
import math
import numpy as np
import scipy.integrate as spi
import scipy.integrate as odeint

# current problems 10/18
# idk how accurate to real life, units may be issue, force applications seem to stack
# need to implement 'sensors' (or what they essentially do)
# idk how to hard-implement bodies, or make them exert forces on each other

# idea -> basically open space
# buttons to generate bodies, or placable
# changable maze (adjustable fidelity and spaces) -> needs to have collision but not move
# adjustable robot properties and behavior
# flexible for future projects
# output data (personal, do last)





dt = 0
res = (1000, 1000)

# pygame setup
pygame.init()
screen = pygame.display.set_mode(res)
clock = pygame.time.Clock()
running = True
space = pymunk.Space()
space.gravity = (0,0)

# Maybe make a class?
# Create a simple robot body (a rectangle)
class falinks(object):

    def __init__(self, space, position):
        # main vars
        self.mass = 23 # need to configure
        self.size = (20, 20*1.22857) # assuming units are mm, these are measured (scale as necessary)
        moment = pymunk.moment_for_box(self.mass, self.size)
        self.body = pymunk.Body(self.mass, moment)
        self.body.position = position
        self.shape = pymunk.Poly.create_box(self.body, self.size)
        self.shape.elasticity = 0.5
        space.add(self.body, self.shape)

        # Sensors
        self.magnet_radius = 10
        self.south_shape = pymunk.Circle(self.body, self.magnet_radius, offset=(0, self.size[1]/2))
        self.north_shape = pymunk.Circle(self.body, self.magnet_radius, offset=(0, -self.size[1]/2))
        self.south_shape.sensor = True
        self.north_shape.sensor = True
        self.south_shape.color = (0, 0, 255, 10)
        self.north_shape.color = (255, 0, 0, 10)
        space.add(self.south_shape)
        space.add(self.north_shape)

        # 'sensed' vars
        self.Ax = 0 # x velocity
        self.Ay = 0 # y velocity

    def southPos(self):
        return self.south_shape.body.position

    def northPos(self):
        return self.north_shape.body.position


    # set max, pass in % ?
    def velocityCalc(self, time, frequency):
        # frame time -> treat as degrees, convert to radians -> make sure it reaches full range
        # pass in frequency, which is a 'randomly' set number, specific to each own object
        return 2000*math.sin(frequency*time) # supposed to return the 'velocity'

def dirValue(number):
    if number == 0:
        return 1
    if number > 0:
        return 1
    else:
        return -1

# PROBABLY CHANGE TO GOAL FOR ANGULAR VELOCITY
def translationalPID(object, goalVel, time, prevError, integral):
    kP, kI, kD = 0.35, 0.001, 0.3

    error = goalVel - object.body.velocity.length
    pOut = kP * error
    
    integral += error * (pygame.time.get_ticks()/1000 - time)
    iOut = kI * integral

    derivative = (error - prevError) / ((pygame.time.get_ticks()/1000 - time))
    dOut = kD * derivative

    output = pOut + iOut + dOut

    return output, error, integral

def rotationalPID(object, goalVel, time, prevError, integral):
    kP, kD = 0.05, 0.3

    error = goalVel - object.body.angular_velocity
    pOut = kP * error
    
    derivative = (error - prevError) / ((pygame.time.get_ticks()/1000 - time))
    dOut = kD * derivative

    output = pOut + dOut

    return output, error, integral

# Just using a specific function so that I don't have to pass over values that will be the same every time
# need to figure out how to measure time difference
def falinksBehavior(objects, time):
    rTError = 0
    lTError = 0
    rTIntegral = 0
    lTIntegral = 0

    rRError = 0
    lRError = 0
    rRIntegral = 0
    lRIntegral = 0
    for n in range(len(objects)):
        # Movement PID and stuff like that
        rTFE = translationalPID(objects[n], 20, time, rTError, rTIntegral)
        rTError = rTFE[1]
        rTIntegral = rTFE[2]
        lTFE = translationalPID(objects[n], 20*0.9, time, lTError, lTIntegral)
        lTError = lTFE[1]
        lTIntegral = lTFE[2]

        # Rotational PID
        rRFE = rotationalPID(objects[n], (objects[n].size[1]/2)*(20-20*0.9), time, rRError, rRIntegral)
        rRError = rRFE[1]
        rRIntegral = rRFE[2]
        #lRFE = rotationalPID(objects[n], 10*dirValue(objects[n].body.angular_velocity), time, lRError, lRIntegral)
        #lRError = lRFE[1]
        #lRIntegral = lRFE[2]

        #print(rFE[0])
        #print(lFE[0])

        # translational movement forces
        objects[n].body.apply_force_at_local_point((rTFE[0]*objects[n].size[1]/2, 0), (0, objects[n].size[1]/2))
        objects[n].body.apply_force_at_local_point((lTFE[0]*objects[n].size[1]/2, 0), (0, -objects[n].size[1]/2))

        # rotational movement forces
        #objects[n].body.angular_velocity = rRFE[0]


        #print(objects[n].body.angular_velocity)

        # max speed
        #maxVelocity = 25.5
        #maxAngVelocity = maxVelocity * (abs(rFE[0]-lFE[0])/(rFE[0]+lFE[0]))
        #print(objects[n].body.angular_velocity)
        
        #if objects[n].body.velocity.length > maxVelocity:
            #objects[n].body.velocity = objects[n].body.velocity * (maxVelocity/objects[n].body.velocity.length)
        
        #if abs(objects[n].body.angular_velocity) > maxAngVelocity:
            #objects[n].body.angular_velocity = objects[n].body.angular_velocity * abs(maxAngVelocity/objects[n].body.angular_velocity)

        # need to determine how to actually calc friction 
        # friction forces 

        # take angel of body
        # apply magnitudes of x and y frictions forces at both wheels
        # they share direction and magnitude
        # Cr dir = at wheles, flipp of 'y' direction on body
        # MUs dir = at wheels, flip of 'x' direction on body
        Cr = 0.02
        MUs = 0.4
        if objects[n].body.angle > 90:
            if objects[n].body.angle < 270:
                objects[n].body.apply_force_at_local_point((-Cr, MUs), (0, objects[n].size[1]/2))
                objects[n].body.apply_force_at_local_point((-Cr, MUs), (0, -objects[n].size[1]/2))
        else:
            objects[n].body.apply_force_at_local_point((-Cr, -MUs), (0, objects[n].size[1]/2))
            objects[n].body.apply_force_at_local_point((-Cr, -MUs), (0, -objects[n].size[1]/2))


        print(objects[n].body.velocity)

        # magnet connectionsC:w
        # 1. check what poles are interacting b/t two objects
        # 2. identify if pushing / pulling, and make vector normal to the side's orientation
        # 3. apply each other's normal vector direction * direction (+/-) * magnet strength
        # 4. try to figure wtf to do for disconnection

        # DID WRONG JUST APPLYING FORCE TO NORMAL OF THE BODY when it should be towards other body, AND NOT WORKINGw
        '''
        mag_force = 1000
        for i in range(len(objects)):
            if n != i:
                if abs(objects[n].southPos()[0]-objects[i].southPos()[0]) < objects[n].south_shape.radius:
                    if abs(objects[n].southPos()[1]-objects[i].southPos()[1]) < objects[n].south_shape.radius:
                        i_pos = (objects[i].southPos()[0], objects[i].southPos()[1])
                        i_angle = objects[i].body.angle
                        i_direction = pymunk.Vec2d(math.cos(i_angle), math.sin(i_angle))

                        n_pos = (objects[n].southPos()[0], objects[n].southPos()[1])
                        n_angle = objects[n].body.angle
                        n_direction = pymunk.Vec2d(math.cos(n_angle), math.sin(n_angle))

                        i_normal = pymunk.Vec2d(((i_pos[0]-n_pos[0])*dirValue(i_pos[0]-n_pos[0]))*i_direction[0], ((i_pos[1]-n_pos[1])*dirValue(i_pos[1]-n_pos[1]))*i_direction[1])
                        n_normal = pymunk.Vec2d(((n_pos[0]-i_pos[0])*dirValue(n_pos[0]-i_pos[0]))*n_direction[0], ((n_pos[1]-i_pos[1])*dirValue(n_pos[1]-i_pos[1]))*n_direction[1])
                        objects[n].body.apply_force_at_local_point((n_normal)*mag_force, (0, objects[n].size[1]/2))
                        objects[i].body.apply_force_at_local_point((i_normal)*mag_force, (0, objects[i].size[1]/2))
                        print("i normal\n")
                        print(i_normal)
                        print("\nn normal\n")
                        print(n_normal)

                if abs(objects[n].southPos()[0]-objects[i].northPos()[0]) < objects[n].south_shape.radius:
                    if abs(objects[n].southPos()[1]-objects[i].northPos()[1]) < objects[n].south_shape.radius:
                        i_pos = (objects[i].northPos()[0], objects[i].northPos()[1])
                        i_angle = objects[i].body.angle
                        i_direction = pymunk.Vec2d(math.cos(i_angle), math.sin(i_angle))

                        n_pos = (objects[n].southPos()[0], objects[n].southPos()[1])
                        n_angle = objects[n].body.angle
                        n_direction = pymunk.Vec2d(math.cos(n_angle), math.sin(n_angle))

                        i_normal = pymunk.Vec2d(((i_pos[0]-n_pos[0])*dirValue(i_pos[0]-n_pos[0]))*i_direction[0], ((i_pos[1]-n_pos[1])*dirValue(i_pos[1]-n_pos[1]))*i_direction[1])
                        n_normal = pymunk.Vec2d(((n_pos[0]-i_pos[0])*dirValue(n_pos[0]-i_pos[0]))*n_direction[0], ((n_pos[1]-i_pos[1])*dirValue(n_pos[1]-i_pos[1]))*n_direction[1])
                        objects[n].body.apply_force_at_local_point(-(n_normal)*mag_force, (0, objects[n].size[1]/2))
                        objects[i].body.apply_force_at_local_point(-(i_normal)*mag_force, (0, -objects[i].size[1]/2))

                if abs(objects[n].northPos()[0]-objects[i].southPos()[0]) < objects[n].north_shape.radius:
                    if abs(objects[n].northPos()[1]-objects[i].southPos()[1]) < objects[n].north_shape.radius:
                        i_pos = (objects[i].southPos()[0], objects[i].southPos()[1])
                        i_angle = objects[i].body.angle
                        i_direction = pymunk.Vec2d(math.cos(i_angle), math.sin(i_angle))

                        n_pos = (objects[n].northPos()[0], objects[n].northPos()[1])
                        n_angle = objects[n].body.angle
                        n_direction = pymunk.Vec2d(math.cos(n_angle), math.sin(n_angle))

                        i_normal = pymunk.Vec2d(((i_pos[0]-n_pos[0])*dirValue(i_pos[0]-n_pos[0]))*i_direction[0], ((i_pos[1]-n_pos[1])*dirValue(i_pos[1]-n_pos[1]))*i_direction[1])
                        n_normal = pymunk.Vec2d(((n_pos[0]-i_pos[0])*dirValue(n_pos[0]-i_pos[0]))*n_direction[0], ((n_pos[1]-i_pos[1])*dirValue(n_pos[1]-i_pos[1]))*n_direction[1])
                        objects[n].body.apply_force_at_local_point(-(n_normal)*mag_force, (0, -objects[n].size[1]/2))
                        objects[i].body.apply_force_at_local_point(-(i_normal)*mag_force, (0, objects[i].size[1]/2))

                if abs(objects[n].northPos()[0]-objects[i].northPos()[0]) < objects[n].north_shape.radius:
                    if abs(objects[n].northPos()[1]-objects[i].northPos()[1]) < objects[n].north_shape.radius:
                        i_pos = (objects[i].northPos()[0], objects[i].northPos()[1])
                        i_angle = objects[i].body.angle
                        i_direction = pymunk.Vec2d(math.cos(i_angle), math.sin(i_angle))

                        n_pos = (objects[n].northPos()[0], objects[n].northPos()[1])
                        n_angle = objects[n].body.angle
                        n_direction = pymunk.Vec2d(math.cos(n_angle), math.sin(n_angle))

                        i_normal = pymunk.Vec2d(((i_pos[0]-n_pos[0])*dirValue(i_pos[0]-n_pos[0]))*i_direction[0], ((i_pos[1]-n_pos[1])*dirValue(i_pos[1]-n_pos[1]))*i_direction[1])
                        n_normal = pymunk.Vec2d(((n_pos[0]-i_pos[0])*dirValue(n_pos[0]-i_pos[0]))*n_direction[0], ((n_pos[1]-i_pos[1])*dirValue(n_pos[1]-i_pos[1]))*n_direction[1])
                        objects[n].body.apply_force_at_local_point((n_normal)*mag_force, (0, -objects[n].size[1]/2))
                        objects[i].body.apply_force_at_local_point((i_normal)*mag_force, (0, -objects[i].size[1]/2))
            '''

        



def updateMovement(objects):
    for x in objects:
        x.body.apply_force_at_local_point((0, -10000), (0, 0))

# just a test
def applyGrav(objects):
    for x in objects:
        x.body.apply_force_at_local_point((0, (x.mass*500)), (0, 10))
        x.body.apply_force_at_local_point((0, (x.mass*500)), (0, -10))

def mazeDefine():
    print("finish")

# turn on and off maze parts
def mazeCheck(mazeObjects, mousePos):
    for n in range(len(mazeObjects)):
        if mousePos[0] >= mazeObjects[n].offset[0] - mazeObjects[n].radius:
            if mousePos[0] <= mazeObjects[n].offset[0] + mazeObjects[n].radius:
                if mousePos[1] >= mazeObjects[n].offset[1] - mazeObjects[n].radius:
                    if mousePos[1] <= mazeObjects[n].offset[1] + mazeObjects[n].radius:
                        mazeObjects[n].sensor = not mazeObjects[n].sensor
                        if mazeObjects[n].sensor == True:
                            mazeObjects[n].color = (100, 255, 0, 10)
                        else:
                            mazeObjects[n].color = (255, 100, 0, 10)

# ratio for falinks body ~43mm long 35mm wide: length = 1.22857 * width
#misc variables
rect_size = 400

# Some lists for storing behavior and data
objects = []

# create main test area

# Make into callable function
mazeDimension = 800
baseX = 100
baseY = 100
nem = 10
maze = []
b = -0.5
for n in range(nem*nem):
    if (n%nem) == 0:
        b = b + 1
    maze.append(pymunk.Circle(space.static_body, 10, (baseX+(n%nem+0.5)*(mazeDimension/nem), baseY+b*(mazeDimension/nem))))
    maze[n].dampening = 0.9
    maze[n].sensor = True
    maze[n].color = (100, 255, 0, 10)
    space.add(maze[n])


points = [(baseX, baseY), (baseX+mazeDimension, baseY), (baseX+mazeDimension, baseY+mazeDimension), (baseX, baseY+mazeDimension)]
for i in range(len(points)):
    seg = pymunk.Segment(space.static_body, points[i], points[(i+1)%4], 2)
    seg.elasticity = 0.9
    space.add(seg)

# MAIN LOOP





draw_options = pymunk.pygame_util.DrawOptions(screen)
prev_time = 0
while running:

    keys = pygame.key.get_pressed()

    # EVENTS & PROCCESSING
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if keys[pygame.K_UP]:
                objects[n].body.apply_force_at_local_point((10000, 0), (0, objects[n].size[1]/2))
                objects[n].body.apply_force_at_local_point((9000, 0), (0, -objects[n].size[1]/2))

        if keys[pygame.K_DOWN]:
            objects.append(falinks(space, (pygame.mouse.get_pos())))

        if event.type == pygame.QUIT:
            running = False
        
        if event.type == pygame.MOUSEBUTTONDOWN:
            mazeCheck(maze, pygame.mouse.get_pos())

    falinksBehavior(objects, prev_time)

    # RENDERS
    screen.fill("white") # White background
    space.debug_draw(draw_options)

    # Display program
    pygame.display.flip()

    # UPDATE PHYSICS
    prev_time = pygame.time.get_ticks()/1000
    space.step(1/60.0)
    clock.tick(60)  # Limits FPS to 60
    dt = clock.tick(60) / 1000 # Update dt for calculations

pygame.quit()