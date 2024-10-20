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
res = (1280, 1000)

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
        self.mass = 50 # need to configure
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
        return self.south_shape.position + self.south_shape.rotation_vector.rotated((0, self.size[1]/2))

    def northPos(self):
        return self.north_shape.position + self.north_shape.rotation_vector.rotated((0, -self.size[1]/2))


    # set max, pass in % ?
    def velocityCalc(self, time, frequency):
        # frame time -> treat as degrees, convert to radians -> make sure it reaches full range
        # pass in frequency, which is a 'randomly' set number, specific to each own object
        return 200*math.sin(frequency*time) # supposed to return the 'velocity'



# Just using a specific function so that I don't have to pass over values that will be the same every time
# need to figure out how to measure time difference
def falinksBehavior(objects, time):
    for n in range(len(objects)):
        currT = pygame.time.get_ticks()/1000
        lvel = objects[n].velocityCalc(time, 1)
        rvel = objects[n].velocityCalc(time, 2)
        laccel = lvel*(currT-time)
        raccel = rvel*(currT-time)

        objects[n].body.apply_force_at_local_point((1000*(objects[n].mass*raccel), 0), (0, 10))
        objects[n].body.apply_force_at_local_point((1000*(objects[n].mass*laccel), 0), (0, -10))

def updateMovement(objects):
    for x in objects:
        x.body.apply_force_at_local_point((0, -10000), (0, 0))

# just a test
def applyGrav(objects):
    for x in objects:
        x.body.apply_force_at_local_point((0, (x.mass*500)), (0, 10))
        x.body.apply_force_at_local_point((0, (x.mass*500)), (0, -10))

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
mazeDimension = 500
baseX = 200
baseY = 200
nem = 10
maze = []
b = -0.5
for n in range(nem*nem):
    if (n%nem) == 0:
        b = b + 1
    maze.append(pymunk.Circle(space.static_body, 10, (baseX+(n%nem+0.5)*(mazeDimension/nem), baseY+b*(mazeDimension/nem))))
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
while running:

    keys = pygame.key.get_pressed()

    # EVENTS & PROCCESSING
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if keys[pygame.K_UP]:
            updateMovement(objects)

        if keys[pygame.K_DOWN]:
            objects.append(falinks(space, (pygame.mouse.get_pos())))

        if event.type == pygame.QUIT:
            running = False
        
        if event.type == pygame.MOUSEBUTTONDOWN:
            mazeCheck(maze, pygame.mouse.get_pos())

    falinksBehavior(objects, pygame.time.get_ticks()/1000)
    # applyGrav(objects)

    # make list of objects
    # append object list when creating
    # updateMovement()... // applyForces() (n) -> update properties
    # applyCollisions() (n^2)
    # then render...
    
    # print(pygame.time.get_ticks())
    print(maze[0].offset)

    # RENDERS
    screen.fill("white") # White background
    space.debug_draw(draw_options)

    # Display program
    pygame.display.flip()

    # UPDATE PHYSICS
    space.step(1/60.0)
    clock.tick(60)  # Limits FPS to 60
    dt = clock.tick(60) / 1000 # Update dt for calculations

pygame.quit()