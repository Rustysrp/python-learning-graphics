# Example file showing a basic pygame "game loop"
import pygame
import pymunk
import pymunk.pygame_util
import pymunk.constraints
import math
#import numpy as np
#import scipy.integrate as spi
#import scipy.integrate as odeint

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

    def __init__(self, space, position, lV, rV):
        # main vars
        self.mass = 23 # need to configure
        self.size = (20, 20*1.22857) # assuming units are mm, these are measured (scale as necessary)
        moment = pymunk.moment_for_box(self.mass, self.size)
        self.body = pymunk.Body(self.mass, moment)
        self.body.position = position
        self.shape = pymunk.Poly.create_box(self.body, self.size)
        self.shape.elasticity = 0.5
        space.add(self.body, self.shape)

        # Velocity
        self.leftVelocity = lV
        self.rightVelocity = rV

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

    def behavior(self, dT):
        translationalVel = (self.leftVelocity + self.rightVelocity) /2
        rotationalVel = (self.rightVelocity - self.leftVelocity) / self.size[1]

        goalVel = (translationalVel * math.cos(self.body.angle), translationalVel * math.sin(self.body.angle))

        # k factor for accel.
        # kFactor = translationalVel / rotationalVel # This seems like the right value, but makes collisions weird
        kFactor = 4 # temp while figuring out weird collision stuff

        aX = (kFactor*(goalVel[0] - self.body.velocity[0]) - self.body.angular_velocity * self.body.velocity[1]) * dT
        aY = (kFactor*(goalVel[1] - self.body.velocity[1]) + self.body.angular_velocity * self.body.velocity[0]) * dT

        self.body.angular_velocity += (rotationalVel - self.body.angular_velocity) * dT
        self.body.velocity += (aX, aY)

        print(self.body.velocity.length - translationalVel)



def magForce(BodyA, BodyB):
    print("")

# just temp -> finish later
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
# Some lists for storing behavior and data
objects = []

# Maze generation
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
    # Event processing
    keys = pygame.key.get_pressed() # Keep track of what keys are pressed
    for event in pygame.event.get():
        if keys[pygame.K_DOWN]:
            objects.append(falinks(space, (pygame.mouse.get_pos()), lV=70, rV=100))
        
        if event.type == pygame.MOUSEBUTTONDOWN:
            mazeCheck(maze, pygame.mouse.get_pos())
            
        if event.type == pygame.QUIT:
            running = False


    # Update object physics
    dT = (pygame.time.get_ticks()/1000-prev_time) # Calculate the difference in time between iterations
    for n in range(len(objects)):
        objects[n].behavior(dT) # Pass over dT for updating object behavior / physics

    # GONNA NEED TO DO THAT TIMESTEP THING SINCE ADDING TOO MANY OBJECTS MESSES UP FRAMERATE (& PHYSICS)

    # RENDERS
    screen.fill("white") # White background
    space.debug_draw(draw_options)

    # Display program
    pygame.display.flip()

    # UPDATE PHYSICS
    prev_time = pygame.time.get_ticks()/1000
    space.step(1/60.0)
    clock.tick(60)  # Limits FPS to 60

pygame.quit()