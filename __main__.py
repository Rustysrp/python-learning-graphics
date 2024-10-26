# Example file showing a basic pygame "game loop"
import pygame
import pymunk
import pymunk.pygame_util
import pymunk.constraints
import math
#import numpy as np
#import scipy.integrate as spi
#import scipy.integrate as odeint

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1000, 1000))
clock = pygame.time.Clock()
running = True
space = pymunk.Space()
space.gravity = (0, 0)

# Maybe make a class?
# Create a simple robot body (a rectangle)
class falinks(object):
    def __init__(self, space, position, lV, rV):
        # main vars
        ratio = 1.22857 # Ratio for width:length, 1:1.22857
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

    def movement(self, dT):
        # Calculate the translational and rotational velocities based on the left and right wheel velocities
        translationalVel = (self.leftVelocity + self.rightVelocity) / 2
        rotationalVel = (self.rightVelocity - self.leftVelocity) / self.size[1]

        # Desired translational velocity for the body
        goalVel = (translationalVel * math.cos(self.body.angle), translationalVel * math.sin(self.body.angle))

        # k factor for accel.
        kFactor = self.size[1] # Multiplyer value, idk what it should be but it seems this works

        # Calculate translational rotation
        aX = (kFactor * (goalVel[0] - self.body.velocity[0]) - self.body.angular_velocity * self.body.velocity[1]) * dT
        aY = (kFactor * (goalVel[1] - self.body.velocity[1]) + self.body.angular_velocity * self.body.velocity[0]) * dT

        self.body.angular_velocity += (rotationalVel - self.body.angular_velocity) * dT # Apply angular rotation
        self.body.velocity += (aX, aY) # Apply translational acceleration

    def magnetForce(dT):
        return dT # Temporary before working on function

    def behavior(self, dT):
        self.movement(dT)
        self.magnetForce(dT)


def magForce(BodyA, BodyB):
    print("")

# Function for turning off and on maze nodes, checking where the mouse position is (called after clicking)
# Will have to adjust for square nodes
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


# Maze generation -> Maybe make into callable function later on but prob. isn't necessary
mazeDimension = 800 # How big the square will be (maze)
mazePoint = (100, 100) # Initial position, top left of maze
nem = 10 # Side number of maze nodes (ttl # of maze nodes = nem*nem)
maze = [] # Initialize maze object list
spacing = -0.5 # Initial spacing of the nodes to make even distribution
for n in range(nem*nem):
    if (n%nem) == 0:
        spacing = spacing + 1
    maze.append(pymunk.Circle(space.static_body, 10, (mazePoint[0]+(n%nem+0.5)*(mazeDimension/nem), mazePoint[1]+spacing*(mazeDimension/nem))))
    # Some maze aspects
    maze[n].dampening = 0.9
    maze[n].sensor = True
    maze[n].color = (100, 255, 0, 10) # Initialize the default colors for disable nodes
    space.add(maze[n]) # Add the maze to the space

# Top left of the box to encapsulate objects
points = [(mazePoint[0], mazePoint[1]), (mazePoint[0]+mazeDimension, mazePoint[0]), (mazePoint[0]+mazeDimension, mazePoint[1]+mazeDimension), (mazePoint[0], mazePoint[1]+mazeDimension)] # Corner points
for i in range(len(points)): # Generate the segments
    seg = pymunk.Segment(space.static_body, points[i], points[(i+1)%4], 2)
    seg.elasticity = 0.9
    space.add(seg) # Add the segments

# Set some variables
dT = 0 # Initialize timestep variable
objects = [] # Initialize object list

# Main loop
while running:
    # PROCESS EVENTS
    keys = pygame.key.get_pressed() # Keep track of what keys are pressed
    for event in pygame.event.get():
        if keys[pygame.K_DOWN]:
            objects.append(falinks(space, (pygame.mouse.get_pos()), lV=70, rV=100))
        
        if event.type == pygame.MOUSEBUTTONDOWN:
            mazeCheck(maze, pygame.mouse.get_pos())
            
        if event.type == pygame.QUIT:
            running = False

    # UPDATE OBJECT PHYSICS
    for n in range(len(objects)):
        objects[n].behavior(pygame.time.get_ticks()/1000 - dT) # Pass over dT for updating object behavior / physics

    # GONNA NEED TO DO THAT TIMESTEP THING SINCE ADDING TOO MANY OBJECTS MESSES UP FRAMERATE (& THEREFORE PHYSICS)

    # RENDERS
    screen.fill("white") # White background
    space.debug_draw(pymunk.pygame_util.DrawOptions(screen)) # Draw the pymunk space
    pygame.display.flip() # Display the program

    # UPDATE PHYSICS VARIABLES
    dT = pygame.time.get_ticks()/1000 # Record what the previous time will be to calculate dT for physics
    space.step(1/60.0) # Step the pymunk space 
    clock.tick(60)  # Limits FPS to 60

pygame.quit()