# Example file showing a basic pygame "game loop"
import pygame
import pymunk
import pymunk.pygame_util
import math
import numpy as np
import scipy.integrate as spi
import scipy.integrate as odeint

# current problems 10/18

# idea -> basically open space
# buttons to generate bodies, or placable
# changable maze (adjustable fidelity and spaces) -> needs to have collision but not move
# adjustable robot properties and behavior
# flexible for future projects
# output data (personal, do last)





dt = 0
res = (1280, 720)

# pygame setup
pygame.init()
screen = pygame.display.set_mode(res)
clock = pygame.time.Clock()
running = True
space = pymunk.Space()
space.gravity = (0,0)

# Maybe make a class?
# Create a simple robot body (a rectangle)

# Just using a specific function so that I don't have to pass over values that will be the same every time
def createFalinks(space, position):
    mass = 1 # need to configure
    size = (20, 20*1.22857) # assuming units are mm, these are measured (scale as necessary)
    moment = pymunk.moment_for_box(mass, size)
    body = pymunk.Body(mass, moment)
    body.position = position
    shape = pymunk.Poly.create_box(body, size)
    shape.elasticity = 0.5
    space.add(body, shape)

    return shape

def updateMovement(objects):
    for x in objects:
        x.body.apply_force_at_local_point((10000, 10000), (0, 0))

# ratio for falinks body ~43mm long 35mm wide: length = 1.22857 * width
#misc variables
rect_size = 400

class button(object):
    def __init__(self, x, y, width, height, color):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color
        self.rect = pygame.Rect(x, y, width, height)
    
    def pressed(self, mouse):
        if mouse[0] > self.rect.topleft[0]:
            if mouse[1] > self.rect.topleft[1]:
                if mouse[0] < self.rect.bottomright[0]:
                    if mouse[1] < self.rect.bottomright[1]:
                        return True
                    else: return False
                else: return False
            else: return False
        else: return False

    def draw(self):
        pygame.draw.rect(screen, self.color, self.rect, width=0)


objects = [0]
objects[0] = createFalinks(space, (300, 300))


# create main test area

points = [(200, 200), (600, 200), (600, 600), (200, 600)]
for i in range(len(points)):
    seg = pymunk.Segment(space.static_body, points[i], points[(i+1)%4], 2)
    seg.elasticity = 0.9
    space.add(seg)

# MAIN LOOP

draw_options = pymunk.pygame_util.DrawOptions(screen)
while running:

    # SET SPACE
    testbutton = button(20, 20, 20, 20, "black") # Create a button to be clicked

    keys = pygame.key.get_pressed()

    # EVENTS & PROCCESSING
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse = pygame.mouse.get_pos()
            # if testbutton.pressed(mouse):
            #     print('hit')
        
        if keys[pygame.K_UP]:
            updateMovement(objects)

        if keys[pygame.K_DOWN]:
            objects.append(createFalinks(space, pygame.mouse.get_pos()))

        if event.type == pygame.QUIT:
            running = False

    # make list of objects
    # append object list when creating
    # updateMovement()... // applyForces() (n) -> update properties
    # applyCollisions() (n^2)
    # then render...
    

    jaw = pygame.Rect(400, 400, 300, 400)


    # RENDERS
    screen.fill("white") # White background
    testbutton.draw()
    space.debug_draw(draw_options)

    # Display program
    pygame.display.flip()

    # UPDATE PHYSICS
    space.step(1/60.0)
    clock.tick(60)  # Limits FPS to 60
    dt = clock.tick(60) / 1000 # Update dt for calculations

pygame.quit()