import pygame
import random
import math

from tile_renderer import TileRenderer


WIDTH = 1500
HEIGHT = 1000
FPS = 60

# Define Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)

## initialize pygame and create window
pygame.init()
pygame.mixer.init()  ## For sound
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Hex Rendering")
clock = pygame.time.Clock()     ## For syncing the FPS

### Rendering constants ###
TILE_BUFFER = 1 # px between drawn tiles
###########################

### Project constants ###
ROOF_WIDTH  = 5 # in
ROOF_HEIGHT = 10 # in

TILE_WIDTH  = 12
TILE_HEIGHT = 12
#########################

MARGINS = 50

tile_renderer = TileRenderer(WIDTH , HEIGHT , ROOF_HEIGHT, ROOF_WIDTH, TILE_BUFFER, MARGINS)

## Game loop
running = True
while running:

    #1 Process input/events
    clock.tick(FPS)     ## will make the loop run at the same speed all the time
    for event in pygame.event.get():        # gets all the events which have occured till now and keeps tab of them.
        ## listening for the the X button at the top
        if event.type == pygame.QUIT:
            running = False

    #3 Draw/render
    screen.fill(WHITE)

    tile_renderer.drawRoof(screen)
    screen.blit(pygame.transform.flip(screen, False, True), (0, 0))

    ## Done after drawing everything to the screen
    pygame.display.flip()

pygame.quit()
